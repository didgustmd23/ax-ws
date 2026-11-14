"""A3 지연 시험에서만 요청별 잠금 대기와 SQLite 시간을 수집합니다."""
import threading
import time


class TimedLock:
    """기존 잠금 순서와 동작을 바꾸지 않고 획득 대기만 측정합니다."""

    def __init__(self, original, profiler):
        self.original, self.profiler = original, profiler

    def __enter__(self):
        begin = time.perf_counter()
        self.original.acquire()
        self.profiler.add('ledger_lock_wait_s', time.perf_counter() - begin)
        return self

    def __exit__(self, *args):
        self.original.release()


class TimedDatabase:
    """실제 SQLite 실행을 그대로 위임하고 문장 종류별 소요 시간을 수집합니다."""

    def __init__(self, original, profiler):
        self.original, self.profiler = original, profiler

    def execute(self, sql, *args, **kwargs):
        word = sql.lstrip().split()[0].upper()
        stage = {'BEGIN': 'begin_s', 'SELECT': 'select_s', 'INSERT': 'write_s',
                 'UPDATE': 'write_s', 'COMMIT': 'commit_s', 'ROLLBACK': 'rollback_s'}.get(word)
        begin = time.perf_counter()
        try:
            return self.original.execute(sql, *args, **kwargs)
        finally:
            if stage:
                self.profiler.add(stage, time.perf_counter() - begin)

    def __getattr__(self, name):
        return getattr(self.original, name)


class LeaseProfiler:
    """시험 객체에만 진단기를 연결합니다. 운영 노드의 판정 기준과 응답은 바꾸지 않습니다."""

    def __init__(self, checker):
        self.local = threading.local()
        self.rows = []
        self.rows_lock = threading.Lock()
        self.original_check = checker.check
        checker.ledger.lock = TimedLock(checker.ledger.lock, self)
        checker.ledger.db = TimedDatabase(checker.ledger.db, self)
        checker.check = self.check

    def add(self, name, elapsed):
        row = getattr(self.local, 'row', None)
        if row is not None:
            row[name] = row.get(name, 0.0) + elapsed

    def check(self, request):
        row = {'request_id': request.request_id, 'thread_id': threading.get_ident()}
        self.local.row = row
        begin = time.perf_counter()
        try:
            return self.original_check(request)
        finally:
            row['server_s'] = time.perf_counter() - begin
            with self.rows_lock:
                self.rows.append(row)
            self.local.row = None

    def report(self, client_samples):
        """같은 요청 ID로 클라이언트 왕복과 서버 상세 시간을 결합합니다."""
        client = {sample[4]: sample[0] for sample in client_samples}
        rows = []
        for source in self.rows:
            row = dict(source)
            row['roundtrip_s'] = client.get(row['request_id'])
            if row['roundtrip_s'] is not None:
                # 전송·실행기 진입·응답 전달 등이 합쳐진 잔여 시간이며 개별 구간을 확정하지 않습니다.
                row['outside_server_s'] = max(0.0, row['roundtrip_s'] - row['server_s'])
            row['inside_without_sql_or_ledger_wait_s'] = max(
                0.0, row['server_s'] - sum(row.get(key, 0.0) for key in (
                    'ledger_lock_wait_s', 'begin_s', 'select_s', 'write_s', 'commit_s', 'rollback_s')))
            rows.append(row)
        stages = {}
        for key in ('roundtrip_s', 'server_s', 'ledger_lock_wait_s', 'begin_s', 'select_s',
                    'write_s', 'commit_s', 'outside_server_s', 'inside_without_sql_or_ledger_wait_s'):
            values = sorted(row[key] for row in rows if row.get(key) is not None)
            if values:
                stages[key] = {'p50_s': values[len(values) // 2],
                               'p99_s': values[max(0, int(len(values) * .99) - 1)],
                               'max_s': values[-1]}
        return {'scope': '시험 전용 계측; 각 구간 p99끼리는 더할 수 없음',
                'stages': stages, 'requests': rows}
