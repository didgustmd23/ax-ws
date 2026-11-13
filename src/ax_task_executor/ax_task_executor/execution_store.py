"""모의 작업의 입력과 완료 결과를 SQLite에 기록하고 중복 실행을 막습니다."""
import json
from pathlib import Path
import sqlite3
import threading


class ExecutionStore:
    def __init__(self, path, robot_id):
        # 역할: 설정과 내부 상태를 준비합니다.
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.robot_id = robot_id
        self.lock = threading.Lock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute("""CREATE TABLE IF NOT EXISTS executions (
            robot TEXT NOT NULL, id TEXT NOT NULL, request TEXT NOT NULL,
            mock_load TEXT NOT NULL, result TEXT,
            PRIMARY KEY (robot, id))""")
        self.db.commit()

    def reserve(self, execution_id, request, mock_load):
        # 작업 전에 주문 내용을 저장합니다. 같은 번호로 다른 내용을 보내면 접수하지 않습니다.
        # 역할: 작업 인자를 기록하고 중복 실행은 기존 결과로 연결합니다.
        with self.lock, self.db:
            self.db.execute('BEGIN IMMEDIATE')
            row = self.db.execute(
                'SELECT request FROM executions WHERE robot=? AND id=?',
                (self.robot_id, execution_id)).fetchone()
            if row is not None:
                return row[0] == request
            self.db.execute('INSERT INTO executions VALUES (?, ?, ?, ?, NULL)',
                            (self.robot_id, execution_id, request, mock_load))
            return True

    def result(self, execution_id, make_result):
        # 저장 결과가 있으면 재사용합니다. 이 모의 복구 방식을 실제 집기 재실행에 사용하면 안 됩니다.
        # 순수 모의 계산을 트랜잭션에 포함해 동시 요청과 프로세스가 결과 하나를 공유합니다.
        # 이 복구 정책은 실제 물리 명령에 사용하지 않습니다.
        # 역할: 저장 결과를 읽거나 결과 생성 함수를 한 번 적용해 기록합니다.
        with self.lock, self.db:
            self.db.execute('BEGIN IMMEDIATE')
            row = self.db.execute(
                'SELECT mock_load, result FROM executions WHERE robot=? AND id=?',
                (self.robot_id, execution_id)).fetchone()
            if row is None:
                raise RuntimeError('Execution was not durably accepted')
            if row[1] is not None:
                return json.loads(row[1])
            result = make_result(row[0])
            self.db.execute('UPDATE executions SET result=? WHERE robot=? AND id=?',
                            (json.dumps(result), self.robot_id, execution_id))
            return result

    def cached_result(self, execution_id):
        """결과를 새로 계산하지 않고 완료된 기록을 확인한다."""
        with self.lock:
            row = self.db.execute(
                'SELECT result FROM executions WHERE robot=? AND id=?',
                (self.robot_id, execution_id)).fetchone()
            return json.loads(row[0]) if row and row[0] is not None else None

    def close(self):
        # 역할: 보유한 원장과 내부 자원을 안전하게 닫습니다.
        self.db.close()
