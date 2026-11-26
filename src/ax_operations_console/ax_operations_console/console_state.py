"""관제 화면 상태 판정·조립(ROS 없음). 시계는 호출자가 주입한다.

표시 상태 5종: missing(미수집) / ok(정상) / stale(만료) / invalid(무효) / no_source(원천 없음).

계약 2.3 소비자 규칙을 그대로 따른다.
- (source_session_id, revision)으로 비교한다. 같은 세션에서 revision이 줄면 버린다.
- 같은 revision이 다시 오면 내용은 바꾸지 않고 stamp와 수신 시각만 갱신한다(주기 재발행).
- 같은 세션에서 stamp가 줄면 버린다. 새 세션의 첫 메시지를 받은 뒤 이전 세션 메시지는 버린다.
- 미래 stamp(현재 sim time + 허용 오차 초과)와 stamp 0은 받지 않는다.
- 표본 나이(sim time)와 무수신 시간(CLOCK_BOOTTIME)을 둘 다 검사해, 어느 쪽이든 만료 기준을 넘으면 만료다.
- valid=false, revision=0이면 무효다.

표시 여유: 판정(status)은 계약 만료값 그대로다. 화면 깜박임을 줄이기 위해 display_status만
만료 직후 display_grace_s 동안 직전 정상 표시를 유지한다. 두 값을 함께 내보낸다.
"""
from dataclasses import dataclass, field

STATUSES = ('missing', 'ok', 'stale', 'invalid', 'no_source')

# 계약 만료(s). pose·battery는 계약 4절 표 밖의 표시 전용 입력이라 표시 기준을 따로 둔다(README).
DEFAULT_LIMITS = {
    'actuation': 0.3,
    'safety': 0.5,
    'health': 1.0,
    'executor': 3.0,
    'pose': 5.0,
    'battery': 5.0,
    'tasks': 3.0,
}
ROBOT_KINDS = ('actuation', 'safety', 'health', 'executor', 'pose', 'battery')
META_KINDS = ('actuation', 'safety', 'health', 'executor', 'tasks')

# enum 이름표(ax_interfaces 상수와의 일치는 시험이 확인한다)
ACTUATION_NAMES = ('UNKNOWN', 'RUNNING', 'STOP_REQUESTED', 'STOPPING', 'STOPPED', 'FAULT')
SAFETY_NAMES = ('CLEAR', 'SLOWDOWN_REQUIRED', 'STOP_REQUIRED', 'UNKNOWN')
HEALTH_NAMES = ('UNKNOWN', 'OK', 'DEGRADED', 'LOST')
TASK_NAMES = ('UNSPECIFIED', 'QUEUED', 'ASSIGNED', 'RUNNING', 'COMPLETED', 'FAILED', 'CANCELED',
              'RETRY_PENDING')
TRANSFER_NAMES = ('UNKNOWN', 'NONE', 'LOCKED', 'IN_PROGRESS', 'UNLOCKED')
RESULT_NAMES = ('UNSPECIFIED', 'SUCCEEDED', 'FAILED', 'CANCELED', 'TIMED_OUT', 'UNCERTAIN')
TERMINAL_TASK_STATES = ('COMPLETED', 'FAILED', 'CANCELED')
NOT_RUNNING = ('FAULT', 'UNKNOWN', None)


# 숫자 enum → 이름(범위 밖은 '?값')
def enum_name(names, value):
    if 0 <= value < len(names):
        return names[value]

    return f'?{value}'


@dataclass
class Sample:
    """수신 메시지 하나의 요약. meta가 없는 표준 메시지는 session·revision이 None이다."""

    value: object
    stamp: float
    session: str = None
    revision: int = None
    valid: bool = True
    reason: str = ''
    source_id: str = ''
    schema: int = None


@dataclass
class Slot:
    """입력 하나의 최근 값과 수신 기록."""

    kind: str
    sample: Sample = None
    recv_boot: float = None
    retired_sessions: set = field(default_factory=set)
    counts: dict = field(default_factory=lambda: {
        'accepted': 0, 'republished': 0, 'old_revision': 0, 'old_stamp': 0,
        'old_session': 0, 'future_stamp': 0, 'zero_stamp': 0})

    # 새 표본 반영. 반영하면 True, 버리면 False(사유는 counts에 센다)
    def update(self, sample, now_sim, now_boot, future_tolerance_s):

        # 시각 검사: stamp 0, 미래 stamp(시계가 유효할 때만 비교)
        if sample.stamp <= 0.0:
            self.counts['zero_stamp'] += 1
            return False

        if now_sim > 0.0 and sample.stamp > now_sim + future_tolerance_s:
            self.counts['future_stamp'] += 1
            return False

        previous = self.sample

        # 세션 검사: 끝난 세션의 늦은 메시지는 버리고, 새 세션이면 이전 세션을 끝난 것으로 기록
        if sample.session is not None:
            if sample.session in self.retired_sessions:
                self.counts['old_session'] += 1
                return False

            if previous is not None and previous.session not in (None, sample.session):
                self.retired_sessions.add(previous.session)
                previous = None

        # 같은 세션(또는 meta 없음) 안에서 stamp·revision 순서 검사
        if previous is not None:
            if sample.stamp < previous.stamp:
                self.counts['old_stamp'] += 1
                return False

            if sample.revision is not None and previous.revision is not None:
                if sample.revision < previous.revision:
                    self.counts['old_revision'] += 1
                    return False

                # 같은 revision 재발행: 내용은 그대로, stamp·수신 시각만 갱신
                if sample.revision == previous.revision:
                    previous.stamp = sample.stamp
                    self.recv_boot = now_boot
                    self.counts['republished'] += 1
                    return True

        self.sample = sample
        self.recv_boot = now_boot
        self.counts['accepted'] += 1
        return True


@dataclass
class Clocks:
    """판정 시각 묶음. sim은 노드 clock(use_sim_time), boot는 CLOCK_BOOTTIME."""

    sim: float
    boot: float


# 슬롯 하나 판정 → 화면 전달 dict(value는 표시 상태가 ok일 때만 넣는다)
def judge(slot, clocks, limit_s, display_grace_s):
    out = {'limit_s': limit_s, 'counts': dict(slot.counts)}

    if slot.sample is None:
        out.update(status='missing', display_status='missing')
        return out

    sample = slot.sample
    recv_age = clocks.boot - slot.recv_boot
    stamp_age = clocks.sim - sample.stamp if clocks.sim > 0.0 else None
    out.update(recv_age_s=round(recv_age, 3),
               stamp_age_s=None if stamp_age is None else round(stamp_age, 3),
               meta=meta_view(sample))

    # 판정 순서: 시계 무효 → 만료 → 생산자 무효 → 정상
    if clocks.sim <= 0.0:
        status, reason = 'invalid', 'CLOCK_INVALID'
    elif recv_age > limit_s or stamp_age > limit_s:
        status, reason = 'stale', 'STALE'
    elif not sample.valid:
        status, reason = 'invalid', sample.reason or 'PRODUCER_INVALID'
    elif sample.revision == 0:
        status, reason = 'invalid', 'REVISION_ZERO'
    else:
        status, reason = 'ok', ''

    # 표시 여유: 만료 기준을 넘은 지 display_grace_s 미만일 때만 직전 정상 표시 유지(판정 status는 그대로).
    # 넘은 정도는 수신 경과·표본 나이 중 큰 쪽 기준이라 폴링 시점과 무관하다
    display = status
    if status == 'stale' and sample.valid and sample.revision != 0:
        overdue = max(recv_age, stamp_age) - limit_s
        if overdue < display_grace_s:
            display = 'ok'

    out.update(status=status, reason=reason, display_status=display)

    if display == 'ok':
        out['value'] = sample.value

    return out


# 화면용 meta 요약
def meta_view(sample):
    if sample.session is None:
        return None

    return {'source_id': sample.source_id, 'session': sample.session, 'revision': sample.revision,
            'valid': sample.valid, 'reason': sample.reason, 'schema': sample.schema,
            'stamp': sample.stamp}


class ConsoleState:
    """로봇별·공용 슬롯 묶음과 화면 상태 조립. 잠금은 호출자(노드)가 맡는다."""

    # 로봇 목록(출처 포함)과 판정 설정으로 슬롯 생성
    def __init__(self, robots, robot_source, limits=None, future_tolerance_s=0.1,
                 display_grace_s=0.5):
        self.robots = [dict(r) for r in robots]
        self.robot_source = robot_source
        self.limits = dict(DEFAULT_LIMITS, **(limits or {}))
        self.future_tolerance_s = future_tolerance_s
        self.display_grace_s = display_grace_s
        self.slots = {r['robot_id']: {kind: Slot(kind) for kind in ROBOT_KINDS}
                      for r in self.robots}
        self.tasks = Slot('tasks')

    # 로봇 입력 반영
    def update_robot(self, robot_id, kind, sample, clocks):
        slot = self.slots[robot_id][kind]
        return slot.update(sample, clocks.sim, clocks.boot, self.future_tolerance_s)

    # 작업 목록 반영
    def update_tasks(self, sample, clocks):
        return self.tasks.update(sample, clocks.sim, clocks.boot, self.future_tolerance_s)

    # 화면 상태 dict 조립
    def build_state(self, clocks):
        tasks = judge(self.tasks, clocks, self.limits['tasks'], self.display_grace_s)
        robots = []

        for robot in self.robots:
            items = {kind: judge(slot, clocks, self.limits[kind], self.display_grace_s)
                     for kind, slot in self.slots[robot['robot_id']].items()}

            mine = assignment(robot['robot_id'], tasks)
            robots.append(dict(robot, items=items, assignment=mine))

        return {
            'schema': 1,
            'clock': {'sim': round(clocks.sim, 3), 'sim_valid': clocks.sim > 0.0},
            'robot_source': self.robot_source,
            'robots': robots,
            'tasks': tasks,
            'kpi': kpi(robots, tasks),
            'no_source': ['/ax/fleet/mode', '/ax/fleet/holds', '/ax/fleet/failures/history',
                          '/ax/robot_<n>/load/state', 'FleetSnapshot.robots(RobotSummary)'],
        }


# 로봇 배정: 작업 목록 표시가 정상일 때만 이 로봇이 낀 미종료 작업
def assignment(robot_id, tasks):
    if tasks['display_status'] != 'ok':
        return {'status': tasks['status'], 'display_status': tasks['display_status']}

    mine = [t for t in tasks['value']
            if robot_id in (t['manipulator'], t['transporter'])
            and t['state'] not in TERMINAL_TASK_STATES]
    return {'status': tasks['status'], 'display_status': 'ok', 'tasks': mine}


# KPI: 값이 없으면 None(숫자를 지어내지 않음)
def kpi(robots, tasks):
    actuation = [r['items']['actuation'] for r in robots]
    all_missing = all(a['status'] == 'missing' for a in actuation)

    # 가동 로봇: 구동 상태가 신선·유효하고 FAULT·UNKNOWN이 아닌 로봇. key는 표시·판정 상태 중 하나
    def running(key):
        return sum(1 for a in actuation
                   if a[key] == 'ok' and a.get('value', {}).get('state') not in NOT_RUNNING)

    by_state = None
    if tasks['display_status'] == 'ok':
        by_state = {}
        for task in tasks['value']:
            by_state[task['state']] = by_state.get(task['state'], 0) + 1

    return {
        'running': None if all_missing else running('display_status'),
        'running_strict': running('status'),
        'registered': len(robots),
        'task_by_state': by_state,
        'task_status': tasks['display_status'],
    }
