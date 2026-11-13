"""
A3 CheckCommandLease 서버(개발 정책).

계약(INTERFACE_CONTRACT 4.3·5.2절)의 요청 형식·권한별 필드·멱등·수락 기록 규칙은 그대로 검사한다.
실제 lease 표·세대·활성 Goal 대조·배정 유효성·재동기화는 구현하지 않았다.
그 판정 자리는 파라미터 `policy`(dev_allow | dev_deny)가 대신하며 기본값은 dev_deny다.
허용 응답의 reason_code는 DEV_POLICY_ALLOW, 수락 기록 ID는 `devpolicy-` 접두어라 개발 정책 결과임을 구분할 수 있다.
"""
from contextlib import nullcontext
import hashlib
import json
from pathlib import Path
import re
import sqlite3
import threading
import uuid

from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.srv import CheckCommandLease
from ax_task_executor.mock_executor import record_dir
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


# 개발 서버가 받는 로봇 ID(시험 구성 robot_1~robot_4)
ROBOT_IDS = ('robot_1', 'robot_2', 'robot_3', 'robot_4')

# 정책 이름과 dev_deny가 쓸 수 있는 거절 사유(계약 D-13 코드 중 lease 판정 결과에 해당하는 것)
POLICIES = ('dev_allow', 'dev_deny')
DENY_REASONS = ('LEASE_EXPIRED', 'LEASE_REVOKED', 'GOAL_MISMATCH', 'ASSIGNMENT_STALE',
                'LEASE_RESYNC_UNCERTAIN')

# 응답 코드. 형식·필드 오류는 모두 AUTHORITY_INVALID다(계약 D-13, 요청 형식 오류 포함).
REASON_AUTHORITY_INVALID = 'AUTHORITY_INVALID'
REASON_DUPLICATE = 'DUPLICATE_REQUEST'
# 수락 기록을 남기지 못한 경우(계약 D-13 A3_RECORD_FAILED). 요청자는 실행하지 않는다.
REASON_RECORD_FAILED = 'A3_RECORD_FAILED'
REASON_DEV_ALLOW = 'DEV_POLICY_ALLOW'
ACCEPT_RECORD_PREFIX = 'devpolicy-'

# valid_for_s 상한(s). 실제 lease·배정 기한이 없으므로 개발용 고정 상한이다.
MAX_VALID_FOR_S = 10.0

# 서비스 요청 큐 깊이. 기본(10)이면 동시 요청이 몰릴 때 요청을 잃어 클라이언트가 무응답으로 끝난다(시험에서 확인).
LEASE_SERVICE_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

# 계약 2.1절 문자열 ID 문자, goal_id(소문자 16진수 32자), SHA-256 소문자 16진수
ID_PATTERN = re.compile(r'[A-Za-z0-9_\-.:@]+')
GOAL_ID_PATTERN = re.compile(r'[0-9a-f]{32}')
DIGEST_PATTERN = re.compile(r'[0-9a-f]{64}')

Request = CheckCommandLease.Request


# 요청 필드를 사전으로 바꾼다(기록·비교용)
def request_fields(request):
    # 역할: 서비스 요청을 기록과 비교에 쓰는 사전으로 변환합니다.
    return {
        'request_id': request.request_id,
        'robot_id': request.robot_id,
        'authority': int(request.authority),
        'execution_id': request.execution_id,
        'goal_id': request.goal_id,
        'lease_generation': int(request.lease_generation),
        'local_task_kind': int(request.local_task_kind),
        'phase': int(request.phase),
        'command_source': int(request.command_source),
        'command_id': request.command_id,
        'command_digest': request.command_digest,
    }


# 사전의 결정적 SHA-256을 만든다
def fingerprint(fields):
    # 역할: 요청 필드에서 결정적인 해시 지문을 계산합니다.
    text = json.dumps(fields, sort_keys=True, separators=(',', ':'))

    return hashlib.sha256(text.encode('utf-8')).hexdigest()


# 같은 command_id인지 판단할 물리 명령 내용(권한 필드 + 출처 + digest)
def command_fields(fields):
    # 역할: 같은 물리 명령인지 비교할 권한과 명령 필드만 추립니다.
    keys = ('authority', 'execution_id', 'goal_id', 'lease_generation', 'local_task_kind',
            'command_source', 'command_digest')

    return {key: fields[key] for key in keys}


# 응답 사전을 만든다
def response(allowed, reason='', record='', valid_for_s=0.0):
    # 역할: 허용 여부와 사유 및 수락 기록을 담은 응답 사전을 만듭니다.
    return {'allowed': allowed, 'reason_code': reason, 'accept_record_id': record,
            'valid_for_s': float(valid_for_s)}


# 계약 형식·권한별 필드를 검사해 거절 사유를 돌려준다. 통과면 빈 문자열
def check_form(fields, robot_id):
    # 글자·필드 조합 검사입니다. 형식이 맞아도 실제 배정·Goal·lease가 유효하다는 뜻은 아닙니다.
    # 역할: 로봇과 식별자 및 권한별 필수 필드 조합을 검사합니다.
    for key in ('request_id', 'execution_id', 'command_id'):
        if not ID_PATTERN.fullmatch(fields[key]):
            return REASON_AUTHORITY_INVALID

    if fields['robot_id'] != robot_id:
        return REASON_AUTHORITY_INVALID

    if fields['phase'] not in (Request.ACCEPT, Request.CONTINUE):
        return REASON_AUTHORITY_INVALID

    if fields['command_source'] not in (Request.SOURCE_A2_TOOL, Request.SOURCE_A4_ARM):
        return REASON_AUTHORITY_INVALID

    if not DIGEST_PATTERN.fullmatch(fields['command_digest']):
        return REASON_AUTHORITY_INVALID

    # 권한 종류별 필수·비움 필드(계약 5.2절 표)
    if fields['authority'] == Request.AUTH_C_MANIPULATION:
        valid = (GOAL_ID_PATTERN.fullmatch(fields['goal_id']) is not None
                 and fields['lease_generation'] > 0
                 and fields['local_task_kind'] == 0)
    elif fields['authority'] == Request.AUTH_A3_LOCAL_TASK:
        valid = (fields['goal_id'] == ''
                 and fields['lease_generation'] == 0
                 and fields['local_task_kind'] == ExecuteLocalTask.Goal.RETRACT_ARM
                 and fields['command_source'] == Request.SOURCE_A4_ARM)
    else:
        valid = False

    return '' if valid else REASON_AUTHORITY_INVALID


class LeaseLedger:
    """로봇별 확인 응답·수락 기록 원장(SQLite WAL + synchronous=FULL). 한 번에 한 요청만 처리한다."""

    # 원장 파일을 열고 표를 만든다
    def __init__(self, path, robot_id):
        # 역할: 설정과 내부 상태를 준비합니다.
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.path = str(path)
        self.robot_id = robot_id
        self.lock = threading.Lock()
        self.db = sqlite3.connect(self.path, check_same_thread=False, isolation_level=None)
        # WAL 저널: 커밋마다 WAL만 fsync해 처리 시간을 줄인다(2026-11-12 측정 p50 22.8 → 6.1 ms).
        # synchronous=FULL이라 응답 전에 커밋한 수락 기록은 전원 차단에도 남는다.
        # WAL을 쓸 수 없는 파일 시스템이면 SQLite가 이전 모드를 유지한다. 내구성은 같으므로 거절하지 않고
        # 실제 모드를 journal_mode에 남겨 서버가 경고하게 한다.
        self.journal_mode = self.db.execute('PRAGMA journal_mode=WAL').fetchone()[0].lower()
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute("""CREATE TABLE IF NOT EXISTS checks (
            robot TEXT NOT NULL, request_id TEXT NOT NULL, fingerprint TEXT NOT NULL,
            response TEXT NOT NULL, PRIMARY KEY (robot, request_id))""")
        self.db.execute("""CREATE TABLE IF NOT EXISTS accepts (
            robot TEXT NOT NULL, command_id TEXT NOT NULL, command TEXT NOT NULL,
            record_id TEXT NOT NULL, request_id TEXT NOT NULL, PRIMARY KEY (robot, command_id))""")

    # 확인 요청 하나를 직렬로 처리한다. decide(fields)는 형식 검사 뒤의 정책 판정이다
    # marks = (허용 사유 코드, 수락 기록 ID 접두어). 개발 정책은 (DEV_POLICY_ALLOW, devpolicy-)다
    def check(self, fields, decide, valid_for_s, marks=(REASON_DEV_ALLOW, ACCEPT_RECORD_PREFIX)):
        # 역할: 요청의 중복과 권한을 검사하고 원장에 응답을 저장합니다.
        with self.lock:
            self.db.execute('BEGIN IMMEDIATE')

            try:
                result = self._check(fields, decide, valid_for_s, marks)
            except BaseException:
                self.db.execute('ROLLBACK')
                raise

            self.db.execute('COMMIT')

            return result

    # 트랜잭션 안의 처리 본체
    def _check(self, fields, decide, valid_for_s, marks):
        # request_id는 질문 번호, command_id는 물리 명령 번호입니다. 둘의 중복을 따로 검사합니다.
        # 역할: 원장 트랜잭션 안에서 요청과 명령의 중복을 먼저 판정합니다.
        key = fingerprint(fields)

        # 1. 같은 request_id: 내용이 같으면 기존 응답, 다르면 DUPLICATE_REQUEST(기록하지 않음).
        row = self.db.execute(
            'SELECT fingerprint, response FROM checks WHERE robot=? AND request_id=?',
            (self.robot_id, fields['request_id'])).fetchone()

        if row is not None:
            return json.loads(row[1]) if row[0] == key else response(False, REASON_DUPLICATE)

        # 2. 같은 command_id: 명령 내용이 다르면 거절, 같으면 기존 수락 기록을 쓴다.
        command = json.dumps(command_fields(fields), sort_keys=True)
        accepted = self.db.execute(
            'SELECT command, record_id FROM accepts WHERE robot=? AND command_id=?',
            (self.robot_id, fields['command_id'])).fetchone()

        if accepted is not None and accepted[0] != command:
            result = response(False, REASON_DUPLICATE)
        else:
            reason = decide(fields)
            result = self._decide(fields, reason, accepted, command, valid_for_s, marks)

        self.db.execute('INSERT INTO checks VALUES (?, ?, ?, ?)',
                        (self.robot_id, fields['request_id'], key, json.dumps(result)))

        return result

    # 정책 판정 결과로 응답을 만들고, 새 ACCEPT 허용이면 수락 기록을 먼저 남긴다
    def _decide(self, fields, reason, accepted, command, valid_for_s, marks):
        # ACCEPT는 수락 영수증을 남깁니다. CONTINUE는 기존 영수증을 확인하고 새로 만들지 않습니다.
        # 역할: 정책 결과를 응답으로 바꾸고 필요한 경우 수락 기록을 남깁니다.
        allow, prefix = marks

        if reason:
            return response(False, reason)

        if fields['phase'] == Request.CONTINUE:
            # 진행 중 확인은 수락된 명령에만 허용하고 새 기록을 만들지 않는다.
            if accepted is None:
                return response(False, REASON_AUTHORITY_INVALID)

            return response(True, allow, '', valid_for_s)

        if accepted is not None:
            return response(True, allow, accepted[1], valid_for_s)

        record = prefix + uuid.uuid4().hex
        self.db.execute(
            'INSERT INTO accepts VALUES (?, ?, ?, ?, ?)',
            (self.robot_id, fields['command_id'], command, record, fields['request_id']))

        return response(True, allow, record, valid_for_s)

    # 재검사 거절만 저장하고 최초 지문과 기존 물리 명령 수락 기록은 보존합니다.
    def deny_cached_allow(self, fields, reason):
        """기존 수락 기록을 보존하면서 재검사 거절을 원장에 저장합니다."""
        with self.lock:
            result = response(False, reason)
            self.db.execute(
                'UPDATE checks SET response=? WHERE robot=? AND request_id=? AND fingerprint=?',
                (json.dumps(result), self.robot_id, fields['request_id'], fingerprint(fields)))
            return result

    def accept_count(self):
        # 역할: 현재 로봇의 수락 기록 개수를 셉니다.
        with self.lock:
            return self.db.execute('SELECT COUNT(*) FROM accepts WHERE robot=?',
                                   (self.robot_id,)).fetchone()[0]

    # 원장을 닫는다
    def close(self):
        # 역할: 보유한 원장과 내부 자원을 안전하게 닫습니다.
        self.db.close()


class DevPolicyAuthority:
    """lease 판정 자리의 개발 정책. 운영 판정은 같은 decide(fields, state) 인터페이스로 교체한다."""

    # 정책 이름과 거절 사유를 검사해 준비한다
    def __init__(self, policy, deny_reason):
        # 역할: 설정과 내부 상태를 준비합니다.
        if policy not in POLICIES:
            raise ValueError('policy must be dev_allow or dev_deny')

        if deny_reason not in DENY_REASONS:
            raise ValueError('deny_reason must be one of ' + ', '.join(DENY_REASONS))

        self.policy = policy
        self.deny_reason = deny_reason
        # 허용 응답 표시(계약 5.2절: 운영 A3는 빈 사유, 개발 정책만 아래 값)
        self.marks = (REASON_DEV_ALLOW, ACCEPT_RECORD_PREFIX)

    # 형식 검사를 통과한 요청의 거절 사유를 돌려준다(허용이면 빈 문자열). state는 보지 않는다
    def decide(self, fields, state=None):
        # 개발 시험용 허용/거절입니다. 운영 권한 검사와 구별해야 합니다.
        # 실제 lease 표·세대·활성 Goal·배정 판정 자리. 운영 판정 클래스는 state(ExecutorState)를 본다.
        # 역할: 요청 형식이나 현재 정책 및 권한을 검사해 거절 사유를 반환합니다.
        return '' if self.policy == 'dev_allow' else self.deny_reason


class LeaseChecker:
    """ROS 없이 쓸 수 있는 판정기. 형식 검사 → lease 판정(정책 객체) → 원장 기록 순서로 처리한다."""

    # 정책·기한·원장을 검사해 준비한다. authority를 주면 policy·deny_reason 대신 그것을 쓴다
    def __init__(self, robot_id, policy, deny_reason, valid_for_s, ledger, authority=None,
                 state=None):
        # 역할: 설정과 내부 상태를 준비합니다.
        if robot_id not in ROBOT_IDS:
            raise ValueError('robot_id must be one of robot_1~robot_4')

        if not 0.0 < float(valid_for_s) <= MAX_VALID_FOR_S:
            raise ValueError(f'valid_for_s must be in (0, {MAX_VALID_FOR_S}]')

        self.authority = authority or DevPolicyAuthority(policy, deny_reason)
        self.robot_id = robot_id
        self.policy = getattr(self.authority, 'policy', 'custom')
        self.valid_for_s = float(valid_for_s)
        self.ledger = ledger
        self.state = state

    # 형식 검사를 통과한 요청에 lease 판정을 적용해 거절 사유를 돌려준다(허용이면 빈 문자열)
    def decide(self, fields):
        # 역할: 요청 형식이나 현재 정책 및 권한을 검사해 거절 사유를 반환합니다.
        reason = check_form(fields, self.robot_id)

        if reason:
            return reason

        return self.authority.decide(fields, self.state)

    # 요청 하나를 처리해 응답 사전을 돌려준다. 원장 오류면 기록 없이 거절한다
    def check(self, request):
        # 역할: 요청의 중복과 권한을 검사하고 원장에 응답을 저장합니다.
        fields = request_fields(request)

        # request_id가 없으면 멱등 기록을 할 수 없으므로 기록 없이 거절한다.
        if not ID_PATTERN.fullmatch(fields['request_id']):
            return response(False, REASON_AUTHORITY_INVALID)

        try:
            marks = getattr(self.authority, 'marks', ('', 'accept-'))
            guard = getattr(self.authority, 'command_guard', nullcontext)
            with guard():
                # 중복 인자와 저장된 거절을 먼저 확인합니다. 최초 거절도 원장에 남깁니다.
                # digest 변경은 철회 사유보다 DUPLICATE_REQUEST를 우선합니다.
                # A-04 연결 시험 기준입니다. 상대 A4는 거절을 받으면 이 요청으로 실행하면 안 됩니다.
                if hasattr(self.authority, 'remaining'):
                    duration = min(self.valid_for_s, self.authority.remaining(fields))
                else:
                    duration = self.valid_for_s
                result = self.ledger.check(fields, self.decide, duration, marks)
                if result['allowed'] and hasattr(self.authority, 'remaining'):
                    # 저장된 거절은 재검사하지 않아 권한 복구 후에도 허용으로 바뀌지 않습니다.
                    # 장부 저장이 늦어 기한이 지났다면, 저장 전에는 유효했어도 허용을 반환하지 않습니다.
                    # 내구성 저장을 기다리는 동안에도 권한 기한이 끝날 수 있습니다.
                    reason = self.decide(fields)
                    if reason:
                        return self.ledger.deny_cached_allow(fields, reason)
                    result['valid_for_s'] = min(
                        result['valid_for_s'], self.authority.remaining(fields))
                return result
        except sqlite3.Error:
            return response(False, REASON_RECORD_FAILED)


# 지정하지 않으면 로봇별 보존 경로의 원장을 쓴다(${ROS_HOME:-~/.ros}/ax_task_executor/<robot_id>)
def default_lease_db(robot_id):
    # 역할: 로봇별 기본 명령 확인 원장 경로를 반환합니다.
    return str(record_dir(robot_id) / 'lease_checks.sqlite3')


class LeaseCheckServer(Node):
    """`/ax/<robot_id>/executor/lease/check_command` 개발 정책 서버."""

    # 파라미터를 읽고 원장·서비스를 준비한다
    def __init__(self):
        # 역할: 설정과 내부 상태를 준비합니다.
        super().__init__('lease_check')
        self.declare_parameter('robot_id', 'robot_2')
        self.declare_parameter('policy', 'dev_deny')
        self.declare_parameter('deny_reason', 'LEASE_EXPIRED')
        self.declare_parameter('valid_for_s', 2.0)
        self.declare_parameter('lease_db', '')

        robot_id = self.get_parameter('robot_id').value

        # 원장 폴더를 만들기 전에 로봇 ID를 확인한다.
        if robot_id not in ROBOT_IDS:
            raise ValueError('robot_id must be one of robot_1~robot_4')

        path = self.get_parameter('lease_db').value or default_lease_db(robot_id)
        self.ledger = LeaseLedger(path, robot_id)
        self.checker = LeaseChecker(
            robot_id, self.get_parameter('policy').value,
            self.get_parameter('deny_reason').value,
            self.get_parameter('valid_for_s').value, self.ledger)

        self.service = self.create_service(
            CheckCommandLease, f'/ax/{robot_id}/executor/lease/check_command', self.on_check,
            qos_profile=LEASE_SERVICE_QOS)
        self.get_logger().warning(
            f'개발 정책 lease 확인 서버({self.checker.policy}). 실제 lease·세대·배정 검증이 없습니다. '
            f'원장: {path}')

        if self.ledger.journal_mode != 'wal':
            self.get_logger().warning(
                f'원장 저널이 WAL이 아닙니다({self.ledger.journal_mode}). 기록은 FULL로 남지만 응답이 느릴 수 있습니다.')

    # 서비스 요청을 판정기에 넘기고 응답을 채운다
    def on_check(self, request, reply):
        # 역할: 명령 확인 요청을 판정하고 서비스 응답 필드를 채웁니다.
        result = self.checker.check(request)

        reply.allowed = result['allowed']
        reply.reason_code = result['reason_code']
        reply.accept_record_id = result['accept_record_id']
        reply.valid_for_s = result['valid_for_s']

        return reply

    # 원장을 닫고 노드를 정리한다
    def destroy_node(self):
        # 역할: 노드가 소유한 원장을 닫고 상위 노드를 정리합니다.
        self.ledger.close()

        return super().destroy_node()


def main(args=None):
    # 역할: 실행 인자를 받아 노드를 시작하고 종료 시 자원을 정리합니다.
    rclpy.init(args=args)
    node = LeaseCheckServer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
