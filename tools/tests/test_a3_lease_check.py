"""A3 CheckCommandLease 개발 정책 서버 시험(계약 4.3·5.2절 형식·권한·멱등·수락 기록, ROS 왕복)."""
# [검사 범위] 개발용 lease 서버의 형식·중복·기록·ROS 응답 검사입니다. dev_allow는 운영 권한이 아닙니다.
import ast
import hashlib
import importlib.util
import os
from pathlib import Path
import sqlite3
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
import uuid

from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.srv import CheckCommandLease
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.lease_check import (  # noqa: E402
    ACCEPT_RECORD_PREFIX, LeaseCheckServer, LeaseChecker, LeaseLedger, MAX_VALID_FOR_S)

Request = CheckCommandLease.Request
GOAL_ID = 'a' * 32
SCRIPTS = Path(__file__).resolve().parents[2] / 'src/ax_control/scripts'


# A2 suction_tool_node 스크립트를 모듈로 읽는다(코드 변경 없이 같은 요청 경로를 쓰기 위해)
def load_suction_tool_node():
    spec = importlib.util.spec_from_file_location('suction_tool_node', SCRIPTS / 'suction_tool_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


# check_tool_attach.py의 GOAL_ID 상수를 실행하지 않고 읽는다(gz·tf 의존을 피하려고)
def check_tool_attach_goal_id():
    tree = ast.parse((SCRIPTS / 'check_tool_attach.py').read_text(encoding='utf-8'))

    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', '') == 'GOAL_ID' for t in node.targets):
            return node.value.value

    raise LookupError('GOAL_ID not found')


# A2 suction_tool_node.check_lease와 같은 필드로 C 조작 ACCEPT 요청을 만든다
def tool_request(robot='robot_1', request_id=None, command_id=None, digest='1' * 64):
    request = Request()
    request.request_id = request_id or uuid.uuid4().hex
    request.robot_id = robot
    request.authority = Request.AUTH_C_MANIPULATION
    request.execution_id = 'exec_test'
    request.goal_id = GOAL_ID
    request.lease_generation = 1
    request.phase = Request.ACCEPT
    request.command_source = Request.SOURCE_A2_TOOL
    # suction_tool_node는 command_id에 ToolCommand.request_id를 그대로 쓴다.
    request.command_id = command_id or request.request_id
    request.command_digest = digest

    return request


# A3 일반 팔 후퇴(AUTH_A3_LOCAL_TASK) ACCEPT 요청을 만든다
def retract_request(robot='robot_1'):
    request = Request()
    request.request_id = uuid.uuid4().hex
    request.robot_id = robot
    request.authority = Request.AUTH_A3_LOCAL_TASK
    request.execution_id = 'exec_retract'
    request.local_task_kind = ExecuteLocalTask.Goal.RETRACT_ARM
    request.phase = Request.ACCEPT
    request.command_source = Request.SOURCE_A4_ARM
    request.command_id = uuid.uuid4().hex
    request.command_digest = hashlib.sha256(b'retract').hexdigest()

    return request


class LeaseCheckerTests(unittest.TestCase):

    # 임시 원장과 판정기를 만든다
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.ledger = LeaseLedger(Path(self.temp.name) / 'lease.sqlite3', 'robot_1')
        self.addCleanup(self.ledger.close)

    # 지정 정책의 판정기를 만든다
    def checker(self, policy='dev_allow', reason='LEASE_EXPIRED'):
        return LeaseChecker('robot_1', policy, reason, 2.0, self.ledger)

    # 정책과 무관하게 형식·권한 필드 오류는 AUTHORITY_INVALID로 거절한다
    def test_form_and_authority_fields_rejected(self):
        checker = self.checker()
        cases = {
            'phase unspecified': ('phase', Request.UNSPECIFIED),
            'source unspecified': ('command_source', Request.SOURCE_UNSPECIFIED),
            'authority unspecified': ('authority', Request.AUTH_UNSPECIFIED),
            'robot mismatch': ('robot_id', 'robot_2'),
            'goal_id uppercase': ('goal_id', 'A' * 32),
            'goal_id short': ('goal_id', 'a' * 31),
            'generation zero': ('lease_generation', 0),
            'local kind on C': ('local_task_kind', ExecuteLocalTask.Goal.RETRACT_ARM),
            'empty execution': ('execution_id', ''),
            'bad digest': ('command_digest', 'xyz'),
            'empty command': ('command_id', ''),
        }

        for name, (field, value) in cases.items():
            with self.subTest(case=name):
                request = tool_request()
                setattr(request, field, value)

                result = checker.check(request)

                self.assertFalse(result['allowed'])
                self.assertEqual(result['reason_code'], 'AUTHORITY_INVALID')
                self.assertEqual(result['accept_record_id'], '')

        self.assertEqual(self.ledger.accept_count(), 0)

    # A3 로컬 권한은 RETRACT_ARM·빈 goal_id·세대 0·SOURCE_A4_ARM만 허용한다
    def test_a3_local_task_authority(self):
        checker = self.checker()

        self.assertTrue(checker.check(retract_request())['allowed'])

        cases = {
            'goal_id set': ('goal_id', GOAL_ID),
            'generation set': ('lease_generation', 1),
            'other local kind': ('local_task_kind', ExecuteLocalTask.Goal.CONFIRM_LOAD),
            'A2 source': ('command_source', Request.SOURCE_A2_TOOL),
        }

        for name, (field, value) in cases.items():
            with self.subTest(case=name):
                request = retract_request()
                setattr(request, field, value)

                self.assertEqual(checker.check(request)['reason_code'], 'AUTHORITY_INVALID')

    # dev_deny(기본 안전 정책)는 형식이 맞아도 지정 사유로 거절하고 기록을 남기지 않는다
    def test_dev_deny_policy(self):
        for reason in ('LEASE_EXPIRED', 'LEASE_REVOKED'):
            with self.subTest(reason=reason):
                result = self.checker('dev_deny', reason).check(tool_request())

                self.assertFalse(result['allowed'])
                self.assertEqual(result['reason_code'], reason)
                self.assertEqual(result['valid_for_s'], 0.0)

        self.assertEqual(self.ledger.accept_count(), 0)

    # dev_allow 허용은 개발 정책 표시(사유·기록 ID 접두어)와 valid_for_s를 담는다
    def test_dev_allow_marks_development_policy(self):
        # [핵심] 개발용 허용 응답에는 개발 표시가 있어야 합니다. 실제 권한 검증 결과와 혼동하면 안 됩니다.
        result = self.checker().check(tool_request())

        self.assertTrue(result['allowed'])
        self.assertEqual(result['reason_code'], 'DEV_POLICY_ALLOW')
        self.assertTrue(result['accept_record_id'].startswith(ACCEPT_RECORD_PREFIX))
        self.assertEqual(result['valid_for_s'], 2.0)
        self.assertEqual(self.ledger.accept_count(), 1)

    # 같은 request_id 재전송은 같은 응답, 내용이 다르면 DUPLICATE_REQUEST
    def test_same_request_id_idempotent(self):
        # [핵심] 같은 질문을 다시 보내도 응답과 수락 기록이 중복 생성되지 않아야 합니다.
        checker = self.checker()
        request = tool_request()

        first = checker.check(request)
        again = checker.check(request)

        self.assertEqual(first, again)
        self.assertEqual(self.ledger.accept_count(), 1)

        changed = tool_request(request_id=request.request_id, command_id=request.command_id,
                               digest='2' * 64)
        self.assertEqual(checker.check(changed)['reason_code'], 'DUPLICATE_REQUEST')

        # 거절된 변경 요청이 원래 응답을 덮어쓰지 않는다.
        self.assertEqual(checker.check(request), first)

    # 같은 command_id의 새 request_id ACCEPT는 기존 수락 기록을 재사용하고, 다른 digest는 거절한다
    def test_same_command_new_request_reuses_record(self):
        checker = self.checker()
        command = uuid.uuid4().hex

        first = checker.check(tool_request(command_id=command))
        retry = checker.check(tool_request(command_id=command))
        other = checker.check(tool_request(command_id=command, digest='3' * 64))

        self.assertEqual(retry['accept_record_id'], first['accept_record_id'])
        self.assertEqual(self.ledger.accept_count(), 1)
        self.assertEqual(other['reason_code'], 'DUPLICATE_REQUEST')

    # CONTINUE는 수락된 명령에만 허용하고 수락 기록을 만들지 않는다
    def test_continue_does_not_create_record(self):
        # [핵심] CONTINUE는 이미 수락한 명령의 지속 확인입니다. 새 명령 수락 기록을 만들면 안 됩니다.
        checker = self.checker()
        command = uuid.uuid4().hex
        checker.check(tool_request(command_id=command))

        follow = tool_request(command_id=command)
        follow.phase = Request.CONTINUE
        result = checker.check(follow)

        self.assertTrue(result['allowed'])
        self.assertEqual(result['accept_record_id'], '')
        self.assertEqual(self.ledger.accept_count(), 1)

        orphan = tool_request()
        orphan.phase = Request.CONTINUE
        self.assertFalse(checker.check(orphan)['allowed'])
        self.assertEqual(self.ledger.accept_count(), 1)

    # 원장 기록에 실패하면 허용하지 않는다
    def test_record_failure_rejects(self):
        checker = self.checker()

        with patch.object(self.ledger, 'check', side_effect=sqlite3.OperationalError('disk')):
            result = checker.check(tool_request())

        self.assertFalse(result['allowed'])
        self.assertEqual(result['reason_code'], 'A3_RECORD_FAILED')

    # 원장은 synchronous=FULL이고 다시 열어도 응답·기록이 남는다
    def test_ledger_durable_and_reopen(self):
        request = tool_request()
        first = self.checker().check(request)
        self.assertEqual(self.ledger.db.execute('PRAGMA synchronous').fetchone()[0], 2)

        reopened = LeaseLedger(self.ledger.path, 'robot_1')
        self.addCleanup(reopened.close)
        again = LeaseChecker('robot_1', 'dev_deny', 'LEASE_EXPIRED', 2.0, reopened).check(request)

        # 같은 request_id 재전송은 정책이 바뀌어도 기존 응답이다.
        self.assertEqual(again, first)

    # 원장은 WAL 저널 + synchronous=FULL로 열린다
    def test_ledger_uses_wal_with_full_sync(self):
        self.assertEqual(self.ledger.journal_mode, 'wal')
        self.assertEqual(self.ledger.db.execute('PRAGMA journal_mode').fetchone()[0], 'wal')
        self.assertEqual(self.ledger.db.execute('PRAGMA synchronous').fetchone()[0], 2)

    # 설정 검사: 로봇 4대 허용, 정책·사유·valid_for_s 상한
    def test_configuration_bounds(self):
        for robot in ('robot_1', 'robot_2', 'robot_3', 'robot_4'):
            LeaseChecker(robot, 'dev_deny', 'LEASE_EXPIRED', 2.0, self.ledger)

        bad = [('robot_5', 'dev_deny', 'LEASE_EXPIRED', 2.0),
               ('robot_1', 'allow', 'LEASE_EXPIRED', 2.0),
               ('robot_1', 'dev_deny', 'OK', 2.0),
               ('robot_1', 'dev_deny', 'LEASE_EXPIRED', 0.0),
               ('robot_1', 'dev_deny', 'LEASE_EXPIRED', MAX_VALID_FOR_S + 0.1)]

        for args in bad:
            with self.subTest(args=args), self.assertRaises(ValueError):
                LeaseChecker(*args, self.ledger)


class LeaseCheckServiceTests(unittest.TestCase):

    # 기본 보존 경로를 쓰는 시험도 사용자 ~/.ros를 건드리지 않게 ROS_HOME을 임시 폴더로 둔다
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        patcher = patch.dict(os.environ, {'ROS_HOME': self.temp.name + '/ros_home'})
        patcher.start()
        self.addCleanup(patcher.stop)

    # 지정 파라미터로 ROS를 초기화하고 서버·클라이언트를 만든다
    def start(self, robot, policy, db=None):
        args = ['--ros-args', '-p', f'robot_id:={robot}', '-p', f'policy:={policy}']

        if db:
            args += ['-p', f'lease_db:={db}']

        rclpy.init(args=args)
        server = LeaseCheckServer()
        client_node = Node('lease_check_test_client')
        client = client_node.create_client(
            CheckCommandLease, f'/ax/{robot}/executor/lease/check_command')
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)

        def stop():
            executor.shutdown()
            client_node.destroy_node()
            server.destroy_node()
            rclpy.shutdown()

        self.addCleanup(stop)
        self.assertTrue(client.wait_for_service(timeout_sec=5))

        return server, client, executor

    # 요청을 보내고 응답을 기다린다
    def call(self, executor, client, request):
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5)
        self.assertTrue(future.done(), 'lease 응답 시간 초과')

        return future.result()

    # A2 suction_tool_node와 같은 요청 필드로 왕복한다(기본 dev_deny, dev_allow 재전송 멱등)
    def test_round_trip_with_tool_node_fields(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        server, client, executor = self.start('robot_1', 'dev_allow', f'{temp.name}/lease.sqlite3')

        request = tool_request('robot_1')
        first = self.call(executor, client, request)
        again = self.call(executor, client, request)

        self.assertTrue(first.allowed)
        self.assertEqual(first.reason_code, 'DEV_POLICY_ALLOW')
        self.assertTrue(first.accept_record_id.startswith(ACCEPT_RECORD_PREFIX))
        self.assertAlmostEqual(first.valid_for_s, 2.0)
        self.assertEqual(again.accept_record_id, first.accept_record_id)
        self.assertEqual(server.ledger.accept_count(), 1)

    # A2 suction_tool_node.check_lease를 코드 그대로 호출해 왕복한다(check_tool_attach와 같은 공구 명령 값)
    def test_suction_tool_node_check_lease_round_trip(self):
        tool = load_suction_tool_node()
        from ax_interfaces.srv import ToolCommand

        for policy, expected in (('dev_allow', (True, '')), ('dev_deny', (False, 'LEASE_EXPIRED'))):
            with self.subTest(policy=policy):
                rclpy.init(args=['--ros-args', '-p', 'robot_id:=robot_1', '-p', f'policy:={policy}',
                                 '-p', f'lease_db:={self.temp.name}/{policy}.sqlite3'])
                server = LeaseCheckServer()
                node = tool.SuctionToolNode()
                executor = MultiThreadedExecutor()
                executor.add_node(server)
                executor.add_node(node)
                spinner = threading.Thread(target=executor.spin, daemon=True)
                spinner.start()

                try:
                    self.assertTrue(node.lease_client.wait_for_service(timeout_sec=5))

                    # check_tool_attach.send_tool과 같은 값
                    command = ToolCommand.Request()
                    command.request_id = uuid.uuid4().hex
                    command.robot_id = 'robot_1'
                    command.execution_id = 'exec_test'
                    command.goal_id = check_tool_attach_goal_id()
                    command.object_id = 'test_box'
                    command.lease_generation = 1
                    command.command = ToolCommand.Request.ATTACH

                    digest = tool.command_digest(command)
                    result = node.check_lease(command, digest, tool.boottime_seconds())

                    self.assertEqual(result, expected)
                    self.assertEqual(server.ledger.accept_count(), 1 if policy == 'dev_allow' else 0)
                finally:
                    executor.shutdown()
                    spinner.join(timeout=5)
                    node.destroy_node()
                    server.destroy_node()
                    rclpy.shutdown()

    # 기본 정책은 dev_deny이고 원장은 ROS_HOME 아래 로봇별 보존 경로다. 재시작해도 응답·기록이 남는다
    def test_default_policy_denies_and_persistent_db(self):
        expected = str(Path(os.environ['ROS_HOME']) / 'ax_task_executor/robot_4/lease_checks.sqlite3')
        request = tool_request('robot_4')

        rclpy.init(args=['--ros-args', '-p', 'robot_id:=robot_4', '-p', 'policy:=dev_allow'])
        server = LeaseCheckServer()
        first = server.checker.check(request)
        self.assertEqual(server.ledger.path, expected)
        server.destroy_node()
        rclpy.shutdown()

        server, client, executor = self.start('robot_4', 'dev_deny')
        self.assertEqual(server.ledger.path, expected)

        # 같은 request_id 재전송은 재시작 뒤에도 기존 허용 응답, 새 요청은 기본 정책으로 거절.
        again = self.call(executor, client, request)
        reply = self.call(executor, client, tool_request('robot_4'))

        self.assertTrue(again.allowed)
        self.assertEqual(again.accept_record_id, first['accept_record_id'])
        self.assertEqual(server.ledger.accept_count(), 1)
        self.assertFalse(reply.allowed)
        self.assertEqual(reply.reason_code, 'LEASE_EXPIRED')
        self.assertEqual(server.get_parameter('policy').value, 'dev_deny')

    # 노드 파라미터 기본값은 dev_deny다(파라미터를 주지 않은 경우)
    def test_node_default_is_dev_deny(self):
        rclpy.init(args=['--ros-args', '-p', 'robot_id:=robot_2'])
        self.addCleanup(rclpy.shutdown)
        server = LeaseCheckServer()
        self.addCleanup(server.destroy_node)

        self.assertEqual(server.checker.policy, 'dev_deny')
        self.assertEqual(server.service.srv_name, '/ax/robot_2/executor/lease/check_command')

    # 동시에 같은 command_id를 여러 request_id로 확인해도 수락 기록은 하나다(직렬 처리)
    def test_concurrent_same_command_single_record(self):
        # [핵심] 여러 요청이 동시에 와도 같은 물리 명령의 수락 영수증은 하나만 남아야 합니다.
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        ledger = LeaseLedger(Path(temp.name) / 'lease.sqlite3', 'robot_3')
        self.addCleanup(ledger.close)
        checker = LeaseChecker('robot_3', 'dev_allow', 'LEASE_EXPIRED', 2.0, ledger)
        command = uuid.uuid4().hex
        results = []

        threads = [threading.Thread(target=lambda: results.append(
            checker.check(tool_request('robot_3', command_id=command)))) for _ in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        self.assertEqual(len({r['accept_record_id'] for r in results}), 1)
        self.assertEqual(ledger.accept_count(), 1)


if __name__ == '__main__':
    unittest.main()
