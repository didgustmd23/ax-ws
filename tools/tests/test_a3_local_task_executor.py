"""로컬 작업 실행기(local_task_executor) 시험: Action·lease 기능, 콜백 그룹 분리, lease 응답 지연.

지연 기준(개발 기준, 결과를 보고 바꾸지 않는다): Action 작업이 계속 도는 동안 lease 왕복 p99 < 0.1 s, 최대 < 0.5 s.
최대 0.5 s는 A2 suction_tool_node의 lease 응답 대기 한도(LEASE_TIMEOUT_SECONDS)다.
측정 요청은 dev_allow 새 command_id ACCEPT라 매번 SQLite(synchronous=FULL) 쓰기를 포함한다.
환경 변수 AX_LOCAL_EXECUTOR_LATENCY_OUT을 주면 측정 결과 json을 그 경로에 쓴다.
AX_LOCAL_EXECUTOR_LEASE_MODE(reentrant 기본 | mutex | dedicated)로 lease 콜백 구성을 바꿔 비교할 수 있다(진단용).
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import statistics
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
import uuid

from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.srv import CheckCommandLease, ToolCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

TESTS = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS.parents[1] / 'src/ax_task_executor'))
sys.path.insert(0, str(TESTS))
from ax_task_executor.local_task_executor import LocalTaskExecutor, lease_service_name  # noqa: E402
from ax_task_executor.lease_check import LeaseCheckServer  # noqa: E402
from ax_task_executor.mock_executor import execute_action_name  # noqa: E402
from test_a3_lease_check import (  # noqa: E402
    check_tool_attach_goal_id, load_suction_tool_node, tool_request)

ROBOT = 'robot_1'
Request = CheckCommandLease.Request

# 개발 지연 기준(s)
P99_LIMIT_S = 0.1
MAX_LIMIT_S = 0.5


# future가 끝날 때까지 기다린다(다른 스레드의 실행기가 돌린다)
def wait_future(future, timeout=10.0):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())

    if not done.wait(timeout):
        raise TimeoutError('ROS 응답 시간 초과')

    return future.result()


# CONFIRM_LOAD Goal을 만든다
def goal(execution_id):
    request = ExecuteLocalTask.Goal()
    request.execution.execution_id = execution_id
    request.execution.operation = 'CONFIRM_LOAD'
    request.task_kind = ExecuteLocalTask.Goal.CONFIRM_LOAD
    request.object_id = 'test_box'

    return request


class ClientThread:
    """자기 노드·실행기 스레드를 가진 시험 클라이언트(lease 서비스·Action)."""

    # 노드를 만들고 실행기 스레드를 시작한다
    def __init__(self, name):
        self.node = Node(name)
        self.lease = self.node.create_client(CheckCommandLease, lease_service_name(ROBOT))
        self.action = ActionClient(self.node, ExecuteLocalTask, execute_action_name(ROBOT))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    # lease 요청 하나의 응답과 왕복 시간(s)을 돌려준다
    def check(self, request, timeout=5.0):
        start = time.monotonic()
        reply = wait_future(self.lease.call_async(request), timeout)

        return reply, time.monotonic() - start

    # 정리한다
    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=5)
        self.action.destroy()
        self.node.destroy_node()


class LocalTaskExecutorTests(unittest.TestCase):

    # ROS_HOME·원장을 임시 폴더로 두고 local_task_executor를 멀티스레드 실행기로 띄운다
    def start(self, policy='dev_allow', delay=0.5, threads=4, extra=()):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        patcher = patch.dict(os.environ, {'ROS_HOME': self.temp.name + '/ros_home'})
        patcher.start()
        self.addCleanup(patcher.stop)

        rclpy.init(args=['--ros-args', '-p', f'robot_id:={ROBOT}', '-p', f'policy:={policy}',
                         '-p', f'mock_delay_s:={delay}', '-p', 'mock_load:=STOWED',
                         '-p', f'threads:={threads}', '-p', 'name_check_s:=0.0',
                         '-p', 'lease_mode:=' + os.environ.get('AX_LOCAL_EXECUTOR_LEASE_MODE', 'reentrant'),
                         *extra])
        self.addCleanup(rclpy.shutdown)

        self.node = LocalTaskExecutor()
        self.executor = MultiThreadedExecutor(num_threads=self.node.thread_count())
        self.executor.add_node(self.node)
        self.node.start_lease_thread()
        spinner = threading.Thread(target=self.executor.spin, daemon=True)
        spinner.start()

        self.clients = []

        def stop():
            for client in self.clients:
                client.close()

            self.executor.shutdown()
            spinner.join(timeout=5)
            self.node.destroy_node()

        self.addCleanup(stop)

        return self.node

    # 시험 클라이언트를 하나 더 만든다
    def client(self):
        client = ClientThread(f'local_task_executor_test_client_{len(self.clients)}')
        self.clients.append(client)
        self.assertTrue(client.lease.wait_for_service(timeout_sec=5))
        self.assertTrue(client.action.wait_for_server(timeout_sec=5))

        return client

    # 상태 발행 사본을 모은다(revision이 바뀐 것만 단계 순서로 쓴다)
    def capture_status(self):
        snapshots = []
        publish = self.node.task_status_publisher.publish

        def status(message):
            snapshots.append(deepcopy(message))
            return publish(message)

        self.node.task_status_publisher = Mock(wraps=self.node.task_status_publisher)
        self.node.task_status_publisher.publish = status

        return snapshots

    # --- 기능 ---

    # Action 수락·중복 Goal·상태 비재발행, 끝난 뒤 재요청도 단계를 되돌리지 않는다
    def test_action_duplicate_and_status_order(self):
        self.start(delay=0.5)
        client = self.client()
        snapshots = self.capture_status()

        first = wait_future(client.action.send_goal_async(goal('a3x-order')))
        second = wait_future(client.action.send_goal_async(goal('a3x-order')))
        results = [wait_future(first.get_result_async()), wait_future(second.get_result_async())]
        later = wait_future(client.action.send_goal_async(goal('a3x-order')))
        results.append(wait_future(later.get_result_async()))

        self.assertTrue(all(r.status == 4 for r in results))  # STATUS_SUCCEEDED
        order, seen = [], set()

        for message in snapshots:
            if message.meta.revision not in seen:
                seen.add(message.meta.revision)
                order.append(message.local_phase)

        self.assertEqual(order, ['MOCK_VALIDATING', 'MOCK_CHECKING_LOAD', 'MOCK_SUCCEEDED'])

    # 재요청 Goal 취소는 거절, 원래 Goal 취소는 Action 상태·결과 모두 CANCELED
    def test_goal_scoped_cancel(self):
        self.start(delay=1.0)
        client = self.client()

        original = wait_future(client.action.send_goal_async(goal('a3x-dup')))
        duplicate = wait_future(client.action.send_goal_async(goal('a3x-dup')))
        self.assertEqual(len(wait_future(duplicate.cancel_goal_async()).goals_canceling), 0)
        self.assertEqual(wait_future(original.get_result_async()).status, 4)

        other = wait_future(client.action.send_goal_async(goal('a3x-cancel')))
        self.assertEqual(len(wait_future(other.cancel_goal_async()).goals_canceling), 1)
        result = wait_future(other.get_result_async())

        self.assertEqual(result.status, 5)  # STATUS_CANCELED
        self.assertEqual(result.result.status.error_code, 'MOCK_CANCELED')

    # lease 형식 거절·같은 request_id 멱등·같은 command_id 기록 재사용
    def test_lease_form_idempotent_and_record(self):
        node = self.start()
        client = self.client()

        bad = tool_request(ROBOT)
        bad.phase = Request.UNSPECIFIED
        self.assertEqual(client.check(bad)[0].reason_code, 'AUTHORITY_INVALID')

        request = tool_request(ROBOT)
        first, _ = client.check(request)
        again, _ = client.check(request)
        retry, _ = client.check(tool_request(ROBOT, command_id=request.command_id))

        self.assertTrue(first.allowed)
        self.assertEqual(first.reason_code, 'DEV_POLICY_ALLOW')
        self.assertEqual(again.accept_record_id, first.accept_record_id)
        self.assertEqual(retry.accept_record_id, first.accept_record_id)
        self.assertEqual(node.ledger.accept_count(), 1)

        # 두 원장은 같은 로봇 폴더의 서로 다른 파일이다.
        self.assertNotEqual(node.ledger.path, node.get_parameter('execution_db').value)
        self.assertEqual(Path(node.ledger.path).parent,
                         Path(node.get_parameter('execution_db').value).parent)

    # 기본 정책은 dev_deny다
    def test_default_policy_dev_deny(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        rclpy.init(args=['--ros-args', '-p', f'robot_id:={ROBOT}', '-p', 'name_check_s:=0.0',
                         '-p', f'execution_db:={self.temp.name}/e.sqlite3',
                         '-p', f'lease_db:={self.temp.name}/l.sqlite3'])
        self.addCleanup(rclpy.shutdown)
        node = LocalTaskExecutor()
        self.addCleanup(node.destroy_node)

        self.assertEqual(node.checker.policy, 'dev_deny')
        self.assertEqual(node.checker.check(tool_request(ROBOT))['reason_code'], 'LEASE_EXPIRED')

    # --- 동시성 ---

    # Action 대기·취소 처리·중복 Goal 대기 중에도 lease 응답이 막히지 않는다
    def test_lease_not_blocked_by_action(self):
        self.start(delay=2.0)
        action = self.client()
        workers = [self.client() for _ in range(3)]

        running = wait_future(action.action.send_goal_async(goal('a3x-block')))
        duplicate = wait_future(action.action.send_goal_async(goal('a3x-block')))
        result_future = running.get_result_async()
        times = []

        # 대기 중(원래 Goal 실행 + 중복 Goal 대기) 병렬 lease 요청
        def burst():
            threads = [threading.Thread(target=lambda c=c: times.append(
                c.check(tool_request(ROBOT))[1])) for c in workers for _ in range(5)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()

        burst()
        self.assertFalse(result_future.done(), 'lease 측정 전에 Action이 끝났다')

        # 취소 처리 중 병렬 lease 요청
        cancel = running.cancel_goal_async()
        burst()
        wait_future(cancel)
        wait_future(result_future)
        wait_future(duplicate.get_result_async())

        self.assertEqual(len(times), 30)
        self.assertLess(max(times), MAX_LIMIT_S)

    # 같은 command_id를 여러 클라이언트가 동시에 ACCEPT해도 수락 기록은 하나다
    def test_concurrent_same_command_single_record(self):
        node = self.start()
        clients = [self.client() for _ in range(4)]
        command = uuid.uuid4().hex
        records = []

        threads = [threading.Thread(target=lambda c=c: records.append(
            c.check(tool_request(ROBOT, command_id=command))[0].accept_record_id))
            for c in clients for _ in range(3)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        self.assertEqual(len(records), 12)
        self.assertEqual(len(set(records)), 1)
        self.assertEqual(node.ledger.accept_count(), 1)

    # --- 지연 측정 ---

    # Action 작업이 계속 도는 동안 lease 1000회 왕복 p50·p99·max
    def test_lease_latency_under_action_load(self):
        node = self.start(delay=0.3)
        action = self.client()
        workers = [self.client() for _ in range(4)]
        stop = threading.Event()
        actions = {'goals': 0}

        # 새 실행 ID Goal을 계속 보내 Action 대기 루프·실행 기록 쓰기가 돌게 한다
        def keep_action_busy():
            while not stop.is_set():
                handle = wait_future(action.action.send_goal_async(goal(f'load-{uuid.uuid4().hex}')))
                wait_future(handle.get_result_async())
                actions['goals'] += 1

        busy = threading.Thread(target=keep_action_busy, daemon=True)
        busy.start()
        times, samples, lock = [], [], threading.Lock()

        # 클라이언트마다 250회 새 command_id ACCEPT(매번 SQLite FULL 쓰기)
        def run(index, client):
            for sequence in range(250):
                sent = time.monotonic() - started
                request = tool_request(ROBOT)
                reply, elapsed = client.check(request)
                self.assertTrue(reply.allowed)

                with lock:
                    times.append(elapsed)
                    samples.append((elapsed, index, sequence, sent, request.request_id))

        # 진단: 서버 판정(형식 검사 + SQLite FULL 기록) 시간을 따로 잰다
        server_times = []
        check = node.checker.check

        def timed_check(request):
            begin = time.monotonic()
            result = check(request)
            server_times.append((time.monotonic() - begin, begin - started, request.request_id))
            return result

        node.checker.check = timed_check

        load_before = os.getloadavg()
        started = time.monotonic()
        threads = [threading.Thread(target=run, args=(i, c)) for i, c in enumerate(workers)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        wall = time.monotonic() - started
        stop.set()
        busy.join(timeout=10)

        ordered = sorted(times)
        summary = {
            'n': len(ordered),
            'p50_s': statistics.median(ordered),
            'p99_s': ordered[int(0.99 * len(ordered)) - 1],
            'max_s': ordered[-1],
            'wall_s': wall,
            'action_goals_during': actions['goals'],
            'clients': len(workers),
            'threads': node.thread_count(),
            'sqlite_synchronous': 'FULL',
            'loadavg_before': load_before,
            'loadavg_after': os.getloadavg(),
            'accept_records': node.ledger.accept_count(),
            # 진단: 가장 느린 5개 (왕복 s, 클라이언트 번호, 그 클라이언트의 몇 번째 요청, 시작 후 송신 시각 s)
            'slowest': sorted(samples, reverse=True)[:5],
            # 진단: 서버 판정 시간 최대·p99와 가장 느린 3개 (판정 s, 시작 후 시각 s)
            'server_max_s': max(server_times)[0],
            'server_p99_s': sorted(s[0] for s in server_times)[int(0.99 * len(server_times)) - 1],
            'server_slowest': sorted(server_times, reverse=True)[:3],
        }

        if os.environ.get('AX_LOCAL_EXECUTOR_LATENCY_DUMP'):
            Path(os.environ['AX_LOCAL_EXECUTOR_LATENCY_DUMP']).write_text(
                json.dumps({'client': samples, 'server': server_times}))

        if os.environ.get('AX_LOCAL_EXECUTOR_LATENCY_OUT'):
            Path(os.environ['AX_LOCAL_EXECUTOR_LATENCY_OUT']).write_text(json.dumps(summary, indent=1))

        print('local executor lease latency', json.dumps(summary))
        self.assertEqual(summary['n'], 1000)
        self.assertEqual(summary['accept_records'], 1000)
        self.assertGreater(summary['action_goals_during'], 0)
        self.assertLess(summary['p99_s'], P99_LIMIT_S)
        self.assertLess(summary['max_s'], MAX_LIMIT_S)

    # --- A2 왕복 ---

    # A2 suction_tool_node.check_lease를 코드 그대로 local_task_executor에 왕복한다
    def test_suction_tool_node_round_trip(self):
        tool = load_suction_tool_node()

        for policy, expected in (('dev_allow', (True, '')), ('dev_deny', (False, 'LEASE_EXPIRED'))):
            with self.subTest(policy=policy):
                self.doCleanups()
                self.start(policy=policy)
                node = tool.SuctionToolNode()
                executor = MultiThreadedExecutor()
                executor.add_node(node)
                spinner = threading.Thread(target=executor.spin, daemon=True)
                spinner.start()

                try:
                    self.assertTrue(node.lease_client.wait_for_service(timeout_sec=5))
                    command = ToolCommand.Request()
                    command.request_id = uuid.uuid4().hex
                    command.robot_id = ROBOT
                    command.execution_id = 'exec_test'
                    command.goal_id = check_tool_attach_goal_id()
                    command.object_id = 'test_box'
                    command.lease_generation = 1
                    command.command = ToolCommand.Request.ATTACH

                    result = node.check_lease(command, tool.command_digest(command),
                                              tool.boottime_seconds())

                    self.assertEqual(result, expected)
                finally:
                    executor.shutdown()
                    spinner.join(timeout=5)
                    node.destroy_node()

        self.doCleanups()

    # --- 시작 검사 ---

    # 같은 로봇의 lease 서버가 이미 있으면 local_task_executor 시작을 거절한다. threads < 2도 거절
    def test_refuses_duplicate_server_and_bad_threads(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        rclpy.init(args=['--ros-args', '-p', f'robot_id:={ROBOT}',
                         '-p', f'execution_db:={self.temp.name}/e.sqlite3',
                         '-p', f'lease_db:={self.temp.name}/l.sqlite3'])
        self.addCleanup(rclpy.shutdown)
        legacy = LeaseCheckServer()
        self.addCleanup(legacy.destroy_node)

        with patch.dict(os.environ, {}), self.assertRaises(RuntimeError):
            LocalTaskExecutor()

        self.doCleanups()

        rclpy.init(args=['--ros-args', '-p', f'robot_id:={ROBOT}', '-p', 'threads:=1',
                         '-p', 'name_check_s:=0.0',
                         '-p', f'execution_db:={tempfile.mkdtemp()}/e.sqlite3',
                         '-p', f'lease_db:={tempfile.mkdtemp()}/l.sqlite3'])
        self.addCleanup(rclpy.shutdown)

        with self.assertRaises(ValueError):
            LocalTaskExecutor()


if __name__ == '__main__':
    unittest.main()
