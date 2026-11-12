"""
로컬 작업 실행기 한 프로세스: ExecuteLocalTask Action 서버(모의 적재 확인)와 CheckCommandLease 서비스(개발 정책).

설계 11.1절에 따라 lease 확인을 Action 대기·실행 기록 쓰기와 다른 콜백 그룹으로 분리한다.
- lease(기본 lease_mode=reentrant): 같은 노드의 Reentrant 그룹. 요청 직렬 처리(계약 5.2절)는
  콜백 그룹이 아니라 LeaseLedger.lock + SQLite BEGIN IMMEDIATE가 보장한다(동시에 둘 이상 기록하지 않음).
  비교용: lease_mode=mutex(같은 노드의 MutuallyExclusive 그룹), dedicated(전용 노드 + 단일 스레드 실행기).
  2026-11-12 비교(WAL 이전)에서 mutex는 4/6, dedicated는 2/8 회차에 0.8~2.5 s 지연, reentrant는 0/8이었다.
- Action 그룹(Reentrant): Goal 수락·취소·실행과 실행 대기 타이머. 공유 값은 ExecutorState 잠금으로 보호한다.
- 상태 그룹(MutuallyExclusive): RobotTaskStatus 1 Hz 재발행 타이머.
MultiThreadedExecutor(파라미터 threads, 기본 4)가 세 그룹을 돌린다(dedicated면 lease는 별도 스레드).

잠금 순서(교착 방지): 한 스레드가 두 잠금을 동시에 잡지 않는다.
- ExecutorState.lock: 사전·집합 갱신만(잎 잠금). 안에서 SQLite·ROS 호출·await 없음.
- ExecutionStore.lock·LeaseLedger.lock: 각자의 SQLite 트랜잭션만. 안에서 ROS 호출·await 없음.
- 상태 메시지 잠금(MockExecutor._status_lock): 메시지 계산·교체만, 발행은 잠금 밖.

원장은 분리된 두 파일이다(${ROS_HOME:-~/.ros}/ax_task_executor/<robot_id>/ 아래):
executions.sqlite3, lease_checks.sqlite3.
실제 lease 판정은 없고 DevPolicyAuthority(dev_allow | dev_deny, 기본 dev_deny)가 대신한다.
"""
import threading
import time

from ax_interfaces.srv import CheckCommandLease
from ax_task_executor.executor_state import ExecutorState
from ax_task_executor.lease_check import (
    default_lease_db, DevPolicyAuthority, LEASE_SERVICE_QOS, LeaseChecker, LeaseLedger)
from ax_task_executor.mock_executor import execute_action_name, MockExecutor
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

# 이 실행 파일의 노드 이름과 lease 전용 노드 이름(dedicated 모드)
NODE_NAME = 'local_task_executor'
LEASE_NODE_NAME = 'local_task_executor_lease'
LEASE_MODES = ('reentrant', 'mutex', 'dedicated')


# 로봇의 lease 확인 서비스 경로(계약 4.3절)
def lease_service_name(robot_id):
    return f'/ax/{robot_id}/executor/lease/check_command'


class LocalTaskExecutor(MockExecutor):
    """한 노드 안의 Action 서버와 lease 서비스. 모의 실행 로직은 MockExecutor를 그대로 쓴다."""

    # 콜백 그룹·공유 상태·lease 원장을 준비하고 같은 이름의 다른 서버가 있으면 거절한다
    def __init__(self):
        state = ExecutorState()
        super().__init__(NODE_NAME, action_group=ReentrantCallbackGroup(),
                         status_group=MutuallyExclusiveCallbackGroup(), state=state)

        self.declare_parameter('threads', 4)
        self.declare_parameter('policy', 'dev_deny')
        self.declare_parameter('deny_reason', 'LEASE_EXPIRED')
        self.declare_parameter('valid_for_s', 2.0)
        self.declare_parameter('lease_db', '')
        # 시작 시 같은 서비스·Action 이름을 가진 다른 노드를 찾는 시간(s). 0이면 확인하지 않는다.
        self.declare_parameter('name_check_s', 1.0)
        self.declare_parameter('lease_mode', 'reentrant')

        if int(self.get_parameter('threads').value) < 2:
            raise ValueError('threads must be >= 2 (Action·상태 콜백을 함께 처리)')

        self.lease_mode = self.get_parameter('lease_mode').value

        if self.lease_mode not in LEASE_MODES:
            raise ValueError('lease_mode must be one of ' + ', '.join(LEASE_MODES))

        path = self.get_parameter('lease_db').value or default_lease_db(self.robot_id)
        self.ledger = LeaseLedger(path, self.robot_id)
        authority = DevPolicyAuthority(self.get_parameter('policy').value,
                                       self.get_parameter('deny_reason').value)
        self.checker = LeaseChecker(
            self.robot_id, authority.policy, authority.deny_reason,
            self.get_parameter('valid_for_s').value, self.ledger,
            authority=authority, state=state)

        # lease 서비스를 둘 노드와 콜백 그룹(모드별)
        self.lease_node = None
        self._lease_executor = None
        self._lease_thread = None

        if self.lease_mode == 'dedicated':
            self.lease_node = Node(LEASE_NODE_NAME)
            host, group = self.lease_node, None
        elif self.lease_mode == 'mutex':
            host, group = self, MutuallyExclusiveCallbackGroup()
        else:
            host, group = self, ReentrantCallbackGroup()

        self.lease_service = host.create_service(
            CheckCommandLease, lease_service_name(self.robot_id), self.on_check,
            qos_profile=LEASE_SERVICE_QOS, callback_group=group)

        others = self.duplicate_servers(self.get_parameter('name_check_s').value)

        if others:
            self.destroy_node()
            raise RuntimeError('같은 로봇의 실행기·lease 서버가 이미 실행 중입니다: ' + ', '.join(others))

        self.get_logger().warning(
            f'로컬 작업 실행기(lease 정책 {authority.policy}, lease {self.lease_mode}, '
            f'스레드 {self.thread_count()}). '
            f'실제 lease·세대·배정 검증이 없습니다. lease 원장: {path}')

    # 실행기 스레드 수
    def thread_count(self):
        return int(self.get_parameter('threads').value)

    # lease 전용 노드를 전용 단일 스레드 실행기로 돌린다(dedicated 모드에서만)
    def start_lease_thread(self):
        if self.lease_node is None or self._lease_thread is not None:
            return

        self._lease_executor = SingleThreadedExecutor()
        self._lease_executor.add_node(self.lease_node)
        self._lease_thread = threading.Thread(
            target=self._lease_executor.spin, name='lease_check', daemon=True)
        self._lease_thread.start()

    # lease 전용 실행기 스레드를 멈춘다
    def stop_lease_thread(self):
        if self._lease_executor is None:
            return

        self._lease_executor.shutdown()
        self._lease_thread.join(timeout=5)
        self._lease_executor = None
        self._lease_thread = None

    # 자기(와 lease 전용 노드) 말고 같은 lease 서비스나 Action 서비스를 내는 노드를 wait_s 동안 찾는다
    def duplicate_servers(self, wait_s):
        names = {lease_service_name(self.robot_id),
                 execute_action_name(self.robot_id) + '/_action/send_goal'}
        mine = {(self.get_name(), self.get_namespace()), (LEASE_NODE_NAME, '/')}
        deadline = time.monotonic() + float(wait_s)
        found = set()

        while True:
            for name, namespace in self.get_node_names_and_namespaces():
                if (name, namespace) in mine:
                    continue

                try:
                    services = self.get_service_names_and_types_by_node(name, namespace)
                except Exception:  # noqa: B902 - 사라진 노드는 건너뛴다
                    continue

                for service, _ in services:
                    if service in names:
                        found.add(f'{namespace.rstrip("/")}/{name}:{service}')

            if found or time.monotonic() >= deadline:
                return sorted(found)

            time.sleep(0.1)

    # lease 요청을 판정기에 넘기고 응답을 채운다(lease 콜백 그룹, 직렬)
    def on_check(self, request, reply):
        result = self.checker.check(request)

        reply.allowed = result['allowed']
        reply.reason_code = result['reason_code']
        reply.accept_record_id = result['accept_record_id']
        reply.valid_for_s = result['valid_for_s']

        return reply

    # lease 원장을 닫고 모의 실행기 정리를 이어 한다
    def destroy_node(self):
        self.stop_lease_thread()

        if getattr(self, 'lease_node', None) is not None:
            self.lease_node.destroy_node()
            self.lease_node = None

        if getattr(self, 'ledger', None) is not None:
            self.ledger.close()
            self.ledger = None

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalTaskExecutor()
    executor = MultiThreadedExecutor(num_threads=node.thread_count())
    executor.add_node(node)
    node.start_lease_thread()

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
