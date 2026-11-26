"""관제 표시 중계 노드의 실제 DDS·HTTP 시험(격리 설치본, 시험 전용 publisher, Gazebo 없음).

확인:
- 시험 publisher(ActuationState·GroupTaskList 개발 Topic) → GET /api/state 반영(정상·값·출처 dev_config)
- 발행 없는 로봇은 미수집, 미래 stamp 표본은 반영하지 않고 횟수만 증가
- 발행 중단 뒤 만료로 바뀜(판정 status), 같은 세션 revision 감소 표본 폐기
- graph: 노드의 publisher는 /rosout·/parameter_events뿐,
  service client·action client·action server 0, 파라미터 Service 없음
- POST 405·다른 Host 403, SIGINT 종료와 잔여 프로세스 0
환경: 검증할 격리 install을 source한 셸. domain은 ros_test_domain으로 60~69에서 빈 곳을 예약한다.
"""
import http.client
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import unittest

from ament_index_python.packages import get_package_prefix
from ax_interfaces.msg import ActuationState, GroupTaskList, GroupTaskStatus, StateMeta
import rclpy
from rclpy.action import get_action_client_names_and_types_by_node
from rclpy.action import get_action_server_names_and_types_by_node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402

NODE = 'ax_operations_console'
DEV_TASKS = '/ax/dev/d2_mock/fleet_tasks'
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
RUN_TAG = f'console_ros_{os.getpid()}'
_domain_state = None


# 모듈 domain 예약과 빈 domain 확인
def setUpModule():
    global _domain_state
    _domain_state = ros_test_domain.enter_module_domain(range(60, 70))
    found = ros_test_domain.conflicting_nodes(
        _domain_state['domain'], r'^(?!/ax_test_domain_probe$).+', wait_s=3.0)

    if found:
        ros_test_domain.leave_module_domain(_domain_state)
        raise AssertionError(ros_test_domain.conflict_message(_domain_state['domain'], found))

    os.environ['AX_RUN_TAG'] = RUN_TAG


# domain 반환
def tearDownModule():
    ros_test_domain.leave_module_domain(_domain_state)


# 비어 있는 TCP 포트 하나
def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class ConsoleRosTest(unittest.TestCase):
    """설치된 operations_console을 하위 프로세스로 띄워 확인한다(모듈에서 한 번)."""

    # 시험 노드·publisher 준비와 관제 노드 실행
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('console_ros_probe')
        cls.act_pub = cls.node.create_publisher(
            ActuationState, '/ax/robot_1/safety/actuation_state', STATE_QOS)
        cls.task_pub = cls.node.create_publisher(GroupTaskList, DEV_TASKS, STATE_QOS)
        cls.session = f'probe_{os.getpid()}'
        cls.revision = 0
        cls.port = free_port()
        executable = Path(get_package_prefix(NODE)) / 'lib' / NODE / 'operations_console'
        args = [str(executable), '--ros-args', '-p', 'topics_profile:=dev',
                '-p', f'port:={cls.port}', '-p', 'use_sim_time:=false']
        cls.proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True)
        cls.wait_http()

    # 관제 노드 정리와 잔여 프로세스 확인
    @classmethod
    def tearDownClass(cls):
        if cls.proc.poll() is None:
            cls.proc.send_signal(signal.SIGINT)
            try:
                cls.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                cls.proc.kill()
                cls.proc.wait()

        cls.node.destroy_node()
        rclpy.shutdown()
        left = ros_test_domain.processes_with(f'AX_RUN_TAG={RUN_TAG}')
        assert [pid for pid in left if pid != os.getpid()] == [], left

    # HTTP가 열릴 때까지 대기(최대 20 s)
    @classmethod
    def wait_http(cls):
        end = time.monotonic() + 20.0

        while time.monotonic() < end:
            if cls.proc.poll() is not None:
                raise AssertionError(f'관제 노드 종료: {cls.proc.stdout.read()}')
            try:
                cls.get('/api/state')
                return
            except OSError:
                time.sleep(0.2)

        raise AssertionError('관제 HTTP 시작 안 됨')

    # HTTP 요청 → (상태, 본문)
    @classmethod
    def http(cls, method, path, host=None):
        conn = http.client.HTTPConnection('127.0.0.1', cls.port, timeout=5)
        conn.putrequest(method, path, skip_host=True)
        conn.putheader('Host', f'127.0.0.1:{cls.port}' if host is None else host)
        conn.endheaders()
        response = conn.getresponse()
        body = response.read()
        conn.close()
        return response.status, body

    # /api/state JSON
    @classmethod
    def get(cls, path):
        status, body = cls.http('GET', path)
        assert status == 200, status
        return json.loads(body)

    # StateMeta 채우기(stamp_offset_s는 현재 ROS 시각 기준)
    def meta(self, revision=None, stamp_offset_s=0.0, valid=True):
        if revision is None:
            type(self).revision += 1
            revision = self.revision

        m = StateMeta()
        now = self.node.get_clock().now().nanoseconds + int(stamp_offset_s * 1e9)
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(now, 10 ** 9)
        m.schema_version = 2
        m.source_id = 'console_ros_probe'
        m.source_session_id = self.session
        m.revision = revision
        m.valid = valid
        return m

    # 구동 상태 발행
    def publish_actuation(self, **meta_args):
        msg = ActuationState()
        msg.meta = self.meta(**meta_args)
        msg.robot_id = 'robot_1'
        msg.state = ActuationState.RUNNING
        msg.drive_allowed = True
        self.act_pub.publish(msg)
        return msg.meta.revision

    # 작업 목록 발행
    def publish_tasks(self):
        msg = GroupTaskList()
        msg.meta = self.meta()
        task = GroupTaskStatus()
        task.task_id = 'task_probe'
        task.state = GroupTaskStatus.RUNNING
        task.transfer_state = GroupTaskStatus.TRANSFER_LOCKED
        task.manipulator_robot_id = 'robot_1'
        task.transporter_robot_id = 'robot_2'
        msg.tasks.append(task)
        self.task_pub.publish(msg)

    # 조건이 맞을 때까지 발행하며 상태 확인(최대 timeout_s)
    def until(self, check, publish=None, timeout_s=10.0):
        end = time.monotonic() + timeout_s
        state = None

        while time.monotonic() < end:
            if publish:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            state = self.get('/api/state')
            if check(state):
                return state
            time.sleep(0.05)

        self.fail(f'조건 미충족. 마지막 상태: {json.dumps(state, ensure_ascii=False)[:1500]}')

    # 로봇 항목 도우미
    @staticmethod
    def item(state, robot, kind):
        return next(r for r in state['robots'] if r['robot_id'] == robot)['items'][kind]

    # robot_1 구동 항목과 그 수신 횟수
    def actuation(self, state):
        return self.item(state, 'robot_1', 'actuation')

    def counts(self, state):
        return self.actuation(state)['counts']

    # 발행 → 반영, 미수집, 배정, 출처, 미래 stamp 거부, 만료, revision 감소 폐기
    def test_flow(self):
        def publish():
            self.publish_actuation()
            self.publish_tasks()

        state = self.until(lambda s: self.item(s, 'robot_1', 'actuation')['status'] == 'ok'
                           and s['tasks']['status'] == 'ok', publish)
        self.assertEqual(self.item(state, 'robot_1', 'actuation')['value']['state'], 'RUNNING')
        self.assertEqual(self.item(state, 'robot_2', 'actuation')['status'], 'missing')
        self.assertNotIn('value', self.item(state, 'robot_2', 'pose'))
        self.assertEqual(state['robot_source'], 'dev_config')
        self.assertEqual(state['topics'], {'profile': 'dev', 'tasks': DEV_TASKS})
        self.assertEqual(state['kpi']['task_by_state'], {'RUNNING': 1})
        robot_1 = next(r for r in state['robots'] if r['robot_id'] == 'robot_1')
        self.assertEqual([t['task_id'] for t in robot_1['assignment']['tasks']], ['task_probe'])

        # 미래 stamp(+2 s): 반영하지 않고 횟수만 증가
        before = self.item(state, 'robot_1', 'actuation')['meta']['revision']
        self.publish_actuation(stamp_offset_s=2.0)
        state = self.until(lambda s: self.counts(s)['future_stamp'] >= 1)
        self.assertEqual(self.item(state, 'robot_1', 'actuation')['meta']['revision'], before)

        # 같은 세션 revision 감소: 높은 revision을 반영한 뒤 낮은 revision은 폐기
        self.publish_actuation(revision=1000)
        self.until(lambda s: (self.actuation(s).get('meta') or {}).get('revision') == 1000)
        self.publish_actuation(revision=999)
        state = self.until(lambda s: self.counts(s)['old_revision'] >= 1)
        self.assertEqual(self.item(state, 'robot_1', 'actuation')['meta']['revision'], 1000)

        # 발행 중단 → 판정 만료(0.3 s 기준), 값 없음은 표시 여유(0.5 s) 뒤
        state = self.until(lambda s: self.actuation(s)['display_status'] == 'stale',
                           timeout_s=5.0)
        self.assertEqual(self.item(state, 'robot_1', 'actuation')['status'], 'stale')
        self.assertNotIn('value', self.item(state, 'robot_1', 'actuation'))

    # graph: 쓰기 경로 없음
    def test_graph_read_only(self):
        end = time.monotonic() + 10.0
        name, namespace = NODE, '/'

        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (name, namespace) in self.node.get_node_names_and_namespaces():
                break

        graph = self.node
        publishers = {t for t, _ in graph.get_publisher_names_and_types_by_node(name, namespace)}
        clients = self.node.get_client_names_and_types_by_node(name, namespace)
        services = {s for s, _ in self.node.get_service_names_and_types_by_node(name, namespace)}
        action_clients = get_action_client_names_and_types_by_node(self.node, name, namespace)
        action_servers = get_action_server_names_and_types_by_node(self.node, name, namespace)
        subscribed = graph.get_subscriber_names_and_types_by_node(name, namespace)
        subscriptions = {t for t, _ in subscribed}

        self.assertEqual(publishers - {'/rosout', '/parameter_events'}, set())
        self.assertEqual((clients, action_clients, action_servers), ([], [], []))
        self.assertFalse([s for s in services if 'parameter' in s], services)
        self.assertIn(DEV_TASKS, subscriptions)
        self.assertIn('/ax/robot_4/battery_state', subscriptions)
        self.assertNotIn('/ax/fleet/tasks', subscriptions)

    # HTTP 쓰기 거절·Host 검사
    def test_http_refusals(self):
        self.assertEqual(self.http('POST', '/api/state')[0], 405)
        self.assertEqual(self.http('PUT', '/')[0], 405)
        self.assertEqual(self.http('GET', '/api/state', host=f'evil.example:{self.port}')[0], 403)


if __name__ == '__main__':
    unittest.main()
