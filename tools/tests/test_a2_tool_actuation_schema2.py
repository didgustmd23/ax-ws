"""A2 suction_tool_node의 ActuationState Schema 2 수신·허가 판단 시험(계약 INTERFACE_CONTRACT Schema 2)."""
# [검사 범위] 공구 노드의 수신 단계 검사(ActuationScreen)와 arm_allowed 판단만 본다.
# 실제 A4 safety_gate·Gazebo·A3 lease는 쓰지 않는다. 시각은 가짜 시계로 고정한다.
import importlib.util
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
import uuid

from ax_interfaces.msg import ActuationState
import rclpy
from rclpy.executors import SingleThreadedExecutor

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[2] / 'src/ax_control/scripts'
ROBOT = 'robot_s2test'
_DOMAIN_STATE = None
# 공구 노드가 시험 로봇 이름 공간을 쓰도록 robot_id를 넘긴다.
ROS_ARGS = ['--ros-args', '-p', f'robot_id:={ROBOT}']


# 모듈 전체에 시험용 domain 하나를 잡는다(같은 이름 공구 노드가 있으면 시작하지 않는다).
def setUpModule():
    global _DOMAIN_STATE
    _DOMAIN_STATE = ros_test_domain.enter_module_domain(ros_test_domain.A3_DOMAINS, rf'^/ax/{ROBOT}/')


# 모듈이 끝나면 domain 환경변수를 되돌린다.
def tearDownModule():
    ros_test_domain.leave_module_domain(_DOMAIN_STATE)


# suction_tool_node 스크립트를 모듈로 읽는다(실행 경로와 같은 코드를 쓰기 위해)
def load_tool_module():
    spec = importlib.util.spec_from_file_location('suction_tool_node_s2', SCRIPTS / 'suction_tool_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


TOOL = load_tool_module()


# 시험용 ActuationState를 만든다. 기본값은 정상 Schema 2 메시지다
def actuation(stamp_s=100.0, robot_id=ROBOT, schema=2, valid=True, source='a4_safety_gate',
              session='s1', revision=1, arm=True, state=ActuationState.RUNNING):
    message = ActuationState()

    # builtin Time은 정수 필드라 NaN·무한대는 표현할 수 없다. 0 stamp로 무효 시각을 시험한다.
    total_ns = int(round(stamp_s * 1e9))
    message.meta.header.stamp.sec = total_ns // 1_000_000_000
    message.meta.header.stamp.nanosec = total_ns % 1_000_000_000
    message.meta.schema_version = schema
    message.meta.valid = valid
    message.meta.source_id = source
    message.meta.source_session_id = session
    message.meta.revision = revision
    message.robot_id = robot_id
    message.state = state
    message.drive_allowed = arm
    message.arm_allowed = arm

    return message


# ActuationScreen 단위 시험(ROS 그래프 없이 순수 판단)
class ActuationScreenTest(unittest.TestCase):
    # 매 시험 새 검사기를 만든다
    def setUp(self):
        self.screen = TOOL.ActuationScreen(ROBOT)

    # 정상 Schema 2는 수용하고 기준값을 갱신한다
    def test_accepts_valid_schema2(self):
        self.assertEqual(self.screen.check(actuation()), '')
        self.assertEqual((self.screen.session, self.screen.revision), ('s1', 1))

    # 식별·형식 오류를 사유별로 거부한다
    def test_rejects_malformed(self):
        cases = [
            (actuation(robot_id='robot_2'), 'ROBOT_MISMATCH'),
            (actuation(schema=1), 'SCHEMA_MISMATCH'),
            (actuation(session=''), 'SOURCE_EMPTY'),
            (actuation(source=''), 'SOURCE_EMPTY'),
            (actuation(revision=0), 'REVISION_ZERO'),
            (actuation(stamp_s=0.0), 'STAMP_INVALID'),
        ]

        for message, reason in cases:
            with self.subTest(reason=reason):
                self.assertEqual(self.screen.check(message), reason)

        # 거부는 추적 상태를 만들지 않는다.
        self.assertIsNone(self.screen.session)

    # valid=false도 형식이 맞으면 수용한다(무효 상태로 덮어써 fail-closed)
    def test_accepts_invalid_flag(self):
        self.assertEqual(self.screen.check(actuation(valid=False)), '')

    # 미래 stamp는 수신 단계에서 거르지 않는다(평가 시점에 판단)
    def test_future_stamp_not_rejected_on_receive(self):
        self.assertEqual(self.screen.check(actuation(stamp_s=1e9)), '')

    # 같은 세션의 revision·stamp 감소는 거부하고 기준값을 유지한다
    def test_rejects_regression(self):
        self.assertEqual(self.screen.check(actuation(revision=5, stamp_s=100.0)), '')
        self.assertEqual(self.screen.check(actuation(revision=4, stamp_s=100.1)), 'REVISION_REGRESSED')
        self.assertEqual(self.screen.check(actuation(revision=6, stamp_s=99.9)), 'REVISION_REGRESSED')
        self.assertEqual((self.screen.revision, self.screen.stamp), (5, 100.0))

        # 같은 revision·stamp 재수신은 감소가 아니다.
        self.assertEqual(self.screen.check(actuation(revision=5, stamp_s=100.0)), '')

    # 새 세션을 받으면 이전 세션은 다시 받지 않는다
    def test_retires_previous_session(self):
        self.assertEqual(self.screen.check(actuation(session='s1', revision=9)), '')
        self.assertEqual(self.screen.check(actuation(session='s2', revision=1)), '')
        self.assertEqual(self.screen.check(actuation(session='s1', revision=10, stamp_s=200.0)), 'SESSION_RETIRED')
        self.assertEqual(self.screen.session, 's2')


# 가짜 ROS 시계. arm_allowed의 sim 표본 나이 계산에 쓴다
class FakeClock:
    # 현재 시각(s)을 정한다
    def __init__(self, now_s):
        self.now_s = now_s

    # rclpy Clock.now()처럼 nanoseconds 속성을 가진 값을 돌려준다
    def now(self):
        return type('Now', (), {'nanoseconds': int(self.now_s * 1e9)})()


# SuctionToolNode의 on_actuation·arm_allowed 시험(프로세스 안 rclpy)
class ToolNodeActuationTest(unittest.TestCase):
    # 노드를 만들고 단조 시각·ROS 시계를 가짜로 바꾼다
    def setUp(self):
        rclpy.init(args=ROS_ARGS)
        self.node = TOOL.SuctionToolNode()
        self.assertEqual(self.node.robot_id, ROBOT)

        self.boot = [1000.0]
        self.clock = FakeClock(100.0)
        self.patches = [
            patch.object(TOOL, 'boottime_seconds', lambda: self.boot[0]),
            patch.object(self.node, 'get_clock', lambda: self.clock),
        ]

        for item in self.patches:
            item.start()

    # 패치와 노드를 정리한다
    def tearDown(self):
        for item in self.patches:
            item.stop()

        self.node.destroy_node()
        rclpy.shutdown()

    # 정상 Schema 2 수용 후 허가, 다른 로봇·schema 1은 허가를 만들지 않는다
    def test_accept_and_reject(self):
        self.node.on_actuation(actuation(robot_id='robot_2'))
        self.node.on_actuation(actuation(schema=1))
        self.node.on_actuation(actuation(session=''))
        self.assertFalse(self.node.arm_allowed())

        self.node.on_actuation(actuation())
        self.assertTrue(self.node.arm_allowed())

    # 거부 메시지가 연속으로 와도 만료(0.3 s)를 늦추지 않는다
    def test_rejected_does_not_extend_expiry(self):
        self.node.on_actuation(actuation(revision=3, stamp_s=100.0))
        self.assertTrue(self.node.arm_allowed())

        for step in range(1, 5):
            self.boot[0] += 0.1
            self.clock.now_s += 0.1
            self.node.on_actuation(actuation(revision=2, stamp_s=100.0 + 0.1 * step))

        self.assertGreater(self.boot[0] - 1000.0, TOOL.STATE_EXPIRY_SECONDS)
        self.assertFalse(self.node.arm_allowed())

    # 이전 세션이 다시 나타나도 허가를 되살리지 않는다
    def test_retired_session_ignored(self):
        self.node.on_actuation(actuation(session='s1', revision=7, arm=True))
        self.node.on_actuation(actuation(session='s2', revision=1, arm=False))
        self.node.on_actuation(actuation(session='s1', revision=8, arm=True, stamp_s=100.0))
        self.assertFalse(self.node.arm_allowed())

    # 유효 수용 뒤 valid=false가 오면 즉시 거절한다
    def test_invalid_flag_closes_immediately(self):
        self.node.on_actuation(actuation(revision=1))
        self.assertTrue(self.node.arm_allowed())

        self.node.on_actuation(actuation(revision=2, valid=False))
        self.assertFalse(self.node.arm_allowed())

    # 미래 stamp는 그 순간만 거절하고, 시계가 따라오면 같은 메시지로 허가한다
    def test_future_stamp_denied_until_clock_catches_up(self):
        self.node.on_actuation(actuation(stamp_s=100.05))
        self.assertFalse(self.node.arm_allowed())
        self.assertFalse(self.node.arm_allowed())
        self.assertEqual(self.node.future_stamp_denials, 2)

        self.clock.now_s = 100.06
        self.assertTrue(self.node.arm_allowed())

    # 허가는 arm_allowed만 따른다. STOPPED는 허가가 아니고 허가를 막지도 않는다
    def test_stopped_is_not_permission(self):
        self.node.on_actuation(actuation(revision=1, state=ActuationState.STOPPED, arm=False))
        self.assertFalse(self.node.arm_allowed())

        self.node.on_actuation(actuation(revision=2, state=ActuationState.STOPPED, arm=True))
        self.assertTrue(self.node.arm_allowed())

    # 오래된 표본(sim 나이 > 0.3 s)은 거절한다(기존 검사 유지)
    def test_stale_sample(self):
        self.node.on_actuation(actuation(stamp_s=99.6))
        self.assertFalse(self.node.arm_allowed())


# 실제 토픽으로 발행해 공구 노드 구독 경로가 Schema 2만 받는지 확인한다
class ToolNodeTopicTest(unittest.TestCase):
    # 실제 시계를 쓰는 노드와 발행 노드를 만든다
    def setUp(self):
        rclpy.init(args=ROS_ARGS)
        self.tool = TOOL.SuctionToolNode()

        self.helper = rclpy.create_node('a2_s2_publisher')
        self.publisher = self.helper.create_publisher(
            ActuationState, f'/ax/{ROBOT}/safety/actuation_state', TOOL.state_qos())
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.tool)
        self.executor.add_node(self.helper)

    # 노드를 정리한다
    def tearDown(self):
        self.executor.shutdown()
        self.tool.destroy_node()
        self.helper.destroy_node()
        rclpy.shutdown()

    # 조건이 참이 될 때까지 spin한다
    def spin_until(self, condition, timeout=3.0):
        end = time.monotonic() + timeout

        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

            if condition():
                return True

        return False

    # 현재 시각 stamp로 메시지를 발행한다
    def publish(self, **kwargs):
        now = self.tool.get_clock().now().nanoseconds / 1e9 - 0.01
        self.publisher.publish(actuation(stamp_s=now, **kwargs))

    # schema 1은 토픽으로 와도 허가하지 않고, schema 2는 허가한다
    def test_topic_round_trip(self):
        session = uuid.uuid4().hex
        deadline = time.monotonic() + 1.0

        while time.monotonic() < deadline:
            self.publish(schema=1, session=session, revision=1)
            self.executor.spin_once(timeout_sec=0.05)

        self.assertIsNone(self.tool.actuation)
        self.assertEqual(self.tool.actuation_reject_reason, 'SCHEMA_MISMATCH')

        revision = [1]

        # 정상 메시지가 수용되고 허가될 때까지 반복 발행한다.
        def accepted():
            revision[0] += 1
            self.publish(session=session, revision=revision[0])

            return self.tool.actuation is not None and self.tool.arm_allowed()

        self.assertTrue(self.spin_until(accepted))


if __name__ == '__main__':
    unittest.main()
