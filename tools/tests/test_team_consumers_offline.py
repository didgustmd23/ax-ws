"""ROS 그래프 없이 내 출력 메시지를 상대 원본 소비 함수에 직접 전달합니다."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
for relative in ('src/ax_environment', 'src/ax_task_executor',
                 'src/ax_safety', 'src/ax_d4', 'tools/tests'):
    sys.path.insert(0, str(ROOT / relative))
# rfc8785는 D4에 선언된 의존성입니다. 개인 임시 폴더를 import 경로로 추가하지 않습니다.

from ax_environment.corridor_observer import CorridorObserver
from ax_environment.regions import observe_robot
from ax_interfaces.msg import LocalizationHealth, ResultStatus, RobotTaskStatus, SafetyState
from ax_safety.b4.b4_risk_node import B4RiskNode
from ax_task_executor.mock_executor import MockExecutor
from rclpy.time import Time
from rosidl_runtime_py.set_message import set_message_fields


class B3B4ConsumerTests(unittest.TestCase):
    """실제 B3 메시지 생성과 B4 소비 코드를 연결하며 센서·시계는 시험값입니다."""

    def produce(self, approved):
        """통로 영역 사본에서 실제 B3 기하 판정과 메시지 게시 함수를 실행합니다."""
        catalog = json.loads((ROOT / 'src/ax_environment/config/ax_warehouse_regions.json').read_text())
        catalog['approved'] = approved
        observations = observe_robot(
            catalog, [-0.5, -0.5, 0.5, 0.5], (100.0, 100.0, 0.0),
            frame_id=catalog['frame_id'], map_revision=catalog['map_revision'],
            position_valid=True, outline_complete=True, age_s=0.0)
        reason = '' if approved else 'REGIONS_NOT_APPROVED'
        source = SimpleNamespace(robot_id='robot_1', session='offline-b3', revision=1,
                                 corridor_ids=list(observations), last_key=None, publisher=Mock(),
                                 get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)),
                                 evaluate=lambda now: ({key: (value.state, value.position_valid)
                                                        for key, value in observations.items()}, reason))
        source.meta = MethodType(CorridorObserver.meta, source)
        CorridorObserver.tick(source)
        return source.publisher.publish.call_args.args[0]

    def consume(self, message):
        """B4 원본 콜백·메타 검사·판정 함수를 그대로 호출해 결과를 읽습니다."""
        loc = LocalizationHealth()
        loc.robot_id, loc.state = 'robot_1', LocalizationHealth.OK
        loc.meta = deepcopy(message.meta)
        loc.meta.valid = True
        now = time.monotonic()
        node = SimpleNamespace(
            robot_id='robot_1', localization=loc, loc_time=now,
            corridor=None, corridor_time=0, front=3.0, rear=3.0,
            front_time=now, rear_time=now, sensor_ttl=0.5, loc_ttl=1.0, corridor_ttl=1.0,
            max_linear=0.67, max_angular=0.5, stop_distance=0.5, slowdown_distance=1.0,
            slowdown_linear=0.2, session='offline-b4', revision=1, publisher=Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))
        for name in ('recent_stamp', 'meta_ok', 'fresh'):
            setattr(node, name, MethodType(getattr(B4RiskNode, name), node))
        B4RiskNode.on_corridor(node, message)
        B4RiskNode.publish_state(node)
        return node.publisher.publish.call_args.args[0]

    def test_approved_b3_outside_is_accepted_by_real_b4(self):
        """신선하고 완전한 B3 출력이면 B4가 정상 판정을 할 수 있어야 합니다."""
        message = self.produce(True)
        self.assertTrue(message.observations)
        self.assertTrue(all(item.observation == item.OUTSIDE for item in message.observations))
        result = self.consume(message)
        self.assertTrue(result.meta.valid)
        self.assertEqual(result.level, SafetyState.CLEAR)

    def test_unapproved_b3_is_blocked_by_real_b4(self):
        """운영 영역 미승인 상태를 B4가 안전한 통로라고 오해하면 안 됩니다."""
        result = self.consume(self.produce(False))
        self.assertFalse(result.meta.valid)
        self.assertFalse(result.allow_forward)

    def test_old_b3_output_is_blocked_by_real_b4(self):
        """옛 시각의 B3 출력은 새로 수신해도 정상 주행으로 처리하지 않습니다."""
        message = self.produce(True)
        message.meta.header.stamp = Time(seconds=90).to_msg()
        result = self.consume(message)
        self.assertFalse(result.meta.valid)
        self.assertFalse(result.allow_forward)

    def test_empty_b3_catalog_is_blocked_by_updated_b4(self):
        """새 B4의 빈 목록 차단이 실제 B3 메시지와 연결돼도 유지되는지 확인합니다."""
        message = self.produce(True)
        # 메타가 유효해도 통로가 사라진 목록을 통로 전체 비점유로 해석하면 안 됩니다.
        message.observations = []
        result = self.consume(message)
        self.assertFalse(result.meta.valid)
        self.assertFalse(result.allow_forward or result.allow_backward or result.allow_rotate)


class A3D4ConsumerTests(unittest.TestCase):
    """A3 실제 상태·결과 생성 함수를 D4 원본 검증과 원장에 직접 연결합니다."""

    def test_mock_output_is_schema_compatible_but_not_operational_authority(self):
        """모의 상태는 형식 검사를 통과해도 운영 원장에서는 UNKNOWN이어야 합니다."""
        import test_d4_w0 as fixture
        # 최신 D4의 패키지 import 및 SQLite 명시적 커밋 경로를 그대로 사용합니다.
        from ax_d4.common_contract import check_a3_action_result, check_status
        from ax_d4.input_source import to_d4_status
        from ax_d4.ledger import accept_status

        status = RobotTaskStatus()
        set_message_fields(status, fixture.make_status())
        status.meta.valid = False
        status.meta.validity_reason = 'MOCK_ONLY'
        source = SimpleNamespace(
            task_status=status, _status_lock=threading.Lock(), task_status_publisher=Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))
        source.publish_task_status = MethodType(MockExecutor.publish_task_status, source)
        result = MockExecutor.make_result('STOWED')
        check_a3_action_result(result)
        last_result = ResultStatus()
        set_message_fields(last_result, result['status'])
        MockExecutor.update_task_status(source, 'COMPLETE', result=last_result)
        message = source.task_status_publisher.publish.call_args.args[0]
        payload = to_d4_status(message)
        check_status(payload)
        with tempfile.TemporaryDirectory() as directory:
            response = accept_status(payload, Path(directory) / 'ledger.db',
                                     now_sim_s=100, received_boottime_s=100)
        self.assertTrue(response.startswith('UNKNOWN'), response)


if __name__ == '__main__':
    unittest.main()
