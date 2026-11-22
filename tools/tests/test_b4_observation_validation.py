"""B4 통로 관측 유효성에 따른 안전 차단 동작을 검증한다.

빈 관측 목록, UNKNOWN 관측, 잘못된 위치와 메타데이터는
통행 허가로 해석되지 않고 UNKNOWN 상태로 차단되어야 한다.
"""

from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from ax_interfaces.msg import CorridorObservation, CorridorObservationList, LocalizationHealth, SafetyState
from builtin_interfaces.msg import Time


# 테스트 실행 위치와 관계없이 작업 공간의 B4 구현을 직접 가져온다.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_safety'))
from ax_safety.b4.b4_risk_node import B4RiskNode


def valid_meta(message, schema_version=1, source_id='test'):
    """테스트 메시지가 공통 StateMeta 계약을 만족하도록 기본값을 채운다."""

    message.meta.valid = True
    message.meta.schema_version = schema_version
    message.meta.revision = 1
    message.meta.source_id = source_id
    message.meta.source_session_id = 'test-session'
    message.meta.header.stamp = Time(sec=100)
    message.robot_id = 'robot_1'


def publish(observations, health_schema=1, health_source='test',
            corridor_schema=1, corridor_source='test'):
    """B4의 입력 상태를 구성하고 발행될 SafetyState를 반환한다."""

    health = LocalizationHealth()
    valid_meta(health, health_schema, health_source)
    health.state = health.OK
    corridors = CorridorObservationList()
    valid_meta(corridors, corridor_schema, corridor_source)
    corridors.observations = observations
    received = time.monotonic()
    node = SimpleNamespace(robot_id='robot_1', localization=health, corridor=corridors,
        front=2., rear=2., front_time=received, rear_time=received,
        loc_time=received, corridor_time=received,
        sensor_ttl=.25, loc_ttl=1., corridor_ttl=1., max_linear=.67, max_angular=.5,
        stop_distance=.5, slowdown_distance=1., session='test', revision=0, publisher=Mock(),
        # fbde940 B4가 __init__에서 만드는 배터리 속성. 기본값(b4_risk_node.py 75~91행)으로
        # 배터리 검사를 끈 상태(battery_required=False)라 통로 관측 판정 기대값은 그대로다.
        battery=None, battery_time=None, battery_required=False, battery_topic='battery_state',
        battery_ttl=.5, battery_low_percent=22.0, battery_critical_percent=-1.0,
        battery_low_action_level=-1, battery_critical_action_level=-1, last_battery_risk=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000, to_msg=lambda: Time(sec=100))))
    node.fresh = lambda *args: B4RiskNode.fresh(node, *args)
    node.recent_stamp = lambda *args: B4RiskNode.recent_stamp(node, *args)
    node.meta_ok = lambda *args: B4RiskNode.meta_ok(node, *args)
    B4RiskNode.publish_state(node)
    return node.publisher.publish.call_args.args[0]


def observation():
    """정상적인 OUTSIDE 통로 관측 하나를 만든다."""

    message = CorridorObservation()
    valid_meta(message)
    message.corridor_id = 'corridor_1'
    message.position_valid = True
    message.observation = message.OUTSIDE
    return message


class B4ObservationValidation(unittest.TestCase):
    """유효하지 않은 통로 관측이 항상 안전 차단으로 이어지는지 확인한다."""

    def assert_blocked(self, message):
        """차단 결과가 UNKNOWN이고 모든 이동 허가가 꺼졌는지 확인한다."""

        self.assertEqual(message.level, SafetyState.UNKNOWN)
        self.assertFalse(message.meta.valid)
        self.assertFalse(message.allow_forward or message.allow_backward or message.allow_rotate)
        self.assertEqual(message.max_linear_speed, 0.)

    def test_empty_list_is_unknown(self):
        """통로 관측이 없으면 B4가 통행을 허가하지 않아야 한다."""

        self.assert_blocked(publish([]))

    def test_known_observation_preserves_clear_path(self):
        """정상 관측은 기존의 CLEAR 판단과 전진 허가를 유지해야 한다."""

        result = publish([observation()])
        self.assertEqual(result.level, SafetyState.CLEAR)
        self.assertTrue(result.meta.valid and result.allow_forward)

    def test_unknown_observation_blocks(self):
        """관측 상태가 UNKNOWN이면 이동을 차단해야 한다."""

        item = observation()
        item.observation = item.UNKNOWN
        self.assert_blocked(publish([item]))

    def test_invalid_position_blocks(self):
        """위치가 검증되지 않은 관측은 안전 입력으로 사용할 수 없어야 한다."""

        item = observation()
        item.position_valid = False
        self.assert_blocked(publish([item]))

    def test_invalid_parent_metadata_blocks(self):
        """LocalizationHealth와 통로 목록의 메타데이터 오류를 차단해야 한다."""

        self.assert_blocked(publish([observation()], health_schema=2))
        self.assert_blocked(publish([observation()], corridor_source=''))

    def test_invalid_observation_metadata_blocks(self):
        """개별 관측의 schema_version 오류도 전체 판단을 차단해야 한다."""

        item = observation()
        item.meta.schema_version = 2
        self.assert_blocked(publish([item]))


if __name__ == '__main__':
    unittest.main()
