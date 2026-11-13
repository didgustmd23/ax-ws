"""An empty or uncertain corridor list must never authorize clear travel."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from ax_interfaces.msg import CorridorObservation, CorridorObservationList, LocalizationHealth, SafetyState
from builtin_interfaces.msg import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_safety'))
from ax_safety.b4.b4_risk_node import B4RiskNode


def valid_meta(message):
    message.meta.valid = True
    message.meta.schema_version = 1
    message.meta.revision = 1
    message.meta.source_id = 'test'
    message.meta.source_session_id = 'test-session'
    message.meta.header.stamp = Time(sec=100)
    message.robot_id = 'robot_1'


def publish(observations):
    health = LocalizationHealth()
    valid_meta(health)
    health.state = health.OK
    corridors = CorridorObservationList()
    valid_meta(corridors)
    corridors.observations = observations
    received = time.monotonic()
    node = SimpleNamespace(robot_id='robot_1', localization=health, corridor=corridors,
        front=2., rear=2., front_time=received, rear_time=received,
        loc_time=received, corridor_time=received,
        sensor_ttl=.25, loc_ttl=1., corridor_ttl=1., max_linear=.67, max_angular=.5,
        stop_distance=.5, slowdown_distance=1., session='test', revision=0, publisher=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000, to_msg=lambda: Time(sec=100))))
    node.fresh = lambda *args: B4RiskNode.fresh(node, *args)
    node.recent_stamp = lambda *args: B4RiskNode.recent_stamp(node, *args)
    node.meta_ok = lambda *args: B4RiskNode.meta_ok(node, *args)
    B4RiskNode.publish_state(node)
    return node.publisher.publish.call_args.args[0]


def observation():
    message = CorridorObservation()
    valid_meta(message)
    message.corridor_id = 'corridor_1'
    message.position_valid = True
    message.observation = message.OUTSIDE
    return message


class B4ObservationValidation(unittest.TestCase):
    def assert_blocked(self, message):
        self.assertEqual(message.level, SafetyState.UNKNOWN)
        self.assertFalse(message.meta.valid)
        self.assertFalse(message.allow_forward or message.allow_backward or message.allow_rotate)
        self.assertEqual(message.max_linear_speed, 0.)

    def test_empty_list_is_unknown(self):
        self.assert_blocked(publish([]))

    def test_known_observation_preserves_clear_path(self):
        result = publish([observation()])
        self.assertEqual(result.level, SafetyState.CLEAR)
        self.assertTrue(result.meta.valid and result.allow_forward)

    def test_unknown_observation_blocks(self):
        item = observation()
        item.observation = item.UNKNOWN
        self.assert_blocked(publish([item]))

    def test_invalid_position_blocks(self):
        item = observation()
        item.position_valid = False
        self.assert_blocked(publish([item]))


if __name__ == '__main__':
    unittest.main()
