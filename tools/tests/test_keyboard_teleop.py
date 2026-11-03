import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('keyboard_teleop', ROOT / 'tools/keyboard_teleop.py')
teleop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teleop)


class KeyboardControl(unittest.TestCase):
    def setUp(self):
        self.limits = yaml.safe_load((ROOT / 'src/ax_description/config/arm_joint_limits.yaml').read_text())['joint_limits']

    def test_jog_preserves_other_joints_and_clamps_limit(self):
        for index, name in enumerate(teleop.JOINTS):
            for direction, bound in [(1, 'max_position'), (-1, 'min_position')]:
                start = [0.] * 5
                start[index] = self.limits[name][bound] - direction * .01
                target, _ = teleop.jog_target(start, index, direction, self.limits)
                self.assertAlmostEqual(target[index], self.limits[name][bound])
                self.assertEqual([v for i, v in enumerate(target) if i != index], [0.] * 4)

    def test_quintic_duration_respects_velocity_and_acceleration(self):
        for index, name in enumerate(teleop.JOINTS):
            target, duration = teleop.jog_target([0.] * 5, index, 1, self.limits, step=.5)
            distance = target[index]
            self.assertLessEqual(1.875 * distance / duration, self.limits[name]['max_velocity'] + 1e-9)
            self.assertLessEqual(5.774 * distance / duration ** 2, self.limits[name]['max_acceleration'] + 1e-9)

    def test_invalid_state_is_rejected(self):
        for values in ([0.], [float('nan')] * 5, [float('inf')] * 5):
            with self.assertRaises(ValueError):
                teleop.jog_target(values, 0, 1, self.limits)

    def test_robot_switch_requests_stop(self):
        calls = []
        state = SimpleNamespace(robot='robot_1', stop=lambda: calls.append('stop'))
        self.assertTrue(teleop.KeyboardTeleop.key(state, ord('r')))
        self.assertEqual(state.robot, 'robot_2')
        self.assertEqual(calls, ['stop'])

    def test_driving_is_blocked_during_arm_action(self):
        state = SimpleNamespace(pending=True, velocity=(0., 0.))
        teleop.KeyboardTeleop.key(state, ord('w'))
        self.assertEqual(state.velocity, (0., 0.))


if __name__ == '__main__':
    unittest.main()
