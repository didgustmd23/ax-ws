# 키 입력의 한계값·설정 변경·정지·관절 목표 시간을 검사한다. 실물 제어 성능 검증과 구분한다.
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

    def test_speed_adjustment_stays_below_controller_limits_and_stops(self):
        calls = []
        # ROS 노드를 시작하지 않고 실제 속도 계산 속성과 현재 공통 제한을 사용한다.
        class DriveState(SimpleNamespace):
            linear_speed = teleop.KeyboardTeleop.linear_speed
            angular_speed = teleop.KeyboardTeleop.angular_speed
        profile = teleop.DRIVE_PROFILE
        state = DriveState(adjust_mode='drive', drive_scale=1., arm_step=.04,
                                min_drive_scale=profile['minimum_linear'] / profile['default_linear'],
                                max_drive_scale=profile['maximum_linear'] / profile['default_linear'],
                                velocity=(.1, 0.), key_deadline=10., publish_drive=lambda: calls.append('publish'))
        for _ in range(30):
            teleop.KeyboardTeleop.adjust(state, 1)
        self.assertAlmostEqual(state.linear_speed, profile['maximum_linear'])
        self.assertLessEqual(state.angular_speed, profile['maximum_angular'])
        self.assertEqual(state.velocity, (0., 0.))
        self.assertEqual(state.key_deadline, 0.)
        for _ in range(30):
            teleop.KeyboardTeleop.adjust(state, -1)
        self.assertAlmostEqual(state.linear_speed, profile['minimum_linear'])

    def test_angle_adjustment_does_not_change_speed_or_move(self):
        state = SimpleNamespace(adjust_mode='arm', drive_scale=1., arm_step=.04, max_arm_step=.15)
        teleop.KeyboardTeleop.adjust(state, 1)
        self.assertAlmostEqual(state.arm_step, .05)
        self.assertEqual(state.drive_scale, 1.)
        for _ in range(30):
            teleop.KeyboardTeleop.adjust(state, -1)
        self.assertEqual(state.arm_step, .01)
        for _ in range(30):
            teleop.KeyboardTeleop.adjust(state, 1)
        self.assertEqual(state.arm_step, .15)


if __name__ == '__main__':
    unittest.main()
