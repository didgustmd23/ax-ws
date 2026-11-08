# A2 Gazebo 독립 관측의 순수 계산(관절각 재계산·TCP 합침·정지 시각·속도·짝짓기)을 Gazebo 없이 검사한다.
import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('gazebo_observer', ROOT / 'src/ax_control/scripts/gazebo_observer.py')
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)

ARM_JOINTS = ['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']

# 고정 관절로 합쳐지는 링크(mount)와 기울어진 관절 원점을 가진 작은 시험 모델
SMALL_URDF = """<robot name="t">
<link name="base"/><link name="mount"/><link name="a"/><link name="b"/><link name="tip"/>
<joint name="fix" type="fixed"><parent link="base"/><child link="mount"/><origin xyz="0.1 0 0.2" rpy="0 0 0.3"/></joint>
<joint name="j1" type="revolute"><parent link="mount"/><child link="a"/><origin xyz="0 0 0.1" rpy="0.2 0 0"/>
  <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
<joint name="j2" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0.3 0 0" rpy="0 1.5707963 0"/>
  <axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="20" velocity="1"/></joint>
<joint name="tipj" type="fixed"><parent link="b"/><child link="tip"/><origin xyz="0 0 0.05" rpy="0 0 0"/></joint>
</robot>"""


# 관절각으로 Gazebo 링크(고정 관절로 합쳐진 뒤 남는 링크) 자세를 만든다. world는 임의의 기준 자세다.
def gazebo_link_poses(joints, angles, world, gazebo_links):
    poses = {'base': world} if 'base' in gazebo_links else {}
    frames = {}
    root = next(link for link in gazebo_links if all(child != link for _, _, child, _, _, _ in joints.values()))
    frames[root] = world
    pending = dict(joints)

    while pending:
        progressed = False

        for name, (kind, parent, child, origin, axis, _) in list(pending.items()):
            if parent not in frames:
                continue

            motion = np.eye(4)

            if kind != 'fixed':
                motion[:3, :3] = observer.axis_rotation(axis, angles.get(name, 0.0))

            frames[child] = frames[parent] @ origin @ motion
            del pending[name]
            progressed = True

        if not progressed:
            raise ValueError('disconnected')

    poses.update({link: frames[link] for link in gazebo_links})

    return poses


# Gazebo에 남는 링크 집합: 뿌리와 고정이 아닌 관절의 자식
def gazebo_links_of(joints):
    children = {child for _, _, child, _, _, _ in joints.values()}
    roots = {parent for _, parent, _, _, _, _ in joints.values()} - children
    movable = {child for kind, _, child, _, _, _ in joints.values() if kind != 'fixed'}

    return roots | movable


class ObserverMath(unittest.TestCase):
    def setUp(self):
        self.world = observer.origin_matrix([1.0, -2.0, 0.4], [0.05, -0.1, 1.2])

    def test_angle_about_axis_keeps_sign(self):
        for angle in (-2.5, -0.5, 0.0, 0.7, 3.0):
            for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0]):
                unit = np.asarray(axis, float) / np.linalg.norm(axis)
                matrix = observer.axis_rotation(unit, angle)
                self.assertAlmostEqual(observer.angle_about_axis(matrix, unit), angle, places=9)

    def test_small_chain_with_lumped_link_and_tilted_origin(self):
        joints = observer.read_urdf_joints(SMALL_URDF)
        links = gazebo_links_of(joints)
        self.assertEqual(links, {'base', 'a', 'b'})

        chain = observer.build_joint_chain(joints, ['j1', 'j2'], links)
        self.assertEqual(chain['j1'][0], 'base')

        for angles in ({'j1': 0.4, 'j2': -1.1}, {'j1': -2.9, 'j2': 2.2}):
            recovered, residual = observer.reconstruct_joints(chain, gazebo_link_poses(joints, angles, self.world, links))

            for name, value in angles.items():
                self.assertAlmostEqual(recovered[name], value, places=7)
                self.assertLess(residual[name], 1e-7)

    def test_tcp_is_found_through_fixed_joints(self):
        joints = observer.read_urdf_joints(SMALL_URDF)
        root, transform = observer.lumped_frame('tip', joints, {'base', 'a', 'b'})

        self.assertEqual(root, 'b')
        np.testing.assert_allclose(transform[:3, 3], [0, 0, 0.05])

        with self.assertRaises(ValueError):
            observer.lumped_frame('tip', joints, {'a'})

    def test_ax_arm_chain_from_generated_urdf(self):
        joints = observer.read_urdf_joints((ROOT / 'src/ax_description/urdf/ax_1.urdf').read_text())
        links = gazebo_links_of(joints)
        chain = observer.build_joint_chain(joints, ARM_JOINTS, links)

        # 팔 링크는 Gazebo에서 각각 남고 TCP는 공구 링크에 합쳐진다.
        self.assertEqual(
            [chain[joint][2] for joint in ARM_JOINTS],
            ['column_link', 'upper_arm_link', 'forearm_link', 'wrist_link', 'gripper_base_link'],
        )
        self.assertEqual(observer.lumped_frame('epick_tcp', joints, links)[0], 'gripper_base_link')

        for target in ([0.35, -0.3, 0.35, -0.2, 0.25], [0.0, -1.35, 1.9, -0.25, 0.0], [-3.0, 1.4, -1.9, 1.5, 3.0]):
            angles = dict(zip(ARM_JOINTS, target))
            recovered, residual = observer.reconstruct_joints(chain, gazebo_link_poses(joints, angles, self.world, links))

            for joint in ARM_JOINTS:
                self.assertAlmostEqual(recovered[joint], angles[joint], places=7)
                self.assertLess(residual[joint], 1e-7)

    def test_effort_limits_are_read(self):
        joints = observer.read_urdf_joints(SMALL_URDF)
        self.assertEqual(joints['j2'][5], 20.0)
        self.assertIsNone(joints['fix'][5])

    def test_first_stable_below_needs_continuous_hold(self):
        samples = [(0.0, 1.0), (0.1, 0.0), (0.2, 0.5), (0.3, 0.0), (0.6, 0.0), (0.9, 0.0)]
        self.assertAlmostEqual(observer.first_stable_below(samples, 0.01, 0.5), 0.3)
        self.assertIsNone(observer.first_stable_below(samples[:5], 0.01, 0.5))
        self.assertIsNone(observer.first_stable_below([], 0.01, 0.5))

    def test_joint_speed_series_matches_constant_rate(self):
        history = [(index * 0.01, {'a': 0.2 * index * 0.01, 'b': 0.0}) for index in range(100)]
        series = observer.joint_speed_series(history, 0.1)

        self.assertTrue(series)
        for _, speed in series:
            self.assertAlmostEqual(speed, 0.2, places=6)

    def test_joint_speed_series_wraps_angle(self):
        history = [(0.0, {'a': math.pi - 0.01}), (0.1, {'a': -math.pi + 0.01})]
        self.assertAlmostEqual(observer.joint_speed_series(history, 0.1)[-1][1], 0.2, places=6)

    def test_nearest_pairs_respects_gap(self):
        first = [(0.0, 'a'), (0.11, 'b'), (0.5, 'c')]
        second = [(0.005, 1), (0.1, 2), (0.2, 3)]
        self.assertEqual(observer.nearest_pairs(first, second, 0.02), [('a', 1), ('b', 2)])
        self.assertEqual(observer.nearest_pairs(first, [], 0.02), [])

    def test_object_in_tcp_frame(self):
        tcp = observer.origin_matrix([1, 2, 3], [0, 0, math.pi / 2])
        box = tcp @ observer.origin_matrix([0, 0, 0.085], [0, 0, 0])
        np.testing.assert_allclose(observer.in_tcp_frame(tcp, box)[:3, 3], [0, 0, 0.085], atol=1e-12)


    def test_yaw_and_unwrap(self):
        pose = observer.origin_matrix([0, 0, 0], [0, 0, 2.5])
        self.assertAlmostEqual(observer.yaw_of(pose), 2.5)
        self.assertAlmostEqual(observer.quaternion_yaw(0, 0, math.sin(0.4), math.cos(0.4)), 0.8)

        # 한 바퀴 넘게 돈 각도 열을 풀면 누적 회전이 나온다.
        wrapped = [math.atan2(math.sin(0.3 * index), math.cos(0.3 * index)) for index in range(30)]
        self.assertAlmostEqual(observer.unwrap_angles(wrapped)[-1], 0.3 * 29, places=9)

    def test_summarize_ignores_missing(self):
        stats = observer.summarize([1.0, 2.0, None, 3.0, math.nan])
        self.assertEqual(stats['count'], 3)
        self.assertAlmostEqual(stats['mean'], 2.0)
        self.assertAlmostEqual(stats['std'], 1.0)
        self.assertIsNone(observer.summarize([None]))


# 실행 환경 불일치 안내: 발견한 로봇·월드에 따라 원인 추정이 달라야 한다.
class EnvironmentHint(unittest.TestCase):
    environment = {'ROS_DOMAIN_ID': '77', 'GZ_PARTITION': 'ax-a2'}

    def test_ids_and_worlds_from_topics(self):
        topics = ['/ax/robot_2/odom', '/ax/robot_1/joint_states', '/ax', '/clock', '/ax/robot_2/tf']
        self.assertEqual(observer.robot_ids_from_topics(topics), ['robot_1', 'robot_2'])
        self.assertEqual(observer.worlds_from_topics(['/world/ax_test/clock', '/stats', '/world/']), ['ax_test'])

    def test_publisher_needed_for_world(self):
        published = 'Publishers [Address, Message Type]:\n  tcp://172.21.0.1:41939, gz.msgs.Clock\nNo subscribers on topic [/world/ax_test/clock]\n'
        subscribed = 'No publishers on topic [/world/ax_test/clock]\nSubscribers [Address, Message Type]:\n  tcp://172.21.0.1:5000, gz.msgs.Clock\n'
        self.assertTrue(observer.info_has_publisher(published))
        self.assertFalse(observer.info_has_publisher(subscribed))
        self.assertFalse(observer.info_has_publisher(''))

    def test_settings_are_shown(self):
        hint = observer.environment_hint('robot_1', self.environment, ['robot_1'], ['ax_test'])
        self.assertIn('ROS_DOMAIN_ID=77', hint)
        self.assertIn('GZ_PARTITION=ax-a2', hint)
        self.assertIn('ROS_AUTOMATIC_DISCOVERY_RANGE=(미설정)', hint)

    def test_cause_follows_what_is_visible(self):
        cases = [
            ([], [], 'ROS_DOMAIN_ID·GZ_PARTITION'),
            ([], ['ax_test'], 'ROS 노드가 보이지 않음'),
            (['robot_2'], ['ax_test'], '--robot-id를 robot_2'),
            (['robot_1'], [], 'GZ_PARTITION을 시뮬레이션 터미널과 맞출 것'),
            (['robot_1'], None, '대상은 보임'),
            (['robot_1'], ['ax_test'], '대상은 보임'),
        ]

        for robots, worlds, expected in cases:
            self.assertIn(expected, observer.environment_hint('robot_1', self.environment, robots, worlds))


# 바퀴 보정 도구의 순수 계산(누적 회전·부호 있는 이동·권장 multiplier)
calibration_spec = importlib.util.spec_from_file_location(
    'check_wheel_calibration', ROOT / 'src/ax_control/scripts/check_wheel_calibration.py')


class WheelCalibrationMath(unittest.TestCase):
    def setUp(self):
        import sys

        sys.path.insert(0, str(ROOT / 'src/ax_control/scripts'))

        try:
            self.calibration = importlib.util.module_from_spec(calibration_spec)
            calibration_spec.loader.exec_module(self.calibration)
        except ImportError as error:
            self.skipTest(f'ROS 환경 필요: {error}')

    def test_accumulated_rotation_over_pi(self):
        samples = [(index * 0.1, math.atan2(math.sin(0.5 * index * 0.1), math.cos(0.5 * index * 0.1)))
                   for index in range(81)]
        self.assertAlmostEqual(self.calibration.accumulated_rotation(samples), 4.0, places=9)
        self.assertIsNone(self.calibration.accumulated_rotation(samples[:1]))

    def test_signed_distance_follows_heading(self):
        self.assertAlmostEqual(self.calibration.signed_distance((0, 0), (1, 1), math.pi / 4), math.sqrt(2))
        self.assertAlmostEqual(self.calibration.signed_distance((0, 0), (-1, 0), 0.0), -1.0)

    def test_rotation_about_front_axle_shows_translation(self):
        # 차체 앞 0.334 m 점을 중심으로 π 회전: 실제 병진 0.668 m, odom은 제자리(0)로 본다.
        centre = 0.334
        truth = [(index * 0.1, centre - centre * math.cos(a), -centre * math.sin(a), a)
                 for index, a in enumerate([math.pi * k / 30 for k in range(31)])]
        odom = [(item[0], 0.0, 0.0, item[3]) for item in truth]
        result = self.calibration.rotation_translation(truth, odom)

        self.assertAlmostEqual(result['truth_translation_m'], 2 * centre, places=6)
        self.assertAlmostEqual(result['odom_translation_m'], 0.0)
        self.assertAlmostEqual(result['translation_error_m'], 2 * centre, places=6)
        np.testing.assert_allclose(result['rotation_centre_body_m'], [centre, 0.0], atol=1e-6)

    def test_centred_rotation_has_no_translation(self):
        truth = [(index * 0.1, 1.0, 2.0, 0.1 * index) for index in range(30)]
        result = self.calibration.rotation_translation(truth, truth)

        self.assertAlmostEqual(result['translation_error_m'], 0.0)
        np.testing.assert_allclose(result['rotation_centre_body_m'], [0.0, 0.0], atol=1e-9)   # 차체 중심 회전
        self.assertEqual(result['rotation_radius_m'], 0.0)

    def test_recommended_multiplier(self):
        # odom이 실제보다 10 % 크게 돌면 multiplier를 10 % 키운다.
        self.assertAlmostEqual(self.calibration.recommended_separation_multiplier(1.2, [1.1, 1.1, None]), 1.32)
        self.assertIsNone(self.calibration.recommended_separation_multiplier(1.2, [None]))


if __name__ == '__main__':
    unittest.main()
