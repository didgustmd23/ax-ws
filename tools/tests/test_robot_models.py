"""Integration checks for the two robot models and their sensor contracts."""
from collections import Counter
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / 'src/ax_description'


def expand(filename, *arguments):
    result = subprocess.run(['xacro', str(DESCRIPTION / 'urdf' / filename), *arguments],
                            text=True, capture_output=True)
    if result.returncode:
        raise AssertionError(result.stderr)
    return ET.fromstring(result.stdout)


def transform(origin):
    result = np.eye(4)
    if origin is not None:
        result[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
        result[:3, :3] = Rotation.from_euler(
            'xyz', list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
    return result


def zero_transforms(root):
    result = {'base_link': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent in result:
                result[joint.find('child').get('link')] = (
                    result[parent] @ transform(joint.find('origin')))
                pending.remove(joint)
                progress = True
        assert progress, 'Disconnected robot graph'
    return result


class RobotModelTests(unittest.TestCase):
    def test_gazebo_exported_paths_resolve_converted_meshes(self):
        # ros_gz_sim이 실제로 읽는 package.xml export로 탐색 경로를 만든다.
        # package:// 파일 존재만 검사하면 model:// 변환 후의 누락을 놓친다.
        launcher = (
            Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py'
        )
        spec = importlib.util.spec_from_file_location('gz_sim_launch', launcher)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        exported, _ = module.GazeboRosPaths.get_paths()
        paths = [Path(p) for p in exported.split(os.pathsep) if p]

        variants = [
            ('ax.urdf.xacro', [f'gripper_model:={tool}'])
            for tool in ('primitive', 'robotiq', 'parallel_wide', 'suction')
        ] + [('ax_2.urdf.xacro', [])]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'robot.urdf'
            for filename, arguments in variants:
                with self.subTest(model=filename, arguments=arguments):
                    ET.ElementTree(expand(filename, *arguments)).write(path)
                    conversion = subprocess.run(
                        ['gz', 'sdf', '-p', str(path)],
                        text=True, capture_output=True, check=True,
                    )
                    sdf = ET.fromstring(conversion.stdout)
                    uris = sdf.findall('.//mesh/uri')
                    self.assertTrue(uris)
                    for element in uris:
                        uri = element.text
                        self.assertTrue(uri.startswith('model://'), uri)
                        relative = uri.removeprefix('model://')
                        self.assertTrue(
                            any((folder / relative).is_file() for folder in paths),
                            f'Gazebo exported resource paths cannot resolve {uri}',
                        )

    def test_ax2_keeps_all_eight_meshes_and_original_assembly(self):
        self.assertTrue((DESCRIPTION / 'urdf/ax_2.urdf.xacro').exists())
        robot = expand('ax_2.urdf.xacro')
        transforms = zero_transforms(robot)
        properties = json.loads((DESCRIPTION / 'config/ax_2_physical_properties.json').read_text())
        base = np.array(properties['links']['base_link']['pivot_m'])
        meshes = []
        for link in robot.findall('link'):
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                path = DESCRIPTION / mesh.get('filename').split('ax_description/', 1)[1]
                self.assertTrue(path.is_file(), path)
                vertices = np.array([list(map(float, line.split()[1:4]))
                                     for line in path.read_text().splitlines()
                                     if line.startswith('v ')]) * .001
                matrix = transforms[link.get('name')] @ transform(visual.find('origin'))
                actual = vertices @ matrix[:3, :3].T + matrix[:3, 3]
                np.testing.assert_allclose(actual, vertices - base, atol=1e-10)
                meshes.append(path.name)
        self.assertEqual(len(meshes), 8)
        self.assertEqual(len(set(meshes)), 8)
        self.assertFalse(any(j.get('name') == 'shoulder_joint' for j in robot.findall('joint')))

    def test_sensor_counts_frames_and_robot_topic_separation(self):
        all_topics = []
        for filename, robot_id, expected in (
                ('ax.urdf.xacro', 'robot_1',
                 {'rgbd_camera': 2, 'camera': 2, 'gpu_lidar': 2, 'imu': 1}),
                ('ax_2.urdf.xacro', 'robot_2',
                 {'rgbd_camera': 1, 'camera': 1, 'gpu_lidar': 2, 'imu': 1, 'contact': 1})):
            with self.subTest(robot=robot_id):
                robot = expand(filename, f'frame_prefix:={robot_id}/')
                sensors = robot.findall('gazebo/sensor')
                self.assertEqual(Counter(s.get('type') for s in sensors), expected)
                links = {f'{robot_id}/' + link.get('name') for link in robot.findall('link')}
                for sensor in sensors:
                    topic = sensor.findtext('topic')
                    self.assertTrue(topic.startswith(f'/ax/{robot_id}/sensors/'), topic)
                    all_topics.append(topic)
                    self.assertIn(sensor.findtext('gz_frame_id'), links)
                    if sensor.get('type') in ('camera', 'rgbd_camera'):
                        self.assertIn(sensor.findtext('camera/optical_frame_id'), links)
                    if sensor.get('type') == 'gpu_lidar':
                        self.assertAlmostEqual(float(sensor.findtext('lidar/scan/horizontal/min_angle')),
                                               -np.pi / 2)
                        self.assertAlmostEqual(float(sensor.findtext('lidar/scan/horizontal/max_angle')),
                                               np.pi / 2)
                transforms = zero_transforms(robot)
                np.testing.assert_allclose(transforms['rear_lidar_link'][:3, :3] @ [1, 0, 0],
                                           [-1, 0, 0], atol=1e-10)
                for joint in robot.findall('joint'):
                    if joint.get('name').endswith('_optical_joint'):
                        np.testing.assert_allclose(transform(joint.find('origin'))[:3, :3] @ [0, 0, 1],
                                                   [1, 0, 0], atol=1e-10)
        self.assertEqual(len(all_topics), len(set(all_topics)))

    def test_arm_camera_parents_and_front_sensor_row(self):
        # 팔 카메라 역할 분리: RGB-D는 손목(공구 근처), RGB는 상완에 장착한다.
        ax1 = expand('ax.urdf.xacro')
        parents = {joint.find('child').get('link'): joint.find('parent').get('link')
                   for joint in ax1.findall('joint')}
        self.assertEqual(parents['arm_rgbd_link'], 'wrist_link')
        self.assertEqual(parents['arm_rgb_link'], 'upper_arm_link')
        # q=0에서 손목 RGB-D 광축은 아래쪽 공구 방향을 향한다(흡착판이 없는 공구의 대체 장착).
        self.assertLess(zero_transforms(ax1)['arm_rgbd_link'][2, 0], -0.5)

        # 흡착 공구: RGB-D는 손목 바깥쪽(wrist_link), 4컵 중앙에는 단일 광선 ToF가 붙어
        # 말단 회전과 함께 돈다. 다른 공구에는 ToF가 없다.
        suction = expand('ax.urdf.xacro', 'gripper_model:=suction')
        joints = {joint.find('child').get('link'): joint for joint in suction.findall('joint')}
        self.assertEqual(joints['arm_rgbd_link'].find('parent').get('link'), 'wrist_link')
        self.assertEqual(joints['suction_tof_link'].find('parent').get('link'), 'suction_base_link')
        self.assertIsNone(ax1.find("link[@name='suction_tof_link']"))
        chain, link = [], 'suction_tof_link'
        while link in joints:
            chain.append(joints[link].get('name'))
            link = joints[link].find('parent').get('link')
        self.assertIn('gripper_rotation_joint', chain)
        # suction_base_link 기준: 측정 방향(+X)이 컵 방향(+Z)이고 몸체가 컵 끝면보다 돌출되지 않는다.
        mount = transform(joints['suction_tof_link'].find('origin'))
        np.testing.assert_allclose(mount[:3, 0], [0, 0, 1], atol=1e-10)
        size = np.array(list(map(float, suction.find(
            "link[@name='suction_tof_link']/visual/geometry/box").get('size').split())))
        corners = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * size / 2
        lip_z = transform(joints['suction_grasp_frame'].find('origin'))[2, 3]
        self.assertLess((corners @ mount[:3, :3].T + mount[:3, 3])[:, 2].max(), lip_z)
        # 문서화한 lip_offset은 센서 원점에서 컵 끝면까지의 거리와 같고, 범위가 접촉 상태를 포함한다.
        tof = yaml.safe_load((DESCRIPTION / 'config/sensor_mounts.yaml').read_text())['ax1']['suction_tof']
        self.assertAlmostEqual(tof['lip_offset'], lip_z - mount[2, 3])
        self.assertLess(tof['min_range'], tof['lip_offset'])
        sensor = suction.find("gazebo/sensor[@name='suction_tof']")
        self.assertEqual(sensor.get('type'), 'gpu_lidar')
        self.assertEqual(sensor.findtext('lidar/scan/horizontal/samples'), '1')
        self.assertEqual(sensor.findtext('topic'), '/ax/robot_1/sensors/suction_tof/scan')

        for filename, robot in (('ax.urdf.xacro', ax1), ('ax_2.urdf.xacro', expand('ax_2.urdf.xacro'))):
            with self.subTest(model=filename):
                transforms = zero_transforms(robot)
                rgb, lidar, rgbd = (transforms[f'front_{name}_link']
                                    for name in ('rgb', 'lidar', 'rgbd'))
                # 카메라(+Y)–LiDAR(중앙)–카메라(-Y)를 같은 높이에 두고 모두 +X를 향한다.
                for sensor in (rgb, lidar, rgbd):
                    self.assertAlmostEqual(sensor[2, 3], lidar[2, 3])
                    np.testing.assert_allclose(sensor[:3, 0], [1, 0, 0], atol=1e-10)
                self.assertGreater(rgb[1, 3], 0)
                self.assertAlmostEqual(lidar[1, 3], 0)
                self.assertLess(rgbd[1, 3], 0)
                # ±90° 스캔면이 카메라 몸체 앞을 지나도록 LiDAR 원점이 카메라 앞면보다 앞에 있다.
                depth = float(robot.find("link[@name='front_rgb_link']/visual/geometry/box")
                              .get('size').split()[0])
                self.assertGreater(lidar[0, 3], max(rgb[0, 3], rgbd[0, 3]) + depth / 2)

    def test_suction_tof_bridge_only_for_suction_tool(self):
        # ToF 거리값은 RGB-D 영상 토픽과 분리된 1빔 LaserScan 토픽이며 suction 공구에서만 브리지한다.
        from ax_control.model_contracts import sensor_bridges
        default = sensor_bridges('robot_1')
        suction = sensor_bridges('robot_1', tool='suction')
        tof = [b for b in suction if 'suction_tof' in b[0]]
        self.assertEqual(tof, [('/ax/robot_1/sensors/suction_tof/scan',
                                'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')])
        self.assertFalse([b for b in default if 'suction_tof' in b[0]])
        self.assertEqual(len(suction), len(default) + 1)
        # robot_2(AX_2)에는 공구 인자와 무관하게 ToF 브리지가 없다.
        self.assertFalse([b for b in sensor_bridges('robot_2', tool='suction') if 'suction_tof' in b[0]])

    def test_suction_tof_absent_without_suction_tool(self):
        # 흡착판이 없는 공구와 AX_2 모델에는 ToF 링크·센서가 생성되지 않는다.
        variants = [('ax.urdf.xacro', [f'gripper_model:={tool}'])
                    for tool in ('primitive', 'robotiq', 'parallel_wide')] + [('ax_2.urdf.xacro', [])]
        for filename, arguments in variants:
            with self.subTest(model=filename, arguments=arguments):
                robot = expand(filename, *arguments)
                self.assertIsNone(robot.find("link[@name='suction_tof_link']"))
                self.assertIsNone(robot.find("gazebo/sensor[@name='suction_tof']"))

    def test_ax2_contact_references_a_real_unmerged_collision(self):
        robot = expand('ax_2.urdf.xacro')
        contact = robot.find("gazebo/sensor[@type='contact']")
        self.assertIsNotNone(contact)
        link = robot.find("link[@name='cargo_contact_link']")
        self.assertIsNotNone(link.find("collision[@name='cargo_contact_collision']"))
        preserve = robot.find("gazebo[@reference='cargo_contact_joint']/preserveFixedJoint")
        self.assertIsNotNone(preserve)
        self.assertEqual(preserve.text, 'true')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'ax2.urdf'
            ET.ElementTree(robot).write(path)
            result = subprocess.run(['gz', 'sdf', '-p', str(path)],
                                    text=True, capture_output=True, check=True)
            sdf = ET.fromstring(result.stdout)
            sensor = sdf.find("model/link/sensor[@type='contact']")
            # Harmonic's Contact system reads contact/topic, independently of
            # the generic sensor/topic field used by the other sensor types.
            self.assertEqual(sensor.findtext('contact/topic'),
                             '/ax/robot_2/sensors/cargo_contact')
            parent = next(link for link in sdf.findall('model/link') if sensor in list(link))
            collision = sensor.findtext('contact/collision')
            self.assertIsNotNone(parent.find(f"collision[@name='{collision}']"))


    def test_ros2_control_is_the_only_drive_and_state_source(self):
        # 같은 관절을 두 제어 경로가 구동하지 않도록 이전 Gazebo 제어·상태 플러그인이 없어야 한다.
        legacy = ('gz-sim-diff-drive-system', 'gz-sim-joint-position-controller-system',
                  'gz-sim-joint-state-publisher-system')
        wheels = {f'{side}_wheel_joint' for side in
                  ('front_left', 'front_right', 'rear_left', 'rear_right')}
        from ax_control.model_contracts import ARM_JOINTS, FINGER_JOINTS
        variants = [('ax.urdf.xacro', tool) for tool in
                    ('primitive', 'robotiq', 'parallel_wide', 'suction')] + [('ax_2.urdf.xacro', None)]
        for filename, tool in variants:
            with self.subTest(model=filename, tool=tool):
                arguments = [f'gripper_model:={tool}'] if tool else []
                robot = expand(filename, *arguments, 'ros_namespace:=/ax/robot_x',
                               'ros2_control_params:=/tmp/ax_params.yaml')
                plugins = [p.get('filename') for p in robot.findall('gazebo/plugin')]
                self.assertFalse(set(plugins) & set(legacy))
                self.assertEqual(plugins.count('gz_ros2_control-system'), 1)
                plugin = robot.find("gazebo/plugin[@filename='gz_ros2_control-system']")
                self.assertEqual(plugin.findtext('parameters'), '/tmp/ax_params.yaml')
                self.assertEqual(plugin.findtext('ros/namespace'), '/ax/robot_x')

                systems = robot.findall('ros2_control')
                self.assertEqual(len(systems), 1)
                commands = {j.get('name'): [c.get('name') for c in j.findall('command_interface')]
                            for j in systems[0].findall('joint')}
                expected = {name: ['velocity'] for name in wheels}
                if tool:
                    expected.update({name: ['position'] for name in ARM_JOINTS})
                if tool == 'primitive':
                    expected.update({name: ['position'] for name in FINGER_JOINTS})
                self.assertEqual(commands, expected)
                # 모든 명령 관절은 모델에 실제로 존재하는 비고정 관절이다.
                movable = {j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'}
                self.assertLessEqual(set(commands), movable)

                # 설정 파일이 없으면(RViz 표시·정적 검사) Gazebo 플러그인을 만들지 않는다.
                plain = expand(filename, *arguments)
                self.assertIsNone(plain.find("gazebo/plugin[@filename='gz_ros2_control-system']"))

    def test_controller_config_matches_model_and_contract(self):
        # 제어기 설정의 바퀴 치수·관절 순서가 모델과 계약 모듈과 같아야 한다.
        from ax_control.model_contracts import ARM_JOINTS, controllers, FINGER_JOINTS
        config = ROOT / 'src/ax_control/config'
        cases = [('ax1', 'ax.urdf.xacro', 'robot_1'), ('ax2', 'ax_2.urdf.xacro', 'robot_2')]
        for kind, filename, robot_id in cases:
            with self.subTest(robot=robot_id):
                params = yaml.safe_load((config / f'{kind}_controllers.yaml').read_text())
                manager = params['/**/controller_manager']['ros__parameters']
                spawned = controllers(robot_id)
                self.assertLessEqual(set(spawned), set(manager) - {'update_rate'})
                drive = params['/**/diff_drive_controller']['ros__parameters']
                robot = expand(filename)
                wheel = robot.find("link[@name='front_left_wheel']/collision/geometry/cylinder")
                self.assertAlmostEqual(drive['wheel_radius'], float(wheel.get('radius')), places=6)
                left = zero_transforms(robot)['front_left_wheel'][1, 3]
                right = zero_transforms(robot)['front_right_wheel'][1, 3]
                self.assertAlmostEqual(drive['wheel_separation'], left - right, places=4)
                # 정지 동작은 diff_drive_controller의 명령 시간 초과에 의존한다.
                self.assertGreater(drive['cmd_vel_timeout'], 0)
        params = yaml.safe_load((config / 'ax1_controllers.yaml').read_text())
        arm = params['/**/arm_controller']['ros__parameters']
        self.assertEqual(tuple(arm['joints']), ARM_JOINTS)
        self.assertEqual(arm['command_interfaces'], ['position'])
        fingers = params['/**/finger_controller']['ros__parameters']
        self.assertEqual(tuple(fingers['joints']), FINGER_JOINTS)
        self.assertNotIn('finger_controller', controllers('robot_1', tool='suction'))


if __name__ == '__main__':
    unittest.main()
