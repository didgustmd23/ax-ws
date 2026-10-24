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


if __name__ == '__main__':
    unittest.main()
