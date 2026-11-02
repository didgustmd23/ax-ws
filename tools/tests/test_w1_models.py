"""W1 checks: physical validity, resource references and sensor interoperability."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / 'src/ax_description'


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def model(file, robot_id='robot_1'):
    return ET.fromstring(xacro.process_file(str(PKG / 'urdf' / file), mappings={
        'sensor_robot_id': robot_id, 'frame_prefix': robot_id + '/',
        'gripper_model': 'suction'}).toxml())


class Models(unittest.TestCase):
    def test_wheel_quadrants_axes_and_controller_sides(self):
        from scipy.spatial.transform import Rotation
        launch = module(PKG / 'launch/robot_test.launch.py', 'robot_test_launch')
        for filename, manipulator in [('ax.urdf.xacro', True), ('ax_2.urdf.xacro', False)]:
            robot = model(filename)
            drive = launch.controller_config('robot_1', manipulator,
                {'radius': .2, 'separation': .6})['/ax/robot_1/diff_drive_controller']['ros__parameters']
            for fore_aft in ('front', 'rear'):
                for side in ('left', 'right'):
                    joint_name = f'{fore_aft}_{side}_wheel_joint'
                    joint = robot.find(f"joint[@name='{joint_name}']")
                    position = np.fromstring(joint.find('origin').get('xyz'), sep=' ')
                    rotation = Rotation.from_euler('xyz', np.fromstring(joint.find('origin').get('rpy', '0 0 0'), sep=' '))
                    axis = rotation.apply(np.fromstring(joint.find('axis').get('xyz'), sep=' '))
                    with self.subTest(model=filename, wheel=joint_name):
                        self.assertGreater(position[0] * (1 if fore_aft == 'front' else -1), 0)
                        self.assertGreater(position[1] * (1 if side == 'left' else -1), 0)
                        np.testing.assert_allclose(axis, [0, 1, 0], atol=1e-8)
                        self.assertIn(joint_name, drive[side + '_wheel_names'])
                        self.assertNotIn(joint_name, drive[('right' if side == 'left' else 'left') + '_wheel_names'])

    def test_drive_world_is_flat_and_obstacle_free(self):
        world = ET.parse(PKG / 'worlds/drive_test.world').getroot().find('world')
        self.assertEqual([element.get('name') for element in world.findall('model')], ['ground_plane'])
        self.assertEqual(world.findtext('model/static'), 'true')
        self.assertEqual(world.findtext('gravity'), '0 0 -9.80665')

    def test_tree_meshes_inertias_and_limits(self):
        for filename in ('ax.urdf.xacro', 'ax_2.urdf.xacro'):
            with self.subTest(model=filename):
                root = model(filename)
                links = {e.get('name'): e for e in root.findall('link')}
                self.assertEqual(len(links), len(root.findall('link')))
                children = []
                graph = {}
                for joint in root.findall('joint'):
                    parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                    self.assertIn(parent, links)
                    self.assertIn(child, links)
                    children.append(child)
                    graph.setdefault(parent, []).append(child)
                    if joint.get('type') in ('revolute', 'prismatic'):
                        limit = joint.find('limit')
                        self.assertLess(float(limit.get('lower')), float(limit.get('upper')))
                        self.assertGreater(float(limit.get('velocity')), 0)
                self.assertEqual(len(children), len(set(children)))
                self.assertEqual(set(links) - set(children), {'base_link'})
                visited, todo = set(), ['base_link']
                while todo:
                    link = todo.pop()
                    self.assertNotIn(link, visited)
                    visited.add(link)
                    todo.extend(graph.get(link, []))
                self.assertEqual(visited, set(links))
                for mesh in root.findall('.//mesh'):
                    package, relative = mesh.get('filename').removeprefix('package://').split('/', 1)
                    self.assertTrue((Path(get_package_share_directory(package)) / relative).is_file())
                for element in root.findall('.//inertial'):
                    self.assertGreater(float(element.find('mass').get('value')), 0)
                    i = element.find('inertia').attrib
                    tensor = np.array([[float(i['ixx']), float(i['ixy']), float(i['ixz'])],
                                       [float(i['ixy']), float(i['iyy']), float(i['iyz'])],
                                       [float(i['ixz']), float(i['iyz']), float(i['izz'])]])
                    values = np.linalg.eigvalsh(tensor)
                    self.assertTrue(np.all(values > 0))
                    self.assertLessEqual(values[-1], sum(values[:2]) + 1e-9)

    def test_sensor_frames_and_shared_world_plugins(self):
        for filename, robot_id, sensor_count in [('ax.urdf.xacro', 'robot_1', 8),
                                                 ('ax_2.urdf.xacro', 'robot_2', 6)]:
            root = model(filename, robot_id)
            self.assertEqual(len(root.findall('.//sensor')), sensor_count)
            for sensor in root.findall('.//sensor'):
                self.assertTrue(sensor.findtext('topic').startswith('/ax/' + robot_id + '/sensors/'))
                self.assertTrue(sensor.findtext('gz_frame_id').startswith(robot_id + '/'))
            # No model-local shared systems or alternative wheel driver.
            self.assertEqual(root.findall('gazebo/plugin'), [])
        world = ET.parse(PKG / 'worlds/ax_arm_test.world').getroot().find('world')
        names = [element.get('name') for element in world.findall('plugin')]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn('gz::sim::systems::Sensors', names)
        self.assertIn('gz::sim::systems::Imu', names)
        self.assertIn('gz::sim::systems::Contact', names)

    def test_cargo_contact_survives_sdf_conversion(self):
        robot = model('ax_2.urdf.xacro', 'robot_2')
        self.assertIsNotNone(robot.find("link[@name='cargo_link']"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.urdf'
            path.write_bytes(ET.tostring(robot))
            result = subprocess.run(['gz', 'sdf', '-p', str(path)], check=True, capture_output=True, text=True)
            sdf = ET.fromstring(result.stdout)
            contact = sdf.find(".//sensor[@name='cargo_contact']/contact")
            self.assertIsNotNone(contact)
            collision_name = contact.findtext('collision')
            self.assertIsNotNone(sdf.find(f".//collision[@name='{collision_name}']"))

    def test_sensor_topic_isolation_and_controller_joints(self):
        launch = module(PKG / 'launch/robot_test.launch.py', 'w1_launch')
        topics = []
        for number in range(1, 5):
            robot_id = 'robot_' + str(number)
            manipulator = number % 2 == 1
            entries = launch.bridges(robot_id, manipulator)
            topics.extend(entry['ros_topic_name'] for entry in entries)
            root = model('ax.urdf.xacro' if manipulator else 'ax_2.urdf.xacro', robot_id)
            joints = {e.get('name') for e in root.findall('joint')}
            config = launch.controller_config(robot_id, manipulator, {'radius': .2, 'separation': .6})
            drive = config['/ax/' + robot_id + '/diff_drive_controller']['ros__parameters']
            self.assertTrue(set(drive['left_wheel_names'] + drive['right_wheel_names']) <= joints)
            if manipulator:
                self.assertTrue(set(launch.ARM_JOINTS) <= joints)
        self.assertEqual(len(topics), len(set(topics)))

    def test_physics_reports_use_current_inputs(self):
        import hashlib
        for filename in ('physical_properties.json', 'ax_2_physical_properties.json'):
            report = json.loads((PKG / 'config' / filename).read_text())
            for link in report['links'].values():
                mesh = PKG / report['mesh_directory'] / link['mesh']
                self.assertEqual(link['mesh_sha256'], hashlib.sha256(mesh.read_bytes()).hexdigest())
                self.assertTrue(link['all_component_vertices_enclosed'])


class ToFSemantics(unittest.TestCase):
    def test_nonreturns_and_invalid_samples_are_not_contact(self):
        convert = module(PKG / 'scripts/tof_range.py', 'tof_range').checked_range
        self.assertEqual(convert([math.inf], .02, .4), math.inf)
        self.assertEqual(convert([-math.inf], .02, .4), -math.inf)
        self.assertTrue(math.isnan(convert([math.nan], .02, .4)))
        self.assertTrue(math.isnan(convert([], .02, .4)))
        self.assertTrue(math.isnan(convert([.1, .2], .02, .4)))
        self.assertEqual(convert([0], .02, .4), -math.inf)
        self.assertEqual(convert([.5], .02, .4), math.inf)
        self.assertAlmostEqual(convert([.031], .02, .4), .031)


class ProcessOwnership(unittest.TestCase):
    def test_cleanup_does_not_stop_another_partition(self):
        import os
        import sys
        import uuid
        processes = module(ROOT / 'tools/w1_processes.py', 'w1_processes')
        tag = 'ax-w1-test-' + uuid.uuid4().hex
        owned = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                  env=dict(os.environ, GZ_PARTITION=tag))
        other = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                  env=dict(os.environ, GZ_PARTITION='unrelated-test-partition'))
        try:
            result = processes.stop_tagged(tag)
            self.assertEqual(result['remaining_count'], 0)
            self.assertEqual(result['signalled_count'], 1)
            owned.wait(timeout=5)
            self.assertIsNone(other.poll())
        finally:
            for proc in (owned, other):
                if proc.poll() is None:
                    proc.terminate()
                proc.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
