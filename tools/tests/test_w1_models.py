# 모델 트리·관성·바퀴 위치·토픽 격리·통합 월드·프로세스 소유권을 검사한다.
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
# 제어기 설정은 A2 ax_control/config가 소유한다. 공통 파일 다음에 모델별 파일을 읽는다.
CONTROL_CONFIG = ROOT / 'src/ax_control/config'
CONTROLLER_FILES = {'ax.urdf.xacro': 'ax1_controllers.yaml', 'ax_2.urdf.xacro': 'ax2_controllers.yaml'}


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


# 공통·모델별 YAML을 노드별 ros__parameters로 합친다(gz_ros2_control의 파일 순서와 같음).
def controller_parameters(model_filename):
    import yaml
    merged = {}
    for filename in ('common_controllers.yaml', CONTROLLER_FILES[model_filename]):
        for node, content in yaml.safe_load((CONTROL_CONFIG / filename).read_text()).items():
            merged.setdefault(node.removeprefix('/**/'), {}).update(content['ros__parameters'])
    return merged


def model(file, robot_id='robot_1'):
    return ET.fromstring(xacro.process_file(str(PKG / 'urdf' / file), mappings={
        'sensor_robot_id': robot_id, 'frame_prefix': robot_id + '/',
        'gripper_model': 'suction'}).toxml())


class Models(unittest.TestCase):
    def test_wheel_quadrants_axes_and_controller_sides(self):
        from scipy.spatial.transform import Rotation
        for filename, manipulator in [('ax.urdf.xacro', True), ('ax_2.urdf.xacro', False)]:
            robot = model(filename)
            drive = controller_parameters(filename)['diff_drive_controller']
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

    def test_only_current_warehouse_is_selectable_and_empty_fixture_is_generated(self):
        self.assertEqual({p.relative_to(PKG / 'worlds').as_posix() for p in (PKG / 'worlds').rglob('*.world')}, {'planned/ax_warehouse.world'})
        world = ET.parse(PKG / 'worlds/planned/ax_warehouse.world').getroot().find('world')
        self.assertEqual(world.get('name'), 'ax_test')
        self.assertEqual(sum(m.get('name').startswith('floor_transfer_corridor') for m in world.findall('model')), 3)
        world = module(ROOT/'tools/empty_test_fixture.py', 'empty_fixture').empty_world().find('world')
        self.assertEqual({m.get('name') for m in world.findall('model')},
                         {'ground_plane'})
        self.assertEqual(world.findtext('gravity'), '0 0 -9.80665')
        ground = world.find("model[@name='ground_plane']")
        self.assertEqual(ground.findtext('static'), 'true')
        self.assertEqual(ground.findtext('.//plane/size'), '30 30')

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
        world = ET.parse(PKG / 'worlds/planned/ax_warehouse.world').getroot().find('world')
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
            filename = 'ax.urdf.xacro' if manipulator else 'ax_2.urdf.xacro'
            root = model(filename, robot_id)
            joints = {e.get('name') for e in root.findall('joint')}
            config = controller_parameters(filename)
            drive = config['diff_drive_controller']
            self.assertTrue(set(drive['left_wheel_names'] + drive['right_wheel_names']) <= joints)
            self.assertEqual('arm_controller' in config, manipulator)
            if manipulator:
                self.assertTrue(set(config['arm_controller']['joints']) <= joints)
        self.assertEqual(len(topics), len(set(topics)))

    def test_controller_wheel_geometry_matches_models(self):
        # 바퀴 기하: robot_1은 모델 관절 원점, robot_2는 ax_2_geometry.xacro 속성과 같아야 한다.
        geometry = {node.get('name'): float(node.get('value'))
                    for node in ET.parse(PKG / 'urdf/ax_2_geometry.xacro').getroot()
                    if node.get('name', '').startswith('ax2_wheel_')}
        drive_2 = controller_parameters('ax_2.urdf.xacro')['diff_drive_controller']
        self.assertAlmostEqual(drive_2['wheel_radius'], geometry['ax2_wheel_radius'], places=9)
        self.assertAlmostEqual(drive_2['wheel_separation'], geometry['ax2_wheel_separation'], places=6)
        robot = model('ax.urdf.xacro')
        left = np.fromstring(robot.find("joint[@name='front_left_wheel_joint']/origin").get('xyz'), sep=' ')
        right = np.fromstring(robot.find("joint[@name='front_right_wheel_joint']/origin").get('xyz'), sep=' ')
        drive_1 = controller_parameters('ax.urdf.xacro')['diff_drive_controller']
        self.assertAlmostEqual(drive_1['wheel_separation'], left[1] - right[1], places=3)

    def test_ros2_control_plugin_reads_common_then_model_config(self):
        for filename in CONTROLLER_FILES:
            root = ET.fromstring(xacro.process_file(str(PKG / 'urdf' / filename), mappings={
                'ros_namespace': '/ax/robot_1',
                'ros2_control_common_params': '/cfg/common_controllers.yaml',
                'ros2_control_params': '/cfg/' + CONTROLLER_FILES[filename]}).toxml())
            plugin = root.find(".//plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
            with self.subTest(model=filename):
                self.assertEqual([p.text for p in plugin.findall('parameters')],
                                 ['/cfg/common_controllers.yaml', '/cfg/' + CONTROLLER_FILES[filename]])
            # 설정이 없으면 플러그인을 만들지 않는다(표시·정적 검사용).
            bare = ET.fromstring(xacro.process_file(str(PKG / 'urdf' / filename)).toxml())
            self.assertIsNone(bare.find(".//plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"))

    def test_tool_attach_offset_matches_epick_tcp_chain(self):
        # 흡착 플러그인의 tcp_offset은 gripper_base_link → epick_tcp 고정 관절 변환과 같아야 한다.
        from scipy.spatial.transform import Rotation
        for tool in ('epick', 'suction'):
            root = ET.fromstring(xacro.process_file(str(PKG / 'urdf/ax.urdf.xacro'), mappings={
                'gripper_model': tool, 'tool_attach': 'true', 'ros_namespace': '/ax/robot_1'}).toxml())
            parents = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
            link, transform = 'epick_tcp', np.eye(4)
            while link != 'gripper_base_link':
                joint = parents[link]
                self.assertEqual(joint.get('type'), 'fixed')
                origin = joint.find('origin')
                step = np.eye(4)
                if origin is not None:
                    step[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                    step[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                transform = step @ transform
                link = joint.find('parent').get('link')
            plugin = root.find(".//plugin[@name='ax_control::SuctionAttachSystem']")
            with self.subTest(tool=tool):
                self.assertIsNotNone(plugin)
                self.assertEqual(plugin.findtext('parent_link'), 'gripper_base_link')
                np.testing.assert_allclose(np.fromstring(plugin.findtext('tcp_offset'), sep=' '),
                                           transform[:3, 3], atol=1e-4)
                self.assertEqual(plugin.findtext('command_topic'), '/ax/robot_1/tool/gz_command')
        # 기본값(tool_attach false)이나 흡착 외 공구에서는 플러그인이 없다.
        for mappings in ({'gripper_model': 'epick'}, {'gripper_model': 'primitive', 'tool_attach': 'true'}):
            root = ET.fromstring(xacro.process_file(str(PKG / 'urdf/ax.urdf.xacro'), mappings=mappings).toxml())
            self.assertIsNone(root.find(".//plugin[@name='ax_control::SuctionAttachSystem']"))

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
