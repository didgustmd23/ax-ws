# 외부 원본 해시·V2 자산·EPick 프레임·시험용 충돌 선별을 검사한다. 실제 흡착이나 안전 인증 시험이 아니다.
import hashlib
import struct
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from arm_geometry import ArmGeometry, ARM_JOINTS

class A1Assets(unittest.TestCase):
    def test_travel_pose_footprint_and_cargo_deck_contract(self):
        import itertools
        config = yaml.safe_load((ROOT/'src/ax_description/config/a1_validation.yaml').read_text())['mobility']
        model = ArmGeometry()
        target = config['robot_1']['travel_pose_rad']
        self.assertTrue(model.screen_path([0.]*5, target)['allowed'])
        polygon = np.array(config['robot_1']['footprint_xy_m'])
        corners = np.array(list(itertools.product([-1, 1], repeat=3)))
        tolerance = config['robot_1']['joint_tolerance_rad']
        for delta in itertools.product([-tolerance, 0., tolerance], repeat=5):
            frames = model.frames(np.array(target)+delta)
            for name, offset, half in model.shapes:
                pose = frames[name] @ offset
                points = (corners*half) @ pose[:3, :3].T + pose[:3, 3]
                self.assertTrue(np.all(points[:, :2] >= polygon.min(0)))
                self.assertTrue(np.all(points[:, :2] <= polygon.max(0)))
        source = ROOT/'src/ax_description/urdf/ax_2.urdf.xacro'
        robot = ET.fromstring(xacro.process_file(str(source)).toxml())
        deck = robot.find("link[@name='cargo_contact_link']/collision/geometry/box")
        np.testing.assert_allclose(np.fromstring(deck.get('size'), sep=' ')[:2], config['robot_2']['deck_size_m'])
        frame = robot.find("joint[@name='cargo_frame_joint']/origin")
        self.assertAlmostEqual(float(frame.get('xyz').split()[2]), .001)
        bounds = config['robot_2']['cargo_bounds_xy_m']
        size = config['robot_2']['test_box_size_m']
        self.assertLess(size[0], bounds[1]-bounds[0])
        self.assertLess(size[1], bounds[3]-bounds[2])

    def test_wheel_slip_is_present_in_converted_models(self):
        import subprocess
        import tempfile
        for filename in ('ax.urdf.xacro', 'ax_2.urdf.xacro'):
            text = xacro.process_file(str(ROOT/'src/ax_description/urdf'/filename)).toxml()
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder)/'model.urdf'
                path.write_text(text)
                converted = subprocess.run(['gz', 'sdf', '-p', str(path)], capture_output=True, text=True, check=True)
            sdf = ET.fromstring(converted.stdout)
            for name in ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel'):
                wheel = sdf.find(f".//link[@name='{name}']")
                values = wheel.findall('collision/surface/friction/ode/slip1')
                self.assertTrue(values, 'URDF setting must reach Gazebo SDF')
                self.assertTrue(all(float(value.text) == .05 for value in values))

    def test_interchangeable_tool_tcp_geometry(self):
        from arm_geometry import origin
        source = ROOT / 'src/ax_description/urdf/ax.urdf.xacro'
        for tool, distance in (('epick', .15034361), ('robotiq', .15034361)):
            robot = ET.fromstring(xacro.process_file(str(source), mappings={
                'gripper_model': tool, 'sensors_enabled': 'false'}).toxml())
            frames = {'base_link': np.eye(4)}
            pending = list(robot.findall('joint'))
            while pending:
                count = len(pending)
                for joint in pending[:]:
                    parent = joint.find('parent').get('link')
                    if parent in frames:
                        frames[joint.find('child').get('link')] = frames[parent] @ origin(joint.find('origin'))
                        pending.remove(joint)
                self.assertLess(len(pending), count, 'Disconnected model')
            relative = np.linalg.inv(frames['gripper_base_link']) @ frames['tool_tcp']
            np.testing.assert_allclose(relative[:3, 3], [distance, 0, 0], atol=1e-8)
            np.testing.assert_allclose(relative[:3, 2], [1, 0, 0], atol=1e-8)
            np.testing.assert_allclose(frames['tool_tcp'], frames['epick_tcp'], atol=1e-12)
            if tool == 'epick':
                np.testing.assert_allclose(frames['tool_tcp'], frames['epick_tip'], atol=1e-12)
            else:
                # 실제 collision 패드의 안쪽 평면 중앙 높이와 TCP를 비교한다.
                share = Path(get_package_share_directory('robotiq_description'))
                heights = []
                for side, sign in (('left', -1), ('right', 1)):
                    data = (share / f'meshes/collision/2f_85/{side}_finger_tip.stl').read_bytes()
                    vertices = np.array([r[3:12] for r in struct.iter_unpack('<12fH', data[84:])]).reshape(-1, 3)
                    pad = vertices[vertices[:, 0]*sign > .025]
                    center = np.array([0., 0., (pad[:, 2].min()+pad[:, 2].max())/2, 1.])
                    local = np.linalg.inv(frames['robotiq_85_base_link']) @ frames[f'robotiq_85_{side}_finger_tip_link'] @ center
                    heights.append(local[2])
                self.assertAlmostEqual(np.mean(heights), .13034361, places=7)

    def test_photo_head_dimensions_cups_and_tof_clearance(self):
        settings = yaml.safe_load((ROOT/'src/ax_description/config/epick.yaml').read_text())
        shape = settings['head_visual']
        mesh = ROOT/'src/ax_description/meshes/tools/parcel_suction_head.stl'
        data = mesh.read_bytes()
        self.assertEqual(len(data), 84+50*struct.unpack_from('<I', data, 80)[0])
        vertices = np.array([record[3:12] for record in struct.iter_unpack('<12fH', data[84:])]).reshape(-1, 3)
        np.testing.assert_allclose(np.ptp(vertices, axis=0),
            [shape['width_m'], shape['depth_m'], settings['body_length_m']+settings['cup_height_m']], atol=1e-8)
        robot = ArmGeometry().robot
        cup = robot.find("link[@name='epick_suction_cup']")
        self.assertEqual(len(cup.findall('collision')), shape['rows']*shape['columns'])
        tof_x = float(settings['tof_xyz'].split()[0])
        self.assertGreater(tof_x, shape['width_m']/2+.01)
        from scipy.spatial.transform import Rotation
        xyz = np.fromstring(settings['tof_xyz'], sep=' ')
        rotation = Rotation.from_euler('xyz', np.fromstring(settings['tof_rpy'], sep=' ')).as_matrix()
        direction = rotation[:, 0]
        lip = settings['body_length_m']+settings['cup_height_m']
        self.assertAlmostEqual(settings['tof_normal_cosine'], direction[2])
        self.assertAlmostEqual(settings['tof_lip_offset_m'], (lip-xyz[2])/direction[2])
        for gap in (0., .05, .15, .30):
            hit = xyz + direction*((lip+gap-xyz[2])/direction[2])
            self.assertTrue(np.all(np.abs(hit[:2]) < .1), 'ToF must hit centered 20 cm box')
        contact = xyz + direction*settings['tof_lip_offset_m']
        outer_cup_x = (shape['columns']-1)/2*shape['pitch_m']+settings['cup_radius_m']
        self.assertGreater(contact[0], outer_cup_x)
        # 센서 본체를 기울여도 하우징의 최대 외곽보다 바깥에 있어야 한다.
        half = np.abs(rotation) @ np.array([.006, .01, .01])
        self.assertGreater(xyz[0]-half[0], shape['width_m']/2)

    def test_upstream_license_and_hashes(self):
        package = ROOT / 'src/epick_description'
        manifest = json.loads((package/'UPSTREAM_FILES.json').read_text())
        self.assertEqual(manifest['commit'], 'd07391307d2d16cacf88daa168ce02839589397e')
        self.assertEqual(manifest['license'], 'BSD-3-Clause')
        for entry in manifest['files']:
            self.assertEqual(hashlib.sha256((package/entry['vendored_path']).read_bytes()).hexdigest(), entry['sha256'])

    def test_rebased_mesh_topology_and_limits_are_current(self):
        directory = ROOT/'src/ax_description/meshes/versions/AX_1_v2'
        manifest = json.loads((directory/'manifest.json').read_text())
        self.assertEqual(manifest['joint_limits_sha256'], hashlib.sha256((ROOT/'src/ax_description/config/arm_joint_limits.yaml').read_bytes()).hexdigest())
        for entry in manifest['parts']:
            source = ROOT/'src/ax_description/meshes/parts/AX_1'/entry['source']
            generated = directory/entry['file']
            self.assertEqual(hashlib.sha256(generated.read_bytes()).hexdigest(), entry['sha256'])
            self.assertEqual([line for line in source.read_text().splitlines() if line.startswith('f ')],
                             [line for line in generated.read_text().splitlines() if line.startswith('f ')])

    def test_epick_has_real_geometry_and_no_fake_vacuum_joint(self):
        model = ArmGeometry()
        robot = model.robot
        self.assertIsNotNone(robot.find("link[@name='epick_body']/visual/geometry/mesh"))
        self.assertIsNotNone(robot.find("link[@name='epick_tcp']"))
        self.assertIsNotNone(robot.find("link[@name='tool_tof_link']"))
        self.assertIsNone(robot.find("joint[@name='gripper']"))
        self.assertIsNone(robot.find("link[@name='epick_vacuum_state']"))
        for plugin in robot.findall('.//hardware/plugin'):
            self.assertNotIn('epick_driver', plugin.text)
        settings = yaml.safe_load((ROOT/'src/ax_description/config/epick.yaml').read_text())
        frames = model.frames([0]*5)
        distance = np.linalg.norm(frames['epick_tcp'][:3,3]-frames['epick_base_link'][:3,3])
        self.assertAlmostEqual(distance, settings['body_length_m']+settings['cup_height_m']-settings['tcp_stroke_compensation_m'])
        np.testing.assert_allclose(frames['epick_tcp'][:3,3], frames['epick_tip'][:3,3], atol=1e-10)

    def test_collision_screen_rejects_invalid_limits(self):
        model = ArmGeometry()
        target = [0.0]*5
        target[0] = model.limits[ARM_JOINTS[0]][1] + .1
        self.assertFalse(model.screen_path([0]*5, target)['allowed'])
        self.assertEqual(model.collisions([0]*5), [])

if __name__ == '__main__':
    unittest.main()
