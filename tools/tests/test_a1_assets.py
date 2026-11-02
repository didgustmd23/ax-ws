import hashlib
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
