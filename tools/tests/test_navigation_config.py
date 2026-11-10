"""D1 boundary checks: robot isolation, full shelf footprint and no gate bypass."""
import importlib.util
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('navigation', ROOT/'src/ax_navigation/launch/navigation.launch.py')
navigation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(navigation)


class NavigationConfig(unittest.TestCase):
    def test_role_footprints_and_sensor_isolation(self):
        mobility = yaml.safe_load((ROOT/'src/ax_description/config/a1_validation.yaml').read_text())['mobility']
        for robot in ('robot_1', 'robot_2', 'robot_3', 'robot_4'):
            namespace = '/ax/'+robot
            cfg = navigation.parameters(robot, mobility, '/fixture/navigate.xml')[namespace]
            for which in ('local_costmap', 'global_costmap'):
                costmap = cfg[which][which]['ros__parameters']
                self.assertEqual(costmap['robot_base_frame'], robot+'/base_link')
                expected = mobility['robot_1' if robot in ('robot_1','robot_3') else 'robot_2']['footprint_xy_m']
                self.assertEqual(yaml.safe_load(costmap['footprint']), expected)
                for source in ('front', 'rear'):
                    self.assertEqual(costmap['obstacle_layer'][source]['topic'],
                        namespace+'/sensors/'+source+'_lidar/scan_filtered')
            controller = cfg['controller_server']['ros__parameters']
            self.assertTrue(controller['enable_stamped_cmd_vel'])
            self.assertEqual(controller['odom_topic'], namespace+'/diff_drive_controller/odom')
            self.assertEqual(cfg['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic'], '/map')

    def test_invalid_robot_rejected(self):
        with self.assertRaises(ValueError):
            navigation.parameters('robot_5', {}, '')

    def test_no_recovery_motion_in_w2_tree(self):
        import xml.etree.ElementTree as ET
        tree = ET.parse(ROOT/'src/ax_navigation/config/navigate.xml')
        for name in ('Spin', 'BackUp', 'DriveOnHeading', 'RecoveryNode', 'ClearEntireCostmap'):
            self.assertFalse(tree.findall('.//'+name))


if __name__ == '__main__':
    unittest.main()
