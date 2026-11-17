"""Launch-level regression checks for the W2 A4/B4 time contract."""
import importlib.util
from pathlib import Path
import unittest

from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[2]


class SafetyLaunchConfigTest(unittest.TestCase):
    def test_b4_explicitly_uses_ros_sim_time(self):
        path = ROOT / 'src' / 'ax_safety' / 'launch' / 'safety.launch.py'
        spec = importlib.util.spec_from_file_location('safety_launch_under_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        b4_nodes = [entity for entity in description.entities
                    if isinstance(entity, Node) and entity.node_executable == 'b4_risk_node']
        self.assertEqual(len(b4_nodes), 1)
        # launch_ros의 정규화된 파라미터 매핑은 내부 속성으로 보관된다.
        # 소스 문자열 검색이 아니라 실제로 생성된 launch action을 검증한다.
        parameters = b4_nodes[0]._Node__parameters
        self.assertTrue(any(
            isinstance(item, dict) and any(
                getattr(key[0], 'text', None) == 'use_sim_time' and value is True
                for key, value in item.items())
            for item in parameters))


if __name__ == '__main__':
    unittest.main()
