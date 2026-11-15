"""지도 교체 뒤 B3와 내 팀 실행 파일이 같은 번호를 쓰고 옛 입력을 차단하는지 검사합니다."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'src/ax_environment'))
import generate_b3_regions as generator
from ax_environment.regions import observe_robot


class RevisionHandoffTests(unittest.TestCase):
    """상대 지도 파일은 읽기만 하며 ROS 노드를 시작하거나 승인 값을 저장하지 않습니다."""

    def metadata(self):
        """현재 채택 지도와 적용 목록을 별도 사본으로 읽습니다."""
        directory = ROOT / 'src/ax_localization/maps/ax_warehouse'
        return [yaml.safe_load((directory / name).read_text(encoding='utf-8'))
                for name in ('map_info.yaml', 'applicable_worlds.yaml')]

    def test_changed_revision_or_transform_requires_revalidation(self):
        """미래 지도 변경을 번호만 자동 추종해 검증된 좌표로 오해하지 않습니다."""
        for key in ('map_revision', 'transform', 'applicable_revision'):
            with self.subTest(key=key):
                info, applicable = deepcopy(self.metadata())
                if key == 'map_revision':
                    info['map_revision'] += 1
                elif key == 'applicable_revision':
                    applicable['map_revision'] += 1
                else:
                    info['T_world_map']['initial']['x_m'] += 0.2
                with patch.object(generator.yaml, 'safe_load', side_effect=[info, applicable]):
                    with self.assertRaises(ValueError):
                        generator.build()

    def test_default_launch_uses_current_catalog_revision_for_both_robots(self):
        """기본 실행 인자가 실제 두 B3 노드까지 전달되는지 검사합니다."""
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        path = ROOT / 'src/ax_task_executor/launch/team_handoff.launch.py'
        spec = importlib.util.spec_from_file_location('revision2_team_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        with patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs):
            nodes = module.setup_nodes(context)
        catalog = generator.build()
        observers = [node for node in nodes if node['package'] == 'ax_environment']
        self.assertEqual(len(observers), 2)
        self.assertTrue(all(node['parameters'][0]['map_revision'] == catalog['map_revision']
                            for node in observers))
        self.assertFalse(catalog['approved'])

    def test_old_revision_stays_unknown_even_with_valid_pose(self):
        """같은 좌표라도 교체 전 지도 번호로 들어온 관측은 사용하지 않습니다."""
        catalog = generator.build()
        catalog['approved'] = True  # 시험 사본만 승인해 revision 차단을 독립적으로 확인한다.
        result = observe_robot(catalog, [-0.5, -0.5, 0.5, 0.5], (100.0, 100.0, 0.0),
                               frame_id='map', map_revision=1, position_valid=True,
                               outline_complete=True, age_s=0.0)
        self.assertTrue(result)
        self.assertTrue(all(row.state == 'UNKNOWN' and not row.position_valid
                            for row in result.values()))

    def test_rebased_catalog_preserves_geometry_and_pending_approval(self):
        """지도 번호를 맞춰도 기존 물리 경계와 미승인 정책을 그대로 보존합니다."""
        path = ROOT / 'src/ax_environment/config/ax_warehouse_regions.json'
        catalog = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(catalog, generator.build())
        self.assertFalse(catalog['approved'])
        self.assertEqual(catalog['map_revision'], self.metadata()[0]['map_revision'])


if __name__ == '__main__':
    unittest.main()
