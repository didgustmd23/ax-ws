"""상대 구현을 시작하지 않고 내 담당 실행 설정과 권한 보류를 검사합니다."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[2]


class KimTeamLaunchTests(unittest.TestCase):
    """다른 담당자의 노드나 승인값을 추가하지 않고 같은 시계와 로봇 ID를 전달합니다."""

    def build(self, **overrides):
        """노드를 생성하는 대신 실행 설정을 수집합니다. ROS 통신은 하지 않습니다."""
        path = ROOT / 'src/ax_task_executor/launch/team_handoff.launch.py'
        spec = importlib.util.spec_from_file_location('kim_full_team_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        context.launch_configurations.update(overrides)
        with patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs):
            return module.setup_nodes(context)

    def test_default_keeps_existing_a3_b3_only(self):
        """기존 실행 명령은 C3/D3를 추가로 켜지 않습니다."""
        self.assertEqual(len(self.build()), 4)

    def test_w3_uses_one_fleet_manager_and_manipulator_candidates(self):
        """같은 시계·로봇·영역을 전달하되 영역이나 허가 정책을 승인하지 않습니다."""
        nodes = self.build(with_w3='true', regions_file='/tmp/regions.json',
                           corridor_database='/tmp/isolated-corridors.db')
        self.assertEqual(len(nodes), 6)
        self.assertTrue(all(node['parameters'][0]['use_sim_time'] for node in nodes))
        self.assertEqual({node['package'] for node in nodes},
                         {'ax_task_executor', 'ax_environment', 'ax_grasp_planner', 'ax_corridor_manager'})
        candidates = [node for node in nodes if node['package'] == 'ax_grasp_planner']
        self.assertEqual([node['parameters'][0]['robot_id'] for node in candidates], ['robot_1'])
        managers = [node for node in nodes if node['package'] == 'ax_corridor_manager']
        self.assertEqual(len(managers), 1)
        config = managers[0]['parameters'][0]
        self.assertEqual(config['robot_ids'], ['robot_1', 'robot_2'])
        self.assertEqual(config['regions_file'], '/tmp/regions.json')
        self.assertEqual(config['database'], '/tmp/isolated-corridors.db')
        self.assertNotIn('grant_policy_approved', config)
        self.assertNotIn('approved', config)

    def test_transport_only_does_not_start_candidate_generator(self):
        """운송 모델에 조작 역할이나 가짜 후보를 만들어 부여하지 않습니다."""
        nodes = self.build(with_w3='true', robot_ids='robot_2', robot_models='AX_2')
        self.assertEqual(len(nodes), 3)
        self.assertNotIn('ax_grasp_planner', [node['package'] for node in nodes])

    def test_bad_flag_is_rejected_before_any_node_starts(self):
        """실행 옵션 오타는 조용히 일부 노드만 켜는 대신 시작 전에 거절합니다."""
        with self.assertRaises(ValueError):
            self.build(with_w3='yes')
