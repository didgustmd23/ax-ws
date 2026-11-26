"""D2 개발 launch의 인자 → 노드 파라미터 전달 시험(ROS 그래프를 띄우지 않는다).

launch 파일을 모듈로 읽어 LaunchDescription의 인자 기본값을 context에 넣고, Node action의 파라미터를
launch_ros 평가 함수로 실제 값까지 계산해 확인한다(소스 문자열 검색이 아님). 패키지 share 경로는
설치본 없이 돌도록 가짜 경로로 바꾼다.
실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_launch_config.py
"""
import importlib.util
from pathlib import Path
import unittest
from unittest import mock

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'src' / 'ax_fleet_manager' / 'launch' / 'group_manager_dev.launch.py'
FAKE_SHARE = '/nonexistent/share/ax_fleet_manager'


# launch 파일을 읽어 LaunchDescription을 만든다(share 경로는 가짜)
def load_description():
    spec = importlib.util.spec_from_file_location('d2_group_launch_under_test', LAUNCH)
    module = importlib.util.module_from_spec(spec)

    with mock.patch('ament_index_python.packages.get_package_share_directory',
                    return_value=FAKE_SHARE):
        spec.loader.exec_module(module)
        return module.generate_launch_description()


# 인자 기본값(+덮어쓰기)으로 context를 채우고 (인자 기본값 사전, 노드, 평가된 파라미터 사전)을 돌려준다
def evaluate(overrides=None):
    description = load_description()
    context = LaunchContext()
    defaults = {}

    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = ''.join(s.perform(context) for s in entity.default_value)
            defaults[entity.name] = default
            context.launch_configurations[entity.name] = default

    for key, value in (overrides or {}).items():
        context.launch_configurations[key] = value

    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    params = {}
    for item in evaluate_parameters(context, nodes[0]._Node__parameters):
        params.update(item)

    return defaults, nodes, params


class GroupManagerDevLaunchTest(unittest.TestCase):
    """use_sim_time 인자가 선언되고 bool 파라미터로 노드에 전달된다."""

    # 기본은 false(/clock 없는 개발 단독 실행)이며 bool로 전달
    def test_use_sim_time_default_false(self):
        defaults, nodes, params = evaluate()

        self.assertEqual(defaults['use_sim_time'], 'false')
        self.assertEqual(len(nodes), 1)
        self.assertIs(params['use_sim_time'], False)

    # use_sim_time:=true면 노드 파라미터가 True
    def test_use_sim_time_true_is_passed(self):
        _, _, params = evaluate({'use_sim_time': 'true'})

        self.assertIs(params['use_sim_time'], True)

    # 기존 인자 전달은 그대로
    def test_other_arguments_unchanged(self):
        _, nodes, params = evaluate({'store_path': '/tmp/x/d2.sqlite'})

        self.assertEqual(nodes[0].node_executable, 'group_dev_publisher')
        self.assertEqual(params['store_path'], '/tmp/x/d2.sqlite')
        self.assertEqual(params['output_topic'], '/ax/dev/d2_mock/fleet_tasks')
        self.assertEqual(params['mode'], 'DEVELOPMENT_MOCK')
        self.assertEqual(params['fixture_file'],
                         str(Path(FAKE_SHARE) / 'config' / 'dev_fixture_transfer.json'))


if __name__ == '__main__':
    unittest.main()
