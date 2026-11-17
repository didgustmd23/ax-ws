"""Launch-level regression checks for the W2 A4/B4 time contract and parameter routing.

노드는 OpaqueFunction 안에서 만들어지므로 LaunchContext로 함수를 실행해 실제 생성된 Node action을 검사한다
(소스 문자열 검색이 아님). ROS 그래프는 띄우지 않는다.
"""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'src' / 'ax_safety' / 'config' / 'safety_limits.yaml'


def load_launch():
    path = ROOT / 'src' / 'ax_safety' / 'launch' / 'safety.launch.py'
    spec = importlib.util.spec_from_file_location('safety_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# launch 인자 기본값을 context에 넣고 OpaqueFunction을 실행해 Node 목록을 얻는다
def nodes_for(overrides=None):
    module = load_launch()
    description = module.generate_launch_description()
    context = LaunchContext()

    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = ''.join(s.perform(context) for s in entity.default_value) if entity.default_value else ''
            context.launch_configurations[entity.name] = default

    for key, value in (overrides or {}).items():
        context.launch_configurations[key] = value

    nodes = []
    for entity in description.entities:
        if isinstance(entity, OpaqueFunction):
            nodes.extend(e for e in entity.execute(context) if isinstance(e, Node))

    return {node.node_executable: node for node in nodes}


def parameter_items(node):
    # launch_ros의 정규화된 파라미터 매핑은 내부 속성으로 보관된다.
    return node._Node__parameters


def dict_value(node, name):
    for item in parameter_items(node):
        if isinstance(item, dict):
            for key, value in item.items():
                if getattr(key[0], 'text', None) == name:
                    return value
    return None


def file_entries(node):
    return [item for item in parameter_items(node) if not isinstance(item, dict)]


def file_path(entry):
    # ParameterFile 또는 경로 substitution 목록을 문자열로 바꾼다
    context = LaunchContext()
    param_file = getattr(entry, 'param_file', entry)
    if isinstance(param_file, (list, tuple)):
        return ''.join(s.perform(context) for s in param_file)
    return str(param_file)


class SafetyLaunchConfigTest(unittest.TestCase):
    def test_b4_explicitly_uses_ros_sim_time(self):
        nodes = nodes_for()
        self.assertIn('b4_risk_node', nodes)
        self.assertIs(dict_value(nodes['b4_risk_node'], 'use_sim_time'), True)

    # 세 노드 모두 공통 파일을 받고, 로봇별 파일이 없으면 공통 파일 하나뿐이다
    def test_common_file_reaches_all_nodes(self):
        nodes = nodes_for()
        self.assertEqual(sorted(nodes), ['a4_protected_arm_server', 'a4_safety_gate', 'b4_risk_node'])

        for name, node in nodes.items():
            files = [file_path(e) for e in file_entries(node)]
            self.assertEqual(len(files), 1, name)
            self.assertTrue(files[0].endswith('safety_limits.yaml'), name)

    # 로봇별 파일은 공통 파일 뒤에 붙고, 없는 경로는 시작 전에 거절한다
    def test_robot_file_is_appended_and_missing_rejected(self):
        with tempfile.NamedTemporaryFile('w', suffix='.yaml') as robot_file:
            robot_file.write('/**/a4_safety_gate:\n  ros__parameters:\n    travel_pose_tolerance_rad: 0.04\n')
            robot_file.flush()
            nodes = nodes_for({'robot_params_file': robot_file.name})

            for name, node in nodes.items():
                files = [file_path(e) for e in file_entries(node)]
                self.assertEqual(files[-1], robot_file.name, name)

        with self.assertRaises(RuntimeError):
            nodes_for({'robot_params_file': '/nonexistent/robot_params.yaml'})

    # 허가 플래그는 launch 인자 사전(마지막 항목)에 있고 기본 false다
    def test_default_flags_stay_locked(self):
        nodes = nodes_for()
        gate = nodes['a4_safety_gate']
        context = LaunchContext()

        for flag in ('enable_simulation_drive', 'operational_permits_verified', 'travel_envelope_verified',
                     'arm_operation_verified'):
            value = dict_value(gate, flag)
            self.assertIsNotNone(value, flag)
            context.launch_configurations[flag] = 'false'
            text = ''.join(s.perform(context) for s in value.value)
            self.assertEqual(text, 'false', flag)

        self.assertIsInstance(parameter_items(gate)[-1], dict)

    # 패키지 yaml: namespaced 노드에 적용되는 /**/ 키, 빈 배열 없음, 허가 false, D-06 값
    def test_package_yaml_keys_and_no_empty_arrays(self):
        data = yaml.safe_load(CONFIG.read_text())
        self.assertEqual(sorted(data), ['/**/a4_protected_arm_server', '/**/a4_safety_gate', '/**/b4_risk_node'])

        for key, section in data.items():
            params = section['ros__parameters']
            empty = [name for name, value in params.items() if isinstance(value, list) and not value]
            self.assertEqual(empty, [], key)

        gate = data['/**/a4_safety_gate']['ros__parameters']
        for flag in ('enable_simulation_drive', 'operational_permits_verified', 'travel_envelope_verified',
                     'arm_operation_verified'):
            self.assertIs(gate[flag], False, flag)
        self.assertNotIn('travel_joint_names', gate)
        self.assertEqual((gate['stop_linear_threshold_m_s'], gate['stop_angular_threshold_rad_s'],
                          gate['stop_joint_threshold_rad_s'], gate['stop_hold_s']), (0.01, 0.02, 0.01, 0.5))

    # 정지 역할 파일은 공통 파일 뒤·로봇별 파일 앞에 붙고, 없는 경로는 거절한다
    def test_stop_evidence_file_order(self):
        evidence = ROOT / 'src' / 'ax_safety' / 'config' / 'stop_evidence_ax1.yaml'
        with tempfile.NamedTemporaryFile('w', suffix='.yaml') as robot_file:
            robot_file.write('/**/a4_safety_gate:\n  ros__parameters:\n    travel_pose_tolerance_rad: 0.04\n')
            robot_file.flush()
            nodes = nodes_for({'stop_evidence_file': str(evidence), 'robot_params_file': robot_file.name})
            files = [file_path(e) for e in file_entries(nodes['a4_safety_gate'])]
            self.assertEqual(files[1:], [str(evidence), robot_file.name])

        with self.assertRaises(RuntimeError):
            nodes_for({'stop_evidence_file': '/nonexistent/stop_evidence.yaml'})

    # 모델별 정지 역할 파일: AX_1 필수 팔 관절은 실제 arm_controller 관절·URDF revolute 관절과 같고, AX_2는 팔 없음
    def test_model_stop_evidence_matches_controllers(self):
        config = ROOT / 'src' / 'ax_safety' / 'config'
        ax1 = yaml.safe_load((config / 'stop_evidence_ax1.yaml').read_text())
        ax2 = yaml.safe_load((config / 'stop_evidence_ax2.yaml').read_text())
        self.assertEqual(sorted(ax1), ['/**/a4_safety_gate'])
        self.assertEqual(sorted(ax2), ['/**/a4_safety_gate'])

        p1 = ax1['/**/a4_safety_gate']['ros__parameters']
        p2 = ax2['/**/a4_safety_gate']['ros__parameters']
        controllers = yaml.safe_load((ROOT / 'src' / 'ax_control' / 'config' / 'ax1_controllers.yaml').read_text())
        arm_joints = controllers['/**/arm_controller']['ros__parameters']['joints']
        self.assertEqual(p1, {'stop_arm_presence': 'present', 'stop_arm_joint_names': arm_joints})

        urdf = (ROOT / 'src' / 'ax_description' / 'urdf' / 'ax.urdf.xacro').read_text()
        for name in arm_joints:
            self.assertIn(f'<joint name="{name}" type="revolute">', urdf)
        self.assertFalse(any('wheel' in name for name in arm_joints))

        self.assertEqual(p2, {'stop_arm_presence': 'absent'})
        ax2_controllers = (ROOT / 'src' / 'ax_control' / 'config' / 'ax2_controllers.yaml').read_text()
        self.assertNotIn('arm_controller', ax2_controllers)


if __name__ == '__main__':
    unittest.main()
