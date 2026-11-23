"""상위 simulation.launch.py가 로봇별 A4/B4 설정 파일을 safety.launch.py에 전달하는지 확인한다.

simulation.launch.py의 OpaqueFunction을 LaunchContext로 실행하고, 로봇별 safety include를 실제 launch
방문 순서(GroupAction scope push/pop, IncludeLaunchDescription, safety.launch.py OpaqueFunction)대로 따라가
생성된 A4/B4 Node의 매개변수(파일 목록·launch 인자 사전)를 평가한다. A1 robot_test include와 spawner·
Gazebo·ROS 그래프는 실행하지 않는다. 패키지 share 경로는 저장소 src로 바꿔 소스 기준으로 검사한다.
"""
import importlib.util
from pathlib import Path
import re
import tempfile
import unittest
from unittest import mock

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).resolve().parents[2]
SIMULATION = ROOT / 'src' / 'ax_control' / 'launch' / 'simulation.launch.py'
SHARES = {name: ROOT / 'src' / name for name in ('ax_safety', 'ax_description', 'ax_control')}
SAFETY_CONFIG = SHARES['ax_safety'] / 'config'
COMMON = SAFETY_CONFIG / 'safety_limits.yaml'
AX1 = SAFETY_CONFIG / 'stop_evidence_ax1.yaml'
AX2 = SAFETY_CONFIG / 'stop_evidence_ax2.yaml'
SAFETY_EXECUTABLES = ('a4_safety_gate', 'a4_protected_arm_server', 'b4_risk_node')
ROBOT_TEST = SHARES['ax_description'] / 'launch' / 'robot_test.launch.py'


# 패키지 share·prefix를 저장소 src로 돌려 설치본 노후와 무관하게 소스를 검사한다
def share_of(name):
    return str(SHARES[name])


# 상위 launch 모듈을 새로 읽는다(시험마다 독립)
def load_simulation():
    spec = importlib.util.spec_from_file_location('simulation_launch_under_test', SIMULATION)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = share_of
    module.get_package_prefix = share_of
    return module


# 상위 launch 인자 기본값과 덮어쓸 값을 넣은 context를 만든다. extra는 미선언 CLI 인자처럼 미리 넣는 값이다
def make_context(description, overrides=None, extra=None):
    context = LaunchContext()

    for key, value in (extra or {}).items():
        context.launch_configurations[key] = value

    for key, value in (overrides or {}).items():
        context.launch_configurations[key] = value

    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)

    return context


# include 대상 launch 파일 경로(substitution을 평가한 값)
def launch_file(include, context):
    source = include.launch_description_source
    location = source._LaunchDescriptionSource__location
    return perform_substitutions(context, normalize_to_list_of_substitutions(location))


# 반환 action을 launch 방문 순서대로 따라가며 safety Node를 그 시점 context로 평가한다
def walk(entities, context, nodes, includes):
    for entity in entities:
        if isinstance(entity, Node):
            if entity.node_package == 'ax_safety':
                raw_namespace = normalize_to_list_of_substitutions(entity._Node__node_namespace)
                namespace = perform_substitutions(context, raw_namespace)
                params = evaluate_parameters(context, entity._Node__parameters)
                nodes.append({'namespace': namespace, 'executable': entity.node_executable, 'params': params})
            continue

        # 프로세스 환경을 바꾸지 않는다
        if isinstance(entity, SetEnvironmentVariable):
            continue

        # 로그는 출력하지 않고 내용만 모은다(정지 역할 파일 출처 확인용)
        if isinstance(entity, LogInfo):
            LOGS.append(perform_substitutions(context, entity.msg))
            continue

        if isinstance(entity, IncludeLaunchDescription):
            location = launch_file(entity, context)
            includes.append(location)
            # A1 월드·모델 launch는 xacro·Gazebo가 필요하므로 따라가지 않는다
            if location.endswith('robot_test.launch.py'):
                continue

        sub_entities = entity.visit(context)
        if sub_entities:
            walk(sub_entities, context, nodes, includes)


# 상위 launch를 실행해 safety Node 평가 결과와 include 목록, 방문 후 context를 돌려준다
def run_simulation(overrides=None, extra=None):
    LOGS.clear()
    with mock.patch('ament_index_python.packages.get_package_share_directory', share_of):
        module = load_simulation()
        description = module.generate_launch_description()
        context = make_context(description, overrides, extra)
        opaque = [entity for entity in description.entities if isinstance(entity, OpaqueFunction)]
        nodes, includes = [], []
        for entity in opaque:
            walk(entity.execute(context), context, nodes, includes)
    return nodes, includes, context


# 평가된 매개변수에서 파일 목록과 launch 인자 사전을 나눈다
def split_params(params):
    files = [str(item) for item in params if isinstance(item, Path)]
    values = {}
    for item in params:
        if isinstance(item, dict):
            values.update(item)
    return files, values


# 로봇 namespace와 실행 파일로 safety Node 하나를 찾는다
def node(nodes, robot_id, executable):
    found = [n for n in nodes if n['namespace'] == f'/ax/{robot_id}' and n['executable'] == executable]
    if len(found) != 1:
        raise AssertionError(f'{robot_id} {executable}: {len(found)}개')
    return split_params(found[0]['params'])


SAFETY_ON = {'safety': 'true', 'gui': 'false'}
# 기존 의미(매핑 없는 로봇은 빈 값)를 확인하는 시험용. 기본값 적용을 끈다
SAFETY_ON_NO_AUTO = {**SAFETY_ON, 'safety_stop_evidence_auto': 'false'}
# walk가 모은 LogInfo 내용(run_simulation마다 비운다)
LOGS = []


class SimulationSafetyLaunchTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        # 경로 내부 공백 보존 확인용 로봇별 파일
        folder = Path(self.tmp.name) / 'robot params'
        folder.mkdir()
        self.robot_a = folder / 'robot a.yaml'
        self.robot_b = folder / 'robot_b.yaml'
        self.common = Path(self.tmp.name) / 'common.yaml'
        for path in (self.robot_a, self.robot_b, self.common):
            path.write_text('/**/a4_safety_gate:\n  ros__parameters:\n    stop_hold_s: 0.5\n')

    # 기본 safety=false: A4/B4 include가 없다
    def test_default_has_no_safety_nodes(self):
        nodes, includes, _ = run_simulation({'gui': 'false'})
        self.assertEqual(nodes, [])
        self.assertFalse(any(path.endswith('safety.launch.py') for path in includes))

    # 두 로봇에 서로 다른 정지 역할·로봇별 파일, 공통 기본 경로, 허가 false
    def test_two_robots_receive_distinct_files(self):
        nodes, _, _ = run_simulation({
            **SAFETY_ON,
            'safety_stop_evidence_files': f'robot_1={AX1},robot_2={AX2}',
            'safety_robot_params_files': f'robot_1={self.robot_a}',
        })

        for executable in SAFETY_EXECUTABLES:
            files, values = node(nodes, 'robot_1', executable)
            self.assertEqual(files, [str(COMMON), str(AX1), str(self.robot_a)])
            self.assertEqual(values['robot_id'], 'robot_1')

            files, values = node(nodes, 'robot_2', executable)
            self.assertEqual(files, [str(COMMON), str(AX2)])
            self.assertEqual(values['robot_id'], 'robot_2')

        for robot_id in ('robot_1', 'robot_2'):
            _, gate = node(nodes, robot_id, 'a4_safety_gate')
            for flag in ('enable_simulation_drive', 'operational_permits_verified',
                         'travel_envelope_verified', 'arm_operation_verified'):
                self.assertIs(gate[flag], False)
            _, arm = node(nodes, robot_id, 'a4_protected_arm_server')
            self.assertIs(arm['enable_simulation_arm'], False)

    # 미지정이면 공통 패키지 기본 파일만 들어가고 빈 경로를 넘기지 않는다
    def test_unspecified_uses_package_common_only(self):
        nodes, _, _ = run_simulation({**SAFETY_ON_NO_AUTO, 'robots': '2'})
        for robot_id in ('robot_1', 'robot_2'):
            for executable in SAFETY_EXECUTABLES:
                files, _ = node(nodes, robot_id, executable)
                self.assertEqual(files, [str(COMMON)])

    # 별도 공통 파일 지정
    def test_custom_common_file(self):
        nodes, _, _ = run_simulation({**SAFETY_ON_NO_AUTO, 'safety_params_file': str(self.common)})
        files, _ = node(nodes, 'robot_2', 'a4_safety_gate')
        self.assertEqual(files, [str(self.common)])

    # robot_ids 부분 선택: 선택한 로봇에만 safety, 매핑도 그 로봇 기준
    def test_partial_selection(self):
        nodes, _, _ = run_simulation({**SAFETY_ON, 'robot_ids': 'robot_3',
                                      'safety_stop_evidence_files': f'robot_3={AX1}'})
        self.assertEqual({n['namespace'] for n in nodes}, {'/ax/robot_3'})
        files, _ = node(nodes, 'robot_3', 'a4_safety_gate')
        self.assertEqual(files, [str(COMMON), str(AX1)])

    # 4대 선택에서 일부만 지정: 나머지는 비어 있어야 한다
    def test_four_robots_no_fill_from_neighbor(self):
        nodes, _, _ = run_simulation({**SAFETY_ON_NO_AUTO, 'robots': '4',
                                      'safety_stop_evidence_files': f'robot_2={AX2},robot_3={AX1}'})
        expected = {'robot_1': [str(COMMON)], 'robot_2': [str(COMMON), str(AX2)],
                    'robot_3': [str(COMMON), str(AX1)], 'robot_4': [str(COMMON)]}
        for robot_id, files in expected.items():
            self.assertEqual(node(nodes, robot_id, 'a4_safety_gate')[0], files)

    # 앞 로봇만 지정/뒤 로봇 미지정, 반대 순서 모두 다음 로봇으로 새지 않는다
    def test_scope_no_leak_between_robots(self):
        for mapping, filled, empty in ((f'robot_1={AX1}', 'robot_1', 'robot_2'),
                                       (f'robot_2={AX2}', 'robot_2', 'robot_1')):
            with self.subTest(mapping=mapping):
                nodes, _, context = run_simulation({**SAFETY_ON_NO_AUTO, 'safety_stop_evidence_files': mapping,
                                                    'safety_robot_params_files': f'{filled}={self.robot_b}'})
                self.assertEqual(len(node(nodes, filled, 'a4_safety_gate')[0]), 3)
                self.assertEqual(node(nodes, empty, 'a4_safety_gate')[0], [str(COMMON)])
                # include 방문이 끝난 뒤 상위 context에 하위 인자가 남지 않는다
                for name in ('robot_id', 'params_file', 'stop_evidence_file', 'robot_params_file'):
                    self.assertNotIn(name, context.launch_configurations)

    # 상위 context에 이미 하위 이름이 있어도(직접 넣은 상속값) 명시 전달값이 이긴다
    def test_inherited_values_do_not_leak(self):
        with mock.patch('ament_index_python.packages.get_package_share_directory', share_of):
            module = load_simulation()
            context = LaunchContext()
            context.launch_configurations.update({
                'stop_evidence_file': str(AX2), 'robot_params_file': str(self.robot_a),
                'params_file': str(self.common), 'enable_simulation_drive': 'true'})
            values = {**{name: '' for name in module.SAFETY_FILE_ARGUMENTS},
                      **{name: 'false' for name in module.SAFETY_FLAG_ARGUMENTS},
                      'safety_stop_evidence_auto': 'false'}
            values['safety_stop_evidence_files'] = f'robot_1={AX1}'
            nodes, includes = [], []
            robot_ids = ['robot_1', 'robot_2']
            per_robot = module.safety_actions(values, robot_ids, module.robot_models(robot_ids))
            walk([action for robot_id in robot_ids for action in per_robot[robot_id]], context, nodes, includes)

        self.assertEqual(node(nodes, 'robot_1', 'a4_safety_gate')[0], [str(COMMON), str(AX1)])
        files, gate = node(nodes, 'robot_2', 'a4_safety_gate')
        self.assertEqual(files, [str(COMMON)])
        self.assertIs(gate['enable_simulation_drive'], False)

    # 매핑 오류는 launch 시작 전에 거부한다
    def test_invalid_mappings_rejected(self):
        cases = {
            'not_selected': (f'robot_3={AX1}', '선택되지 않은'),
            'duplicate': (f'robot_1={AX1},robot_1={AX2}', '중복'),
            'missing_value': ('robot_1=', '형식'),
            'missing_separator': (str(AX1), '형식'),
            'empty_item': (f'robot_1={AX1},', '형식'),
            'missing_file': ('robot_1=/nonexistent/ax.yaml', '파일 없음'),
            'comma_in_path': (f'robot_1={self.tmp.name}/a,b.yaml', '형식'),
            'bad_robot_id': (f'robot_9={AX1}', '선택되지 않은'),
        }
        for name, (mapping, message) in cases.items():
            with self.subTest(name):
                with self.assertRaisesRegex(ValueError, message):
                    run_simulation({**SAFETY_ON, 'safety_stop_evidence_files': mapping})

    # 공통 파일이 없으면 거부한다
    def test_missing_common_rejected(self):
        with self.assertRaisesRegex(ValueError, '파일 없음'):
            run_simulation({**SAFETY_ON, 'safety_params_file': '/nonexistent/common.yaml'})

    # 경로 내부 공백은 보존하고 '='는 첫 구분자만 쓴다
    def test_path_spaces_and_equals(self):
        odd = Path(self.tmp.name) / 'a=b c.yaml'
        odd.write_text('/**/a4_safety_gate:\n  ros__parameters:\n    stop_hold_s: 0.5\n')
        nodes, _, _ = run_simulation({**SAFETY_ON_NO_AUTO, 'robot_ids': 'robot_1',
                                      'safety_robot_params_files': f' robot_1 = {odd} '})
        self.assertEqual(node(nodes, 'robot_1', 'a4_safety_gate')[0], [str(COMMON), str(odd)])

    # 상위 진입점에서 하위 단일 로봇 인자(전역 별칭)를 거부한다
    def test_global_aliases_rejected(self):
        for alias in ('stop_evidence_file', 'robot_params_file', 'params_file', 'enable_simulation_drive'):
            for safety in ('true', 'false'):
                with self.subTest(alias=alias, safety=safety):
                    with self.assertRaisesRegex(ValueError, alias):
                        run_simulation({'safety': safety, 'gui': 'false'}, extra={alias: str(AX1)})

    # safety=false인데 파일 인자를 주면 조용히 무시하지 않는다
    def test_file_arguments_without_safety_rejected(self):
        with self.assertRaisesRegex(ValueError, 'safety:=true'):
            run_simulation({'gui': 'false', 'safety_stop_evidence_files': f'robot_1={AX1}'})

    # A1 robot_test에는 safety 인자를 넘기지 않는다
    def test_robot_test_gets_no_safety_arguments(self):
        with mock.patch('ament_index_python.packages.get_package_share_directory', share_of):
            module = load_simulation()
            description = module.generate_launch_description()
            context = make_context(description, {**SAFETY_ON, 'safety_stop_evidence_files': f'robot_1={AX1}'})
            actions = [a for e in description.entities if isinstance(e, OpaqueFunction) for a in e.execute(context)]
        includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
        robot_test = [a for a in includes if launch_file(a, context).endswith('robot_test.launch.py')]
        self.assertEqual(len(robot_test), 1)
        names = {perform_substitutions(context, normalize_to_list_of_substitutions(key))
                 for key, _ in robot_test[0].launch_arguments}
        self.assertFalse({name for name in names if 'safety' in name or name.endswith('_file')})
        self.assertIn('controller_config_dir', names)
        # 로봇별 safety include는 scope를 가진 GroupAction 안에만 있다
        groups = [a for a in actions if isinstance(a, GroupAction)]
        self.assertEqual(len(groups), 2)
        safety_includes = [a for a in includes if launch_file(a, context).endswith('safety.launch.py')]
        self.assertEqual(safety_includes, [])

    # 매핑 없음(기본 auto=true): 각 로봇이 실제 모델의 기본 정지 역할 파일을 받는다
    def test_auto_default_by_model(self):
        nodes, _, _ = run_simulation({**SAFETY_ON, 'robots': '2'})
        for robot_id, evidence in (('robot_1', AX1), ('robot_2', AX2)):
            for executable in SAFETY_EXECUTABLES:
                self.assertEqual(node(nodes, robot_id, executable)[0], [str(COMMON), str(evidence)])
        # 로봇별 출처 로그
        self.assertEqual(len(LOGS), 2)
        self.assertIn('robot_1', LOGS[0])
        self.assertIn(str(AX1), LOGS[0])
        self.assertIn('모델 기본값(ax.urdf.xacro)', LOGS[0])
        self.assertIn('모델 기본값(ax_2.urdf.xacro)', LOGS[1])

    # robot_3·robot_4 포함 선택(4대·부분 선택)도 모델 기준
    def test_auto_default_four_and_partial(self):
        nodes, _, _ = run_simulation({**SAFETY_ON, 'robots': '4'})
        expected = {'robot_1': AX1, 'robot_2': AX2, 'robot_3': AX1, 'robot_4': AX2}
        for robot_id, evidence in expected.items():
            self.assertEqual(node(nodes, robot_id, 'a4_safety_gate')[0], [str(COMMON), str(evidence)])

        nodes, _, _ = run_simulation({**SAFETY_ON, 'robot_ids': 'robot_4,robot_3'})
        self.assertEqual({n['namespace'] for n in nodes}, {'/ax/robot_3', '/ax/robot_4'})
        self.assertEqual(node(nodes, 'robot_4', 'a4_safety_gate')[0], [str(COMMON), str(AX2)])
        self.assertEqual(node(nodes, 'robot_3', 'a4_safety_gate')[0], [str(COMMON), str(AX1)])

    # 부분 매핑: 명시 항목이 우선이고 나머지는 모델 기본값
    def test_explicit_mapping_overrides_default(self):
        custom = Path(self.tmp.name) / 'custom_evidence.yaml'
        custom.write_text('/**/a4_safety_gate:\n  ros__parameters:\n    stop_arm_presence: absent\n')
        nodes, _, _ = run_simulation({**SAFETY_ON, 'robots': '2', 'safety_stop_evidence_files': f'robot_1={custom}'})
        self.assertEqual(node(nodes, 'robot_1', 'a4_safety_gate')[0], [str(COMMON), str(custom)])
        self.assertEqual(node(nodes, 'robot_2', 'a4_safety_gate')[0], [str(COMMON), str(AX2)])
        self.assertIn('명시 매핑', LOGS[0])
        self.assertIn('모델 기본값(ax_2.urdf.xacro)', LOGS[1])

    # auto=false: 매핑 없는 로봇은 빈 값이고 로그에 그 사실을 남긴다. 잘못된 값은 거부
    def test_auto_false_and_invalid_value(self):
        nodes, _, _ = run_simulation({**SAFETY_ON_NO_AUTO, 'robots': '2'})
        for robot_id in ('robot_1', 'robot_2'):
            self.assertEqual(node(nodes, robot_id, 'a4_safety_gate')[0], [str(COMMON)])
        self.assertTrue(all('(빈 값)' in line and 'auto:=false' in line for line in LOGS))

        for value in ('yes', '', '1'):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, 'safety_stop_evidence_auto'):
                    run_simulation({**SAFETY_ON, 'safety_stop_evidence_auto': value})

    # 역할을 모르는 로봇(A1 ROBOT_ROLES에 없음)은 기본값을 넣지 않고 시작 전에 거부한다
    def test_unknown_model_rejected(self):
        with mock.patch('ament_index_python.packages.get_package_share_directory', share_of):
            module = load_simulation()
            module.a1_robot_roles = lambda: {'robot_1': 'MANIPULATOR', 'robot_2': 'INSPECTOR'}
            with self.assertRaisesRegex(ValueError, "robot_2.*모델을 정할 수 없다"):
                module.robot_models(['robot_1', 'robot_2'])
            with self.assertRaisesRegex(ValueError, "robot_3.*모델을 정할 수 없다"):
                module.robot_models(['robot_3'])

    # 모델 기본 파일이 없으면 빈 값으로 넘기지 않고 거부한다
    def test_missing_default_file_rejected(self):
        with mock.patch('ament_index_python.packages.get_package_share_directory', share_of):
            module = load_simulation()
            module.MODEL_STOP_EVIDENCE = {**module.MODEL_STOP_EVIDENCE, 'ax_2.urdf.xacro': 'missing.yaml'}
            values = {**{name: '' for name in module.SAFETY_FILE_ARGUMENTS},
                      **{name: 'false' for name in module.SAFETY_FLAG_ARGUMENTS},
                      'safety_stop_evidence_auto': 'true'}
            with self.assertRaisesRegex(ValueError, '모델 기본 파일 없음'):
                module.safety_actions(values, ['robot_2'], module.robot_models(['robot_2']))

    # 두 출처 일치: 역할 표는 A1 파일 그대로 읽고, 역할→모델 규칙·모델 목록은 A1 원문과 같다
    def test_roles_and_models_match_a1(self):
        with mock.patch('ament_index_python.packages.get_package_share_directory', share_of):
            module = load_simulation()
            spec = importlib.util.spec_from_file_location('robot_test_under_test', ROBOT_TEST)
            a1 = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(a1)

            self.assertEqual(module.a1_robot_roles(), a1.ROBOT_ROLES)
            # A1 모델 선택 원문: 'ax.urdf.xacro' if manipulator else 'ax_2.urdf.xacro', manipulator = 역할 MANIPULATOR
            source = ROBOT_TEST.read_text(encoding='utf-8')
            self.assertRegex(source, r"manipulator = ROBOT_ROLES\[robot_id\] == 'MANIPULATOR'")
            rule = re.search(r"model_filename = '([^']+)' if manipulator else '([^']+)'", source)
            self.assertIsNotNone(rule)
            for role in set(a1.ROBOT_ROLES.values()):
                expected = rule.group(1) if role == 'MANIPULATOR' else rule.group(2)
                self.assertEqual(module.ROLE_MODELS[role], expected)
            # 모델 목록은 A1 제어기 설정 표와 같고, 각 모델에 정지 역할 파일이 있다
            self.assertEqual(set(module.MODEL_STOP_EVIDENCE), set(a1.CONTROLLER_FILES))
            for name in module.MODEL_STOP_EVIDENCE.values():
                self.assertTrue((SAFETY_CONFIG / name).is_file(), name)
            # 팔 모델(arm_controller·공구 대상)은 MANIPULATOR 모델뿐
            self.assertEqual(module.ARM_MODELS, {rule.group(1)})

    # 팔 제어기·공구 노드는 모델 기준(robot_1·robot_3)으로만 붙는다
    def test_arm_nodes_follow_model(self):
        with mock.patch('ament_index_python.packages.get_package_share_directory', share_of):
            module = load_simulation()
            description = module.generate_launch_description()
            context = make_context(description, {'gui': 'false', 'robots': '4'})
            actions = [a for e in description.entities if isinstance(e, OpaqueFunction) for a in e.execute(context)]
        spawned = {}
        tools = set()
        for action in actions:
            if isinstance(action, Node):
                namespace = perform_substitutions(context, normalize_to_list_of_substitutions(
                    action._Node__node_namespace))
                if action.node_executable == 'spawner':
                    args = [perform_substitutions(context, normalize_to_list_of_substitutions(a))
                            for a in action._Node__arguments]
                    spawned.setdefault(namespace, set()).update(args)
                if action.node_executable == 'suction_tool_node.py':
                    tools.add(namespace)
        arm = {ns for ns, args in spawned.items() if 'arm_controller' in args}
        self.assertEqual(arm, {'/ax/robot_1', '/ax/robot_3'})
        self.assertEqual(tools, {'/ax/robot_1', '/ax/robot_3'})


if __name__ == '__main__':
    unittest.main()
