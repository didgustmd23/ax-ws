# B2 위치 추정 launch(공통 map_server + 로봇별 AMCL)의 인자 해석·로봇별 분리·설정 값과
# 평가 계산(check_localization.py)을 Gazebo 없이 검사한다.
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'src/ax_localization'
CONFIG = PACKAGE / 'config/amcl.yaml'


# launch 파일을 모듈로 읽는다. ROS launch 패키지가 없으면 None
def load_launch():
    spec = importlib.util.spec_from_file_location('localization_launch', PACKAGE / 'launch/localization.launch.py')
    module = importlib.util.module_from_spec(spec)

    try:
        spec.loader.exec_module(module)
    except ImportError:
        return None

    return module


LAUNCH = load_launch()


# amcl.yaml의 ros__parameters를 읽는다.
def amcl_config():
    return yaml.safe_load(CONFIG.read_text(encoding='utf-8'))['/**/amcl']['ros__parameters']


# 설정 파일 값만 검사한다(ROS 없이 가능).
class AmclConfig(unittest.TestCase):

    # 프레임·지도·입력 범위가 계약과 B1 출력에 맞는지 확인한다.
    def test_frames_and_ranges(self):
        params = amcl_config()

        self.assertEqual(params['global_frame_id'], 'map')
        self.assertEqual(params['map_topic'], '/map')
        self.assertTrue(params['tf_broadcast'])
        self.assertTrue(params['use_sim_time'])
        self.assertEqual(params['robot_model_type'], 'nav2_amcl::DifferentialMotionModel')
        self.assertEqual((params['laser_min_range'], params['laser_max_range']), (0.12, 10.0))
        self.assertFalse(params['set_initial_pose'])

    # 기본 초기 위치 묶음이 nav2 이름(initial_pose.x/y/z/yaw)을 따르는지 확인한다.
    def test_initial_pose_keys(self):
        self.assertEqual(set(amcl_config()['initial_pose']), {'x', 'y', 'z', 'yaw'})


@unittest.skipIf(LAUNCH is None, 'ROS launch 패키지 필요(source install/setup.bash)')
class LaunchArguments(unittest.TestCase):

    # 로봇별 초기 위치 문자열을 해석한다.
    def test_parse_initial_poses(self):
        poses = LAUNCH.parse_initial_poses('robot_2=0,0,0; robot_1=-2.5,1.5,0.1;', ['robot_1', 'robot_2'])

        self.assertEqual(poses, {'robot_2': (0.0, 0.0, 0.0), 'robot_1': (-2.5, 1.5, 0.1)})
        self.assertEqual(LAUNCH.parse_initial_poses('', ['robot_2']), {})

    # 형식이 틀리거나 목록에 없는 로봇·중복·비유한 값은 거절한다.
    def test_parse_initial_poses_rejects(self):
        for text in ('robot_3=0,0,0', 'robot_2=0,0', 'robot_2:0,0,0', 'robot_2=0,0,0;robot_2=1,1,0', 'robot_2=nan,0,0'):
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    LAUNCH.parse_initial_poses(text, ['robot_1', 'robot_2'])

    # 로봇 ID 목록 검사
    def test_parse_robot_ids(self):
        self.assertEqual(LAUNCH.parse_robot_ids('robot_1, robot_2'), ['robot_1', 'robot_2'])

        for text in ('', 'robot_5', 'robot_1,robot_1'):
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    LAUNCH.parse_robot_ids(text)

    # 로봇마다 프레임·입력이 자기 이름으로 바뀌고 설정 파일의 robot_2 기본값이 모두 덮어써지는지 확인한다.
    def test_overrides_cover_robot_specific_values(self):
        robot_keys = {key for key, value in amcl_config().items() if isinstance(value, str) and 'robot_' in value}

        for robot_id in ('robot_1', 'robot_2'):
            overrides = LAUNCH.amcl_overrides(robot_id, 'filtered')

            self.assertTrue(robot_keys <= set(overrides), robot_keys - set(overrides))
            self.assertEqual(overrides['odom_frame_id'], f'{robot_id}/odom')
            self.assertEqual(overrides['base_frame_id'], f'{robot_id}/base_link')
            self.assertEqual(overrides['scan_topic'], f'/ax/{robot_id}/sensors/front_lidar/scan_filtered')
            self.assertFalse(overrides['set_initial_pose'])

        self.assertEqual(LAUNCH.amcl_overrides('robot_1', 'raw')['scan_topic'], '/ax/robot_1/sensors/front_lidar/scan')

    # 초기 위치를 준 로봇만 set_initial_pose가 켜진다.
    def test_overrides_initial_pose(self):
        overrides = LAUNCH.amcl_overrides('robot_1', 'filtered', (-2.5, 1.5, 0.1))

        self.assertTrue(overrides['set_initial_pose'])
        self.assertEqual((overrides['initial_pose.x'], overrides['initial_pose.y'], overrides['initial_pose.yaw']),
                         (-2.5, 1.5, 0.1))

    # 채택 기록이 없는 지도는 개발용으로, map_info.yaml이 있는 map.yaml은 revision과 함께 안내한다.
    def test_map_notice(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder / 'map.yaml').write_text('image: map.pgm\n')

            self.assertIn('개발용 지도', LAUNCH.map_notice(folder / 'map.yaml'))

            (folder / 'map_info.yaml').write_text('map_revision: 3\n')

            self.assertIn('map_revision: 3', LAUNCH.map_notice(folder / 'map.yaml'))
            self.assertIn('개발용 지도', LAUNCH.map_notice(folder / 'slam_map.yaml'))


@unittest.skipIf(LAUNCH is None, 'ROS launch 패키지 필요(source install/setup.bash)')
class LaunchNodes(unittest.TestCase):

    # 실행 그래프 조회 없이 setup을 불러 만들어지는 노드 목록을 돌려준다.
    # start_poses: 고정 출발 위치 yaml 내용 함수(지도 이미지 sha256 → 문자열). 지도 이미지는 임시 pgm이다.
    def nodes(self, robot_ids, poses='', conflicts=(), start_poses=None, health='true'):
        from launch import LaunchContext
        from launch_ros.actions import Node

        original = LAUNCH.running_conflicts
        original_share = LAUNCH.get_package_share_directory
        LAUNCH.running_conflicts = lambda ids: list(conflicts)
        LAUNCH.get_package_share_directory = lambda name: str(PACKAGE)

        try:
            with tempfile.TemporaryDirectory() as folder:
                map_path = Path(folder) / 'map.yaml'
                map_path.write_text('image: map.pgm\n')
                (Path(folder) / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
                start_file = ''

                if start_poses is not None:
                    import hashlib

                    sha = hashlib.sha256((Path(folder) / 'map.pgm').read_bytes()).hexdigest()
                    start_file = str(Path(folder) / 'start.yaml')
                    Path(start_file).write_text(start_poses(sha))

                context = LaunchContext()
                context.launch_configurations.update({
                    'map': str(map_path), 'robot_ids': robot_ids, 'scan': 'filtered',
                    'initial_poses': poses, 'startup_timeout': '60.0', 'startup_report': '',
                    'start_poses': start_file, 'health': health, 'health_config': ''})

                actions = LAUNCH.setup(context)
        finally:
            LAUNCH.running_conflicts = original
            LAUNCH.get_package_share_directory = original_share

        return [action for action in actions if isinstance(action, Node)]

    # launch가 정규화한 값(TextSubstitution 묶음)을 문자열로 되돌린다. 다른 값은 그대로 둔다.
    @staticmethod
    def plain(value):
        if isinstance(value, tuple) and value and all(hasattr(part, 'text') for part in value):
            return ''.join(part.text for part in value)

        # 문자열 목록은 항목마다 되돌린다.
        if isinstance(value, (list, tuple)) and value and all(isinstance(item, (list, tuple)) for item in value):
            return [LaunchNodes.plain(tuple(item)) for item in value]

        return value

    # 노드의 (패키지, namespace, 이름, 덮어쓴 매개변수) 요약
    @classmethod
    def describe(cls, node):
        parameters = {}

        for item in node._Node__parameters:
            if isinstance(item, dict):
                parameters.update({cls.plain(key): cls.plain(value) for key, value in item.items()})

        return node._Node__package, node._Node__node_namespace, node._Node__node_name, parameters

    # 지도 서버는 1개, AMCL은 로봇마다 자기 namespace에 하나씩 만들고 모두 lifecycle 노드다.
    # nav2 lifecycle_manager는 쓰지 않는다(응답 유실 시 무기한 대기, 재연결 시 초기 위치 재초기화).
    def test_one_map_server_and_amcl_per_robot(self):
        from launch_ros.actions import LifecycleNode

        nodes = self.nodes('robot_1,robot_2', 'robot_2=0,0,0')
        described = [self.describe(node) for node in nodes]
        map_servers = [item for item in described if item[0] == 'nav2_map_server']
        amcls = {item[1]: item for item in described if item[0] == 'nav2_amcl'}

        self.assertEqual(len(map_servers), 1)
        self.assertEqual(set(amcls), {'/ax/robot_1', '/ax/robot_2'})
        lifecycle = [node for node in nodes if isinstance(node, LifecycleNode)]
        supervisors = {item[1]: item for item in described if item[0] == 'ax_localization'}

        self.assertEqual(len(lifecycle), 3)
        self.assertEqual(set(supervisors), {'/ax/robot_1', '/ax/robot_2'})
        self.assertEqual(len(nodes), 5)
        self.assertFalse([item for item in described if item[0] == 'nav2_lifecycle_manager'])

        # health:=false면 감독 노드를 띄우지 않는다.
        self.assertEqual(len(self.nodes('robot_2', health='false')), 2)

    # 고정 출발 위치: 지도와 맞으면 감독 노드 매개변수로 넘기고 AMCL에는 initial pose를 넣지 않는다.
    def test_start_poses(self):
        text = lambda sha: (f'map: {{revision: 0, image_sha256: {sha}}}\n'
                            'robots:\n  robot_1: {x: -2.5, y: 1.5, yaw: 0.0, position_std_m: 0.1, yaw_std_rad: 0.05}\n'
                            '  robot_2: {x: 0.0, y: 0.0, yaw: 0.0, position_std_m: 0.1, yaw_std_rad: 0.05}\n')
        described = [self.describe(node) for node in self.nodes('robot_1,robot_2', start_poses=text)]
        supervisors = {item[1]: item[3] for item in described if item[0] == 'ax_localization'}
        amcls = {item[1]: item[3] for item in described if item[0] == 'nav2_amcl'}

        self.assertTrue(supervisors['/ax/robot_1']['start_pose_enabled'])
        self.assertEqual(supervisors['/ax/robot_1']['start_pose_x'], -2.5)
        self.assertFalse(amcls['/ax/robot_1']['set_initial_pose'])

    # 고정 출발 위치 거절: 로봇 누락·지도 해시 불일치·initial_poses와 겹침·health 꺼짐
    def test_start_poses_rejected(self):
        good = lambda sha: (f'map: {{revision: 0, image_sha256: {sha}}}\n'
                            'robots: {robot_2: {x: 0, y: 0, yaw: 0, position_std_m: 0.1, yaw_std_rad: 0.05}}\n')
        cases = [
            ('로봇 누락', dict(robot_ids='robot_1,robot_2', start_poses=good)),
            ('해시 불일치', dict(robot_ids='robot_2', start_poses=lambda sha: good('f' * 64))),
            ('revision 불일치', dict(robot_ids='robot_2', start_poses=lambda sha: good(sha).replace('revision: 0', 'revision: 4'))),
            ('initial_poses와 겹침', dict(robot_ids='robot_2', poses='robot_2=0,0,0', start_poses=good)),
            ('health 꺼짐', dict(robot_ids='robot_2', start_poses=good, health='false')),
        ]

        for name, arguments in cases:
            with self.subTest(case=name):
                with self.assertRaises(ValueError):
                    self.nodes(**arguments)

    # 노드마다 configure 이벤트와 inactive→activate, active 안내 처리기를 하나씩 둔다.
    def test_lifecycle_events(self):
        from launch.actions import EmitEvent, RegisterEventHandler

        original = LAUNCH.running_conflicts
        original_share = LAUNCH.get_package_share_directory
        LAUNCH.running_conflicts = lambda ids: []
        LAUNCH.get_package_share_directory = lambda name: str(PACKAGE)

        try:
            from launch import LaunchContext

            with tempfile.TemporaryDirectory() as folder:
                map_path = Path(folder) / 'map.yaml'
                map_path.write_text('image: map.pgm\n')
                context = LaunchContext()
                context.launch_configurations.update({
                    'map': str(map_path), 'robot_ids': 'robot_1,robot_2', 'scan': 'filtered', 'initial_poses': '',
                    'startup_timeout': '60.0', 'startup_report': '', 'start_poses': '', 'health': 'true',
                    'health_config': ''})
                actions = LAUNCH.setup(context)
        finally:
            LAUNCH.running_conflicts = original
            LAUNCH.get_package_share_directory = original_share

        from launch.actions import TimerAction

        # 노드마다 configure 이벤트 1개, 전이·프로세스 종료 처리기 2개, 전체 제한시간 타이머 1개
        self.assertEqual(sum(isinstance(action, EmitEvent) for action in actions), 3)
        self.assertEqual(sum(isinstance(action, RegisterEventHandler) for action in actions), 6)
        self.assertEqual([action.period for action in actions if isinstance(action, TimerAction)], [60.0])


    # 로봇마다 프레임·입력·초기 위치가 분리되고 공유하는 것은 /map뿐인지 확인한다.
    def test_robot_separation(self):
        described = [self.describe(node) for node in self.nodes('robot_1,robot_2', 'robot_2=0,0,0')]
        amcls = {item[1]: item[3] for item in described if item[0] == 'nav2_amcl'}
        one, two = amcls['/ax/robot_1'], amcls['/ax/robot_2']

        for key in ('odom_frame_id', 'base_frame_id', 'scan_topic'):
            self.assertNotEqual(one[key], two[key])

        self.assertEqual(one['map_topic'], two['map_topic'])
        self.assertFalse(one['set_initial_pose'])
        self.assertTrue(two['set_initial_pose'])

    # 이미 실행 중인 SLAM·AMCL·map_server가 있으면 시작하지 않는다.
    def test_refuses_when_running(self):
        with self.assertRaises(RuntimeError):
            self.nodes('robot_2', conflicts=['/ax/robot_2/slam_toolbox'])

    # 지도 파일이 없으면 시작하지 않는다.
    def test_missing_map(self):
        from launch import LaunchContext

        context = LaunchContext()
        context.launch_configurations.update({
            'map': '/nonexistent/map.yaml', 'robot_ids': 'robot_2', 'scan': 'filtered',
            'initial_poses': '', 'startup_timeout': '60.0', 'startup_report': '', 'start_poses': '',
            'health': 'true', 'health_config': ''})

        with self.assertRaises(FileNotFoundError):
            LAUNCH.setup(context)



@unittest.skipIf(LAUNCH is None, 'ROS launch 패키지 필요(source install/setup.bash)')
class StartupMonitorLogic(unittest.TestCase):

    # 두 노드를 감시하는 기록기
    def monitor(self, report=None):
        return LAUNCH.StartupMonitor(['/map_server', '/ax/robot_2/amcl'], 30.0, report)

    # configure 성공이면 activate 동작을 돌려주고, 모두 active면 status active다.
    def test_success_sequence(self):
        monitor = self.monitor()

        self.assertEqual(monitor.on_transition('/map_server', 'configuring', 'inactive', 'ACT'), ['ACT'])
        self.assertEqual(monitor.nodes['/map_server']['stage'], 'activate')
        monitor.on_transition('/map_server', 'activating', 'active', 'ACT')
        self.assertEqual(monitor.status, 'starting')
        monitor.on_transition('/ax/robot_2/amcl', 'configuring', 'inactive', 'ACT')
        monitor.on_transition('/ax/robot_2/amcl', 'activating', 'active', 'ACT')
        self.assertEqual(monitor.status, 'active')
        self.assertEqual(monitor.on_timeout(), [])

    # 거절·실패·오류 전이와 시작 중 종료, 제한시간 초과는 StartupFailure이고 기록을 남긴다.
    def test_failures(self):
        cases = [
            (lambda m: m.on_transition('/ax/robot_2/amcl', 'configuring', 'unconfigured', 'ACT'), 'configure'),
            (lambda m: m.on_transition('/ax/robot_2/amcl', 'activating', 'inactive', 'ACT'), 'activate'),
            (lambda m: m.on_transition('/ax/robot_2/amcl', 'configuring', 'errorprocessing', 'ACT'), 'configure'),
            (lambda m: m.on_exit('/ax/robot_2/amcl', -9), 'configure'),
            (lambda m: m.on_timeout(), 'timeout'),
        ]

        for call, stage in cases:
            with self.subTest(stage=stage):
                with tempfile.TemporaryDirectory() as folder:
                    report = Path(folder) / 'startup.json'
                    monitor = self.monitor(report)

                    with self.assertRaises(LAUNCH.StartupFailure):
                        call(monitor)

                    data = json.loads(report.read_text(encoding='utf-8'))

                self.assertEqual(data['status'], 'failed')
                self.assertEqual(data['failure']['stage'], stage)
                self.assertIn('/ax/robot_2/amcl', data['failure']['node'])

    # 실패가 확정된 뒤의 사건(정리 중 종료 등)은 다시 실패로 만들지 않는다.
    def test_after_failure_and_shutdown(self):
        monitor = self.monitor()

        with self.assertRaises(LAUNCH.StartupFailure):
            monitor.on_timeout()

        self.assertEqual(monitor.on_exit('/map_server', -2), [])
        self.assertEqual(monitor.on_exit('/ax/robot_2/amcl', -2, shutting_down=True), [])
        self.assertEqual(monitor.nodes['/map_server']['exit_code'], -2)

    # 활성화 완료 후 노드 종료는 기록만 한다: 예외 없음, status active·failure 없음 유지, exit_code 기록·보고 갱신.
    def test_exit_after_active(self):
        with tempfile.TemporaryDirectory() as folder:
            report = Path(folder) / 'startup.json'
            monitor = self.monitor(report)

            for label in monitor.nodes:
                monitor.on_transition(label, 'configuring', 'inactive', 'ACT')
                monitor.on_transition(label, 'activating', 'active', 'ACT')

            actions = monitor.on_exit('/ax/robot_2/amcl', 4)
            data = json.loads(report.read_text(encoding='utf-8'))

        self.assertEqual(monitor.status, 'active')
        self.assertIsNone(monitor.failure)
        self.assertEqual(len(actions), 1)
        self.assertEqual(data['status'], 'active')
        self.assertIsNone(data['failure'])
        self.assertEqual(data['nodes']['/ax/robot_2/amcl']['exit_code'], 4)

    # 시작 중 노드 종료는 실패 처리다(활성화 완료 후 종료와 구분).
    def test_exit_during_startup_is_failure(self):
        monitor = self.monitor()
        monitor.on_transition('/map_server', 'configuring', 'inactive', 'ACT')
        monitor.on_transition('/map_server', 'activating', 'active', 'ACT')

        with self.assertRaises(LAUNCH.StartupFailure):
            monitor.on_exit('/ax/robot_2/amcl', 3)

        self.assertEqual(monitor.status, 'failed')
        self.assertIn('시작 중 프로세스 종료(종료 코드 3)', monitor.failure['reason'])


# 평가 계산(check_localization.py)을 ROS 없이 검사한다.
def load_check():
    import sys

    sys.path.insert(0, str(PACKAGE / 'scripts'))
    spec = importlib.util.spec_from_file_location('check_localization', PACKAGE / 'scripts/check_localization.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


CHECK = load_check()


class LocalizationErrors(unittest.TestCase):

    # world에서 (2, -1)만큼 옮긴 map, 직진하는 로봇. AMCL map→odom이 정확하면 오차 0이다.
    def setUp(self):
        import numpy as np

        self.np = np
        self.world_map = CHECK.se2(2.0, -1.0, 0.0)
        times = np.arange(0.0, 5.01, 0.1)
        self.odom = [(t, 0.2 * t, 0.0, 0.0, 0.2, 0.0) for t in times]
        self.truth = [(t, 2.0 + 0.2 * t, -1.0, 0.0) for t in times]

    # 정확한 map→odom이면 위치·방향 오차가 0이다.
    def test_exact_estimate(self):
        errors = CHECK.pose_errors([(0.0, 0.0, 0.0, 0.0)], self.odom, self.truth, self.world_map)

        self.assertGreater(errors['samples'], 5)
        self.assertAlmostEqual(errors['position_error_m']['max'], 0.0, places=9)
        self.assertAlmostEqual(errors['yaw_error_rad']['max'], 0.0, places=9)

    # map→odom이 0.3 m 어긋나면 그만큼 위치 오차가 난다. 유효 시각 이전 표본만 쓴다.
    def test_offset_and_validity_time(self):
        map_odom = [(0.0, 0.0, 0.0, 0.0), (3.0, 0.3, 0.0, 0.0)]
        errors = CHECK.pose_errors(map_odom, self.odom, self.truth, self.world_map)
        rows = self.np.array(errors['series']['rows'])

        self.assertAlmostEqual(rows[rows[:, 0] < 3.0, 7].max(), 0.0, places=9)
        self.assertAlmostEqual(rows[rows[:, 0] >= 3.0, 7].min(), 0.3, places=9)
        self.assertAlmostEqual(errors['final']['position_error_m'], 0.3, places=9)

    # map→odom이나 실제 자세가 없으면 계산하지 않는다.
    def test_no_samples(self):
        self.assertIsNone(CHECK.pose_errors([], self.odom, self.truth, self.world_map))
        self.assertIsNone(CHECK.pose_errors([(0.0, 0.0, 0.0, 0.0)], self.odom, [], self.world_map))

    # 한 로봇의 초기 위치 입력 뒤 그 로봇만 map→odom이 바뀌었는지 로봇별로 따로 센다.
    def test_initial_pose_effects(self):
        records = {
            'robot_1': {'initialpose': [[10.0, -2.0, 1.8, 0.0, 0.5, 'map']],
                        'map_odom': [(9.0, 0.0, 0.0, 0.0), (10.5, 0.5, 0.3, 0.0)],
                        'amcl_pose': [[10.5, -2.0, 1.8, 0.0, 0.5, 'map']]},
            'robot_2': {'initialpose': [],
                        'map_odom': [(9.0, 0.0, 0.0, 0.0), (10.5, 0.0, 0.0, 0.0)],
                        'amcl_pose': []},
        }
        effects = CHECK.initial_pose_effects(records)

        self.assertEqual(len(effects), 1)
        self.assertEqual(effects[0]['input_robot'], 'robot_1')
        self.assertAlmostEqual(effects[0]['robots']['robot_1']['map_odom_change']['max_position_change_m'],
                               math.hypot(0.5, 0.3))
        self.assertEqual(effects[0]['robots']['robot_2']['map_odom_change']['max_position_change_m'], 0.0)
        self.assertEqual(effects[0]['robots']['robot_2']['amcl_pose_messages'], 0)

    # 수신 주기 요약
    def test_rate_summary(self):
        self.assertEqual(CHECK.rate_summary([1.0])['rate_hz'], None)
        summary = CHECK.rate_summary([0.0, 0.1, 0.2, 0.5])

        self.assertAlmostEqual(summary['rate_hz'], 6.0)
        self.assertAlmostEqual(summary['max_gap_s'], 0.3)

    # 지도 검증 기록에서 정렬 보정값(optimal)을 우선 읽는다.
    def test_world_map_from_accuracy(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'accuracy.json'
            path.write_text('{"transforms": {"initial_T_world_map": {"x_m": 2.5, "y_m": -1.5, "yaw_rad": 0.0},'
                            ' "optimal_T_world_map": {"x_m": 2.49, "y_m": -1.49, "yaw_rad": 0.001}}}')

            values, source = CHECK.world_map_from_accuracy(path)

        self.assertEqual(values, [2.49, -1.49, 0.001])
        self.assertTrue(source.endswith('optimal_T_world_map'))



# 평가 결과의 검증 가능 여부(status)와 로봇별 누락 사유를 검사한다.
class InputCompleteness(unittest.TestCase):

    # 정상 입력 한 로봇분 기록(직진, 정확한 map→odom)과 그 실제 자세
    @staticmethod
    def good_record():
        times = [i * 0.1 for i in range(51)]

        return ({'map_odom': [(0.0, 0.0, 0.0, 0.0)],
                 'odom': [(t, 0.2 * t, 0.0, 0.0, 0.2, 0.0) for t in times],
                 'amcl_pose': [[0.0, 0.0, 0.0, 0.0, 0.1, 'map']], 'initialpose': [],
                 'scan': [(t, 'robot_2/front_lidar_link') for t in times]},
                CHECK.np.array([(t, 0.2 * t, 0.0, 0.0) for t in times]))

    # 모든 연결이 된 TF 조회 결과
    @staticmethod
    def good_chain(robot_id):
        return {name: {'ok': True, 'x': 0.0, 'y': 0.0} for name in
                (f'map->{robot_id}/odom', f'{robot_id}/odom->{robot_id}/base_link', f'map->{robot_id}/base_link')}

    # 기록·실제 자세·TF 조회 결과로 summarize → finalize 결과를 만든다.
    def result(self, records, truth, chains, start=0.0, end=5.0, duration=5.0):
        import argparse

        arguments = argparse.Namespace(scan='filtered', duration=duration, world_map=[0.0, 0.0, 0.0],
                                       world_map_source='test', tf_future=1.0, label='test')
        result = CHECK.summarize(records, truth, CHECK.se2(0.0, 0.0, 0.0), chains, [], [], arguments)

        return CHECK.finalize(result, start, end, duration, 900.0)

    # 정상 입력이면 completed, 판정은 not_judged
    def test_complete(self):
        record, truth = self.good_record()
        result = self.result({'robot_2': record}, {'robot_2': truth}, {'robot_2': self.good_chain('robot_2')})

        self.assertEqual(result['status'], 'completed', result['problems'])
        self.assertTrue(result['verifiable'])
        self.assertTrue(result['recording']['duration_met'])
        self.assertEqual(result['problems'], [])
        self.assertEqual(result['judgment'], 'not_judged')

    # 입력이 전혀 없으면 incomplete이고 로봇별 사유가 모두 남는다.
    def test_no_input(self):
        empty = {key: [] for key in ('map_odom', 'odom', 'amcl_pose', 'initialpose', 'scan')}
        chain = {name: {'ok': False, 'error': 'LookupException'} for name in self.good_chain('robot_2')}
        result = self.result({'robot_2': empty}, {}, {'robot_2': chain})
        problems = result['robots']['robot_2']['problems']

        self.assertEqual(result['status'], 'incomplete')
        self.assertEqual(result['judgment'], 'not_judged')

        for text in ('스캔 수신 없음', 'odom 수신 없음', 'map→odom 수신 없음', 'Gazebo 실제 자세 표본 없음',
                     '오차 계산 표본 없음', 'TF 연결 실패 map->robot_2/odom(LookupException)'):
            self.assertIn(text, problems)

    # 한 로봇만 누락돼도 전체가 incomplete이고, 정상 로봇의 계산 결과는 남는다.
    def test_one_robot_missing(self):
        record, truth = self.good_record()
        missing = dict(record, map_odom=[])
        result = self.result({'robot_1': missing, 'robot_2': record}, {'robot_1': truth, 'robot_2': truth},
                             {'robot_1': self.good_chain('robot_1'), 'robot_2': self.good_chain('robot_2')})

        self.assertEqual(result['status'], 'incomplete')
        self.assertFalse(result['robots']['robot_1']['verifiable'])
        self.assertTrue(result['robots']['robot_2']['verifiable'])
        self.assertIsNotNone(result['robots']['robot_2']['errors'])
        self.assertIn('robot_1: map→odom 수신 없음', result['problems'])
        self.assertFalse([problem for problem in result['problems'] if problem.startswith('robot_2')])

    # TF 연결 하나라도 실패하면 incomplete
    def test_tf_chain_failure(self):
        record, truth = self.good_record()
        chain = self.good_chain('robot_2')
        chain['robot_2/odom->robot_2/base_link'] = {'ok': False, 'error': 'ExtrapolationException'}
        result = self.result({'robot_2': record}, {'robot_2': truth}, {'robot_2': chain})

        self.assertEqual(result['status'], 'incomplete')
        self.assertEqual(result['robots']['robot_2']['problems'],
                         ['TF 연결 실패 robot_2/odom->robot_2/base_link(ExtrapolationException)'])

    # 실제 자세가 있어도 시각이 겹치지 않으면 오차 표본이 없어 incomplete
    def test_no_error_samples(self):
        record, truth = self.good_record()
        late_truth = truth.copy()
        late_truth[:, 0] += 100.0
        result = self.result({'robot_2': record}, {'robot_2': late_truth}, {'robot_2': self.good_chain('robot_2')})

        self.assertEqual(result['status'], 'incomplete')
        self.assertEqual(result['robots']['robot_2']['problems'], ['오차 계산 표본 없음'])

    # 기록 시간을 못 채우면 입력이 있어도 incomplete. 검증 가능 여부는 따로 남는다.
    def test_short_recording(self):
        record, truth = self.good_record()
        result = self.result({'robot_2': record}, {'robot_2': truth}, {'robot_2': self.good_chain('robot_2')},
                             end=3.0)

        self.assertEqual(result['status'], 'incomplete')
        self.assertTrue(result['verifiable'])
        self.assertFalse(result['recording']['duration_met'])
        self.assertTrue(result['problems'][0].startswith('기록 시간 부족'))


if __name__ == '__main__':
    unittest.main()
