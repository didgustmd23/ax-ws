# B2 위치 신뢰성 원시 기록·분석(check_localization.py)과 신뢰 기록 진단 사유(localization_health.py)를 ROS 없이 검사한다.
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'src/ax_localization/scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import localization_health as lh  # noqa: E402
import ros_test_domain  # noqa: E402

spec = importlib.util.spec_from_file_location('check_localization', SCRIPTS / 'check_localization.py')
CHECK = importlib.util.module_from_spec(spec)
spec.loader.exec_module(CHECK)


# 원시 기록 파일 형식·로봇 분리·중단 보존
class RawLogFormat(unittest.TestCase):

    # 로봇·자료별 파일에 줄 단위로 쓰고, 닫기 전에도 이미 쓴 줄을 다시 읽을 수 있다(flush).
    def test_lines_readable_before_close(self):
        with tempfile.TemporaryDirectory() as folder:
            raw = CHECK.RawLog(folder)
            raw.write('robot_1', 'odom', {'stamp': 1.0, 'x': 0.5})
            raw.write('robot_2', 'odom', {'stamp': 1.0, 'x': 9.0})
            raw.write(None, 'tf_publishers', {'publishers': ['/a']})

            rows, broken = CHECK.read_jsonl(Path(folder) / 'raw/robot_1/odom.jsonl')

            self.assertEqual((rows, broken), ([{'stamp': 1.0, 'x': 0.5}], 0))
            self.assertEqual(CHECK.read_jsonl(Path(folder) / 'raw/robot_2/odom.jsonl')[0][0]['x'], 9.0)
            self.assertTrue((Path(folder) / 'raw/common/tf_publishers.jsonl').is_file())
            raw.close()

    # 비유한 값은 null로 쓰고(JSON 표준 유지), 닫은 뒤 기록은 버린다.
    def test_non_finite_and_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            raw = CHECK.RawLog(folder)
            raw.write('robot_1', 'health', {'position_std_m': float('nan'), 'yaw_std_rad': float('inf')})
            raw.close()
            raw.write('robot_1', 'health', {'position_std_m': 0.1})
            text = (Path(folder) / 'raw/robot_1/health.jsonl').read_text()

            self.assertEqual(text.count('\n'), 1)
            self.assertIn('"position_std_m": null', text)
            self.assertNotIn('NaN', text)

    # 마지막 줄이 쓰다 잘린 파일은 앞 줄을 살리고 깨진 줄 수를 센다.
    def test_truncated_last_line(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'raw/robot_2/amcl_pose.jsonl'
            path.parent.mkdir(parents=True)
            path.write_text('{"stamp": 1.0}\n{"stamp": 2.0}\n{"stamp": 3.')

            rows, broken = CHECK.read_jsonl(path)

            self.assertEqual([row['stamp'] for row in rows], [1.0, 2.0])
            self.assertEqual(broken, 1)

    # 공분산 36개를 모두 남기고 길이가 다르면 표시한다.
    def test_covariance_serialization(self):
        covariance = [float(index) for index in range(36)]
        stored = CHECK.covariance_list(covariance)

        self.assertEqual(stored, {'values': covariance, 'complete': True})
        self.assertFalse(CHECK.covariance_list([1.0, 2.0])['complete'])
        self.assertEqual(CHECK.covariance_std(lh.pose_covariance(0.2, 0.1)), (0.2, 0.1))
        self.assertEqual(CHECK.covariance_std([0.0] * 5), (None, None))

    # 로봇별로 빠진 자료를 missing에 남긴다.
    def test_inventory_missing(self):
        with tempfile.TemporaryDirectory() as folder:
            raw = CHECK.RawLog(folder)

            for stream in CHECK.EXPECTED_STREAMS:
                raw.write('robot_2', stream, {'stamp': 1.0})

            raw.write('robot_1', 'odom', {'stamp': 1.0})
            raw.close()
            inventory = CHECK.raw_inventory(folder, ['robot_1', 'robot_2'])

            self.assertNotIn('robot_2', inventory['missing'])
            self.assertIn('health', inventory['missing']['robot_1'])
            self.assertNotIn('odom', inventory['missing']['robot_1'])
            self.assertEqual(inventory['files']['robot_1/odom.jsonl']['lines'], 1)

    # 조건 파일: 시작에 쓰며 git·해시·환경·같은 순간의 벽시계/단조 시각이 있고 원자적으로 바뀐다.
    def test_conditions(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / 'a.yaml'
            target.write_text('x: 1\n')
            conditions = CHECK.collect_conditions(ROOT, {'a': target, 'none': Path(folder) / 'missing'}, None,
                                                  ['robot_2'], {'duration': 5.0},
                                                  environment={'ROS_DOMAIN_ID': '42', 'GZ_PARTITION': 'p'})
            CHECK.write_json_atomic(Path(folder) / 'conditions.json', conditions)
            stored = json.loads((Path(folder) / 'conditions.json').read_text())

            self.assertEqual(stored['environment']['ROS_DOMAIN_ID'], '42')
            self.assertEqual(len(stored['files_sha256']['a']['sha256']), 64)
            self.assertIsNone(stored['files_sha256']['none']['sha256'])
            self.assertIn('monotonic_s', stored['time_reference'])
            self.assertIsNotNone(stored['git']['commit'])
            self.assertFalse((Path(folder) / 'conditions.json.tmp').exists())


# 분석 계산
class RawAnalysis(unittest.TestCase):

    # TF stamp 정리: 같은 stamp·같은 값은 중복, 같은 stamp·다른 값은 충돌, 작은 stamp는 역행으로 세고 뺀다.
    # 값이 같은 재발행과 바뀐 갱신을 나눈다.
    def test_map_odom_steps(self):
        tf_rows = [
            {'stamp': 1.0, 'valid_time': 0.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            {'stamp': 1.1, 'valid_time': 0.1, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},     # 같은 값 재발행
            {'stamp': 1.1, 'valid_time': 0.1, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},     # 중복 stamp(같은 값)
            {'stamp': 1.1, 'valid_time': 0.1, 'x': 5.0, 'y': 0.0, 'yaw': 0.0},     # 충돌 stamp(다른 값)
            {'stamp': 0.9, 'valid_time': -0.1, 'x': 5.0, 'y': 0.0, 'yaw': 0.0},    # 역행
            {'stamp': 1.2, 'valid_time': 0.2, 'x': 0.03, 'y': 0.04, 'yaw': 0.1},   # 갱신
        ]
        odom = [{'stamp': 0.15, 'x': 1.0, 'y': 0.0, 'yaw': 0.0}]
        steps = CHECK.map_odom_steps(tf_rows, odom)

        self.assertEqual((steps['received'], steps['accepted'], steps['duplicate_stamp'], steps['conflicting_stamp'],
                          steps['reversed_stamp'], steps['repeated_value']), (6, 3, 1, 1, 1, 1))
        self.assertEqual(steps['conflicts'][0]['other'], [5.0, 0.0, 0.0])
        self.assertAlmostEqual(steps['changed'][0]['odom_age_s'], 0.05)
        self.assertEqual(len(steps['changed']), 1)
        change = steps['changed'][0]
        self.assertAlmostEqual(change['map_odom_m'], 0.05)
        self.assertAlmostEqual(change['map_odom_rad'], 0.1)
        # 같은 odom (1, 0)에서 본 추정 위치 차이: 회전 0.1 rad로 odom 1 m 앞 점이 더 움직인다.
        expected = math.hypot(0.03 + math.cos(0.1) - 1.0, 0.04 + math.sin(0.1))
        self.assertAlmostEqual(change['estimate_m'], expected)

    # 추정 변화에 쓸 odom이 없거나 나이 상한을 넘으면 대응 불가로 세고 값은 비운다.
    def test_estimate_odom_age_limit(self):
        tf_rows = [{'stamp': 1.0, 'valid_time': 0.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                   {'stamp': 2.0, 'valid_time': 1.0, 'x': 0.1, 'y': 0.0, 'yaw': 0.0},
                   {'stamp': 3.0, 'valid_time': 2.0, 'x': 0.2, 'y': 0.0, 'yaw': 0.0}]
        odom = [{'stamp': 0.95, 'x': 0.0, 'y': 0.0, 'yaw': 0.0}]
        steps = CHECK.map_odom_steps(tf_rows, odom, max_odom_age_s=0.1)

        self.assertEqual(steps['estimate_unmatched_odom'], 1)
        self.assertAlmostEqual(steps['changed'][0]['estimate_m'], 0.1)
        self.assertIsNone(steps['changed'][1]['estimate_m'])
        self.assertAlmostEqual(steps['changed'][1]['odom_age_s'], 1.05)

    # 기록 품질: 기준이 없으면 미확인. 기준은 stamp 집합으로 대조해 중복이 누락을 상쇄하지 않고,
    # 수집 구간 밖 기준 사건은 따로 센다. 구독 준비는 증명할 자료가 없어 unknown이다.
    def test_record_quality(self):
        rows = [{'stamp': 0.2 * i, 'recv_mono': 10.0 + 0.2 * i} for i in (0, 1, 3, 3, 4)]   # 0.6 누락, 0.6 중복
        unverified = CHECK.record_quality(rows, None, 'raw/robot_2/odom.jsonl')

        self.assertEqual((unverified['observed'], unverified['observed_duplicate_stamps'], unverified['loss'],
                          unverified['subscriber_ready']), (5, 1, 'unverified', 'unknown'))
        self.assertAlmostEqual(unverified['max_recv_gap_mono_s'], 0.4)

        reference = [(0.2 * i, 10.0 + 0.2 * i) for i in range(5)] + [(9.0, 99.0)]           # 마지막은 구간 밖
        checked = CHECK.record_quality(rows, reference, 'raw/robot_2/health.jsonl', 'tick', window=[9.0, 20.0])
        window = checked['collection_window']

        self.assertEqual((checked['reference_in_window'], checked['reference_out_of_window']), (5, 1))
        self.assertEqual((window['matched'], window['missing_in_record'], window['loss']), (4, 1, 'confirmed'))
        self.assertEqual(checked['observed_interval']['missing_in_record'], 1)
        self.assertIn('DDS', checked['loss_meaning'])

    # 회차 품질: 수집 구간(시작 mono~끝)과 health 진단 전체·구간 대조를 나누고, initialpose는 개수만 비교한다.
    def test_session_record_quality(self):
        with tempfile.TemporaryDirectory() as folder:
            CHECK.write_json_atomic(Path(folder) / 'conditions.json', {'time_reference': {'monotonic_s': 5.0}})
            raw = CHECK.RawLog(folder)
            raw.write('robot_2', 'health', {'stamp': 1.0, 'recv_mono': 6.0})
            raw.write('robot_2', 'odom', {'stamp': 1.0, 'recv_mono': 7.0})
            raw.write('robot_2', 'initialpose', {'stamp': 0.0, 'recv_mono': 6.5})
            raw.close()
            path = Path(folder) / 'supervisor/robot_2_supervisor.jsonl'
            path.parent.mkdir()
            path.write_text('\n'.join(json.dumps(row) for row in (
                {'kind': 'tick', 'sim': 0.5, 'mono': 4.0}, {'kind': 'tick', 'sim': 1.0, 'mono': 6.0},
                {'kind': 'tick', 'sim': 1.2, 'mono': 6.8}, {'kind': 'initialpose_sent', 'sim': 0.0, 'mono': 4.5},
                {'kind': 'initialpose_sent', 'sim': 3.0, 'mono': 6.4})) + '\n')
            quality = CHECK.session_record_quality(folder, ['robot_2'])
            robot = quality['robots']['robot_2']

            self.assertEqual((quality['collection_window_mono']['start'], quality['collection_window_mono']['end']),
                             (5.0, 7.0))
            self.assertEqual(robot['health']['all_ticks']['missing_in_record'], 2)
            self.assertEqual(robot['health']['reference_out_of_window'], 1)
            self.assertEqual(robot['health']['collection_window']['missing_in_record'], 1)
            self.assertEqual((robot['initialpose']['coverage'], robot['initialpose']['reference_in_window'],
                              robot['initialpose']['unmatched_count'], robot['initialpose']['loss']),
                             ('count_only', 1, 0, 'count_equal_unproven'))
            self.assertEqual(robot['odom']['loss'], 'unverified')

    # TF·amcl_pose 대응은 허용 안의 짝만 세고 무대응을 따로 남긴다.
    def test_pairing(self):
        tf_rows = [{'valid_time': t} for t in (1.0, 1.1, 1.2)]
        amcl_rows = [{'stamp': 1.0}, {'stamp': 1.1004}, {'stamp': 1.5}]
        pairing = CHECK.pair_tf_amcl(tf_rows, amcl_rows, tolerance=0.001)

        self.assertEqual((pairing['matched_amcl'], pairing['unmatched_amcl'], pairing['unmatched_tf']), (2, 1, 1))
        self.assertEqual(pairing['cause_of_unmatched'], '미확정')

    # health 시간 비중: 다음 표본까지를 그 상태로 세고 긴 간격은 뺀다.
    def test_health_shares(self):
        rows = [{'stamp': 0.0, 'state': 'OK', 'reason': ''}, {'stamp': 0.2, 'state': 'DEGRADED',
                                                               'reason': 'COVARIANCE_HIGH'},
                {'stamp': 0.6, 'state': 'OK', 'reason': ''}, {'stamp': 5.0, 'state': 'OK', 'reason': ''}]
        shares = CHECK.health_time_shares(rows, max_gap_s=1.0)

        self.assertAlmostEqual(shares['state_s']['OK'], 0.2)
        self.assertAlmostEqual(shares['state_s']['DEGRADED'], 0.4)
        self.assertAlmostEqual(shares['reason_s']['COVARIANCE_HIGH'], 0.4)
        self.assertEqual(len(shares['excluded_gaps_s']), 1)

    # 분포 요약은 표본 수·p99를 포함하고 빈 목록은 samples 0
    def test_distribution(self):
        self.assertEqual(CHECK.distribution([None]), {'samples': 0})
        summary = CHECK.distribution([1.0, -2.0, 3.0])
        self.assertEqual(summary['samples'], 3)
        self.assertEqual(summary['max'], 3.0)
        self.assertIn('p99', summary)

    # 오차 표본과 공분산은 그 시각 이전 1 s 안의 amcl_pose와만 묶는다.
    def test_error_with_covariance(self):
        series = [[1.0, 0, 0, 0, 0, 0, 0, 0.05, 0.01], [5.0, 0, 0, 0, 0, 0, 0, 0.1, -0.02]]
        amcl = [{'stamp': 0.8, 'covariance': CHECK.covariance_list(lh.pose_covariance(0.2, 0.1))}]
        joined = CHECK.error_with_covariance(series, amcl, max_age_s=1.0)

        self.assertEqual((joined['joined'], joined['unmatched']), (1, 1))
        self.assertAlmostEqual(joined['rows'][0]['position_std_m'], 0.2)

    # 감독 진단 요약: 판단별 횟수와 재위치 요청·보낸 initialpose를 모은다.
    def test_trust_summary(self):
        rows = [{'kind': 'tick', 'trust_stored': True, 'trust_rejection': None, 'trust_record_age_s': 0.0,
                 'trust_path_m': 0.0, 'session': 's1'},
                {'kind': 'tick', 'trust_stored': False, 'trust_rejection': 'YAW_STD_HIGH', 'trust_record_age_s': 3.0,
                 'trust_path_m': 0.5, 'session': 's1'},
                {'kind': 'reinit_request', 'sim': 9.0, 'code': 'REINIT_ACCEPTED', 'accepted': True},
                {'kind': 'initialpose_sent', 'sim': 9.0, 'source': 'REINIT', 'pose': [0, 0, 0]}]
        summary = CHECK.trust_summary(rows)

        self.assertEqual(summary['decisions'], {'STORED': 1, 'YAW_STD_HIGH': 1})
        self.assertEqual(summary['reinit_requests'][0]['code'], 'REINIT_ACCEPTED')
        self.assertEqual(summary['initialpose_sent'][0]['source'], 'REINIT')
        self.assertEqual(summary['sessions'], ['s1'])

    # 회차 폴더 전체 분석: 로봇별로 나뉘고 빠진 자료가 inventory에 남는다.
    def test_analyze_session(self):
        with tempfile.TemporaryDirectory() as folder:
            CHECK.write_json_atomic(Path(folder) / 'conditions.json', {'robot_ids': ['robot_2']})
            raw = CHECK.RawLog(folder)

            for index in range(5):
                t = 1.0 + 0.1 * index
                raw.write('robot_2', 'tf_map_odom', {'stamp': t, 'valid_time': t - 1.0, 'x': 0.01 * index,
                                                     'y': 0.0, 'yaw': 0.0})
                raw.write('robot_2', 'amcl_pose', {'stamp': t - 1.0, 'covariance':
                                                   CHECK.covariance_list(lh.pose_covariance(0.1, 0.05))})
                raw.write('robot_2', 'odom', {'stamp': t - 1.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': 0.0, 'w': 0.0})

            raw.close()
            analysis = CHECK.analyze_session(folder)
            entry = analysis['robots']['robot_2']

            self.assertEqual(entry['map_odom_steps']['changed_count'], 4)
            self.assertAlmostEqual(entry['map_odom_steps']['map_odom_m']['max'], 0.01)
            self.assertEqual(entry['tf_amcl_pairing']['matched_amcl'], 5)
            self.assertIn('health', analysis['inventory']['missing']['robot_2'])
            self.assertEqual(analysis['judgment'], 'not_judged')


# 신뢰 기록 진단 사유는 갱신 판단(consider)과 항상 같다.
class TrustRejection(unittest.TestCase):

    # 조건별 첫 거절 사유와 consider 결과가 일치한다.
    def test_rejection_matches_consider(self):
        config = lh.merge_config()
        cases = [
            (lh.Health(lh.OK, True, '', 0.1, 0.05), 'map', False, None),
            (lh.Health(lh.OK, True, '', 0.1, 0.05), 'map', True, 'REINIT_ACTIVE'),
            (lh.Health(lh.DEGRADED, True, 'COVARIANCE_HIGH', 0.1, 0.2), 'map', False, 'HEALTH_NOT_OK'),
            (lh.Health(lh.OK, True, '', 0.1, 0.05), None, False, 'MAP_UNKNOWN'),
            (lh.Health(lh.OK, True, '', 0.2, 0.05), 'map', False, 'POSITION_STD_HIGH'),
            (lh.Health(lh.OK, True, '', 0.1, 0.12), 'map', False, 'YAW_STD_HIGH'),
            (lh.Health(lh.OK, True, '', float('nan'), 0.05), 'map', False, 'POSITION_STD_HIGH'),
        ]
        inputs = lh.Inputs(map_odom=(0.0, 0.0, 0.0), odom_pose=(1.0, 0.0, 0.0))

        for health, map_id, active, expected in cases:
            with self.subTest(expected=expected):
                tracker = lh.TrustedPoseTracker(config)
                self.assertEqual(tracker.rejection(health, inputs, map_id, active), expected)
                self.assertEqual(tracker.consider(health, inputs, 1.0, map_id, active), expected is None)

        tracker = lh.TrustedPoseTracker(config)
        bad = lh.Inputs(map_odom=(float('nan'), 0.0, 0.0), odom_pose=(1.0, 0.0, 0.0))
        self.assertEqual(tracker.rejection(cases[0][0], bad, 'map', False), 'POSE_INVALID')
        self.assertFalse(tracker.consider(cases[0][0], bad, 1.0, 'map', False))



# ROS 2와 ax_interfaces가 있는지
def ros_available():
    import shutil

    return (shutil.which('ros2') is not None and importlib.util.find_spec('rclpy') is not None
            and importlib.util.find_spec('ax_interfaces') is not None)


# 기록 노드가 health를 빠짐없이 받는지 실제 ROS 통신으로 확인한다(격리 domain, 같은 프로세스의 가짜 발행자).
# 발행은 감독 노드와 같은 STATE QoS(깊이 1·TRANSIENT_LOCAL·RELIABLE)이고 메시지마다 고유 stamp(연번)를 넣는다.
# 다른 자료 부하로 odom을 200 Hz로 함께 보낸다. Gazebo·감독 노드 통합 시험이 아니다.
@unittest.skipUnless(ros_available(), 'ROS 2와 ax_interfaces 필요(source install/setup.bash)')
class HealthStreamDelivery(unittest.TestCase):

    # 기록 노드를 띄우고 발행자를 만든다.
    def setUp(self):
        import os
        import threading

        domain, lock = ros_test_domain.reserve_domain(ros_test_domain.HEALTH_AND_SESSION_DOMAINS)
        self.addCleanup(ros_test_domain.release, lock)
        os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')

        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from ax_interfaces.msg import LocalizationHealth
        from nav_msgs.msg import Odometry

        self.folder = tempfile.TemporaryDirectory()
        self.rclpy = rclpy
        rclpy.init()
        self.raw = CHECK.RawLog(self.folder.name)
        self.recorder = CHECK.make_recorder(['robot_2'], 'filtered', 1.0, self.raw, None)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.recorder)
        self.spinner = threading.Thread(target=self.executor.spin, daemon=True)
        self.spinner.start()
        self.source = rclpy.create_node('health_stream_source')
        state = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.health = self.source.create_publisher(LocalizationHealth, '/ax/robot_2/localization/health', state)
        self.odom = self.source.create_publisher(Odometry, '/ax/robot_2/diff_drive_controller/odom', 50)
        self.Health, self.Odometry = LocalizationHealth, Odometry
        self.sequence = 0

        deadline = time.monotonic() + 10.0

        while self.health.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)

        self.assertGreater(self.health.get_subscription_count(), 0, '기록 노드 구독 연결 실패')

    # 노드를 정리한다.
    def tearDown(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.spinner.join(timeout=2.0)
        self.recorder.destroy_node()
        self.source.destroy_node()
        self.rclpy.try_shutdown()
        self.raw.close()
        self.folder.cleanup()

    # 고유 stamp(연번 × 1 ms)의 health 한 건을 보낸다.
    def send(self):
        self.sequence += 1
        message = self.Health()
        message.meta.header.stamp.sec = self.sequence // 1000
        message.meta.header.stamp.nanosec = (self.sequence % 1000) * 1_000_000
        message.meta.revision = self.sequence
        message.robot_id = 'robot_2'
        self.health.publish(message)

    # 기록 파일에 남은 health stamp(연번) 집합. 닫기 전에 읽어 줄마다 flush됐는지도 함께 본다.
    def recorded(self):
        rows = CHECK.read_jsonl(Path(self.folder.name) / 'raw/robot_2/health.jsonl')[0]
        return {row['revision'] for row in rows}

    # 받은 수가 기대에 닿거나 시간이 다 될 때까지 기다린다.
    def wait_for(self, expected, seconds=10.0):
        deadline = time.monotonic() + seconds

        while len(self.recorded()) < expected and time.monotonic() < deadline:
            time.sleep(0.05)

    # 짧은 버스트(깊이보다 적은 60건 연속)와 odom 부하 아래 지속 5 Hz(3 s)를 모두 받고, 닫기 전 파일에 남는다.
    def test_burst_and_sustained_5hz(self):
        import threading

        stop = threading.Event()

        def load():
            message = self.Odometry()

            while not stop.is_set():
                self.odom.publish(message)
                time.sleep(0.005)

        loader = threading.Thread(target=load, daemon=True)
        loader.start()

        try:
            for _ in range(60):
                self.send()

            self.wait_for(60)
            self.assertEqual(self.recorded(), set(range(1, 61)), '버스트 손실')

            for _ in range(15):
                self.send()
                time.sleep(0.2)

            self.wait_for(75)
        finally:
            stop.set()
            loader.join(timeout=2.0)

        before_close = self.recorded()
        self.assertEqual(before_close, set(range(1, 76)), f'지속 5 Hz 손실: {sorted(set(range(1, 76)) - before_close)}')
        self.raw.close()
        self.assertEqual(self.recorded(), before_close)
        self.assertEqual(CHECK.HEALTH_STREAM_DEPTH, 100)


if __name__ == '__main__':
    unittest.main()
