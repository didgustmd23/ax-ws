# B2 위치 신뢰성·고정 출발 위치·마지막 신뢰 위치·제한된 재위치 판단(localization_health.py)을 ROS 없이 검사한다.
import hashlib
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_localization/scripts'))

import localization_health as lh  # noqa: E402

CONFIG = lh.merge_config()


# 정상 입력(10 s 시각, 정지, 공분산 작음, 모든 입력 최신)
def good_inputs(now=10.0, odom=(1.0, 0.0, 0.0), map_odom=(0.5, 0.2, 0.0), std=0.05, yaw_std=0.03):
    return lh.Inputs(
        scan_stamp=now - 0.1, odom_stamp=now - 0.02, odom_pose=odom,
        tf_valid_time=now - 0.1, map_odom=map_odom, tf_chain_ok=True,
        amcl_stamp=now - 5.0, amcl_pose=lh.compose(map_odom, odom),
        amcl_covariance=tuple(lh.pose_covariance(std, yaw_std)), amcl_odom_pose=odom, amcl_updates=3,
        actuation_state=lh.ACTUATION_STOPPED, actuation_valid=True, actuation_stamp=now - 0.05)


# 위치 신뢰성 판정
class HealthEvaluation(unittest.TestCase):

    # 정상 입력이면 OK·valid·사유 없음, 현재 자세는 map→odom · odom
    def test_ok(self):
        health = lh.evaluate(good_inputs(), 10.0, CONFIG)

        self.assertEqual((health.state, health.valid, health.reason), (lh.OK, True, ''))
        self.assertAlmostEqual(health.position_std_m, 0.05)
        self.assertAlmostEqual(health.pose[0], 1.5)

    # 정지 중에는 AMCL 추정이 오래돼도(5 s 전) odom이 움직이지 않았으면 갱신 정체로 보지 않는다.
    def test_stationary_sparse_amcl(self):
        inputs = good_inputs()
        inputs.amcl_stamp = -100.0

        self.assertEqual(lh.evaluate(inputs, 10.0, CONFIG).state, lh.OK)

    # AMCL 갱신 없이 odom이 기준 이상 움직이면 ESTIMATE_STALE
    def test_moving_without_amcl_update(self):
        inputs = good_inputs(odom=(1.6, 0.0, 0.0))
        inputs.amcl_odom_pose = (1.0, 0.0, 0.0)

        health = lh.evaluate(inputs, 10.0, CONFIG)

        self.assertEqual(health.state, lh.DEGRADED)
        self.assertEqual(health.reason, 'ESTIMATE_STALE')

    # 단절: 스캔이 잠깐 늦으면 DEGRADED, 오래 끊기면 LOST(INPUT_LOST)
    def test_scan_disconnect(self):
        inputs = good_inputs()
        inputs.scan_stamp = 10.0 - 1.0
        self.assertEqual((lh.evaluate(inputs, 10.0, CONFIG).state, lh.evaluate(inputs, 10.0, CONFIG).reason),
                         (lh.DEGRADED, 'SCAN_STALE'))

        inputs.scan_stamp = 10.0 - 3.0
        health = lh.evaluate(inputs, 10.0, CONFIG)
        self.assertEqual((health.state, health.reason), (lh.LOST, 'INPUT_LOST'))

    # 만료: odom이 늦으면 ODOM_STALE, 받은 적이 없으면 INPUT_LOST
    def test_odom_expired(self):
        inputs = good_inputs()
        inputs.odom_stamp = 10.0 - 0.5
        self.assertEqual(lh.evaluate(inputs, 10.0, CONFIG).reason, 'ODOM_STALE')

        inputs.odom_stamp = None
        self.assertEqual(lh.evaluate(inputs, 10.0, CONFIG).state, lh.LOST)

    # TF: map→odom이 늦으면 TF_STALE, 오래되거나 map→base_link 조회가 안 되면 TF_MISSING
    def test_tf(self):
        inputs = good_inputs()
        inputs.tf_valid_time = 10.0 - 1.0
        self.assertEqual(lh.evaluate(inputs, 10.0, CONFIG).reason, 'TF_STALE')

        inputs.tf_valid_time = 10.0 - 3.0
        self.assertEqual(lh.evaluate(inputs, 10.0, CONFIG).reason, 'TF_MISSING')

        inputs = good_inputs()
        inputs.tf_chain_ok = False
        health = lh.evaluate(inputs, 10.0, CONFIG)
        self.assertEqual((health.state, health.reason), (lh.LOST, 'TF_MISSING'))

    # 비정상 수치: NaN·음수 분산·무한 자세는 ESTIMATE_INVALID(LOST)
    def test_invalid_numbers(self):
        for covariance in (lh.pose_covariance(float('nan'), 0.1), [0.0] * 35 + [-1.0], [0.1] * 10):
            inputs = good_inputs()
            inputs.amcl_covariance = tuple(covariance)

            with self.subTest(covariance=covariance[:2]):
                health = lh.evaluate(inputs, 10.0, CONFIG)
                self.assertEqual(health.state, lh.LOST)
                self.assertIn('ESTIMATE_INVALID', health.reason)

        inputs = good_inputs()
        inputs.map_odom = (float('inf'), 0.0, 0.0)
        self.assertIn('ESTIMATE_INVALID', lh.evaluate(inputs, 10.0, CONFIG).reason)

    # 공분산이 크면 DEGRADED, 매우 크면 LOST. 공분산이 작아도 다른 입력이 나쁘면 OK가 아니다.
    def test_covariance_levels(self):
        self.assertEqual(lh.evaluate(good_inputs(std=0.4), 10.0, CONFIG).reason, 'COVARIANCE_HIGH')
        self.assertEqual(lh.evaluate(good_inputs(std=1.5), 10.0, CONFIG).reason, 'COVARIANCE_LOST')

        inputs = good_inputs(std=0.01)
        inputs.scan_stamp = None
        self.assertNotEqual(lh.evaluate(inputs, 10.0, CONFIG).state, lh.OK)

    # 사유가 여럿이면 심각한 순으로 잇는다.
    def test_reason_order(self):
        inputs = good_inputs(std=0.4)
        inputs.tf_chain_ok = False
        inputs.odom_stamp = 10.0 - 0.5

        self.assertEqual(lh.evaluate(inputs, 10.0, CONFIG).reason, 'TF_MISSING,ODOM_STALE,COVARIANCE_HIGH')

    # 초기화 전이면 판정 불가(UNKNOWN, valid=false)
    def test_not_initialized(self):
        health = lh.evaluate(lh.Inputs(scan_stamp=9.9, odom_stamp=9.9), 10.0, CONFIG)

        self.assertEqual((health.state, health.valid, health.reason), (lh.UNKNOWN, False, 'NOT_INITIALIZED'))

    # 시계 이상: 역행은 계속 무효, 벽시계로 오래 멈추면 무효, 다시 흐르면 유효(역행 표시는 유지)
    def test_clock(self):
        clock = lh.ClockMonitor(2.0)

        self.assertEqual(clock.update(5.0, 100.0), (True, False))
        self.assertEqual(clock.update(5.5, 100.5), (True, False))
        self.assertEqual(clock.update(5.5, 103.0), (False, False))
        self.assertEqual(clock.update(6.0, 103.5), (True, False))
        self.assertEqual(clock.update(1.0, 104.0), (False, True))
        self.assertEqual(clock.update(1.5, 104.5)[0], False)

        health = lh.evaluate(good_inputs(), 10.0, CONFIG, clock_valid=False)
        self.assertEqual((health.state, health.valid, health.reason), (lh.UNKNOWN, False, 'CLOCK_INVALID'))


# 고정 출발 위치 설정
class StartPoses(unittest.TestCase):

    # 임시 지도(pgm)와 설정 파일을 만든다.
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp(prefix='ax_start_'))
        (self.folder / 'map.pgm').write_bytes(b'P5\n2 2\n255\n\x00\x00\xff\xff')
        (self.folder / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
        self.sha = hashlib.sha256((self.folder / 'map.pgm').read_bytes()).hexdigest()

    # 설정 파일을 쓰고 읽는다.
    def load(self, text, robots=('robot_2',)):
        path = self.folder / 'start.yaml'
        path.write_text(text)

        return lh.load_start_poses(path, self.folder / 'map.yaml', list(robots))

    # 정상 설정(개발 지도 revision 0, 해시 일치)
    def test_valid(self):
        result = self.load(f'map: {{revision: 0, image_sha256: {self.sha}}}\n'
                           'robots: {robot_2: {x: 1, y: -2, yaw: 0.5, position_std_m: 0.1, yaw_std_rad: 0.05}}\n')

        self.assertEqual(result['robots']['robot_2']['x'], 1.0)
        self.assertEqual(result['map_revision'], 0)

    # 채택 지도는 map_info.yaml의 map_revision과 맞아야 한다.
    def test_adopted_revision(self):
        (self.folder / 'map_info.yaml').write_text('map_revision: 3\n')
        good = f'map: {{revision: 3, image_sha256: {self.sha}}}\nrobots: {{robot_2: {{x: 0, y: 0, yaw: 0, position_std_m: 0.1, yaw_std_rad: 0.1}}}}\n'

        self.assertEqual(self.load(good)['map_revision'], 3)

        with self.assertRaisesRegex(ValueError, 'revision 불일치'):
            self.load(good.replace('revision: 3', 'revision: 2'))

    # 누락·비정상 값·지도 불일치는 거절한다.
    def test_rejections(self):
        pose = '{x: 0, y: 0, yaw: 0, position_std_m: 0.1, yaw_std_rad: 0.1}'
        cases = {
            'map 누락': f'robots: {{robot_2: {pose}}}\n',
            '로봇 누락': f'map: {{revision: 0, image_sha256: {self.sha}}}\nrobots: {{robot_1: {pose}}}\n',
            '키 누락': f'map: {{revision: 0, image_sha256: {self.sha}}}\nrobots: {{robot_2: {{x: 0, y: 0}}}}\n',
            '비유한 값': f'map: {{revision: 0, image_sha256: {self.sha}}}\nrobots: {{robot_2: {{x: .nan, y: 0, yaw: 0, position_std_m: 0.1, yaw_std_rad: 0.1}}}}\n',
            '문자 값': f'map: {{revision: 0, image_sha256: {self.sha}}}\nrobots: {{robot_2: {{x: a, y: 0, yaw: 0, position_std_m: 0.1, yaw_std_rad: 0.1}}}}\n',
            '표준편차 0': f'map: {{revision: 0, image_sha256: {self.sha}}}\nrobots: {{robot_2: {{x: 0, y: 0, yaw: 0, position_std_m: 0, yaw_std_rad: 0.1}}}}\n',
            'revision 불일치': f'map: {{revision: 1, image_sha256: {self.sha}}}\nrobots: {{robot_2: {pose}}}\n',
            '해시 불일치': f'map: {{revision: 0, image_sha256: {"0" * 64}}}\nrobots: {{robot_2: {pose}}}\n',
        }

        for name, text in cases.items():
            with self.subTest(case=name):
                with self.assertRaises(ValueError):
                    self.load(text)

    # 저장소의 개발용 출발 위치 설정은 형식이 맞고 두 로봇을 모두 가진다(지도 해시는 지도가 있을 때만 확인).
    def test_repository_dev_config(self):
        import yaml

        data = yaml.safe_load((ROOT / 'src/ax_localization/config/start_poses/aws_warehouse_dev.yaml').read_text())

        self.assertEqual(data['map']['revision'], 0)
        self.assertEqual(set(data['robots']), {'robot_1', 'robot_2'})

    # 설정 파일의 모르는 키·음수는 거절한다.
    def test_health_config(self):
        with self.assertRaises(ValueError):
            lh.merge_config({'health': {'unknown_key': 1}})

        with self.assertRaises(ValueError):
            lh.merge_config({'reinit': {'timeout_s': -1}})

        self.assertEqual(lh.merge_config({'reinit': {'timeout_s': 5}})['reinit']['timeout_s'], 5.0)


# 마지막 신뢰 위치와 odom 연속성
class TrustedPose(unittest.TestCase):

    # OK·작은 공분산이면 기록하고, 재위치 중·DEGRADED·공분산 큼이면 갱신하지 않는다.
    def test_update_and_hold(self):
        tracker = lh.TrustedPoseTracker(CONFIG)
        inputs = good_inputs()
        health = lh.evaluate(inputs, 10.0, CONFIG)

        self.assertTrue(tracker.consider(health, inputs, 10.0, 'map-a', reinit_active=False))
        kept = tracker.record

        moved = good_inputs(now=11.0, odom=(1.2, 0.0, 0.0))
        self.assertFalse(tracker.consider(lh.evaluate(moved, 11.0, CONFIG), moved, 11.0, 'map-a', reinit_active=True))
        self.assertFalse(tracker.consider(lh.Health(lh.DEGRADED, True, 'SCAN_STALE', 0.05, 0.03), moved, 11.0,
                                          'map-a', reinit_active=False))
        wide = good_inputs(now=11.0, std=0.2)
        self.assertFalse(tracker.consider(lh.evaluate(wide, 11.0, CONFIG), wide, 11.0, 'map-a', reinit_active=False))
        self.assertIs(tracker.record, kept)

    # 지도 변경·시계 역행·odom 끊김이면 기록을 무효로 하고 사유를 남긴다.
    def test_invalidation(self):
        for reason, action in (('MAP_CHANGED', lambda t: t.check_map('map-b')),
                               ('CLOCK_REVERSED', lambda t: t.invalidate('CLOCK_REVERSED')),
                               ('ODOM_JUMP', lambda t: t.invalidate('ODOM_JUMP'))):
            tracker = lh.TrustedPoseTracker(CONFIG)
            inputs = good_inputs()
            tracker.consider(lh.evaluate(inputs, 10.0, CONFIG), inputs, 10.0, 'map-a', False)
            action(tracker)

            with self.subTest(reason=reason):
                self.assertIsNone(tracker.record)
                self.assertEqual(tracker.invalid_reason, reason)

    # odom 연속성: 속도로 설명되는 이동은 정상, 도약·시각 역행·비유한 값은 끊김
    def test_odom_continuity(self):
        continuity = lh.OdomContinuity(CONFIG['odom_continuity'])

        self.assertIsNone(continuity.update(1.0, (0.0, 0.0, 0.0), 0.3, 0.0))
        self.assertIsNone(continuity.update(1.02, (0.006, 0.0, 0.0), 0.3, 0.0))
        self.assertEqual(continuity.update(1.04, (2.0, 0.0, 0.0), 0.3, 0.0), 'ODOM_JUMP')
        self.assertEqual(continuity.update(1.00, (2.0, 0.0, 0.0), 0.0, 0.0), 'ODOM_TIME_REVERSED')
        self.assertEqual(continuity.update(1.10, (float('nan'), 0.0, 0.0), 0.0, 0.0), 'ODOM_INVALID')

    # 두 로봇의 기록은 서로 영향을 주지 않는다.
    def test_two_robots(self):
        one, two = lh.TrustedPoseTracker(CONFIG), lh.TrustedPoseTracker(CONFIG)
        first = good_inputs(map_odom=(0.0, 0.0, 0.0))
        second = good_inputs(map_odom=(-2.5, 1.5, 0.0))
        one.consider(lh.evaluate(first, 10.0, CONFIG), first, 10.0, 'map-a', False)
        two.consider(lh.evaluate(second, 10.0, CONFIG), second, 10.0, 'map-a', False)
        one.invalidate('ODOM_JUMP')

        self.assertIsNone(one.record)
        self.assertEqual(two.record.map_odom, (-2.5, 1.5, 0.0))


# A4 구동 상태를 now 시각의 최신 STOPPED로 갱신한다(실제 A4는 20 Hz로 발행).
def fresh_actuation(inputs, now):
    inputs.actuation_stamp = now - 0.02


# 제한된 재위치
class Reinit(unittest.TestCase):

    # 10 s에 신뢰 기록을 만든 뒤, 단절로 LOST가 된 상태의 입력을 돌려준다.
    def prepared(self, travel=0.2):
        tracker = lh.TrustedPoseTracker(CONFIG)
        inputs = good_inputs()
        tracker.consider(lh.evaluate(inputs, 10.0, CONFIG), inputs, 10.0, 'map-a', False)
        later = good_inputs(now=20.0, odom=(1.0 + travel, 0.0, 0.0))
        later.tf_valid_time = 15.0
        reinit = lh.Reinitializer(CONFIG)
        health = lh.evaluate(later, 20.0, CONFIG)

        return tracker, later, reinit, health

    # 접수 → 전달 → 회복한 새 추정 3개·TF·스캔 최신 → SUCCEEDED. 진행 중에는 LOST·REINIT_IN_PROGRESS
    def test_success(self):
        tracker, inputs, reinit, health = self.prepared()

        self.assertEqual(health.state, lh.LOST)
        self.assertEqual(reinit.request(health, inputs, tracker, 20.0, 'map-a', True), (True, 'REINIT_ACCEPTED'))
        candidate = reinit.attempt.candidate['pose']
        self.assertAlmostEqual(candidate[0], 0.5 + 1.2)
        reinit.mark_sent(20.0)

        progress = reinit.apply(health)
        self.assertEqual((progress.state, progress.reason.split(',')[0]), (lh.LOST, 'REINIT_IN_PROGRESS'))

        for index in range(3):
            stamp = 20.2 + 0.2 * index
            reinit.on_amcl(stamp, candidate, lh.pose_covariance(0.08, 0.05))
            inputs.tf_valid_time = stamp
            inputs.scan_stamp = stamp
            fresh_actuation(inputs, stamp)
            result = reinit.step(inputs, stamp)

        self.assertEqual(result, 'SUCCEEDED')
        self.assertFalse(reinit.active())

    # 입력 pose가 같은 공분산으로 돌아오기만 하면 완료가 아니다(시간초과).
    def test_echo_is_not_recovery(self):
        tracker, inputs, reinit, health = self.prepared()
        reinit.request(health, inputs, tracker, 20.0, 'map-a', True)
        reinit.mark_sent(20.0)
        attempt = reinit.attempt.candidate

        for index in range(5):
            stamp = 20.2 + 0.2 * index
            reinit.on_amcl(stamp, attempt['pose'], lh.pose_covariance(attempt['position_std_m'], attempt['yaw_std_rad']))
            inputs.tf_valid_time = inputs.scan_stamp = stamp
            fresh_actuation(inputs, stamp)
            self.assertIsNone(reinit.step(inputs, stamp))

        inputs.scan_stamp = inputs.tf_valid_time = 36.0
        fresh_actuation(inputs, 36.0)
        self.assertEqual(reinit.step(inputs, 36.0), 'REINIT_FAILED_TIMEOUT')

    # 전달 이전 시각의 추정은 회복 근거가 아니다.
    def test_old_estimate_ignored(self):
        tracker, inputs, reinit, health = self.prepared()
        reinit.request(health, inputs, tracker, 20.0, 'map-a', True)
        reinit.mark_sent(20.0)
        reinit.on_amcl(19.9, reinit.attempt.candidate['pose'], lh.pose_covariance(0.05, 0.03))

        self.assertEqual(reinit.attempt.updates, [])

    # AMCL 무응답(새 추정 없음) → 시간초과에 AMCL_UNAVAILABLE. 실패는 OK 회복 전까지 유지
    def test_no_response(self):
        tracker, inputs, reinit, health = self.prepared()
        reinit.request(health, inputs, tracker, 20.0, 'map-a', True)
        reinit.mark_sent(20.0)

        fresh_actuation(inputs, 30.0)
        self.assertIsNone(reinit.step(inputs, 30.0))
        fresh_actuation(inputs, 35.5)
        self.assertEqual(reinit.step(inputs, 35.5), 'REINIT_FAILED_AMCL_UNAVAILABLE')
        self.assertIn('REINIT_FAILED_AMCL_UNAVAILABLE', reinit.apply(health).reason)

        # 입력이 정상으로 보여도(OK 판정) 실패 뒤 AMCL 회복 근거가 없으면 LOST·실패 사유 유지
        ok = lh.evaluate(good_inputs(), 10.0, CONFIG)
        self.assertEqual(reinit.apply(ok).state, lh.LOST)

        # 실패 전 시각의 추정·전달 pose를 같은 공분산으로 돌려준 추정은 근거가 아니다.
        candidate = reinit.attempt.candidate
        reinit.on_amcl(35.0, candidate['pose'], lh.pose_covariance(0.05, 0.03))
        for index in range(3):
            reinit.on_amcl(36.0 + index, candidate['pose'],
                           lh.pose_covariance(candidate['position_std_m'], candidate['yaw_std_rad']))
        self.assertEqual(reinit.apply(ok).state, lh.LOST)

        # 실패 뒤 수렴한 새 추정 3개가 쌓이면 회복으로 보고 실패 표시를 지운다.
        for index in range(3):
            reinit.on_amcl(40.0 + index, candidate['pose'], lh.pose_covariance(0.05, 0.03))
        self.assertEqual(reinit.apply(ok).state, lh.OK)
        self.assertIsNone(reinit.failure)

    # 발산(후보에서 크게 벗어남) → NOT_RECOVERED
    def test_divergence(self):
        tracker, inputs, reinit, health = self.prepared()
        reinit.request(health, inputs, tracker, 20.0, 'map-a', True)
        reinit.mark_sent(20.0)
        far = lh.compose(reinit.attempt.candidate['pose'], (1.5, 0.0, 0.0))

        for index in range(3):
            reinit.on_amcl(20.2 + 0.2 * index, far, lh.pose_covariance(0.05, 0.03))

        fresh_actuation(inputs, 21.0)
        self.assertEqual(reinit.step(inputs, 21.0), 'REINIT_FAILED_NOT_RECOVERED')

    # 진행 중 정지 조건 상실(구동 시작·상태 만료) → NOT_STOPPED
    def test_stop_lost(self):
        for change in (lambda i: setattr(i, 'actuation_state', 1), lambda i: setattr(i, 'actuation_stamp', 19.0)):
            tracker, inputs, reinit, health = self.prepared()
            reinit.request(health, inputs, tracker, 20.0, 'map-a', True)
            reinit.mark_sent(20.0)
            change(inputs)

            with self.subTest():
                self.assertEqual(reinit.step(inputs, 20.2), 'REINIT_FAILED_NOT_STOPPED')

    # 중복 요청은 같은 재위치(새 시도·전달 없음)
    def test_duplicate(self):
        tracker, inputs, reinit, health = self.prepared()
        reinit.request(health, inputs, tracker, 20.0, 'map-a', True)
        first = reinit.attempt

        self.assertEqual(reinit.request(health, inputs, tracker, 20.5, 'map-a', True), (True, 'REINIT_IN_PROGRESS'))
        self.assertIs(reinit.attempt, first)

    # 거절: 정지 아님·구동 상태 만료·근거 없음·근거 만료·이동 과다·odom 끊김·지도 변경·시계 무효·이미 OK
    def test_rejections(self):
        cases = []

        def case(name, expected, mutate):
            cases.append((name, expected, mutate))

        case('정지 아님', 'REINIT_REJECTED_NOT_STOPPED', lambda c: setattr(c['inputs'], 'actuation_state', 1))
        case('구동 상태 만료', 'REINIT_REJECTED_ACTUATION_STALE', lambda c: setattr(c['inputs'], 'actuation_stamp', 19.0))
        case('구동 상태 무효', 'REINIT_REJECTED_ACTUATION_STALE', lambda c: setattr(c['inputs'], 'actuation_valid', False))
        case('근거 없음', 'REINIT_REJECTED_NO_TRUSTED_POSE', lambda c: setattr(c['tracker'], 'record', None))
        case('근거 만료', 'REINIT_REJECTED_TRUSTED_POSE_EXPIRED', lambda c: c.update(now=75.0))
        case('이동 과다', 'REINIT_REJECTED_MOTION_TOO_LARGE', lambda c: setattr(c['inputs'], 'odom_pose', (2.5, 0.0, 0.0)))
        case('odom 끊김', 'REINIT_REJECTED_ODOM_DISCONTINUOUS', lambda c: c['tracker'].invalidate('ODOM_JUMP'))
        case('지도 변경', 'REINIT_REJECTED_MAP_CHANGED', lambda c: c['tracker'].check_map('map-b'))
        case('시계 역행', 'REINIT_REJECTED_CLOCK_INVALID', lambda c: c['tracker'].invalidate('CLOCK_REVERSED'))
        case('시계 무효', 'REINIT_REJECTED_CLOCK_INVALID', lambda c: c.update(clock=False))
        case('이미 OK', 'REINIT_REJECTED_NOT_NEEDED', lambda c: c.update(health=lh.Health(lh.OK, True, '')))

        for name, expected, mutate in cases:
            tracker, inputs, reinit, health = self.prepared()
            context = {'tracker': tracker, 'inputs': inputs, 'now': 20.0, 'clock': True, 'health': health}
            mutate(context)

            if name == '근거 만료':
                context['inputs'].actuation_stamp = 74.95

            with self.subTest(case=name):
                accepted, code = reinit.request(context['health'], context['inputs'], context['tracker'],
                                                context['now'], 'map-a', context['clock'])
                self.assertEqual((accepted, code), (False, expected))
                self.assertFalse(reinit.active())

    # 후보 공분산은 이동량에 따라 커진다.
    def test_candidate_covariance_grows(self):
        small = self.prepared(travel=0.1)
        large = self.prepared(travel=0.8)
        a, _ = lh.reinit_candidate(small[0], small[1].odom_pose, 20.0, 'map-a', CONFIG)
        b, _ = lh.reinit_candidate(large[0], large[1].odom_pose, 20.0, 'map-a', CONFIG)

        self.assertLess(a['position_std_m'], b['position_std_m'])
        self.assertTrue(math.isclose(a['travel_m'], 0.1))



# 검토 지적 3건의 재현 시험
class ReviewFindings(unittest.TestCase):

    # [1] odom에 NaN이 들어오면 OK가 아니라 LOST(ODOM_INVALID)이고, 현재 자세를 못 구하면 정상이 아니다.
    def test_nan_odom_not_ok(self):
        inputs = good_inputs()
        inputs.odom_pose = (float('nan'), 0.0, 0.0)
        health = lh.evaluate(inputs, 10.0, CONFIG)

        self.assertEqual(health.state, lh.LOST)
        self.assertIn('ODOM_INVALID', health.reason)
        self.assertIsNone(health.pose)

    # [1] odom 연속성 상실은 판정에 LOST(ODOM_DISCONTINUOUS)로 반영되고, 끊긴 뒤 AMCL 새 추정 3개가 쌓여야 풀린다.
    def test_odom_break_reaches_health(self):
        continuity = lh.OdomContinuity(CONFIG['odom_continuity'])
        inputs = good_inputs()

        continuity.update(9.0, (1.0, 0.0, 0.0), 0.0, 0.0)
        self.assertEqual(continuity.update(9.02, (3.0, 0.0, 0.0), 0.0, 0.0), 'ODOM_JUMP')
        inputs.odom_break_at, inputs.amcl_after_break = continuity.broken_at, continuity.amcl_after_break
        health = lh.evaluate(inputs, 10.0, CONFIG)
        self.assertEqual((health.state, health.reason), (lh.LOST, 'ODOM_DISCONTINUOUS'))

        continuity.note_amcl(8.0)
        for stamp in (9.5, 9.6, 9.7):
            continuity.note_amcl(stamp)
        inputs.amcl_after_break = continuity.amcl_after_break
        self.assertEqual(inputs.amcl_after_break, 3)
        self.assertEqual(lh.evaluate(inputs, 10.0, CONFIG).state, lh.OK)

    # [1] NaN 표본은 기준으로 남기지 않아, 그 뒤의 도약도 마지막 정상 표본과 비교해 잡는다.
    def test_nan_sample_not_baseline(self):
        continuity = lh.OdomContinuity(CONFIG['odom_continuity'])

        continuity.update(1.0, (0.0, 0.0, 0.0), 0.0, 0.0)
        self.assertEqual(continuity.update(1.02, (float('nan'), 0.0, 0.0), 0.0, 0.0), 'ODOM_INVALID')
        self.assertEqual(continuity.update(1.04, (5.0, 0.0, 0.0), 0.0, 0.0), 'ODOM_JUMP')

    # [2] 1 m 전진 후 1 m 복귀(누적 2 m, 순변위 0)는 이동 제한(1 m)에 걸린다. 반복 회전도 같다.
    def test_round_trip_counts(self):
        for name, path in (('왕복', [(0.1 * i, 0.0, 0.0) for i in range(11)] + [(1.0 - 0.1 * i, 0.0, 0.0) for i in range(1, 11)]),
                           ('반복 회전', [(0.0, 0.0, lh.wrap(0.2 * i)) for i in range(40)] + [(0.0, 0.0, 0.0)])):
            tracker = lh.TrustedPoseTracker(CONFIG)
            start = good_inputs(odom=(0.0, 0.0, 0.0))
            tracker.consider(lh.evaluate(start, 10.0, CONFIG), start, 10.0, 'map-a', False)

            for pose in path:
                tracker.on_odom(pose)

            candidate, code = lh.reinit_candidate(tracker, path[-1], 20.0, 'map-a', CONFIG)

            with self.subTest(case=name):
                self.assertIsNone(candidate)
                self.assertEqual(code, 'REINIT_REJECTED_MOTION_TOO_LARGE')

    # [2] 허용 범위 안의 왕복(누적 0.6 m)도 불확실성은 누적량으로 키운다(순변위 0으로 보지 않음).
    def test_cumulative_covariance(self):
        tracker = lh.TrustedPoseTracker(CONFIG)
        start = good_inputs(odom=(0.0, 0.0, 0.0))
        tracker.consider(lh.evaluate(start, 10.0, CONFIG), start, 10.0, 'map-a', False)

        for pose in [(0.1 * i, 0.0, 0.0) for i in range(4)] + [(0.3 - 0.1 * i, 0.0, 0.0) for i in range(1, 4)]:
            tracker.on_odom(pose)

        candidate, _ = lh.reinit_candidate(tracker, (0.0, 0.0, 0.0), 20.0, 'map-a', CONFIG)

        self.assertAlmostEqual(candidate['travel_m'], 0.6)
        self.assertAlmostEqual(candidate['net_travel_m'], 0.0)
        self.assertAlmostEqual(candidate['position_std_m'], 0.10 + 0.2 * 0.6)

    # [2] 새 신뢰 기록을 만들면 누적량을 0부터 다시 센다.
    def test_cumulative_resets_on_record(self):
        tracker = lh.TrustedPoseTracker(CONFIG)
        start = good_inputs(odom=(0.0, 0.0, 0.0))
        tracker.consider(lh.evaluate(start, 10.0, CONFIG), start, 10.0, 'map-a', False)
        tracker.on_odom((0.5, 0.0, 0.0))
        again = good_inputs(now=11.0, odom=(0.5, 0.0, 0.0))
        self.assertTrue(tracker.consider(lh.evaluate(again, 11.0, CONFIG), again, 11.0, 'map-a', False))

        self.assertEqual((tracker.path_m, tracker.path_rad), (0.0, 0.0))

    # [3] 재위치 중 시계 무효는 UNKNOWN·valid=false로 남고(LOST·valid=true로 덮지 않음), 재위치는 실패로 끝난다.
    def test_clock_invalid_during_reinit(self):
        tracker, inputs, reinit, health = Reinit().prepared()
        reinit.request(health, inputs, tracker, 20.0, 'map-a', True, wall=100.0)
        reinit.mark_sent(20.0)

        unknown = lh.evaluate(inputs, 20.5, CONFIG, clock_valid=False)
        shown = reinit.apply(unknown)
        self.assertEqual((shown.state, shown.valid), (lh.UNKNOWN, False))
        self.assertEqual(shown.reason, 'CLOCK_INVALID,REINIT_IN_PROGRESS')

        self.assertEqual(reinit.step(inputs, 20.5, clock_valid=False, wall=100.5), 'REINIT_FAILED_CLOCK_INVALID')
        after = reinit.apply(unknown)
        self.assertEqual((after.state, after.valid, after.reason), (lh.UNKNOWN, False,
                                                                   'CLOCK_INVALID,REINIT_FAILED_CLOCK_INVALID'))

    # [3] sim time이 멈춰도(같은 now) 벽시계 제한시간이 지나면 재위치를 끝낸다.
    def test_wall_timeout_when_sim_frozen(self):
        tracker, inputs, reinit, health = Reinit().prepared()
        reinit.request(health, inputs, tracker, 20.0, 'map-a', True, wall=100.0)
        reinit.mark_sent(20.0)
        fresh_actuation(inputs, 20.0)

        self.assertIsNone(reinit.step(inputs, 20.0, clock_valid=True, wall=150.0))
        self.assertEqual(reinit.step(inputs, 20.0, clock_valid=True, wall=161.0), 'REINIT_FAILED_AMCL_UNAVAILABLE')



# 최종 검토 지적 2건의 재현 시험
class FinalReviewFindings(unittest.TestCase):

    # AMCL 추정 → 최초 odom → 새 AMCL 없이 기준 초과 이동이면 ESTIMATE_STALE(최초 odom이 기준)
    def test_amcl_before_odom_baseline(self):
        inputs = good_inputs()
        inputs.amcl_odom_pose = lh.estimate_baseline(None, None, 1, on_amcl=True)
        self.assertIsNone(inputs.amcl_odom_pose)

        first = (1.0, 0.0, 0.0)
        inputs.amcl_odom_pose = lh.estimate_baseline(inputs.amcl_odom_pose, first, 1)
        self.assertEqual(inputs.amcl_odom_pose, first)

        # 이후 odom마다 기준을 덮어쓰지 않는다.
        for step in range(1, 7):
            pose = (1.0 + 0.1 * step, 0.0, 0.0)
            inputs.amcl_odom_pose = lh.estimate_baseline(inputs.amcl_odom_pose, pose, 1)
            inputs.odom_pose = pose

        self.assertEqual(inputs.amcl_odom_pose, first)
        health = lh.evaluate(inputs, 10.0, CONFIG)
        self.assertEqual((health.state, health.reason), (lh.DEGRADED, 'ESTIMATE_STALE'))

    # 비정상·끊김 odom은 기준으로 채택하지 않고, AMCL 수신 때 odom이 비유한 값이면 기준을 비운다.
    def test_baseline_rejects_bad_odom(self):
        nan = (float('nan'), 0.0, 0.0)

        self.assertIsNone(lh.estimate_baseline(None, nan, 1))
        self.assertIsNone(lh.estimate_baseline(None, (1.0, 0.0, 0.0), 1, odom_problem='ODOM_JUMP'))
        self.assertIsNone(lh.estimate_baseline(None, (1.0, 0.0, 0.0), 0))
        self.assertIsNone(lh.estimate_baseline((0.0, 0.0, 0.0), nan, 1, on_amcl=True))
        self.assertEqual(lh.estimate_baseline(None, (1.0, 0.0, 0.0), 1, on_amcl=True), (1.0, 0.0, 0.0))

    # 현재·기준 자세의 NaN·±inf에서 예외 없이 비정상 상태를 돌려준다.
    def test_non_finite_poses_no_exception(self):
        inf, nan = float('inf'), float('nan')
        cases = {
            '현재 yaw +inf': dict(odom_pose=(0.0, 0.0, inf)),
            '현재 yaw -inf': dict(odom_pose=(0.0, 0.0, -inf)),
            '현재 x NaN': dict(odom_pose=(nan, 0.0, 0.0)),
            '기준 yaw +inf': dict(amcl_odom_pose=(0.0, 0.0, inf)),
            '기준 x -inf': dict(amcl_odom_pose=(-inf, 0.0, 0.0)),
            '기준 NaN': dict(amcl_odom_pose=(nan, nan, nan)),
            'map→odom yaw inf': dict(map_odom=(0.0, 0.0, inf)),
        }

        for name, values in cases.items():
            inputs = good_inputs()

            for key, value in values.items():
                setattr(inputs, key, value)

            with self.subTest(case=name):
                health = lh.evaluate(inputs, 10.0, CONFIG)
                self.assertNotEqual(health.state, lh.OK)
                self.assertTrue(health.reason)


if __name__ == '__main__':
    unittest.main()
