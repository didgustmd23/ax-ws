# B2 AMCL 재시작 대책(localization_health.py)을 ROS 없이 검사한다.
# P1: 공분산 0(붕괴) 추정은 OK가 아니다(DEGRADED COVARIANCE_COLLAPSED). launch 초기 위치 산포 만들기(InitialSpread).
# P2: AMCL 재시작(lifecycle 전이·산포 뒤 공분산 0 추정) 감지 → LOST(ESTIMATE_DISCONTINUOUS) latch,
#     기존 재위치(신뢰 기록 + odom)가 성공해야 해제.
# 근거: log/b2_amcl_restart/20261113_073413/REPORT.md(nav2_amcl 1.3.13 on_cleanup·on_activate 공분산 0 재초기화).
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_localization/scripts'))

import localization_health as lh  # noqa: E402

CONFIG = lh.merge_config()
MAP_ODOM = (0.0, 0.0, 0.0)
ODOM = (0.669, 0.0, 0.0)


# 정상 입력(정지, 모든 입력 최신). std·yaw_std로 공분산 대각을 정한다.
def inputs(now=10.0, std=0.05, yaw_std=0.03, map_odom=MAP_ODOM, odom=ODOM):
    return lh.Inputs(
        scan_stamp=now - 0.1, odom_stamp=now - 0.02, odom_pose=odom,
        tf_valid_time=now - 0.1, map_odom=map_odom, tf_chain_ok=True,
        amcl_stamp=now - 0.1, amcl_pose=lh.compose(map_odom, odom),
        amcl_covariance=tuple(lh.pose_covariance(std, yaw_std)), amcl_odom_pose=odom, amcl_updates=3,
        actuation_state=lh.ACTUATION_STOPPED, actuation_valid=True, actuation_stamp=now - 0.05)


# start부터 end까지 50 Hz로 같은 odom 자세를 넣은 기록
def history(start=0.0, end=40.0, pose=ODOM):
    record = lh.OdomHistory(1000.0)

    for index in range(int(round((end - start) / 0.02)) + 1):
        record.add(start + 0.02 * index, pose)

    return record


# P1: 공분산 붕괴 판정
class CollapsedCovariance(unittest.TestCase):

    # 공분산 0(Jazzy 재초기화 실측 1e-14 수준)은 DEGRADED(COVARIANCE_COLLAPSED)이고 std는 그대로 낸다.
    def test_zero_is_degraded(self):
        for std, yaw_std in ((0.0, 0.0), (1e-7, 3e-7), (0.0, 0.03), (0.05, 0.0)):
            with self.subTest(std=std, yaw_std=yaw_std):
                health = lh.evaluate(inputs(std=std, yaw_std=yaw_std), 10.0, CONFIG)

                self.assertEqual((health.state, health.reason), (lh.DEGRADED, 'COVARIANCE_COLLAPSED'))
                self.assertEqual(health.position_std_m, std)

    # 하한 경계: 하한 미만은 붕괴, 하한 이상(정상 최솟값 0.029 m 포함)은 OK
    def test_boundary(self):
        low = CONFIG['health']['position_std_min_m']
        cases = ((low * 0.99, lh.DEGRADED), (low, lh.OK), (0.029, lh.OK))

        for std, state in cases:
            with self.subTest(std=std):
                self.assertEqual(lh.evaluate(inputs(std=std, yaw_std=0.03), 10.0, CONFIG).state, state)

    # 공분산이 큰 경우(COVARIANCE_HIGH·LOST)는 기존 판정 그대로이고 붕괴로 보지 않는다.
    def test_high_unchanged(self):
        self.assertEqual(lh.evaluate(inputs(std=0.3), 10.0, CONFIG).reason, 'COVARIANCE_HIGH')
        self.assertEqual(lh.evaluate(inputs(std=1.5), 10.0, CONFIG).reason, 'COVARIANCE_LOST')

    # 하한은 설정으로 바꿀 수 있고 모르는 키는 여전히 거절한다.
    def test_config(self):
        config = lh.merge_config({'health': {'position_std_min_m': 0.0, 'yaw_std_min_rad': 0.0}})

        self.assertEqual(lh.evaluate(inputs(std=0.0, yaw_std=0.0), 10.0, config).state, lh.OK)

        with self.assertRaises(ValueError):
            lh.merge_config({'initial_spread': {'unknown': 1.0}})

    # 붕괴 추정은 신뢰 기록이 되지 않는다(health OK 아님).
    def test_not_trusted(self):
        tracker = lh.TrustedPoseTracker(CONFIG)
        current = inputs(std=0.0, yaw_std=0.0)
        health = lh.evaluate(current, 10.0, CONFIG)

        self.assertFalse(tracker.consider(health, current, 10.0, 'map-a', False, None))
        self.assertEqual(tracker.rejection(health, current, 'map-a', False, None), 'HEALTH_NOT_OK')


# P1: launch 초기 위치 산포 만들기
class InitialSpreadFlow(unittest.TestCase):

    # 조건을 모두 만족할 때만 한 번 보낸다(launch 초기 위치 사용·확인됨·공분산 붕괴·자세 있음·시계 유효).
    def test_due_conditions(self):
        spread = lh.InitialSpread(CONFIG)
        collapsed, spread_std, pose = (0.0, 0.0), (0.1, 0.1), (0.0, 0.0, 0.0)

        self.assertTrue(spread.due(True, True, collapsed, pose, True))

        for args in ((False, True, collapsed, pose, True), (True, False, collapsed, pose, True),
                     (True, True, spread_std, pose, True), (True, True, collapsed, None, True),
                     (True, True, collapsed, pose, False), (True, True, None, pose, True)):
            with self.subTest(args=args):
                self.assertFalse(spread.due(*args))

        spread.mark_sent(5.0)
        self.assertFalse(spread.due(True, True, collapsed, pose, True))

    # 보낸 뒤 무이동 갱신을 주기마다 요청하고, OK + 새 추정 min_updates개면 멈춘다(SPREAD).
    def test_step_until_spread(self):
        spread = lh.InitialSpread(CONFIG)
        spread.mark_sent(5.0)
        period = CONFIG['reinit']['nomotion_period_s']

        self.assertTrue(spread.step(5.1, False))
        self.assertFalse(spread.step(5.1 + period / 2, False))
        self.assertTrue(spread.step(5.1 + period, False))

        # 보낸 시각 이전·붕괴 추정은 세지 않는다.
        spread.on_amcl(4.9, (0.1, 0.1))
        spread.on_amcl(5.3, (0.0, 0.0))
        self.assertEqual(spread.updates, 0)

        for stamp in (5.4, 5.9, 6.4):
            spread.on_amcl(stamp, (0.08, 0.06))

        self.assertTrue(spread.step(6.5, False))       # 아직 OK 아님: 계속 요청
        self.assertFalse(spread.done)
        self.assertFalse(spread.step(6.6, True))
        self.assertEqual((spread.done, spread.result), (True, 'SPREAD'))
        self.assertFalse(spread.step(7.5, True))

    # 제한시간이 지나면 요청을 멈춘다(TIMEOUT). 정지 중 재표본 반복으로 다시 붕괴하지 않게 한다.
    def test_timeout(self):
        spread = lh.InitialSpread(CONFIG)
        spread.mark_sent(5.0)

        self.assertFalse(spread.step(5.0 + CONFIG['initial_spread']['timeout_s'] + 0.1, False))
        self.assertEqual(spread.result, 'TIMEOUT')

    # 산포 송신 의도(INITIAL_SPREAD)가 확인되면 established이고 비교 기준이 유지된다(도약 아님).
    def test_spread_intent_established(self):
        odom = history()
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.on_tf(1.0, MAP_ODOM, odom)
        pose = lh.compose(MAP_ODOM, ODOM)
        monitor.expect(pose, 'INITIAL_SPREAD', 5.0, 0.0, True)

        self.assertEqual(monitor.on_tf(5.5, (0.002, 0.0, 0.0), odom), 'INTENDED')
        self.assertTrue(monitor.established)
        self.assertIsNone(monitor.latched)


# P2: AMCL 재시작 감지
class ResetDetection(unittest.TestCase):

    # 최초 초기화의 공분산 0은 재시작이 아니고, 산포 있는 추정 뒤 다시 공분산 0이면 재시작이다.
    def test_collapsed_after_spread(self):
        detector = lh.AmclResetDetector(CONFIG)

        self.assertIsNone(detector.on_estimate((0.0, 0.0), 3.0, 2.0))
        self.assertIsNone(detector.on_estimate(None, 3.1, 2.0))
        self.assertIsNone(detector.on_estimate((0.1, 0.1), 3.2, 2.0))
        self.assertIsNone(detector.on_estimate((0.03, 0.02), 3.3, 2.0))
        self.assertEqual(detector.on_estimate((0.0, 3e-7), 3.4, 2.0), 'COVARIANCE_COLLAPSED_AFTER_SPREAD')

    # 감독 노드 시계 유효 전(basis_at 없음)·기준 전 stamp의 산포 추정(이전 세션 transient local)은 세지 않는다.
    def test_old_estimate_ignored(self):
        detector = lh.AmclResetDetector(CONFIG)

        self.assertIsNone(detector.on_estimate((0.05, 0.03), 1.0, None))
        self.assertIsNone(detector.on_estimate((0.05, 0.03), 1.0, 2.0))
        self.assertFalse(detector.spread_seen)
        self.assertIsNone(detector.on_estimate((0.0, 0.0), 2.5, 2.0))

    # lifecycle: active를 떠나는 전이만 재시작이다(최초 기동 전이는 해당 없음).
    def test_transition(self):
        startup = (('unconfigured', 'configuring'), ('configuring', 'inactive'),
                   ('inactive', 'activating'), ('activating', 'active'))

        for start, goal in startup:
            with self.subTest(start=start):
                self.assertIsNone(lh.AmclResetDetector.on_transition(start, goal))

        self.assertEqual(lh.AmclResetDetector.on_transition('active', 'deactivating'), 'LIFECYCLE_ACTIVE_TO_DEACTIVATING')
        self.assertEqual(lh.AmclResetDetector.on_transition('active', 'shuttingdown'), 'LIFECYCLE_ACTIVE_TO_SHUTTINGDOWN')

    # 재시작 latch: 차이가 도약 기준보다 작아도 LOST(ESTIMATE_DISCONTINUOUS), 신뢰 기록 덮어쓰기 금지, 처음 기록 유지.
    def test_reset_latch(self):
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.reset('amcl_lifecycle', 31.2, 'LIFECYCLE_ACTIVE_TO_DEACTIVATING')
        monitor.reset('amcl_reset', 33.4, 'COVARIANCE_COLLAPSED_AFTER_SPREAD')

        self.assertEqual(monitor.latched['source'], 'amcl_lifecycle')
        self.assertIsNone(monitor.latched['jump_m'])
        self.assertEqual(monitor.trust_block(), 'ESTIMATE_DISCONTINUOUS')

        health = monitor.apply(lh.evaluate(inputs(now=40.0, std=0.0, yaw_std=0.0), 40.0, CONFIG))
        self.assertEqual(health.state, lh.LOST)
        self.assertEqual(health.reason, 'ESTIMATE_DISCONTINUOUS,COVARIANCE_COLLAPSED')

        # 공분산이 다시 커져도(산포) latch는 풀리지 않는다.
        self.assertEqual(monitor.apply(lh.evaluate(inputs(now=40.0, std=0.05), 40.0, CONFIG)).state, lh.LOST)


# P2 + P3: 재시작 뒤 재위치로 회복(R1 재현 상황: 출발 정지 중 신뢰 기록 → 0.669 m 직진 → 재시작)
class RecoveryAfterReset(unittest.TestCase):

    # 정지 중 기록한 신뢰 위치 + odom으로 후보를 만들고, 산포 있는 새 추정이 맞으면 성공·latch 해제
    def test_reinit_recovers(self):
        tracker = lh.TrustedPoseTracker(CONFIG)
        start = inputs(now=20.0, odom=(0.0, 0.0, 0.0))
        self.assertTrue(tracker.consider(lh.evaluate(start, 20.0, CONFIG), start, 20.0, 'map-a', False, None))

        for x in (0.2, 0.4, 0.669):
            tracker.on_odom((x, 0.0, 0.0))

        monitor = lh.EstimateMonitor(CONFIG)
        monitor.reset('amcl_lifecycle', 31.2, 'LIFECYCLE_ACTIVE_TO_DEACTIVATING')
        # 재시작 뒤 AMCL: 마지막 갱신 자세 0.556으로 공분산 0 재초기화(map→odom x −0.113)
        after = inputs(now=45.0, std=0.0, yaw_std=0.0, map_odom=(-0.113, 0.0, 0.0))
        health = monitor.apply(lh.evaluate(after, 45.0, CONFIG))
        self.assertEqual(health.state, lh.LOST)

        reinit = lh.Reinitializer(CONFIG)
        accepted, code = reinit.request(health, after, tracker, 45.0, 'map-a', True)
        self.assertEqual((accepted, code), (True, 'REINIT_ACCEPTED'))
        candidate = reinit.attempt.candidate
        self.assertAlmostEqual(candidate['pose'][0], 0.669, places=6)
        reinit.mark_sent(45.0)

        # 공분산 0 추정은 수렴 근거가 아니다(완료하지 않음).
        for stamp in (45.2, 45.7, 46.2):
            reinit.on_amcl(stamp, (0.669, 0.0, 0.0), lh.pose_covariance(0.0, 0.0))

        recovered = inputs(now=46.3, std=0.05, yaw_std=0.03, map_odom=MAP_ODOM)
        self.assertIsNone(reinit.step(recovered, 46.3))

        # 산포 있는 새 추정이 후보와 맞으면 성공
        for stamp in (46.5, 47.0, 47.5):
            reinit.on_amcl(stamp, (0.67, 0.0, 0.0), lh.pose_covariance(0.08, 0.05))

        recovered = inputs(now=47.6, std=0.08, yaw_std=0.05, map_odom=MAP_ODOM)
        self.assertEqual(reinit.step(recovered, 47.6), 'SUCCEEDED')
        monitor.release(recovered.map_odom)
        self.assertEqual(monitor.apply(lh.evaluate(recovered, 47.6, CONFIG)).state, lh.OK)

    # 실패 뒤 스스로 회복 근거에도 공분산 0 추정은 넣지 않는다.
    def test_failure_recovery_ignores_collapsed(self):
        reinit = lh.Reinitializer(CONFIG)
        reinit.failure, reinit.failure_at = 'REINIT_FAILED_TIMEOUT', 10.0

        for stamp in (11.0, 11.5, 12.0):
            reinit.on_amcl(stamp, (0.0, 0.0, 0.0), lh.pose_covariance(0.0, 0.0))

        self.assertFalse(reinit.recovered_after_failure())


if __name__ == '__main__':
    unittest.main()
