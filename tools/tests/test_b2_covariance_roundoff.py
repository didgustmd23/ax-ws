# B2 AMCL 공분산 대각의 반올림 수준 음수 처리(localization_health.covariance_std)를 ROS 없이 검사한다.
# 실제 표본: initial_poses 정지 smoke(log/b2_health/20261109_231523_robot2_initial_poses_smoke) amcl_pose stamp 7.4.
# ROS 연결부(진단 표시·신뢰 기록)는 test_localization_supervisor.py LaunchInitialPose에서 검사한다.
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_localization/scripts'))

import localization_health as lh  # noqa: E402

CONFIG = lh.merge_config()
EPS = lh.ROUNDOFF_VARIANCE


# smoke 회차 실제 amcl_pose 공분산(36개, 행 우선)
def real_sample():
    values = [0.0] * 36
    values[0] = 7.420008617947671e-19
    values[1] = values[6] = -5.149960319306146e-19
    values[7] = -5.664956351236761e-18
    values[35] = -1.0924594562311242e-13
    return values


# 대각(xx, yy, yaw)만 정한 공분산
def diagonal(xx, yy, yaw):
    values = [0.0] * 36
    values[0], values[7], values[35] = xx, yy, yaw
    return values


# 정상 입력(공분산만 바꿔 넣는다)
def inputs_with(covariance, now=10.0, map_odom=(0.5, 0.2, 0.0), odom=(1.0, 0.0, 0.0)):
    return lh.Inputs(
        scan_stamp=now - 0.1, odom_stamp=now - 0.02, odom_pose=odom,
        tf_valid_time=now - 0.1, map_odom=map_odom, tf_chain_ok=True,
        amcl_stamp=now - 0.1, amcl_pose=lh.compose(map_odom, odom),
        amcl_covariance=tuple(covariance), amcl_odom_pose=odom, amcl_updates=3,
        actuation_state=lh.ACTUATION_STOPPED, actuation_valid=True, actuation_stamp=now - 0.05)


# 수치 경계
class RoundoffBoundary(unittest.TestCase):

    # 경계값은 분산 단위 1e-12(Jazzy pf 통계 yaw 반올림의 설명용 오차 규모 추정, 엄밀 상한 아님)
    def test_epsilon(self):
        self.assertEqual(EPS, 1e-12)

    # 실제 표본: yy·yaw 극소 음수를 0으로 보고 표준편차를 계산한다. 정규화 대상·원래 값을 돌려준다.
    def test_real_sample(self):
        std, normalized = lh.covariance_std_detail(real_sample())

        self.assertIsNotNone(std)
        self.assertAlmostEqual(std[0], math.sqrt(7.420008617947671e-19))
        self.assertEqual(std[1], 0.0)
        self.assertEqual([item['index'] for item in normalized], [7, 35])
        self.assertEqual(normalized[1]['value'], -1.0924594562311242e-13)
        self.assertIsNotNone(lh.covariance_std(real_sample()))

    # 원시 배열은 바꾸지 않는다.
    def test_raw_unchanged(self):
        values = real_sample()
        before = list(values)
        lh.covariance_std_detail(values)
        lh.covariance_std(values)

        self.assertEqual(values, before)

    # [−epsilon, 0)만 0. 경계 포함, 그 바깥 음수는 무효. 양수·정확한 0은 그대로(정규화 표시 없음).
    def test_boundary(self):
        self.assertEqual(lh.covariance_std_detail(diagonal(0.01, 0.01, -EPS))[0], (0.1, 0.0))
        self.assertIsNone(lh.covariance_std(diagonal(0.01, 0.01, -EPS * 1.0001)))
        self.assertIsNone(lh.covariance_std(diagonal(-2e-12, 0.01, 0.01)))
        self.assertEqual(lh.covariance_std_detail(diagonal(0.01, 0.0, 0.0)), ((0.1, 0.0), []))
        self.assertEqual(lh.covariance_std_detail(diagonal(1e-20, 0.04, 0.01))[1], [])

    # 유의한 음수·NaN·inf·길이 오류는 계속 무효(절댓값을 씌우지 않는다).
    def test_invalid_kept(self):
        for name, values in (('유의한 음수 yaw', diagonal(0.01, 0.01, -1e-6)),
                             ('유의한 음수 xx', diagonal(-0.01, 0.01, 0.01)),
                             ('NaN', diagonal(float('nan'), 0.01, 0.01)),
                             ('inf', diagonal(0.01, float('inf'), 0.01)),
                             ('-inf', diagonal(0.01, 0.01, float('-inf'))),
                             ('길이 35', [0.0] * 35)):
            with self.subTest(case=name):
                self.assertEqual(lh.covariance_std_detail(values), (None, []))
                self.assertIsNone(lh.covariance_std(values))


# 판정·도약·신뢰 기록·재위치에 미치는 범위
class RoundoffEffects(unittest.TestCase):

    # 실제 표본: ESTIMATE_INVALID가 아니고 다른 검사가 진행된다. 다만 공분산 0(붕괴)이라 OK가 아니라
    # DEGRADED(COVARIANCE_COLLAPSED)다(2026-11-13 AMCL 재시작 대책 P1: 공분산 0은 정확도 근거가 아님).
    def test_evaluate_real_sample(self):
        health = lh.evaluate(inputs_with(real_sample()), 10.0, CONFIG)

        self.assertEqual((health.state, health.reason), (lh.DEGRADED, 'COVARIANCE_COLLAPSED'))

    # 기존 유의한 음수는 그대로 ESTIMATE_INVALID
    def test_evaluate_significant_negative(self):
        health = lh.evaluate(inputs_with(diagonal(0.01, 0.01, -1e-6)), 10.0, CONFIG)

        self.assertEqual(health.state, lh.LOST)
        self.assertIn('ESTIMATE_INVALID', health.reason)

    # 시계·TF 이상 우선순위는 그대로(정규화와 무관)
    def test_priorities_kept(self):
        self.assertEqual(lh.evaluate(inputs_with(real_sample()), 10.0, CONFIG, clock_valid=False).reason,
                         'CLOCK_INVALID')
        stale = inputs_with(real_sample())
        stale.tf_valid_time = 5.0
        self.assertIn('TF_MISSING', lh.evaluate(stale, 10.0, CONFIG).reason)

    # 정규화된 추정이라도 큰 도약은 LOST·신뢰 기록 차단, 초기화 미확인이면 신뢰 기록 차단
    def test_jump_and_establish_still_apply(self):
        odom = lh.OdomHistory(1000.0)

        for index in range(501):
            odom.add(0.02 * index, (1.0, 0.0, 0.0))

        monitor = lh.EstimateMonitor(CONFIG)
        monitor.on_tf(5.0, (0.5, 0.2, 0.0), odom)
        inputs = inputs_with(real_sample())
        tracker = lh.TrustedPoseTracker(CONFIG)

        self.assertEqual(monitor.trust_block(), 'NOT_ESTABLISHED')
        self.assertFalse(tracker.consider(lh.evaluate(inputs, 10.0, CONFIG), inputs, 10.0, 'map-a', False,
                                          monitor.trust_block()))

        self.assertEqual(monitor.on_tf(6.0, (3.5, 0.2, 0.0), odom), 'JUMP')
        health = monitor.apply(lh.evaluate(inputs, 10.0, CONFIG))
        self.assertEqual((health.state, health.reason), (lh.LOST, 'ESTIMATE_DISCONTINUOUS,COVARIANCE_COLLAPSED'))
        self.assertEqual(monitor.trust_block(), 'ESTIMATE_DISCONTINUOUS')
        self.assertIsNone(tracker.record)

    # 재위치·실패 뒤 회복 기록도 같은 함수를 쓴다: 반올림 음수는 0, 유의한 음수는 무한대(회복 근거 아님)
    def test_reinit_callers(self):
        reinit = lh.Reinitializer(CONFIG)
        reinit.failure, reinit.failure_at = 'REINIT_FAILED_TIMEOUT', 10.0
        reinit.on_amcl(11.0, (0.0, 0.0, 0.0), real_sample())
        reinit.on_amcl(11.2, (0.0, 0.0, 0.0), diagonal(0.01, 0.01, -1e-6))

        self.assertEqual(reinit.after_failure[0][1:], (math.sqrt(7.420008617947671e-19), 0.0))
        self.assertEqual(reinit.after_failure[1][1:], (float('inf'), float('inf')))


if __name__ == '__main__':
    unittest.main()
