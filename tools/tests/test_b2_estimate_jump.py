# B2 추정 도약 감지·신뢰 기록 보호(localization_health.py)를 ROS 없이 검사한다.
# 1단계 중복·역행 입력, 2단계 도약 감지·신뢰 기록 보호, 3단계 의도된 초기화·재위치, 4단계 latch 해제.
# ROS 연결부는 test_localization_supervisor.py(가짜 세계)에서 검사한다.
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_localization/scripts'))

import localization_health as lh  # noqa: E402

CONFIG = lh.merge_config()
MAP_ODOM = (0.5, 0.2, 0.0)
ODOM = (1.0, 0.0, 0.0)


# 정상 입력(정지, 공분산 작음, 모든 입력 최신)
def good_inputs(now=10.0, odom=ODOM, map_odom=MAP_ODOM, std=0.05, yaw_std=0.03):
    return lh.Inputs(
        scan_stamp=now - 0.1, odom_stamp=now - 0.02, odom_pose=odom,
        tf_valid_time=now - 0.1, map_odom=map_odom, tf_chain_ok=True,
        amcl_stamp=now - 0.1, amcl_pose=lh.compose(map_odom, odom),
        amcl_covariance=tuple(lh.pose_covariance(std, yaw_std)), amcl_odom_pose=odom, amcl_updates=3,
        actuation_state=lh.ACTUATION_STOPPED, actuation_valid=True, actuation_stamp=now - 0.05)


# start부터 end까지 50 Hz로 같은 odom 자세를 넣은 기록(시험에서는 보관 기간을 길게 둔다)
def history(start=0.0, end=10.0, pose=ODOM, keep_s=1000.0):
    record = lh.OdomHistory(keep_s)
    count = int(round((end - start) / 0.02))

    for index in range(count + 1):
        record.add(start + 0.02 * index, pose)

    return record


# START_POSE가 확인된 감시기(비교 기준 MAP_ODOM, 최신 amcl_pose와 TF 대응)
def established_monitor(odom=None, now=5.0):
    odom = odom or history()
    monitor = lh.EstimateMonitor(CONFIG)
    start = lh.compose(MAP_ODOM, ODOM)
    monitor.expect(start, 'START_POSE', now, 100.0, True)
    monitor.on_amcl(now + 0.1, start, odom)
    monitor.on_tf(now + 0.1, MAP_ODOM, odom, now + 0.2, 100.2, True)

    return monitor


# 신뢰 기록 하나를 만든 tracker
def trusted_tracker(now=10.0, monitor=None):
    tracker = lh.TrustedPoseTracker(CONFIG)
    inputs = good_inputs(now)
    block = None if monitor is None else monitor.trust_block()
    tracker.consider(lh.evaluate(inputs, now, CONFIG), inputs, now, 'map-a', False, block)

    return tracker


# 재위치 한 건을 진행한다. updates: 전달 뒤 AMCL 추정 (stamp, pose, std). 판정 시각은 20.2 s부터 0.2 s 간격이다
# (stamp가 비유한 값이어도 시각은 유한). 결과 코드를 돌려준다.
def run_reinit(reinit, inputs, updates):
    result = None

    for index, (stamp, pose, std) in enumerate(updates):
        now = 20.2 + 0.2 * index
        reinit.on_amcl(stamp, pose, lh.pose_covariance(std, std))
        inputs.tf_valid_time = now
        inputs.scan_stamp = now
        inputs.odom_stamp = now
        inputs.actuation_stamp = now
        result = reinit.step(inputs, now) or result

    return result


# 1단계: 같은·역행·비유한 stamp
class DuplicateAndReversedInputs(unittest.TestCase):

    # 늦은 stamp만 받아들이고, 같은 stamp는 값으로 중복·충돌을 나누며 기준 값은 처음 것을 유지한다.
    def test_stamp_gate(self):
        gate = lh.StampGate()

        self.assertEqual(gate.check(1.0, (0.0, 0.0, 0.0)), 'ACCEPTED')
        self.assertEqual(gate.check(1.0, (0.0, 0.0, 0.0)), 'DUPLICATE')
        self.assertEqual(gate.check(1.0, (0.5, 0.0, 0.0)), 'CONFLICT')
        self.assertEqual(gate.value, (0.0, 0.0, 0.0))
        self.assertEqual(gate.check(0.9, (0.0, 0.0, 0.0)), 'REVERSED')
        self.assertEqual(gate.check(float('nan'), (0.0, 0.0, 0.0)), 'INVALID_STAMP')
        self.assertEqual(gate.check(float('inf'), (0.0, 0.0, 0.0)), 'INVALID_STAMP')
        self.assertEqual(gate.check(1.1, (0.5, 0.0, 0.0)), 'ACCEPTED')
        self.assertEqual(gate.stamp, 1.1)
        self.assertEqual(gate.counts, {'ACCEPTED': 2, 'DUPLICATE': 1, 'CONFLICT': 1, 'REVERSED': 1,
                                       'INVALID_STAMP': 2})

    # NaN 공분산이 든 같은 메시지 재수신은 충돌이 아니라 중복이다.
    def test_gate_nan_duplicate(self):
        gate = lh.StampGate()
        gate.check(2.0, (float('nan'), 1.0))

        self.assertEqual(gate.check(2.0, (float('nan'), 1.0)), 'DUPLICATE')

    # 같은 추정 3회로는 재위치 회복 조건(새 추정 3개)을 채우지 못한다. 늘어나는 stamp면 성공한다.
    def test_reinit_same_estimate_three_times(self):
        for name, stamps, expected in (('같은 stamp', (20.2, 20.2, 20.2), None),
                                       ('역행', (20.4, 20.3, 20.2), None),
                                       ('비유한', (20.2, float('nan'), float('inf')), None),
                                       ('정상 증가', (20.2, 20.4, 20.6), 'SUCCEEDED')):
            with self.subTest(case=name):
                tracker = trusted_tracker()
                inputs = good_inputs(20.0, odom=(1.2, 0.0, 0.0))
                inputs.tf_valid_time = 15.0
                reinit = lh.Reinitializer(CONFIG)
                health = lh.evaluate(inputs, 20.0, CONFIG)
                self.assertEqual(reinit.request(health, inputs, tracker, 20.0, 'map-a', True)[1], 'REINIT_ACCEPTED')
                reinit.mark_sent(20.0)
                candidate = reinit.attempt.candidate['pose']
                updates = [(stamp, candidate, 0.08) for stamp in stamps]
                result = run_reinit(reinit, inputs, updates)

                self.assertEqual(result, expected)
                self.assertEqual(len(reinit.attempt.updates), 3 if expected else 1)

    # odom 끊김 뒤 같은·역행·비유한 stamp AMCL 추정은 회복 수에 넣지 않는다(ODOM_DISCONTINUOUS 유지).
    def test_odom_break_needs_new_stamps(self):
        continuity = lh.OdomContinuity(CONFIG['odom_continuity'])
        continuity.update(1.0, (0.0, 0.0, 0.0), 0.0, 0.0)
        self.assertEqual(continuity.update(1.02, (2.0, 0.0, 0.0), 0.0, 0.0), 'ODOM_JUMP')

        for stamp in (1.5, 1.5, 1.5, 1.4, float('nan')):
            continuity.note_amcl(stamp)

        self.assertEqual(continuity.amcl_after_break, 1)
        inputs = good_inputs(2.0)
        inputs.odom_break_at, inputs.amcl_after_break = continuity.broken_at, continuity.amcl_after_break
        self.assertIn('ODOM_DISCONTINUOUS', lh.evaluate(inputs, 2.0, CONFIG).reason)

        continuity.note_amcl(1.6)
        continuity.note_amcl(1.7)
        inputs.amcl_after_break = continuity.amcl_after_break
        self.assertEqual(continuity.amcl_after_break, 3)
        self.assertNotIn('ODOM_DISCONTINUOUS', lh.evaluate(inputs, 2.0, CONFIG).reason)

    # 실패 뒤 회복 근거도 같은 stamp는 한 번만 센다.
    def test_after_failure_counts_new_stamps(self):
        reinit = lh.Reinitializer(CONFIG)
        reinit.failure, reinit.failure_at = 'REINIT_FAILED_TIMEOUT', 10.0

        for _ in range(3):
            reinit.on_amcl(11.0, (0.0, 0.0, 0.0), lh.pose_covariance(0.05, 0.03))

        self.assertEqual(len(reinit.after_failure), 1)
        self.assertFalse(reinit.recovered_after_failure())

    # sim time 역행은 기존대로 시계 무효(UNKNOWN·CLOCK_INVALID)를 유지한다.
    def test_clock_reversal_kept(self):
        clock = lh.ClockMonitor(2.0)
        clock.update(5.0, 0.0)
        valid, reversed_ = clock.update(4.0, 0.1)

        self.assertEqual((valid, reversed_), (False, True))
        health = lh.EstimateMonitor(CONFIG).apply(lh.evaluate(good_inputs(), 4.0, CONFIG, valid))
        self.assertEqual((health.state, health.valid, health.reason), (lh.UNKNOWN, False, 'CLOCK_INVALID'))


# 2단계: 도약 감지와 신뢰 기록 보호
class JumpDetection(unittest.TestCase):

    # 정상 보정(0.03 m·0.03 rad)은 도약이 아니고 비교 기준을 새 추정으로 옮긴다.
    def test_normal_correction(self):
        odom = history()
        monitor = established_monitor(odom)
        corrected = (MAP_ODOM[0] + 0.03, MAP_ODOM[1], 0.03)

        self.assertEqual(monitor.on_tf(5.5, corrected, odom), 'CONTINUOUS')
        self.assertIsNone(monitor.latched)
        self.assertEqual(monitor.reference, corrected)
        self.assertEqual(monitor.apply(lh.Health(lh.OK, True, '')).state, lh.OK)

    # 위치 도약: LOST·ESTIMATE_DISCONTINUOUS latch, 비교 기준은 도약 전 값 유지, 신뢰 기록 차단
    def test_position_jump(self):
        odom = history()
        monitor = established_monitor(odom)

        self.assertEqual(monitor.on_tf(5.5, (MAP_ODOM[0] + 0.5, MAP_ODOM[1], 0.0), odom), 'JUMP')
        self.assertAlmostEqual(monitor.latched['jump_m'], 0.5)
        self.assertEqual(monitor.reference, MAP_ODOM)
        health = monitor.apply(lh.Health(lh.OK, True, '', 0.05, 0.03))
        self.assertEqual((health.state, health.valid, health.reason), (lh.LOST, True, 'ESTIMATE_DISCONTINUOUS'))
        self.assertEqual(monitor.trust_block(), 'ESTIMATE_DISCONTINUOUS')

    # 방향 도약(0.5 rad): 같은 odom 자세에서 비교한 회전량으로 잡는다.
    def test_yaw_jump(self):
        odom = history()
        monitor = established_monitor(odom)
        robot = lh.compose(MAP_ODOM, ODOM)
        rotated = lh.compose((robot[0], robot[1], robot[2] + 0.5), lh.inverse(ODOM))

        self.assertEqual(monitor.on_tf(5.5, rotated, odom), 'JUMP')
        self.assertAlmostEqual(monitor.latched['jump_rad'], 0.5)
        self.assertAlmostEqual(monitor.latched['jump_m'], 0.0, places=9)

    # map→odom 평행이동 크기를 도약량으로 쓰지 않는다: odom 원점에서 먼 로봇의 작은 회전 보정은
    # map→odom 이동이 커도(≈0.5 m) 로봇 자세 차이는 0.05 rad뿐이라 도약이 아니다.
    def test_translation_is_not_jump_amount(self):
        far = (10.0, 0.0, 0.0)
        odom = history(pose=far)
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.on_tf(1.0, (0.0, 0.0, 0.0), odom)
        robot = lh.compose((0.0, 0.0, 0.0), far)
        corrected = lh.compose((robot[0], robot[1], 0.05), lh.inverse(far))

        self.assertGreater(math.hypot(corrected[0], corrected[1]), 0.45)
        self.assertEqual(monitor.on_tf(1.5, corrected, odom), 'CONTINUOUS')

    # TF 공백(4 s) 뒤 첫 TF는 공백 전 마지막 비교 기준과 비교한다. 유효한 신뢰 기록이 없어도 감시한다.
    def test_tf_gap_keeps_reference(self):
        odom = history(0.0, 12.0)
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.on_tf(1.0, MAP_ODOM, odom)
        self.assertFalse(monitor.established)

        self.assertEqual(monitor.on_tf(9.0, (MAP_ODOM[0] - 5.9, MAP_ODOM[1], 1.59), odom), 'JUMP')
        self.assertEqual(monitor.reference, MAP_ODOM)

    # 수신 순서 교환: 도약한 amcl_pose가 TF보다 먼저 와도 비교 기준 · 그 시각 odom과 비교해 잡는다.
    # 반대로 정상 갱신은 amcl_pose가 먼저 오면 TF가 올 때까지 갱신 대응이 없어 신뢰 기록을 보류한다.
    def test_order_swap(self):
        odom = history()
        monitor = established_monitor(odom)
        self.assertIsNone(monitor.trust_block())

        corrected = (MAP_ODOM[0] + 0.05, MAP_ODOM[1], 0.0)
        self.assertEqual(monitor.on_amcl(6.0, lh.compose(corrected, ODOM), odom), 'CONTINUOUS')
        self.assertEqual(monitor.trust_block(), 'ESTIMATE_UNPAIRED')
        self.assertEqual(monitor.on_tf(6.0, corrected, odom), 'CONTINUOUS')
        self.assertIsNone(monitor.trust_block())

        self.assertEqual(monitor.on_amcl(7.0, (0.0, 0.0, 0.0), odom), 'JUMP')
        self.assertEqual(monitor.latched['source'], 'amcl')
        self.assertEqual(monitor.on_tf(8.2, (-1.0, 0.0, 0.0), odom), 'JUMP')
        self.assertEqual(monitor.latched['source'], 'amcl')

    # TF가 먼저 바뀌고 amcl_pose가 늦으면 이전 amcl 추정과 섞지 않는다(대응 실패로 보류).
    def test_tf_first_not_mixed(self):
        odom = history()
        monitor = established_monitor(odom)
        corrected = (MAP_ODOM[0] + 0.1, MAP_ODOM[1], 0.0)

        self.assertEqual(monitor.on_tf(6.0, corrected, odom), 'CONTINUOUS')
        self.assertEqual(monitor.trust_block(), 'ESTIMATE_UNPAIRED')

    # odom 대응 누락: 도약량을 확정하지 않고 DEGRADED(ODOM_STALE)·신뢰 기록 보류, 비교 기준 유지.
    # odom이 늦게 오면 다시 비교해 해소한다(정상이면 CONTINUOUS, 도약이면 latch).
    def test_odom_missing_or_late(self):
        for name, new_value, expected in (('정상', (MAP_ODOM[0] + 0.02, MAP_ODOM[1], 0.0), 'CONTINUOUS'),
                                          ('도약', (MAP_ODOM[0] + 2.0, MAP_ODOM[1], 0.0), 'JUMP')):
            with self.subTest(case=name):
                odom = history(0.0, 5.5)
                monitor = established_monitor(odom)

                self.assertEqual(monitor.on_tf(7.0, new_value, odom), 'UNVERIFIED')
                self.assertEqual(monitor.reference, MAP_ODOM)
                self.assertEqual(monitor.trust_block(), 'ESTIMATE_UNVERIFIED')
                health = monitor.apply(lh.Health(lh.OK, True, '', 0.05, 0.03))
                self.assertEqual((health.state, health.reason), (lh.DEGRADED, 'ODOM_STALE'))
                self.assertEqual(monitor.apply(lh.Health(lh.LOST, True, 'TF_MISSING')).state, lh.LOST)

                for index in range(1, 101):
                    odom.add(5.5 + 0.02 * index, ODOM)

                self.assertIn(('tf', expected), monitor.retry(odom, 7.5, 102.0, True))
                self.assertIsNone(monitor.unverified)

    # odom 표본이 추정 시각에서 match 허용(0.1 s)보다 멀면 대응 없음이다.
    def test_odom_lookup_limit(self):
        odom = history(0.0, 5.0)

        self.assertEqual(odom.lookup(5.09, 0.1), ODOM)
        self.assertIsNone(odom.lookup(5.2, 0.1))
        self.assertIsNone(odom.lookup(float('nan'), 0.1))

        # 보관 기간(5 s)보다 오래된 표본은 지우고, 시각이 역행하면 기록을 비운다.
        kept = history(0.0, 10.0, keep_s=CONFIG['estimate_jump']['odom_history_s'])
        self.assertIsNone(kept.lookup(4.0, 0.1))
        self.assertEqual(kept.lookup(6.0, 0.1), ODOM)
        kept.add(3.0, ODOM)
        self.assertIsNone(kept.lookup(6.0, 0.1))

    # 작은 공분산 도약: evaluate만 보면 OK지만 latch로 LOST이고 신뢰 기록을 덮어쓰지 않는다.
    def test_small_covariance_jump_blocks_trust(self):
        odom = history(0.0, 30.0)
        monitor = established_monitor(odom)
        tracker = trusted_tracker(10.0, monitor)
        self.assertIsNotNone(tracker.record)
        jumped = (MAP_ODOM[0] + 3.0, MAP_ODOM[1], 0.0)
        self.assertEqual(monitor.on_tf(19.9, jumped, odom), 'JUMP')

        inputs = good_inputs(20.0, map_odom=jumped, std=0.02, yaw_std=0.01)
        raw = lh.evaluate(inputs, 20.0, CONFIG)
        self.assertEqual(raw.state, lh.OK)
        health = monitor.apply(raw)
        block = monitor.trust_block()

        self.assertEqual(health.state, lh.LOST)
        self.assertFalse(tracker.consider(health, inputs, 20.0, 'map-a', False, block))
        self.assertFalse(tracker.consider(raw, inputs, 20.0, 'map-a', False, block))
        self.assertEqual(tracker.rejection(raw, inputs, 'map-a', False, block), 'ESTIMATE_DISCONTINUOUS')
        self.assertEqual(tracker.record.map_odom, MAP_ODOM)

    # 보존한 기록도 기존 나이·누적 이동 조건을 만족할 때만 재위치 후보가 된다(보존이 곧 허용은 아니다).
    def test_preserved_record_still_limited(self):
        tracker = trusted_tracker(10.0)
        candidate, code = lh.reinit_candidate(tracker, ODOM, 80.0, 'map-a', CONFIG)
        self.assertEqual((candidate, code), (None, 'REINIT_REJECTED_TRUSTED_POSE_EXPIRED'))

        tracker.on_odom((2.5, 0.0, 0.0))
        candidate, code = lh.reinit_candidate(tracker, (2.5, 0.0, 0.0), 20.0, 'map-a', CONFIG)
        self.assertEqual(code, 'REINIT_REJECTED_MOTION_TOO_LARGE')

    # 같은 stamp·다른 값 입력은 다음 새 입력까지 신뢰 기록을 보류한다.
    def test_conflict_blocks_until_new_input(self):
        odom = history()
        monitor = established_monitor(odom)
        monitor.conflict('amcl', 5.1)

        self.assertEqual(monitor.trust_block(), 'INPUT_CONFLICT')
        monitor.on_amcl(5.3, lh.compose(MAP_ODOM, ODOM), odom)
        self.assertIsNone(monitor.trust_block())

    # 설정: 새 키는 estimate_jump 묶음이며 재위치 consistency와 따로 바뀐다.
    def test_config_keys(self):
        config = lh.merge_config({'estimate_jump': {'position_m': 0.4}})

        self.assertEqual(config['estimate_jump']['position_m'], 0.4)
        self.assertEqual(config['reinit']['consistency_m'], 0.5)
        self.assertEqual((CONFIG['estimate_jump']['position_m'], CONFIG['estimate_jump']['yaw_rad']), (0.3, 0.3))

        with self.assertRaises(ValueError):
            lh.merge_config({'estimate_jump': {'unknown': 1.0}})


# 3단계: 의도된 초기화·재위치
class IntendedChanges(unittest.TestCase):

    # 정상 초기화: START_POSE 송신 뒤 새 TF가 후보와 맞으면 확인(established)·비교 기준 설정·의도 소진
    def test_start_pose_confirmed(self):
        odom = history()
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.expect(lh.compose(MAP_ODOM, ODOM), 'START_POSE', 5.0, 100.0, True)

        self.assertEqual(monitor.on_tf(5.1, MAP_ODOM, odom, 5.2, 100.2, True), 'INTENDED')
        self.assertTrue(monitor.established)
        self.assertIsNone(monitor.intent)

    # 최초 TF는 비교 기준일 뿐 초기화 확인이 아니다(의도 없음 → 신뢰 기록 불가).
    def test_first_tf_not_trust(self):
        odom = history()
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.on_amcl(5.0, lh.compose(MAP_ODOM, ODOM), odom)

        self.assertEqual(monitor.on_tf(5.0, MAP_ODOM, odom), 'BASELINE')
        self.assertEqual(monitor.trust_block(), 'NOT_ESTABLISHED')

    # 후보 불일치: 송신 뒤 첫 TF가 후보와 다르면 비교 기준으로만 두고 확인하지 않는다.
    def test_mismatch_not_established(self):
        odom = history()
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.expect((5.0, 5.0, 0.0), 'START_POSE', 5.0, 100.0, True)

        self.assertEqual(monitor.on_tf(5.1, MAP_ODOM, odom, 5.2, 100.2, True), 'BASELINE')
        self.assertFalse(monitor.established)
        self.assertEqual(monitor.trust_block(), 'NOT_ESTABLISHED')

    # 오래된 입력: 송신 전 시각의 TF는 후보와 맞아도 의도 확인이 아니다.
    def test_old_input_not_intended(self):
        odom = history()
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.on_tf(3.0, (0.0, 0.0, 0.0), odom)
        monitor.expect(lh.compose(MAP_ODOM, ODOM), 'REINIT', 5.0, 100.0, True)

        self.assertEqual(monitor.on_tf(4.0, MAP_ODOM, odom, 5.1, 100.1, True), 'JUMP')
        self.assertIsNotNone(monitor.intent)

    # 재위치 후보와 맞는 새 추정은 도약이 아니다(비교 기준 전환). 확인은 재위치 성공(release) 때다.
    def test_reinit_intent(self):
        odom = history()
        monitor = established_monitor(odom)
        target = (MAP_ODOM[0] + 1.0, MAP_ODOM[1], 0.0)
        monitor.expect(lh.compose(target, ODOM), 'REINIT', 6.0, 101.0, True)

        self.assertEqual(monitor.on_tf(6.2, target, odom, 6.3, 101.3, True), 'INTENDED')
        self.assertIsNone(monitor.latched)
        self.assertEqual(monitor.reference, target)

    # 후속 임의 도약: 의도를 한 번 쓴 뒤의 도약은 latch(예상 기간 전체를 면제하지 않는다).
    def test_later_jump_after_intent(self):
        odom = history()
        monitor = established_monitor(odom)

        self.assertEqual(monitor.on_tf(5.6, (MAP_ODOM[0] + 1.0, MAP_ODOM[1], 0.0), odom, 5.7, 100.7, True), 'JUMP')

    # 의도 기간 안이라도 후보와 다른 도약은 latch한다.
    def test_mismatching_jump_during_intent(self):
        odom = history()
        monitor = established_monitor(odom)
        monitor.expect(lh.compose(MAP_ODOM, ODOM), 'REINIT', 6.0, 101.0, True)

        self.assertEqual(monitor.on_tf(6.2, (MAP_ODOM[0] + 2.0, MAP_ODOM[1], 0.0), odom, 6.3, 101.3, True), 'JUMP')

    # 제한시간: sim time·단조 벽시계 상한 어느 쪽이 지나도 의도를 닫는다. 그 뒤 후보와 맞는 도약도 latch
    def test_intent_timeout(self):
        for name, now, wall in (('sim', 21.5, 101.0), ('wall', 6.5, 161.5)):
            with self.subTest(case=name):
                odom = history(0.0, 25.0)
                monitor = established_monitor(odom)
                target = (MAP_ODOM[0] + 1.0, MAP_ODOM[1], 0.0)
                monitor.expect(lh.compose(target, ODOM), 'REINIT', 6.0, 101.0, True)

                self.assertEqual(monitor.on_tf(now - 0.1, target, odom, now, wall, True), 'JUMP')
                self.assertIsNone(monitor.intent)

    # 시계 미준비: 송신 시각을 정할 수 없으면 의도를 기록하지 않고, 기록 뒤 시계가 무효가 되면 닫는다.
    def test_clock_not_ready(self):
        monitor = lh.EstimateMonitor(CONFIG)

        self.assertFalse(monitor.expect((0.0, 0.0, 0.0), 'START_POSE', 0.0, 1.0, False))
        self.assertIsNone(monitor.intent)
        monitor.expect((0.0, 0.0, 0.0), 'START_POSE', 5.0, 1.0, True)
        monitor.expire(5.1, 1.1, False)
        self.assertIsNone(monitor.intent)


# 3단계 보완(검토 hop 3): 기존 TF와 같은 초기 위치, launch initial_poses 후보, 후보 없음 모드
class InitialModes(unittest.TestCase):

    # [검토 재현] 이미 받은 비교 기준과 의도한 초기 위치가 같은 값이어도 odom·송신 뒤 시각·후보 일치로 확인한다.
    def test_unchanged_tf_still_confirms_intent(self):
        odom = history(pose=(0.0, 0.0, 0.0))
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.reference = (0.0, 0.0, 0.0)
        monitor.expect((0.0, 0.0, 0.0), 'START_POSE', 10.0, 100.0, True)

        self.assertEqual(monitor.on_tf(10.1, (0.0, 0.0, 0.0), odom, now=10.2, wall=100.2), 'INTENDED')
        self.assertTrue(monitor.established)
        self.assertIsNone(monitor.intent)

    # 같은 값이라도 송신 전 시각이거나 odom 대응이 없으면 확인하지 않는다(근거 부족으로 내리지도 않음).
    def test_unchanged_tf_old_or_no_odom(self):
        odom = history(0.0, 10.0, pose=(0.0, 0.0, 0.0))
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.reference = (0.0, 0.0, 0.0)
        monitor.expect((0.0, 0.0, 0.0), 'START_POSE', 10.0, 100.0, True)

        self.assertEqual(monitor.on_tf(9.9, (0.0, 0.0, 0.0), odom, now=10.2, wall=100.2), 'UNCHANGED')
        self.assertEqual(monitor.on_tf(10.5, (0.0, 0.0, 0.0), odom, now=10.6, wall=100.6), 'UNCHANGED')
        self.assertFalse(monitor.established)
        self.assertIsNone(monitor.unverified)

    # launch 초기 위치: TF만으로는 확인하지 않고, 기록 뒤 새 amcl_pose가 후보와 맞은 다음 TF로 확인한다.
    def test_launch_initial_pose(self):
        odom = history()
        start = lh.compose(MAP_ODOM, ODOM)
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.expect(start, 'LAUNCH_INITIAL_POSE', 5.0, 100.0, True)

        self.assertEqual(monitor.on_tf(5.1, MAP_ODOM, odom, 5.1, 100.1, True), 'BASELINE')
        self.assertEqual(monitor.on_tf(5.2, MAP_ODOM, odom, 5.2, 100.2, True), 'UNCHANGED')
        self.assertFalse(monitor.established)
        self.assertEqual(monitor.on_amcl(5.3, start, odom), 'INTENDED_PENDING')
        self.assertEqual(monitor.on_tf(5.4, MAP_ODOM, odom, 5.4, 100.4, True), 'INTENDED')
        self.assertTrue(monitor.established)
        self.assertIsNone(monitor.trust_block())

    # 감독 노드 재시작 한계 대응: 기록 전 stamp의 amcl_pose(transient local로 다시 받은 이전 추정)와
    # 재발행 TF만 있으면 후보와 같아도 초기화가 확인되지 않는다.
    def test_launch_initial_pose_old_estimate_only(self):
        odom = history(0.0, 30.0)
        start = lh.compose(MAP_ODOM, ODOM)
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.on_amcl(2.0, start, odom)
        monitor.expect(start, 'LAUNCH_INITIAL_POSE', 5.0, 100.0, True)

        for stamp in (5.1, 6.1, 7.1, 8.1):
            monitor.on_tf(stamp, MAP_ODOM, odom, stamp, 95.0 + stamp, True)

        self.assertFalse(monitor.established)
        self.assertEqual(monitor.trust_block(), 'NOT_ESTABLISHED')

    # launch 초기 위치 기준 구분: 기준 시각은 시계 유효(CLOCK_VALID) 기록 시각, 제한시간은 AMCL active 확인부터.
    # AMCL active 전에는 만료하지 않고, 확인 뒤 sim 15 s(또는 벽시계 60 s)가 지나면 닫는다.
    def test_launch_initial_pose_basis_and_window(self):
        odom = history(0.0, 40.0)
        start = lh.compose(MAP_ODOM, ODOM)
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.expect(start, 'LAUNCH_INITIAL_POSE', 5.0, 100.0, True)

        self.assertEqual((monitor.intent['basis'], monitor.intent['basis_at']), ('CLOCK_VALID', 5.0))
        monitor.expire(25.0, 170.0, True)
        self.assertIsNotNone(monitor.intent)

        monitor.amcl_active(25.0, 170.0)
        monitor.amcl_active(26.0, 171.0)
        self.assertEqual(monitor.intent['window_at'], 25.0)
        monitor.expire(39.0, 180.0, True)
        self.assertIsNotNone(monitor.intent)
        monitor.expire(40.5, 181.0, True)
        self.assertIsNone(monitor.intent)

        # 송신한 후보(START_POSE)는 기준·제한시간 모두 송신 시각부터
        monitor.expect(start, 'START_POSE', 5.0, 100.0, True)
        self.assertEqual((monitor.intent['basis'], monitor.intent['window_at']), ('SENT', 5.0))

    # launch 후보와 다른 새 추정은 확인하지 않는다.
    def test_launch_initial_pose_mismatch(self):
        odom = history()
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.expect((5.0, 5.0, 0.0), 'LAUNCH_INITIAL_POSE', 5.0, 100.0, True)
        monitor.on_amcl(5.3, lh.compose(MAP_ODOM, ODOM), odom)
        monitor.on_tf(5.4, MAP_ODOM, odom, 5.4, 100.4, True)

        self.assertFalse(monitor.established)

    # 후보 없음 모드(외부 initialpose·AMCL 자체 초기화): 첫 TF는 비교 기준일 뿐 확인이 아니며 신뢰 기록이 없다
    # (재위치는 NO_TRUSTED_POSE). 도약 감시는 그대로 한다.
    def test_no_candidate_mode(self):
        odom = history()
        monitor = lh.EstimateMonitor(CONFIG)
        monitor.on_amcl(5.0, lh.compose(MAP_ODOM, ODOM), odom)
        monitor.on_tf(5.0, MAP_ODOM, odom)
        tracker = lh.TrustedPoseTracker(CONFIG)
        inputs = good_inputs(10.0)

        self.assertFalse(tracker.consider(lh.evaluate(inputs, 10.0, CONFIG), inputs, 10.0, 'map-a', False,
                                          monitor.trust_block()))
        self.assertEqual(lh.reinit_candidate(tracker, ODOM, 10.0, 'map-a', CONFIG)[1],
                         'REINIT_REJECTED_NO_TRUSTED_POSE')
        self.assertEqual(monitor.on_tf(6.0, (MAP_ODOM[0] + 1.0, MAP_ODOM[1], 0.0), odom), 'JUMP')


# 4단계: latch 해제
class LatchRelease(unittest.TestCase):

    # 도약 latch 상태를 만든다: 10 s 신뢰 기록, 19.9 s 도약 TF, 20 s 입력(odom 1.2 m)
    def latched(self):
        odom = history(0.0, 40.0)
        monitor = established_monitor(odom)
        tracker = trusted_tracker(10.0, monitor)
        jumped = (MAP_ODOM[0] + 3.0, MAP_ODOM[1], 0.0)
        monitor.on_tf(19.9, jumped, odom)
        inputs = good_inputs(20.0, map_odom=jumped)

        return odom, monitor, tracker, inputs

    # 공분산 감소·시간 경과·중복 표본만으로는 해제하지 않는다.
    def test_no_release_without_reinit(self):
        odom, monitor, tracker, inputs = self.latched()

        for stamp in (21.0, 25.0, 25.0, 30.0):
            monitor.on_amcl(stamp, lh.compose(inputs.map_odom, ODOM), odom)
            monitor.on_tf(stamp, inputs.map_odom, odom)

        self.assertIsNotNone(monitor.latched)
        self.assertEqual(monitor.apply(lh.Health(lh.OK, True, '', 0.01, 0.01)).state, lh.LOST)

    # 성공 해제: 허용된 재위치가 새 추정·수렴·후보 일치·TF 일치·입력 신선도로 성공하면 latch를 풀고
    # 비교 기준을 새 추정으로 바꾼 뒤 신뢰 기록 갱신을 재개한다. 재위치 판정은 latch 반영 전 입력으로 한다.
    def test_success_releases(self):
        odom, monitor, tracker, inputs = self.latched()
        health = monitor.apply(lh.evaluate(inputs, 20.0, CONFIG))
        reinit = lh.Reinitializer(CONFIG)

        self.assertEqual(health.state, lh.LOST)
        self.assertEqual(reinit.request(health, inputs, tracker, 20.0, 'map-a', True), (True, 'REINIT_ACCEPTED'))
        candidate = reinit.attempt.candidate['pose']
        monitor.expect(candidate, 'REINIT', 20.0, 120.0, True)
        reinit.mark_sent(20.0)

        # AMCL이 후보로 돌아옴 → TF가 도약 전 비교 기준과 같은 값이어도 의도 확인은 한다(latch는 그대로)
        inputs.map_odom = MAP_ODOM
        self.assertEqual(monitor.on_tf(20.1, MAP_ODOM, odom, 20.1, 120.1, True), 'INTENDED')
        self.assertIsNotNone(monitor.latched)
        result = run_reinit(reinit, inputs, [(20.2 + 0.2 * i, candidate, 0.08) for i in range(3)])
        self.assertEqual(result, 'SUCCEEDED')

        monitor.on_amcl(20.6, candidate, odom)
        monitor.release(inputs.map_odom)
        health = reinit.apply(monitor.apply(lh.evaluate(inputs, 20.6, CONFIG)))
        self.assertEqual(health.state, lh.OK)
        self.assertIsNone(monitor.trust_block())
        self.assertTrue(tracker.consider(health, inputs, 20.6, 'map-a', False, monitor.trust_block()))
        self.assertEqual(monitor.reference, MAP_ODOM)

    # TF가 후보와 다르면(다른 갱신) 새 추정이 맞아도 재위치를 성공으로 보지 않는다.
    def test_tf_must_match_candidate(self):
        odom, monitor, tracker, inputs = self.latched()
        reinit = lh.Reinitializer(CONFIG)
        reinit.request(monitor.apply(lh.evaluate(inputs, 20.0, CONFIG)), inputs, tracker, 20.0, 'map-a', True)
        candidate = reinit.attempt.candidate['pose']
        reinit.mark_sent(20.0)

        result = run_reinit(reinit, inputs, [(20.2 + 0.2 * i, candidate, 0.08) for i in range(3)])
        self.assertIsNone(result)

    # 실패 유지: 실패 뒤 AMCL 회복 근거(recovered_after_failure)로 실패 표시가 지워져도 latch는 LOST를 유지한다.
    def test_failure_keeps_lost(self):
        odom, monitor, tracker, inputs = self.latched()
        reinit = lh.Reinitializer(CONFIG)
        reinit.request(monitor.apply(lh.evaluate(inputs, 20.0, CONFIG)), inputs, tracker, 20.0, 'map-a', True)
        reinit.mark_sent(20.0)
        inputs.actuation_stamp = 36.0
        self.assertEqual(reinit.step(inputs, 36.0), 'REINIT_FAILED_AMCL_UNAVAILABLE')
        monitor.cancel_intent('REINIT_FAILED_AMCL_UNAVAILABLE')

        for stamp in (37.0, 37.2, 37.4):
            reinit.on_amcl(stamp, (0.0, 0.0, 0.0), lh.pose_covariance(0.05, 0.03))

        good = lh.evaluate(good_inputs(37.5, map_odom=inputs.map_odom), 37.5, CONFIG)
        health = reinit.apply(monitor.apply(good))
        self.assertEqual(health.state, lh.LOST)
        self.assertIn('ESTIMATE_DISCONTINUOUS', health.reason)

        # 기존 회복 근거 경로로 실패 표시가 지워져도 latch는 남는다.
        cleared = reinit.apply(good)
        self.assertEqual(cleared.state, lh.OK)
        self.assertIsNone(reinit.failure)
        self.assertEqual(monitor.apply(cleared).state, lh.LOST)

    # 오래된 후보 거절: 기록이 만료되면 재위치를 접수하지 않으므로 latch가 남는다.
    def test_expired_candidate_rejected(self):
        odom, monitor, tracker, inputs = self.latched()
        late = good_inputs(75.0, map_odom=inputs.map_odom)
        reinit = lh.Reinitializer(CONFIG)

        self.assertEqual(reinit.request(monitor.apply(lh.evaluate(late, 75.0, CONFIG)), late, tracker, 75.0,
                                        'map-a', True), (False, 'REINIT_REJECTED_TRUSTED_POSE_EXPIRED'))
        self.assertIsNotNone(monitor.latched)

    # 우선순위: 시계 무효면 UNKNOWN·valid=false를 유지하고 latch 사유만 덧붙인다.
    # odom 끊김·지도 변경은 latch 중에도 신뢰 기록을 무효로 하고, 해제해도 되살리지 않는다.
    def test_priorities(self):
        odom, monitor, tracker, inputs = self.latched()
        health = monitor.apply(lh.evaluate(inputs, 20.0, CONFIG, clock_valid=False))

        self.assertEqual((health.state, health.valid), (lh.UNKNOWN, False))
        self.assertEqual(health.reason, 'CLOCK_INVALID,ESTIMATE_DISCONTINUOUS')

        for reason, action in (('ODOM_JUMP', lambda t: t.invalidate('ODOM_JUMP')),
                               ('MAP_CHANGED', lambda t: t.check_map('map-b'))):
            with self.subTest(case=reason):
                odom, monitor, tracker, inputs = self.latched()
                action(tracker)
                self.assertIsNone(tracker.record)
                self.assertEqual(lh.reinit_candidate(tracker, ODOM, 20.0, 'map-a', CONFIG)[0], None)
                monitor.release(MAP_ODOM)
                self.assertIsNone(tracker.record)
                self.assertEqual(tracker.invalid_reason, reason)

    # latch 사유 순서: 재위치 진행 표시와 함께면 진행 표시가 앞이다.
    def test_reason_order_with_reinit(self):
        odom, monitor, tracker, inputs = self.latched()
        reinit = lh.Reinitializer(CONFIG)
        reinit.request(monitor.apply(lh.evaluate(inputs, 20.0, CONFIG)), inputs, tracker, 20.0, 'map-a', True)
        health = reinit.apply(monitor.apply(lh.evaluate(inputs, 20.0, CONFIG)))

        self.assertEqual(health.reason, 'REINIT_IN_PROGRESS,ESTIMATE_DISCONTINUOUS')


if __name__ == '__main__':
    unittest.main()
