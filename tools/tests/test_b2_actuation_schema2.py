# B2 위치 감독의 A4 ActuationState Schema 2 소비 규칙(localization_health.ActuationGate·apply_actuation)을 ROS 없이 검사한다.
# 수신 단계: robot·schema·source·session·revision·stamp 유한·양수·은퇴 세션·감소만 거부하고 미래 stamp는 받는다.
# 판정 단계: actuation_problem이 만료·미래·무효를 ACTUATION_STALE, STOPPED 아님을 NOT_STOPPED로 낸다.
# 실제 A4·Gazebo 통합 시험이 아니다.
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_localization/scripts'))

import localization_health as lh  # noqa: E402

CONFIG = lh.merge_config()
ROBOT = 'robot_2'
RUNNING = 1


# 재위치 후보 판단에 필요한 정상 입력(구동 상태는 아직 없음)
def base_inputs(now=10.0, odom=(1.0, 0.0, 0.0), map_odom=(0.5, 0.2, 0.0)):
    return lh.Inputs(
        scan_stamp=now - 0.1, odom_stamp=now - 0.02, odom_pose=odom,
        tf_valid_time=now - 0.1, map_odom=map_odom, tf_chain_ok=True,
        amcl_stamp=now - 5.0, amcl_pose=lh.compose(map_odom, odom),
        amcl_covariance=tuple(lh.pose_covariance(0.05, 0.03)), amcl_odom_pose=odom, amcl_updates=3)


# 구동 상태 메시지 필드 묶음(기본은 정상 Schema 2 STOPPED)
def message(stamp, revision=1, session='s-a', state=lh.ACTUATION_STOPPED, valid=True, robot_id=ROBOT,
            schema_version=2, source_id='a4_safety_gate'):
    return dict(robot_id=robot_id, schema_version=schema_version, source_id=source_id, session=session,
                revision=revision, stamp=stamp, valid=valid, state=state)


# gate·inputs에 메시지 한 건을 반영하고 판정을 돌려준다.
def feed(gate, inputs, fields):
    return lh.apply_actuation(gate, inputs, fields['robot_id'], fields['schema_version'], fields['source_id'],
                              fields['session'], fields['revision'], fields['stamp'], fields['valid'],
                              fields['state'])


# 수신 단계 거부·수용 규칙
class ActuationGateRules(unittest.TestCase):

    # 새 gate·입력을 만들고 정상 Schema 2 STOPPED 한 건을 받아 둔다.
    def setUp(self):
        self.gate = lh.ActuationGate(ROBOT)
        self.inputs = base_inputs()
        self.assertEqual(feed(self.gate, self.inputs, message(9.9, revision=5)), 'ACCEPTED')

    # 거부 메시지는 상태·유효성·stamp를 바꾸지 않는다.
    def assert_unchanged(self, fields, verdict):
        before = (self.inputs.actuation_state, self.inputs.actuation_valid, self.inputs.actuation_stamp)

        self.assertEqual(feed(self.gate, self.inputs, fields), verdict)
        self.assertEqual((self.inputs.actuation_state, self.inputs.actuation_valid, self.inputs.actuation_stamp),
                         before)

    # 정상 Schema 2는 상태·stamp를 반영하고, 허가 필드와 무관하게 STOPPED만으로 재위치 정지 조건을 만족한다.
    def test_accept_schema2(self):
        self.assertEqual(self.inputs.actuation_state, lh.ACTUATION_STOPPED)
        self.assertTrue(self.inputs.actuation_valid)
        self.assertEqual(self.inputs.actuation_stamp, 9.9)
        self.assertIsNone(lh.actuation_problem(self.inputs, 10.0, CONFIG))

    # 각 거부 사유: 다른 robot, Schema 1·0, 빈 source·session, revision 0, 비유한·0 이하 stamp
    def test_rejections(self):
        cases = [
            ('다른 robot', message(10.0, revision=6, robot_id='robot_1'), 'WRONG_ROBOT'),
            ('Schema 1', message(10.0, revision=6, schema_version=1), 'WRONG_SCHEMA'),
            ('Schema 0', message(10.0, revision=6, schema_version=0), 'WRONG_SCHEMA'),
            ('빈 source', message(10.0, revision=6, source_id=''), 'NO_SOURCE'),
            ('빈 session', message(10.0, revision=6, session=''), 'NO_SESSION'),
            ('revision 0', message(10.0, revision=0), 'REVISION_ZERO'),
            ('NaN stamp', message(math.nan, revision=6), 'INVALID_STAMP'),
            ('inf stamp', message(math.inf, revision=6), 'INVALID_STAMP'),
            ('0 stamp', message(0.0, revision=6), 'INVALID_STAMP'),
            ('음수 stamp', message(-1.0, revision=6), 'INVALID_STAMP'),
        ]

        for name, fields, verdict in cases:
            with self.subTest(name):
                self.assert_unchanged({**fields, 'state': RUNNING}, verdict)

    # 같은 세션에서 revision 감소·stamp 감소는 폐기, 같은 revision·늦은 stamp 재발행은 수용한다.
    def test_order_same_session(self):
        self.assert_unchanged(message(10.0, revision=4, state=RUNNING), 'REGRESSED')
        self.assert_unchanged(message(9.8, revision=6, state=RUNNING), 'REGRESSED')

        self.assertEqual(feed(self.gate, self.inputs, message(10.0, revision=5)), 'ACCEPTED')
        self.assertEqual(self.inputs.actuation_stamp, 10.0)
        self.assertEqual(feed(self.gate, self.inputs, message(10.0, revision=5)), 'ACCEPTED')

    # 새 세션은 revision이 작아도 받아들이고, 이전 세션이 다시 오면 은퇴 세션으로 폐기한다.
    def test_new_session_retires_old(self):
        self.assertEqual(feed(self.gate, self.inputs, message(10.0, revision=1, session='s-b', state=RUNNING)),
                         'ACCEPTED')
        self.assertEqual(self.inputs.actuation_state, RUNNING)

        self.assert_unchanged(message(10.1, revision=99, session='s-a'), 'RETIRED_SESSION')
        self.assertEqual(lh.actuation_problem(self.inputs, 10.1, CONFIG), 'NOT_STOPPED')

    # 미래 stamp는 수신에서 버리지 않는다. 판정 시점에 아직 미래면 ACTUATION_STALE, 시계가 따라오면 정상이다.
    def test_future_stamp_kept_and_judged_at_eval(self):
        self.assertEqual(feed(self.gate, self.inputs, message(10.2, revision=6)), 'ACCEPTED')

        self.assertTrue(lh.actuation_future(self.inputs, 10.1))
        self.assertEqual(lh.actuation_problem(self.inputs, 10.1, CONFIG), 'ACTUATION_STALE')
        self.assertFalse(lh.actuation_future(self.inputs, 10.2))
        self.assertIsNone(lh.actuation_problem(self.inputs, 10.2, CONFIG))

    # 식별·순서를 통과한 valid=false는 '무효 상태'로 즉시 반영한다(직전 STOPPED를 남기지 않음, state 버림).
    def test_invalid_meta_fail_closed(self):
        self.assertEqual(feed(self.gate, self.inputs, message(10.0, revision=6, valid=False)), 'ACCEPTED')

        self.assertFalse(self.inputs.actuation_valid)
        self.assertIsNone(self.inputs.actuation_state)
        self.assertEqual(self.inputs.actuation_stamp, 10.0)
        self.assertEqual(lh.actuation_problem(self.inputs, 10.0, CONFIG), 'ACTUATION_STALE')

    # 거부 메시지가 계속 와도 마지막 수용 값은 actuation_max_age_s(0.3 s) 뒤 만료된다(수명 연장 없음).
    def test_rejected_does_not_extend(self):
        for index in range(1, 6):
            feed(self.gate, self.inputs, message(9.9 + 0.1 * index, revision=6 + index, schema_version=1))

        self.assertEqual(self.inputs.actuation_stamp, 9.9)
        self.assertIsNone(lh.actuation_problem(self.inputs, 10.1, CONFIG))
        self.assertEqual(lh.actuation_problem(self.inputs, 10.3, CONFIG), 'ACTUATION_STALE')

    # 판정 횟수가 사유별로 쌓인다(진단용).
    def test_counts(self):
        feed(self.gate, self.inputs, message(10.0, revision=6, schema_version=1))
        feed(self.gate, self.inputs, message(10.0, revision=6, schema_version=1))

        self.assertEqual(self.gate.counts['ACCEPTED'], 1)
        self.assertEqual(self.gate.counts['WRONG_SCHEMA'], 2)


# 재위치 접수 판단과 Schema 2 STOPPED 의미
class ReinitWithSchema2(unittest.TestCase):

    # 신뢰 위치 기록이 있고 지금 TF가 만료된(LOST) 상태를 만든다.
    def prepared(self):
        tracker = lh.TrustedPoseTracker(CONFIG)
        first = base_inputs()
        first.actuation_state, first.actuation_valid, first.actuation_stamp = lh.ACTUATION_STOPPED, True, 9.95
        tracker.consider(lh.evaluate(first, 10.0, CONFIG), first, 10.0, 'map-a', False)
        later = base_inputs(now=20.0, odom=(1.2, 0.0, 0.0))
        later.tf_valid_time = 15.0
        health = lh.evaluate(later, 20.0, CONFIG)

        return tracker, later, lh.Reinitializer(CONFIG), health, lh.ActuationGate(ROBOT)

    # STOPPED가 아니면 허가 필드와 무관하게 거절이 유지된다(RUNNING·STOPPING·UNKNOWN). 허가는 입력에도 없다.
    def test_not_stopped_rejected(self):
        tracker, inputs, reinit, health, gate = self.prepared()

        for revision, state in enumerate((RUNNING, 2, 0), start=1):
            with self.subTest(state=state):
                self.assertEqual(feed(gate, inputs, message(19.95, revision=revision, state=state)), 'ACCEPTED')
                self.assertEqual(reinit.request(health, inputs, tracker, 20.0, 'map-a', True),
                                 (False, 'REINIT_REJECTED_NOT_STOPPED'))
                self.assertFalse(reinit.active())

    # Schema 1 STOPPED만 받은 경우는 구동 상태가 없으므로 ACTUATION_STALE로 거절한다.
    def test_schema1_only_stale(self):
        tracker, inputs, reinit, health, gate = self.prepared()

        self.assertEqual(feed(gate, inputs, message(19.95, schema_version=1)), 'WRONG_SCHEMA')
        self.assertEqual(reinit.request(health, inputs, tracker, 20.0, 'map-a', True),
                         (False, 'REINIT_REJECTED_ACTUATION_STALE'))

    # 정상 Schema 2 STOPPED면 접수하고, 진행 중 RUNNING이 오면 REINIT_FAILED_NOT_STOPPED로 끝난다.
    def test_stopped_accepted_then_lost(self):
        tracker, inputs, reinit, health, gate = self.prepared()

        self.assertEqual(health.state, lh.LOST)
        self.assertEqual(feed(gate, inputs, message(19.95, revision=3)), 'ACCEPTED')
        self.assertEqual(reinit.request(health, inputs, tracker, 20.0, 'map-a', True), (True, 'REINIT_ACCEPTED'))
        reinit.mark_sent(20.0)

        self.assertEqual(feed(gate, inputs, message(20.05, revision=4, state=RUNNING)), 'ACCEPTED')
        self.assertEqual(reinit.step(inputs, 20.1, True), 'REINIT_FAILED_NOT_STOPPED')


if __name__ == '__main__':
    unittest.main()
