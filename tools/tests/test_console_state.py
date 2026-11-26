"""관제 화면 판정(console_state·console_messages) 단위 시험. ROS 실행 없음(메시지 상수 대조만 ax_interfaces 사용).

확인: 5상태(미수집·정상·만료·무효·원천 없음 목록), 표본 나이·무수신 시간 둘 다 만료, 미래 stamp·stamp 0 거부,
같은 세션 revision·stamp 감소 폐기, 같은 revision 재발행 갱신, 이전 세션 폐기, revision 0·시계 무효,
표시 여유(판정 유지), 위치 없음일 때 좌표 미전송, KPI(가동 로봇·작업 수·값 없음), enum 이름표 일치.
"""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_operations_console'))

from ax_operations_console import console_messages as cm  # noqa: E402
from ax_operations_console import console_state as cs  # noqa: E402
from ax_operations_console.console_state import Clocks, ConsoleState, Sample  # noqa: E402

ROBOTS = [{'robot_id': 'robot_1', 'role': 'MANIPULATOR', 'spare': False},
          {'robot_id': 'robot_2', 'role': 'TRANSPORTER', 'spare': True}]


# meta 있는 표본
def sample(stamp, revision=1, session='s1', valid=True, value=None):
    value = {'state': 'RUNNING'} if value is None else value
    return Sample(value=value, stamp=stamp, session=session, revision=revision, valid=valid,
                  reason='' if valid else 'BAD', source_id='src', schema=2)


# 작업 목록 표본
def task_sample(stamp, rows, revision=1):
    return Sample(value=rows, stamp=stamp, session='d', revision=revision, valid=True,
                  source_id='d2', schema=1)


# 판정 도우미: 슬롯 하나를 지정 시각에 판정
def judge(state, robot, kind, sim, boot):
    slot = state.slots[robot][kind]
    return cs.judge(slot, Clocks(sim, boot), state.limits[kind], state.display_grace_s)


class StatusTest(unittest.TestCase):
    """5상태와 만료 판정."""

    # 기본 상태
    def setUp(self):
        self.state = ConsoleState(ROBOTS, 'dev_config', future_tolerance_s=0.1,
                                  display_grace_s=0.0)

    # 받은 적 없으면 미수집이고 값이 없다
    def test_missing(self):
        out = judge(self.state, 'robot_1', 'actuation', 100.0, 50.0)
        self.assertEqual((out['status'], out['display_status']), ('missing', 'missing'))
        self.assertNotIn('value', out)

    # 신선하면 정상과 값
    def test_ok(self):
        self.state.update_robot('robot_1', 'actuation', sample(100.0), Clocks(100.0, 50.0))
        out = judge(self.state, 'robot_1', 'actuation', 100.2, 50.2)
        self.assertEqual(out['status'], 'ok')
        self.assertEqual(out['value'], {'state': 'RUNNING'})

    # 무수신 시간(CLOCK_BOOTTIME)만 넘어도 만료
    def test_stale_by_receive_age(self):
        self.state.update_robot('robot_1', 'actuation', sample(100.0), Clocks(100.0, 50.0))
        out = judge(self.state, 'robot_1', 'actuation', 100.1, 50.31)
        self.assertEqual(out['status'], 'stale')
        self.assertNotIn('value', out)

    # 표본 나이(sim)만 넘어도 만료(늦게 도착한 오래된 표본)
    def test_stale_by_stamp_age(self):
        self.state.update_robot('robot_1', 'actuation', sample(99.0), Clocks(100.0, 50.0))
        out = judge(self.state, 'robot_1', 'actuation', 100.0, 50.0)
        self.assertEqual(out['status'], 'stale')

    # valid=false는 무효(사유 노출), 값 없음
    def test_invalid_producer(self):
        self.state.update_robot('robot_1', 'actuation', sample(100.0, valid=False),
                                Clocks(100.0, 50.0))
        out = judge(self.state, 'robot_1', 'actuation', 100.0, 50.0)
        self.assertEqual((out['status'], out['reason']), ('invalid', 'BAD'))
        self.assertNotIn('value', out)

    # 음수 표본 나이(시계 역행): 표시 나이 0·플래그, 판정은 정상 그대로, 원값 유지
    def test_negative_stamp_age_display(self):
        self.state.update_robot('robot_1', 'actuation', sample(100.0), Clocks(100.0, 50.0))
        out = judge(self.state, 'robot_1', 'actuation', 98.8, 50.1)
        self.assertEqual(out['status'], 'ok')
        self.assertEqual((out['stamp_age_s'], out['stamp_age_shown_s']), (-1.2, 0.0))
        self.assertTrue(out['clock_regressed'])

        normal = judge(self.state, 'robot_1', 'actuation', 100.2, 50.2)
        self.assertEqual((normal['stamp_age_shown_s'], normal['clock_regressed']), (0.2, False))

    # revision 0은 무효
    def test_revision_zero_invalid(self):
        self.state.update_robot('robot_1', 'actuation', sample(100.0, revision=0),
                                Clocks(100.0, 50.0))
        out = judge(self.state, 'robot_1', 'actuation', 100.0, 50.0)
        self.assertEqual((out['status'], out['reason']), ('invalid', 'REVISION_ZERO'))

    # sim 시계가 0이면 시계 무효
    def test_clock_invalid(self):
        self.state.update_robot('robot_1', 'actuation', sample(100.0), Clocks(0.0, 50.0))
        out = judge(self.state, 'robot_1', 'actuation', 0.0, 50.0)
        self.assertEqual((out['status'], out['reason']), ('invalid', 'CLOCK_INVALID'))

    # 원천 없음 목록을 상태에 함께 낸다
    def test_no_source_list(self):
        built = self.state.build_state(Clocks(100.0, 50.0))
        self.assertIn('/ax/fleet/mode', built['no_source'])
        self.assertEqual(built['robot_source'], 'dev_config')


class OrderingTest(unittest.TestCase):
    """계약 2.3 순서·세션·시각 규칙."""

    # 슬롯 하나 준비
    def setUp(self):
        self.slot = cs.Slot('actuation')

    # 반영 도우미
    def put(self, smp, sim=200.0, boot=10.0):
        return self.slot.update(smp, sim, boot, 0.1)

    # 미래 stamp(허용 오차 초과)는 받지 않고, 허용 오차 안은 받는다
    def test_future_stamp(self):
        self.assertFalse(self.put(sample(200.2)))
        self.assertEqual(self.slot.counts['future_stamp'], 1)
        self.assertIsNone(self.slot.sample)
        self.assertTrue(self.put(sample(200.05)))

    # stamp 0은 받지 않는다
    def test_zero_stamp(self):
        self.assertFalse(self.put(sample(0.0)))
        self.assertEqual(self.slot.counts['zero_stamp'], 1)

    # 같은 세션 revision 감소는 버린다
    def test_revision_decrease(self):
        self.put(sample(100.0, revision=5, value={'v': 5}))
        self.assertFalse(self.put(sample(101.0, revision=4, value={'v': 4})))
        self.assertEqual(self.slot.sample.value, {'v': 5})
        self.assertEqual(self.slot.counts['old_revision'], 1)

    # 같은 세션 stamp 감소는 버린다
    def test_stamp_decrease(self):
        self.put(sample(100.0, revision=5))
        self.assertFalse(self.put(sample(99.0, revision=6)))
        self.assertEqual(self.slot.counts['old_stamp'], 1)

    # 같은 revision인데 stamp가 1.2 s 줄어든 재발행(시스템 시계 역행): 받고 수신 시각만 갱신,
    # stamp는 큰 값 유지, 전용 횟수 1
    def test_same_revision_stamp_regressed(self):
        self.put(sample(100.0, revision=5, value={'v': 'first'}), boot=10.0)
        self.assertTrue(self.put(sample(98.8, revision=5, value={'v': 'changed'}), boot=11.0))
        self.assertEqual(self.slot.sample.value, {'v': 'first'})
        self.assertEqual((self.slot.sample.stamp, self.slot.recv_boot), (100.0, 11.0))
        self.assertEqual(self.slot.counts['stamp_regressed_same_revision'], 1)
        self.assertEqual(self.slot.counts['old_stamp'], 0)

        clocks = Clocks(100.0, 11.0)
        out = cs.judge(self.slot, clocks, 3.0, 0.0)
        self.assertEqual((out['recv_age_s'], out['stamp_age_s'], out['status']), (0.0, 0.0, 'ok'))

    # revision이 오르고 stamp가 줄면 버린다(기존 규칙)
    def test_new_revision_stamp_decrease_rejected(self):
        self.put(sample(100.0, revision=5))
        self.assertFalse(self.put(sample(98.8, revision=6)))
        self.assertEqual(self.slot.counts['old_stamp'], 1)
        self.assertEqual(self.slot.sample.revision, 5)

    # revision이 줄면 stamp와 관계없이 버린다(기존 규칙)
    def test_revision_decrease_rejected_any_stamp(self):
        self.put(sample(100.0, revision=5))
        self.assertFalse(self.put(sample(101.0, revision=4)))
        self.assertFalse(self.put(sample(98.0, revision=4)))
        self.assertEqual(self.slot.counts['old_revision'], 2)

    # 같은 revision 재발행: 내용은 그대로, stamp·수신 시각 갱신
    def test_same_revision_republish(self):
        self.put(sample(100.0, revision=5, value={'v': 'first'}), boot=10.0)
        self.assertTrue(self.put(sample(101.0, revision=5, value={'v': 'changed'}), boot=11.0))
        self.assertEqual(self.slot.sample.value, {'v': 'first'})
        self.assertEqual((self.slot.sample.stamp, self.slot.recv_boot), (101.0, 11.0))
        self.assertEqual(self.slot.counts['republished'], 1)

    # 새 세션은 revision이 작아도 받고, 그 뒤 이전 세션 메시지는 버린다
    def test_session_change(self):
        self.put(sample(100.0, revision=50, session='old'))
        self.assertTrue(self.put(sample(101.0, revision=1, session='new')))
        self.assertFalse(self.put(sample(102.0, revision=51, session='old')))
        self.assertEqual(self.slot.counts['old_session'], 1)
        self.assertEqual(self.slot.sample.session, 'new')

    # meta 없는 표준 메시지도 stamp 감소는 버린다
    def test_no_meta_stamp_decrease(self):
        self.put(Sample(value={'x': 1}, stamp=100.0))
        self.assertFalse(self.put(Sample(value={'x': 0}, stamp=99.5)))
        self.assertTrue(self.put(Sample(value={'x': 2}, stamp=100.5)))


class DisplayGraceTest(unittest.TestCase):
    """표시 여유: 판정 status는 계약값, display_status만 잠시 유지."""

    # 만료 직후 grace 안에서는 표시 정상·판정 만료, grace 뒤에는 둘 다 만료
    def test_grace(self):
        state = ConsoleState(ROBOTS, 'dev_config', display_grace_s=0.5)
        state.update_robot('robot_1', 'actuation', sample(100.0), Clocks(100.0, 50.0))
        first = judge(state, 'robot_1', 'actuation', 100.4, 50.4)
        self.assertEqual((first['status'], first['display_status']), ('stale', 'ok'))
        self.assertIn('value', first)
        later = judge(state, 'robot_1', 'actuation', 101.0, 51.0)
        self.assertEqual((later['status'], later['display_status']), ('stale', 'stale'))
        self.assertNotIn('value', later)

    # 무효 표본은 grace로 정상 표시하지 않는다
    def test_no_grace_for_invalid(self):
        state = ConsoleState(ROBOTS, 'dev_config', display_grace_s=0.5)
        state.update_robot('robot_1', 'actuation', sample(100.0, valid=False),
                           Clocks(100.0, 50.0))
        out = judge(state, 'robot_1', 'actuation', 100.4, 50.4)
        self.assertEqual(out['display_status'], 'stale')


class BuildStateTest(unittest.TestCase):
    """화면 상태 조립·KPI·위치 미전송."""

    # 상태 준비
    def setUp(self):
        self.state = ConsoleState(ROBOTS, 'dev_config', display_grace_s=0.0)
        self.clocks = Clocks(100.0, 50.0)

    # 아무것도 없으면 KPI 값 없음
    def test_kpi_missing(self):
        kpi = self.state.build_state(self.clocks)['kpi']
        self.assertIsNone(kpi['running'])
        self.assertIsNone(kpi['task_by_state'])
        self.assertEqual(kpi['task_status'], 'missing')

    # 가동 로봇: 신선·유효·FAULT 아님만 센다
    def test_kpi_running(self):
        running = sample(100.0, value={'state': 'RUNNING'})
        self.state.update_robot('robot_1', 'actuation', running, self.clocks)
        self.state.update_robot('robot_2', 'actuation', sample(100.0, value={'state': 'FAULT'}),
                                self.clocks)
        kpi = self.state.build_state(self.clocks)['kpi']
        self.assertEqual((kpi['running'], kpi['registered']), (1, 2))

    # 작업 상태별 수와 로봇 배정(미종료 작업만)
    def test_tasks_and_assignment(self):
        pair = {'manipulator': 'robot_1', 'transporter': 'robot_2'}
        rows = [dict(pair, task_id='t1', state='RUNNING', transfer='LOCKED'),
                dict(pair, task_id='t0', state='COMPLETED', transfer='UNLOCKED'),
                {'task_id': 't2', 'state': 'QUEUED', 'transfer': 'NONE', 'manipulator': '',
                 'transporter': ''}]
        self.state.update_tasks(task_sample(100.0, rows), self.clocks)
        built = self.state.build_state(self.clocks)
        self.assertEqual(built['kpi']['task_by_state'],
                         {'RUNNING': 1, 'COMPLETED': 1, 'QUEUED': 1})
        mine = built['robots'][0]['assignment']['tasks']
        self.assertEqual([t['task_id'] for t in mine], ['t1'])

    # 작업 목록이 만료되면 배정도 값이 없다
    def test_assignment_follows_tasks(self):
        self.state.update_tasks(task_sample(100.0, []), self.clocks)
        built = self.state.build_state(Clocks(104.0, 54.0))
        self.assertEqual(built['robots'][0]['assignment']['display_status'], 'stale')
        self.assertNotIn('tasks', built['robots'][0]['assignment'])

    # 위치가 만료·미수집이면 좌표를 보내지 않는다
    def test_pose_not_sent_without_fresh_value(self):
        built = self.state.build_state(self.clocks)
        self.assertNotIn('value', built['robots'][0]['items']['pose'])
        pose = Sample(value={'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'frame': 'map'}, stamp=100.0)
        self.state.update_robot('robot_1', 'pose', pose, self.clocks)
        self.assertIn('value', self.state.build_state(self.clocks)['robots'][0]['items']['pose'])
        stale = self.state.build_state(Clocks(106.0, 56.0))['robots'][0]['items']['pose']
        self.assertEqual(stale['status'], 'stale')
        self.assertNotIn('value', stale)


class MessageTest(unittest.TestCase):
    """메시지 요약과 enum 이름표."""

    # enum 이름표가 ax_interfaces 상수와 같다
    def test_enum_tables_match_interfaces(self):
        from ax_interfaces.msg import ActuationState, GroupTaskStatus, LocalizationHealth
        from ax_interfaces.msg import ResultStatus, SafetyState
        tables = [(ActuationState, cs.ACTUATION_NAMES), (SafetyState, cs.SAFETY_NAMES),
                  (LocalizationHealth, cs.HEALTH_NAMES), (ResultStatus, cs.RESULT_NAMES)]

        for msg_type, names in tables:
            for index, name in enumerate(names):
                self.assertEqual(getattr(msg_type, name), index, f'{msg_type.__name__}.{name}')

        for index, name in enumerate(cs.TASK_NAMES):
            self.assertEqual(getattr(GroupTaskStatus, name), index)
        for index, name in enumerate(cs.TRANSFER_NAMES):
            self.assertEqual(getattr(GroupTaskStatus, 'TRANSFER_' + name), index)

    # 범위 밖 enum은 ?값으로 표시(정상으로 보이지 않음)
    def test_enum_out_of_range(self):
        self.assertEqual(cs.enum_name(cs.ACTUATION_NAMES, 9), '?9')

    # 배터리 NaN은 값 없음
    def test_battery_nan(self):
        header = NS(stamp=NS(sec=1, nanosec=0), frame_id='')
        msg = NS(percentage=float('nan'), present=True, header=header)
        self.assertIsNone(cm.battery(msg).value['percentage'])

    # meta 변환: session·revision·valid
    def test_meta_conversion(self):
        meta = NS(header=NS(stamp=NS(sec=10, nanosec=500000000)), source_session_id='abc',
                  revision=7, valid=False, validity_reason='X', source_id='a4', schema_version=2)
        msg = NS(meta=meta, state=4, drive_allowed=False, arm_allowed=True,
                 block_reasons=['HANDOFF_LOCK'], linear_speed=0.0)
        out = cm.actuation(msg)
        self.assertEqual((out.stamp, out.session, out.revision, out.valid),
                         (10.5, 'abc', 7, False))
        self.assertEqual(out.value['state'], 'STOPPED')


if __name__ == '__main__':
    unittest.main()
