"""D3의 A4 Schema 2 수신 경계값·세션 교체·정지 증거를 ROS 그래프 없이 검증합니다."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_d3_team_inputs as fixtures
from ax_corridor_manager.node import CorridorNode
from ax_interfaces.msg import ActuationState


class StopMetadataTests(unittest.TestCase):
    """실제 D3 원장과 수신 콜백을 사용하며 상대 A4 코드는 변경하지 않습니다."""

    def setUp(self):
        """기존 D3 시험의 격리 원장·시계를 준비합니다."""
        fixtures.D3TeamInputTests.setUp(self)

    def message(self, session='a4-session', revision=1):
        """생산자가 0.5초 정지를 확인했다고 보고하는 정상 형식의 시험 메시지를 만듭니다."""
        message = ActuationState(robot_id='robot_1', state=ActuationState.STOPPED)
        fixtures.D3TeamInputTests.meta(self, message.meta)
        message.meta.schema_version = 2
        message.meta.source_id = 'a4_safety_gate'
        message.meta.source_session_id = session
        message.meta.revision = revision
        return message

    def receive(self, message):
        """정지 여부와 유효기한을 실제 수신 경로로 읽습니다."""
        CorridorNode.on_stop(self.node, message, 'robot_1')
        return self.manager.stops['robot_1']

    def test_exact_expiry_boundary_uses_integer_nanoseconds(self):
        """정확히 300ms 된 표본도 부동소수점 오차로 새 표본이 되어서는 안 됩니다."""
        for nanosec, expected in ((700_000_001, True), (700_000_000, False),
                                 (699_999_999, False)):
            with self.subTest(nanosec=nanosec):
                message = self.message()
                message.meta.header.stamp.sec = 99
                message.meta.header.stamp.nanosec = nanosec
                confirmed, _ = self.receive(message)
                self.assertEqual(confirmed, expected)

    def test_blank_identifiers_are_not_identity(self):
        """공백만 있는 출처·세션은 권한 근거가 아니며 정상 세션을 교체하지 않습니다."""
        for field in ('source_id', 'source_session_id'):
            for value in ('', ' ', '\t'):
                with self.subTest(field=field, value=value):
                    self.receive(self.message())
                    bad = self.message()
                    setattr(bad.meta, field, value)
                    self.assertFalse(self.receive(bad)[0])
                    self.assertEqual(self.node.sources[('stop', 'robot_1')][0], 'a4-session')

    def test_epoch_sized_clock_preserves_one_nanosecond_boundary(self):
        """큰 ROS 시각에서도 만료 직전·정확한 만료·미래 1ns를 구분합니다."""
        self.now = 1_800_000_000.0
        for sec, nanosec, expected in ((1_799_999_999, 700_000_001, True),
                                       (1_799_999_999, 700_000_000, False),
                                       (1_800_000_000, 1, False)):
            message = self.message()
            message.meta.header.stamp.sec = sec
            message.meta.header.stamp.nanosec = nanosec
            self.assertEqual(self.receive(message)[0], expected)

    def test_each_measured_speed_is_checked_independently(self):
        """STOPPED enum만 믿지 않고 세 속도의 경계·초과·비정상 값을 모두 검사합니다."""
        for field, limit in (('linear_speed', 0.01), ('angular_speed', 0.02),
                             ('max_joint_speed', 0.01)):
            for value, expected in ((limit, True), (limit + 0.001, False),
                                    (-0.001, False), (float('nan'), False),
                                    (float('inf'), False)):
                with self.subTest(field=field, value=value):
                    message = self.message()
                    setattr(message, field, value)
                    self.assertEqual(self.receive(message)[0], expected)

    def test_noncanonical_and_invalid_timestamps_cannot_confirm_stop(self):
        """0·음수·미래와 정규 범위 밖 nanosec를 정상 시각으로 합산하지 않습니다."""
        for sec, nanosec in ((0, 0), (-1, 0), (100, 1), (101, 0),
                             (99, 1_000_000_000), (98, 2_000_000_000)):
            with self.subTest(sec=sec, nanosec=nanosec):
                message = self.message()
                message.meta.header.stamp.sec = sec
                message.meta.header.stamp.nanosec = nanosec
                confirmed, deadline = self.receive(message)
                self.assertFalse(confirmed)
                self.assertLessEqual(deadline, self.now)

    def test_session_restart_and_lower_revision_cannot_restore_stopped(self):
        """새 세션에서는 revision을 다시 세되 폐기 세션·낮은 revision은 버립니다."""
        self.assertTrue(self.receive(self.message('old', 9))[0])
        running = self.message('new', 1)
        running.state = ActuationState.RUNNING
        self.assertFalse(self.receive(running)[0])
        self.assertFalse(self.receive(self.message('old', 10))[0])
        self.assertTrue(self.receive(self.message('new', 3))[0])
        running.meta.revision = 2
        self.assertTrue(self.receive(running)[0])
        self.assertEqual(self.node.sources[('stop', 'robot_1')], ('new', 3))

    def test_invalid_flag_revision_and_states_do_not_confirm_stop(self):
        """형식이 맞아도 invalid·revision 0·실제 비정지 상태는 정지가 아닙니다."""
        for state in (ActuationState.UNKNOWN, ActuationState.RUNNING,
                      ActuationState.STOP_REQUESTED, ActuationState.STOPPING,
                      ActuationState.FAULT):
            message = self.message()
            message.state = state
            self.assertFalse(self.receive(message)[0])
        message = self.message()
        message.meta.valid = False
        self.assertFalse(self.receive(message)[0])
        message = self.message(revision=0)
        self.assertFalse(self.receive(message)[0])

    def test_other_robot_does_not_clear_current_robot_stop(self):
        """다른 로봇의 메시지는 현재 로봇의 정지 상태를 갱신하지 않습니다."""
        before = self.receive(self.message())
        message = self.message()
        message.robot_id = 'robot_2'
        self.assertEqual(self.receive(message), before)

    def test_heartbeat_uses_sample_age_without_extending_original_deadline(self):
        """같은 표본 재수신만으로 정지 증거의 만료 시각이 뒤로 늘어나지 않습니다."""
        message = self.message()
        _, deadline = self.receive(message)
        self.now += 0.1
        _, repeated_deadline = self.receive(message)
        self.assertAlmostEqual(repeated_deadline, deadline)
        message.meta.header.stamp.nanosec = 100_000_000
        _, fresh_deadline = self.receive(message)
        self.assertGreater(fresh_deadline, deadline)
