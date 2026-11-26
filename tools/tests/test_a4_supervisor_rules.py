"""A4 W3 감독기 핵심 규칙 단위 테스트.

ROS 2가 없는 환경에서도 시계·고장·정지·복구 안전 규칙을 검증한다.
실제 DDS topic/service 연결은 ROS 2 통합 시험에서 별도로 확인해야 한다.
"""

import tempfile
import unittest
from pathlib import Path

from ax_safety.a4.a4_supervisor_rules import (
    ClockMonitor,
    ClockVerdict,
    RestartLedger,
    SafetySupervisorRules,
    SupervisorState,
    TimeState,
)


class A4SupervisorRulesTest(unittest.TestCase):
    def test_clock_pause_and_reverse_are_distinct(self):
        monitor = ClockMonitor(window_s=5.0, pause_s=3.0)
        self.assertEqual(monitor.observe(1.0, 0.0).state, TimeState.UNKNOWN)
        for index in range(1, 31):
            verdict = monitor.observe(1.0, index * 0.1)
        self.assertEqual(verdict.state, TimeState.PAUSED)
        self.assertEqual(monitor.observe(0.5, 3.2).state, TimeState.REVERSED)

    def test_rtf_window_and_invalid_clock_fail_closed(self):
        monitor = ClockMonitor(window_s=5.0, pause_s=3.0, min_rtf=0.5, max_rtf=1.5)
        monitor.observe(1.0, 0.0)
        verdict = monitor.observe(6.0, 5.0)
        self.assertEqual(verdict.state, TimeState.RUNNING)
        self.assertEqual(monitor.observe(float("nan"), 6.0).state, TimeState.UNKNOWN)

    def test_fault_requires_actual_stopped_for_safe_stop(self):
        rules = SafetySupervisorRules("robot_1")
        rules.update_clock(ClockVerdict(TimeState.RUNNING, 1.0, "CLOCK_RUNNING"))
        rules.update_safety(5.0, True)
        rules.update_actuation(5.0, True, False)
        rules.observe_fault("TIME_INVALID", "clock paused", 5.0)
        self.assertEqual(rules.confirm_stop(5.1).state, SupervisorState.SAFE_STOP_PENDING)
        self.assertFalse(rules.decision(5.1).stop_confirmed)
        rules.update_actuation(5.2, True, True)
        self.assertEqual(rules.confirm_stop(5.2).state, SupervisorState.SAFE_STOP)
        self.assertTrue(rules.decision(5.2).stop_confirmed)

    def test_clear_fault_revalidates_all_conditions(self):
        rules = SafetySupervisorRules("robot_1")
        rules.update_clock(ClockVerdict(TimeState.RUNNING, 1.0, "CLOCK_RUNNING"))
        rules.update_safety(10.0, True)
        rules.update_actuation(10.0, True, True)
        rules.observe_fault("SENSOR_STALE", "test", 10.0)
        accepted, code, blockers = rules.clear_fault(10.1, "req", "SENSOR_STALE", False)
        self.assertFalse(accepted)
        self.assertEqual(code, "CAUSE_ACTIVE")
        self.assertIn("CAUSE_ACTIVE", blockers)
        accepted, code, blockers = rules.clear_fault(10.1, "req", "SENSOR_STALE", True)
        self.assertTrue(accepted)
        self.assertEqual(code, "NONE")
        self.assertEqual(blockers, [])

    def test_restart_budget_and_persistent_ledger(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "restart.json")
            first = SafetySupervisorRules("robot_1", max_restart_attempts=1, ledger=RestartLedger(path))
            first.observe_fault("PROCESS_HEARTBEAT_STALE", "node stopped", 1.0)
            self.assertTrue(first.record_restart("a4_gate"))
            self.assertFalse(first.record_restart("a4_gate"))
            second = RestartLedger(path)
            self.assertEqual(second.count("robot_1:a4_gate"), 1)

    def test_stale_stream_blocks_recovery(self):
        rules = SafetySupervisorRules("robot_2", stream_ttl_s=0.5)
        rules.update_clock(ClockVerdict(TimeState.RUNNING, 1.0, "CLOCK_RUNNING"))
        rules.update_safety(1.0, True)
        rules.update_actuation(1.0, True, True)
        rules.observe_fault("SENSOR_STALE", "test", 1.0)
        accepted, code, blockers = rules.clear_fault(2.0, "req", "SENSOR_STALE", True)
        self.assertFalse(accepted)
        self.assertIn(code, {"SENSOR_STALE", "PEER_STATE_STALE"})
        self.assertTrue(blockers)

    def test_process_heartbeat_stale_is_a_fault_lock(self):
        rules = SafetySupervisorRules("robot_1", stream_ttl_s=0.5)
        rules.update_heartbeat("b4_risk_node", 1.0)
        rules.observe_fault("PEER_STATE_STALE", "heartbeat expired", 2.0)
        self.assertFalse(rules.record_restart("b4_risk_node"))
        self.assertTrue(rules.fault.recovery_exhausted)


if __name__ == "__main__":
    unittest.main()
