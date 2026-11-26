#!/usr/bin/env python3
"""A4 W3 안전 감독 ROS 2 노드.

기존 A4 safety_gate가 발행하는 ActuationState를 소비하여 실제 STOPPED 여부를
확인하고, /clock과 안전 상태 스트림의 유효성을 감시한다. 주행·팔 허가는
여전히 safety_gate가 결정하므로 이 노드는 Schema 2를 중복 발행하지 않는다.
"""

from __future__ import annotations

import math
import time
import uuid

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from ax_interfaces.msg import ActuationState, FailureEvent, SafetyState, TimeValidity
from ax_interfaces.srv import ClearFault, RequestSafeStop

from ax_safety.a4.a4_supervisor_rules import (
    ClockMonitor,
    ClockVerdict,
    SafetySupervisorRules,
    SupervisorState,
    TimeState,
)


TIME_STATES = {
    TimeState.UNKNOWN: TimeValidity.UNKNOWN,
    TimeState.RUNNING: TimeValidity.RUNNING,
    TimeState.PAUSED: TimeValidity.PAUSED,
    TimeState.REVERSED: TimeValidity.REVERSED,
    TimeState.RTF_OUT_OF_RANGE: TimeValidity.RTF_OUT_OF_RANGE,
}


class A4SupervisorNode(Node):
    """로봇 하나의 상태 스트림과 시뮬레이션 시계를 감독한다."""

    def __init__(self) -> None:
        super().__init__("a4_safety_supervisor")
        self.robot_id = str(self.declare_parameter("robot_id", "robot_1").value)
        self.source_id = f"a4_supervisor:{self.robot_id}"
        self.session_id = str(uuid.uuid4())
        self.revision = 0
        self.last_sim_time = 0.0
        self.last_clock_wall = self._wall_now()
        self.last_failure_code = ""
        self.last_clear_request_id = ""

        window_s = float(self.declare_parameter("clock_window_s", 5.0).value)
        pause_s = float(self.declare_parameter("clock_pause_s", 3.0).value)
        min_rtf = float(self.declare_parameter("clock_min_rtf", 0.5).value)
        max_rtf = float(self.declare_parameter("clock_max_rtf", 1.5).value)
        ttl_s = float(self.declare_parameter("state_ttl_s", 0.5).value)
        max_restarts = int(self.declare_parameter("max_restart_attempts", 0).value)
        ledger_path = str(self.declare_parameter("restart_ledger_path", "").value)
        self.auto_request_system_stop = bool(
            self.declare_parameter("request_system_safe_stop", False).value
        )

        self.clock_monitor = ClockMonitor(window_s, pause_s, min_rtf, max_rtf)
        from ax_safety.a4.a4_supervisor_rules import RestartLedger
        self.rules = SafetySupervisorRules(
            self.robot_id, max_restarts, RestartLedger(ledger_path), ttl_s
        )
        self._safe_stop_client = self.create_client(
            RequestSafeStop, "/ax/fleet/safe_stop/request"
        )
        self._last_safe_stop_request = 0.0

        prefix = f"/ax/{self.robot_id}"
        self.time_pub = self.create_publisher(TimeValidity, "/ax/fleet/time_validity", 10)
        self.failure_pub = self.create_publisher(
            FailureEvent, f"{prefix}/failures/report", 10
        )
        self.clock_sub = self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.safety_sub = self.create_subscription(
            SafetyState, f"{prefix}/safety/state", self._on_safety, 10
        )
        self.actuation_sub = self.create_subscription(
            ActuationState, f"{prefix}/safety/actuation_state", self._on_actuation, 10
        )
        self.clear_srv = self.create_service(
            ClearFault, f"{prefix}/safety/clear_fault", self._on_clear_fault
        )
        self.timer = self.create_timer(0.1, self._on_tick)

    def _wall_now(self) -> float:
        # 시뮬레이션 시계가 멈추거나 역행해도 timeout을 측정해야 하므로
        # 감독기의 경과 시간은 ROS sim time이 아닌 단조 증가 wall clock을 쓴다.
        return time.monotonic()

    def _stamp_seconds(self, stamp: Time) -> float:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if math.isfinite(value) else 0.0

    def _on_clock(self, message: Clock) -> None:
        """시뮬레이션 시계를 관찰한다. 역행·정지는 fault 입력이다."""
        now = self._wall_now()
        sim = self._stamp_seconds(message.clock)
        self.last_sim_time = sim
        self.last_clock_wall = now
        verdict = self.clock_monitor.observe(sim, now)
        self.rules.update_clock(verdict)
        if verdict.state in (TimeState.PAUSED, TimeState.REVERSED, TimeState.RTF_OUT_OF_RANGE):
            self._raise_fault("TIME_INVALID", verdict.reason, now)

    def _on_safety(self, message: SafetyState) -> None:
        """B4 SafetyState가 현재 로봇의 유효한 상태인지 기록한다."""
        now = self._wall_now()
        valid = bool(getattr(message.meta, "valid", False)) and message.robot_id == self.robot_id
        stop_required = getattr(message, "level", SafetyState.UNKNOWN) in (
            SafetyState.STOP_REQUIRED,
            SafetyState.UNKNOWN,
        )
        self.rules.update_safety(now, valid, stop_required)
        if not valid or stop_required:
            self._raise_fault("SENSOR_INVALID", "B4 SafetyState invalid or stop-required", now)

    def _on_actuation(self, message: ActuationState) -> None:
        """Schema 2의 실제 정지 결과를 감독 상태에 반영한다."""
        now = self._wall_now()
        valid = bool(getattr(message.meta, "valid", False)) and message.robot_id == self.robot_id
        stopped = getattr(message, "state", ActuationState.UNKNOWN) == ActuationState.STOPPED
        self.rules.update_actuation(now, valid, stopped)
        if self.rules.fault:
            self.rules.confirm_stop(now)

    def _raise_fault(self, code: str, detail: str, now: float) -> None:
        if self.last_failure_code == code and self.rules.fault is not None:
            self.rules.observe_fault(code, detail, now)
            return
        self.last_failure_code = code
        self.rules.observe_fault(code, detail, now)
        self._publish_failure(code, detail)
        # D4의 SAFE_STOP 서비스 수락은 실제 정지 완료가 아니다. 자동 요청은
        # 명시적으로 켠 경우에만 보내며, 어떤 경우에도 로컬 fault latch는 유지한다.
        if self.auto_request_system_stop and now - self._last_safe_stop_request >= 1.0:
            self._last_safe_stop_request = now
            if self._safe_stop_client.service_is_ready():
                request = RequestSafeStop.Request()
                request.request_id = str(uuid.uuid4())
                request.reason = f"{self.robot_id}:{code}:{detail}"
                self._safe_stop_client.call_async(request)

    def _publish_failure(self, code: str, detail: str) -> None:
        event = FailureEvent()
        event.failure_event_id = str(uuid.uuid4())
        event.owner_domain = "A"
        event.code = code
        event.classification = FailureEvent.CLASSIFIED
        event.robot_id = self.robot_id
        event.recovery_context_id = self.session_id
        event.recovery_exhausted = bool(self.rules.fault and self.rules.fault.recovery_exhausted)
        event.post_review_required = True
        event.occurred_at = self.get_clock().now().to_msg()
        event.detail = detail
        self.failure_pub.publish(event)

    def _on_tick(self) -> None:
        """스트림 만료를 확인하고 TimeValidity를 주기적으로 발행한다."""
        now = self._wall_now()
        if self.rules.time_verdict.state is not TimeState.RUNNING:
            self._raise_fault("TIME_INVALID", self.rules.time_verdict.reason or "clock not running", now)
        for name in ("safety_state", "actuation_state"):
            stream = self.rules.streams.get(name)
            if stream is None or not stream.fresh(now, self.rules.stream_ttl_s):
                self._raise_fault("SENSOR_STALE", f"{name} stream stale", now)
        # 외부 서비스 매니저가 heartbeat를 연결한 경우에도 동일한 만료
        # 규칙으로 처리한다. manager가 없으면 이 목록 자체가 비어 있다.
        for name, stream in self.rules.streams.items():
            if name.startswith("process:") and not stream.fresh(now, self.rules.stream_ttl_s):
                self._raise_fault("PEER_STATE_STALE", f"{name} stale", now)
        if now - self.last_clock_wall >= self.clock_monitor.pause_s:
            self._raise_fault("TIME_INVALID", "clock topic stale", now)
        if self.rules.fault:
            self.rules.confirm_stop(now)
        self.revision += 1
        message = TimeValidity()
        message.meta.schema_version = 1
        message.meta.source_id = self.source_id
        message.meta.source_session_id = self.session_id
        message.meta.revision = self.revision
        message.meta.header.stamp = self.get_clock().now().to_msg()
        message.meta.valid = self.rules.time_verdict.state is not TimeState.UNKNOWN
        message.meta.validity_reason = self.rules.time_verdict.reason
        message.state = TIME_STATES[self.rules.time_verdict.state]
        message.measured_rtf = float(self.rules.time_verdict.measured_rtf)
        message.window_s = float(self.clock_monitor.window_s)
        self.time_pub.publish(message)

    def _on_clear_fault(self, request: ClearFault.Request, response: ClearFault.Response):
        """ClearFault 요청을 실제 정지·시계·원인 해소 조건으로 검증한다."""
        self.last_clear_request_id = request.request_id
        # 입력 문자열을 원인 해소의 증거로 사용하지 않는다. 요청은 기록용이고,
        # 실제 해소 여부는 rules가 현재 시계·상태 스트림·STOPPED를 재검증한다.
        root_cause_resolved = bool(request.reason.strip()) and self.rules.time_verdict.state is TimeState.RUNNING
        accepted, code, blockers = self.rules.clear_fault(
            self._wall_now(), request.request_id, request.fault_code,
            root_cause_resolved=root_cause_resolved,
        )
        response.accepted = accepted
        response.reason_code = code
        response.remaining_block_reasons = blockers
        if accepted:
            self.last_failure_code = ""
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = A4SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
