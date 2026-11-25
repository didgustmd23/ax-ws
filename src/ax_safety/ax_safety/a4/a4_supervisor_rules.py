"""A4 W3 안전 감독기의 ROS 비의존 판단 규칙.

이 모듈은 ROS 메시지를 직접 만들지 않는다. 시계, 상태 스트림, 고장 래치,
정지 확인, 재시작 예산을 순수 Python 값으로 판단하여 단위 시험과 실제 ROS
연결부가 같은 안전 규칙을 사용하게 한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import json
import math
from pathlib import Path
from typing import Dict, Optional


class TimeState(str, Enum):
    UNKNOWN = "UNKNOWN"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    REVERSED = "REVERSED"
    RTF_OUT_OF_RANGE = "RTF_OUT_OF_RANGE"


class SupervisorState(str, Enum):
    CLEAR = "CLEAR"
    SAFE_STOP_PENDING = "SAFE_STOP_PENDING"
    SAFE_STOP = "SAFE_STOP"
    RECOVERY_CHECK = "RECOVERY_CHECK"
    FAULT_LATCHED = "FAULT_LATCHED"


@dataclass(frozen=True)
class ClockVerdict:
    state: TimeState
    measured_rtf: float = 0.0
    reason: str = ""


class ClockMonitor:
    """/clock의 역행·정지·RTF를 측정한다.

    wall_time은 실제 경과 시간, sim_time은 ROS 시뮬레이션 시간이다.
    처음 들어온 표본은 비교 기준만 만들므로 RUNNING으로 확정하지 않는다.
    """

    def __init__(self, window_s: float = 5.0, pause_s: float = 3.0,
                 min_rtf: float = 0.5, max_rtf: float = 1.5) -> None:
        self.window_s = window_s
        self.pause_s = pause_s
        self.min_rtf = min_rtf
        self.max_rtf = max_rtf
        self._previous: Optional[tuple[float, float]] = None
        self._window_start: Optional[tuple[float, float]] = None

    def observe(self, sim_time: float, wall_time: float) -> ClockVerdict:
        """새 시계 표본을 검사하고 현재 TimeValidity 상태를 반환한다."""
        if not math.isfinite(sim_time) or not math.isfinite(wall_time) or sim_time <= 0:
            return ClockVerdict(TimeState.UNKNOWN, reason="CLOCK_INVALID")

        current = (sim_time, wall_time)
        if self._previous is not None and sim_time < self._previous[0]:
            self._previous = current
            self._window_start = current
            return ClockVerdict(TimeState.REVERSED, reason="CLOCK_REVERSED")

        if self._previous is None:
            self._previous = current
            self._window_start = current
            return ClockVerdict(TimeState.UNKNOWN, reason="CLOCK_WARMUP")

        assert self._window_start is not None
        wall_delta = wall_time - self._previous[1]
        sim_delta = sim_time - self._previous[0]
        if wall_delta < 0:
            return ClockVerdict(TimeState.UNKNOWN, reason="WALL_CLOCK_REVERSED")

        window_wall = wall_time - self._window_start[1]
        window_sim = sim_time - self._window_start[0]
        # 표본 간 간격이 짧아도 전체 관찰 창에서 sim 시간이 전혀 늘지
        # 않았다면 PAUSED이다. 그래야 0.1초 주기 callback이 3초 정지를
        # 누적해서 감지할 수 있다.
        if window_wall >= self.pause_s and window_sim <= 0:
            self._previous = current
            return ClockVerdict(TimeState.PAUSED, 0.0, "CLOCK_PAUSED")

        if window_wall < self.window_s:
            self._previous = current
            return ClockVerdict(TimeState.UNKNOWN, reason="CLOCK_WINDOW_WARMUP")

        measured = window_sim / window_wall if window_wall > 0 else 0.0
        self._previous = current
        self._window_start = current
        if not math.isfinite(measured) or not self.min_rtf <= measured <= self.max_rtf:
            return ClockVerdict(TimeState.RTF_OUT_OF_RANGE, measured, "RTF_OUT_OF_RANGE")
        return ClockVerdict(TimeState.RUNNING, measured, "CLOCK_RUNNING")


@dataclass
class StreamStatus:
    last_time: Optional[float] = None
    valid: bool = False
    detail: str = ""

    def fresh(self, now: float, ttl_s: float) -> bool:
        return self.valid and self.last_time is not None and now - self.last_time <= ttl_s


@dataclass
class FaultRecord:
    code: str
    detail: str
    first_time: float
    last_time: float
    recovery_attempts: int = 0
    recovery_exhausted: bool = False
    stop_confirmed: bool = False


class RestartLedger:
    """재시작 횟수를 선택적으로 영속화한다.

    경로가 비어 있으면 파일을 만들지 않는다. 기본 max_attempts도 0으로 두어
    팀이 정한 재시작 정책 없이 자동 재시작이 일어나지 않게 한다.
    """

    def __init__(self, path: str = "") -> None:
        self.path = Path(path) if path else None
        self.counts: Dict[str, int] = {}
        if self.path and self.path.exists():
            try:
                data = json.loads(self.path.read_text(encoding="utf-8"))
                self.counts = {str(k): int(v) for k, v in data.items()}
            except (OSError, ValueError, TypeError):
                self.counts = {}

    def count(self, key: str) -> int:
        return self.counts.get(key, 0)

    def record(self, key: str) -> int:
        value = self.count(key) + 1
        self.counts[key] = value
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(self.path.suffix + ".tmp")
            temporary.write_text(json.dumps(self.counts, sort_keys=True), encoding="utf-8")
            temporary.replace(self.path)
        return value


@dataclass
class SupervisorDecision:
    state: SupervisorState
    fault_code: str = ""
    stop_confirmed: bool = False
    block_reasons: list[str] = field(default_factory=list)
    restart_allowed: bool = False


class SafetySupervisorRules:
    """A4가 독립적으로 유지해야 하는 고장·복구 상태 머신."""

    def __init__(self, robot_id: str, max_restart_attempts: int = 0,
                 ledger: Optional[RestartLedger] = None, stream_ttl_s: float = 0.5) -> None:
        self.robot_id = robot_id
        self.max_restart_attempts = max(0, int(max_restart_attempts))
        self.ledger = ledger or RestartLedger()
        self.stream_ttl_s = stream_ttl_s
        self.state = SupervisorState.CLEAR
        self.fault: Optional[FaultRecord] = None
        self.streams: Dict[str, StreamStatus] = {}
        self.time_verdict = ClockVerdict(TimeState.UNKNOWN, reason="CLOCK_WARMUP")
        self._actuation_stopped = False
        self._actuation_valid = False
        self._safety_valid = False
        self._local_task_active = False

    def update_stream(self, name: str, now: float, valid: bool, detail: str = "") -> None:
        """노드/상태 스트림의 최신 수신 시각과 유효성을 저장한다."""
        self.streams[name] = StreamStatus(now, valid, detail)

    def update_heartbeat(self, component: str, now: float, healthy: bool = True) -> None:
        """외부 서비스 매니저가 전달한 핵심 프로세스 heartbeat를 기록한다.

        A4가 OS 프로세스를 직접 재시작하지 않도록 heartbeat 기록과 재시작
        실행을 분리했다. 실제 manager 연결 전에도 같은 규칙을 모의 입력으로
        시험할 수 있다.
        """
        self.update_stream(f"process:{component}", now, healthy)

    def update_actuation(self, now: float, valid: bool, stopped: bool) -> None:
        self.update_stream("actuation_state", now, valid)
        self._actuation_valid = valid
        self._actuation_stopped = stopped if valid else False

    def update_safety(self, now: float, valid: bool, stop_required: bool = False) -> None:
        self.update_stream("safety_state", now, valid, "STOP_REQUIRED" if stop_required else "")
        self._safety_valid = valid and not stop_required

    def set_local_task_active(self, active: bool) -> None:
        self._local_task_active = active

    def update_clock(self, verdict: ClockVerdict) -> None:
        self.time_verdict = verdict

    def observe_fault(self, code: str, detail: str, now: float) -> SupervisorDecision:
        """고장을 래치하고 SAFE_STOP 요청 대상 상태로 전환한다."""
        if self.fault is None or self.fault.code != code:
            self.fault = FaultRecord(code, detail, now, now)
        else:
            self.fault.last_time = now
            self.fault.detail = detail
        self.state = SupervisorState.SAFE_STOP_PENDING
        return self.decision(now)

    def confirm_stop(self, now: float) -> SupervisorDecision:
        """A4 게이트가 실제 odom/joint 증거로 STOPPED를 확인했는지 반영한다."""
        if self.fault is None:
            return self.decision(now)
        self.fault.stop_confirmed = self._actuation_valid and self._actuation_stopped
        self.state = SupervisorState.SAFE_STOP if self.fault.stop_confirmed else SupervisorState.SAFE_STOP_PENDING
        return self.decision(now)

    def record_restart(self, component: str) -> bool:
        """남은 재시작 예산이 있을 때만 횟수를 기록한다."""
        key = f"{self.robot_id}:{component}"
        if self.ledger.count(key) >= self.max_restart_attempts:
            if self.fault:
                self.fault.recovery_exhausted = True
            return False
        self.ledger.record(key)
        if self.fault:
            self.fault.recovery_attempts += 1
        return True

    def clear_fault(self, now: float, request_id: str, fault_code: str,
                    root_cause_resolved: bool) -> tuple[bool, str, list[str]]:
        """ClearFault 계약의 조건을 모두 재검증한다.

        accepted=True는 A4 로컬 fault latch 해제 요청이 수락됐다는 뜻뿐이며,
        다른 로봇·D4 SAFE_STOP·B4 차단 조건까지 제거한다는 뜻은 아니다.
        """
        del request_id  # 중복성 검증은 ROS 연결부에서 요청 ID를 기록하며 수행한다.
        del fault_code
        blockers: list[str] = []
        if self.fault is None:
            return False, "CAUSE_ACTIVE", []
        if not self._actuation_valid or not self._actuation_stopped:
            blockers.append("NOT_STOPPED")
        if self.time_verdict.state is not TimeState.RUNNING:
            blockers.append("TIME_INVALID")
        if not root_cause_resolved:
            blockers.append("CAUSE_ACTIVE")
        if self._local_task_active:
            blockers.append("CAUSE_ACTIVE")
        for name, stream in self.streams.items():
            if not stream.fresh(now, self.stream_ttl_s):
                if name.startswith("process:"):
                    blockers.append("FAULT_LOCK")
                else:
                    blockers.append("PEER_STATE_STALE" if name != "actuation_state" else "SENSOR_STALE")
        if blockers:
            self.state = SupervisorState.FAULT_LATCHED
            return False, blockers[0], blockers
        self.state = SupervisorState.RECOVERY_CHECK
        self.fault = None
        self.state = SupervisorState.CLEAR
        return True, "NONE", []

    def decision(self, now: float) -> SupervisorDecision:
        del now
        reasons = []
        if self.fault:
            reasons.append("FAULT_LOCK")
        if self.time_verdict.state is not TimeState.RUNNING:
            reasons.append("TIME_INVALID")
        if not self._actuation_valid:
            reasons.append("SENSOR_STALE")
        return SupervisorDecision(
            self.state,
            self.fault.code if self.fault else "",
            bool(self.fault and self.fault.stop_confirmed),
            reasons,
            bool(self.fault and self.ledger.count(f"{self.robot_id}:default") < self.max_restart_attempts),
        )
