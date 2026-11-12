"""B4 위험 판단: 운영 코드에서 사용하는 ROS 독립 규칙 엔진.

판정 불가능한 신호는 반드시 UNKNOWN / zero-speed이다. 거리 기준은
운영 승인 전까지 보수적 예시이며 실제 안전 한계로 주장하지 않는다.
"""
from dataclasses import dataclass
from math import isfinite
from typing import Optional

CLEAR, SLOWDOWN_REQUIRED, STOP_REQUIRED, UNKNOWN = 0, 1, 2, 3

@dataclass(frozen=True)
class RiskInputs:
    front_sensor_valid: bool = False
    rear_sensor_valid: bool = False
    sensor_fresh: bool = False
    localization_valid: bool = False
    localization_fresh: bool = False
    corridor_valid: bool = False
    emergency_stop: bool = False
    obstacle_distance_m: Optional[float] = None
    rear_distance_m: Optional[float] = None
    max_linear_m_s: float = 0.67
    max_angular_rad_s: float = 0.50
    stop_distance_m: float = 0.50
    slowdown_distance_m: float = 1.00
    slowdown_linear_m_s: float = 0.20
    allow_rotation_when_slow: bool = False

@dataclass(frozen=True)
class RiskDecision:
    level: int
    reason_code: str
    sensor_valid: bool
    front_sensor_valid: bool
    rear_sensor_valid: bool
    localization_valid: bool
    allow_forward: bool
    allow_backward: bool
    allow_rotate: bool
    max_linear_speed: float
    max_angular_speed: float


def blocked(level: int, reason: str, v: RiskInputs) -> RiskDecision:
    return RiskDecision(level, reason, False, v.front_sensor_valid,
                        v.rear_sensor_valid, v.localization_valid,
                        False, False, False, 0.0, 0.0)


def assess_risk(v: RiskInputs) -> RiskDecision:
    """B4 결정. 위험해제는 모든 필수 최신 신호 복원 뒤에만 가능하다."""
    if v.emergency_stop:
        return blocked(STOP_REQUIRED, "OBSTACLE_STOP", v)
    values = (v.max_linear_m_s, v.max_angular_rad_s, v.stop_distance_m,
              v.slowdown_distance_m, v.slowdown_linear_m_s)
    if not all(isfinite(x) and x >= 0 for x in values) or not (0 < v.stop_distance_m < v.slowdown_distance_m):
        return blocked(UNKNOWN, "SENSOR_INVALID", v)
    if not v.sensor_fresh or not v.corridor_valid:
        return blocked(UNKNOWN, "SENSOR_STALE", v)
    if not v.localization_valid or not v.localization_fresh:
        return blocked(UNKNOWN, "LOCALIZATION_INVALID", v)
    if not v.front_sensor_valid:
        return blocked(UNKNOWN, "SENSOR_INVALID", v)
    d = v.obstacle_distance_m
    if d is None or not isfinite(d) or d < 0:
        return blocked(UNKNOWN, "SENSOR_INVALID", v)
    if d <= v.stop_distance_m:
        return blocked(STOP_REQUIRED, "OBSTACLE_STOP", v)
    rear_ok = v.rear_sensor_valid and v.rear_distance_m is not None and isfinite(v.rear_distance_m) and v.rear_distance_m > v.stop_distance_m
    rear_near = rear_ok and v.rear_distance_m <= v.slowdown_distance_m
    if d <= v.slowdown_distance_m or rear_near or not rear_ok:
        reason = "OBSTACLE_SLOWDOWN" if d <= v.slowdown_distance_m or rear_near else "REAR_SENSOR_INVALID"
        # 전방 위험과 후방 위험을 분리. 속도는 보수적으로 제한한다.
        forward = d > v.stop_distance_m
        return RiskDecision(SLOWDOWN_REQUIRED, reason, rear_ok, True, rear_ok, True,
                            forward, rear_ok, v.allow_rotation_when_slow and rear_ok,
                            min(v.max_linear_m_s, v.slowdown_linear_m_s),
                            v.max_angular_rad_s if v.allow_rotation_when_slow and rear_ok else 0.0)
    return RiskDecision(CLEAR, "NONE", True, True, True, True,
                        True, True, True, v.max_linear_m_s, v.max_angular_rad_s)
