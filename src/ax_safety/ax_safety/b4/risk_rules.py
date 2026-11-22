"""B4 위험 판단: 운영 코드에서 사용하는 ROS 독립 규칙 엔진.

판정 불가능한 신호는 반드시 UNKNOWN / zero-speed이다. 거리 기준은
운영 승인 전까지 보수적 예시이며 실제 안전 한계로 주장하지 않는다.
"""
from dataclasses import dataclass, replace
from math import isfinite
from typing import Optional

CLEAR, SLOWDOWN_REQUIRED, STOP_REQUIRED, UNKNOWN = 0, 1, 2, 3

# B4가 판단에 사용할 입력 묶음이다.
# valid/fresh 플래그는 센서가 연결되어 있는지와 최근 값을 보냈는지를 분리한다.
# 거리·속도 값의 단위는 각각 m, m/s, rad/s이며, 기본값은 개발용 보수값이다.
@dataclass(frozen=True)
class RiskInputs:
    """B4가 한 번의 안전 판단에 사용하는 검증 완료 입력 모음.

    `battery_required`가 False이면 기존 센서 판단을 그대로 사용한다.
    True일 때 무효·stale 배터리는 기존 UNKNOWN 차단 경로로 보낸다.
    LOW/CRITICAL 행동 level은 명시적인 parameter가 있을 때만 적용한다.
    """
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
    battery_required: bool = False
    battery_valid: bool = True
    battery_risk: str = "NORMAL"
    battery_reason_code: str = "NONE"
    # Negative means policy is not configured.  B4 must not choose LOW or
    # CRITICAL behavior before the team approves it.
    battery_action_level: int = -1

# B4 판단 결과는 A4가 실제 명령을 허용할 수 있는 정보로 변환되어 발행된다.
# level/reason_code는 사람이 읽는 상태와 차단 원인을 함께 보존하고,
# 방향별 allow 값과 속도 상한은 A4의 최종 게이트 입력이 된다.
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
    """허가 플래그와 속도 상한을 안전한 기본값으로 만든다.

    A4는 이 결과를 받아 주행·팔 명령을 차단하며, 실제 정지 완료는
    별도의 odom/joint 피드백으로 확인한다.
    """
    # 입력이 불완전하거나 위험하면 허용 플래그와 속도를 모두 안전한 기본값으로 만든다.
    # sensor_valid는 전체 입력을 신뢰할 수 없다는 뜻이며, 개별 센서 상태는 별도로 보존한다.


    return RiskDecision(level, reason, False, v.front_sensor_valid,
                        v.rear_sensor_valid, v.localization_valid,
                        False, False, False, 0.0, 0.0)


def apply_battery_policy(decision: RiskDecision, v: RiskInputs) -> RiskDecision:
    """명시된 배터리 행동 정책만 기존 위험 결과에 합성한다.

    기본값은 no-op이다. LOW를 감속으로 볼지 정지로 볼지 합의되지 않았기
    때문에, `battery_action_level=-1`에서는 분류 결과를 허가 차단으로
    바꾸지 않는다. 환경 안전 결과가 더 위험하면 기존 결과를 보존한다.
    """
    if (not v.battery_required or not v.battery_valid or
            v.battery_action_level < 0 or
            v.battery_action_level not in (SLOWDOWN_REQUIRED, STOP_REQUIRED, UNKNOWN) or
            v.battery_risk not in ("LOW", "CRITICAL")):
        return decision
    if decision.level >= v.battery_action_level:
        return decision
    return replace(decision, level=v.battery_action_level,
                   reason_code=v.battery_reason_code or decision.reason_code,
                   allow_forward=False, allow_backward=False,
                   allow_rotate=False, max_linear_speed=0.0,
                   max_angular_speed=0.0)


def assess_risk(v: RiskInputs) -> RiskDecision:
    """환경·위치·통로·배터리 입력을 합쳐 B4 위험 결과를 반환한다.

    반환 결과는 실제 정지 완료가 아니다. A4가 허가를 차단하고,
    별도의 odom/joint 표본으로 STOPPED를 확인한다.
    """


    if v.emergency_stop:
        # 비상 정지는 다른 센서나 거리 조건보다 우선해야 즉시 차단된다.


        return apply_battery_policy(blocked(STOP_REQUIRED, "OBSTACLE_STOP", v), v)
    if v.battery_required and not v.battery_valid:
        # Existing A4 contract treats UNKNOWN/invalid SafetyState as a hard
        # safety gate.  No new battery reason code is introduced here.
        return blocked(UNKNOWN, v.battery_reason_code or "SENSOR_INVALID", v)
    values = (v.max_linear_m_s, v.max_angular_rad_s, v.stop_distance_m,
              v.slowdown_distance_m, v.slowdown_linear_m_s)
    if not all(isfinite(x) and x >= 0 for x in values) or not (0 < v.stop_distance_m < v.slowdown_distance_m):
        # 임계값 관계가 깨지면 비교 결과를 믿을 수 없으므로 UNKNOWN으로 차단한다.


        return apply_battery_policy(blocked(UNKNOWN, "SENSOR_INVALID", v), v)
    if not v.sensor_fresh or not v.corridor_valid:
        # 오래된 센서·통로 관측으로 주행을 허용하면 현재 장애물을 놓칠 수 있다.


        return apply_battery_policy(blocked(UNKNOWN, "SENSOR_STALE", v), v)
    if not v.localization_valid or not v.localization_fresh:
        # 위치가 불확실한 상태에서는 거리와 통로 판정의 기준 좌표도 신뢰할 수 없다.


        return apply_battery_policy(blocked(UNKNOWN, "LOCALIZATION_INVALID", v), v)
    if not v.front_sensor_valid:
        # 전방 거리 없이는 정지·감속 여부를 계산할 수 없다.


        return apply_battery_policy(blocked(UNKNOWN, "SENSOR_INVALID", v), v)
    d = v.obstacle_distance_m
    if d is None or not isfinite(d) or d < 0:
        # None/NaN/음수 거리는 실제 거리로 해석하지 않고 즉시 차단한다.


        return apply_battery_policy(blocked(UNKNOWN, "SENSOR_INVALID", v), v)
    if d <= v.stop_distance_m:
        # 정지거리 안쪽의 장애물은 속도 제한만으로 해결하지 않고 STOP으로 처리한다.


        return apply_battery_policy(blocked(STOP_REQUIRED, "OBSTACLE_STOP", v), v)
    rear_ok = v.rear_sensor_valid and v.rear_distance_m is not None and isfinite(v.rear_distance_m) and v.rear_distance_m > v.stop_distance_m
    rear_near = rear_ok and v.rear_distance_m <= v.slowdown_distance_m
    if d <= v.slowdown_distance_m or rear_near or not rear_ok:
        # 전방·후방 중 하나라도 안전 운용 조건을 충족하지 못하면 감속하고,
        # 후방 이동과 회전은 확인된 경우에만 별도로 허용한다.
        reason = "OBSTACLE_SLOWDOWN" if d <= v.slowdown_distance_m or rear_near else "REAR_SENSOR_INVALID"
        # 전방 위험과 후방 위험을 분리. 속도는 보수적으로 제한한다.
        forward = d > v.stop_distance_m
        return apply_battery_policy(
            RiskDecision(SLOWDOWN_REQUIRED, reason, rear_ok, True, rear_ok, True,
                         forward, rear_ok, v.allow_rotation_when_slow and rear_ok,
                         min(v.max_linear_m_s, v.slowdown_linear_m_s),
                         v.max_angular_rad_s if v.allow_rotation_when_slow and rear_ok else 0.0), v)
    return apply_battery_policy(
        RiskDecision(CLEAR, "NONE", True, True, True, True,
                     True, True, True, v.max_linear_m_s, v.max_angular_rad_s), v)
