"""B4 배터리 입력 검증과 정책 중립적 위험도 분류.

`sim_battery.py`가 발행하는 표준 `sensor_msgs/BatteryState`를 그대로
검증한다. 이 모듈은 잔량을 계산하거나 충전 상태를 만들어 내지 않는다.
그 책임은 배터리 생산자에게 있고, B4는 받은 값이 안전 판단의 근거로
사용 가능한지만 확인한다.

LOW는 22%를 기본 기준으로 사용할 수 있지만, LOW를 감속·정지로 바꾸는
행동은 팀 정책이므로 이 모듈에서 결정하지 않는다. CRITICAL 기준도
확정 전에는 음수로 비활성화한다.
"""

from dataclasses import dataclass
from math import isfinite


UNKNOWN, NORMAL, LOW, CRITICAL = "UNKNOWN", "NORMAL", "LOW", "CRITICAL"


@dataclass(frozen=True)
class BatteryAssessment:
    valid: bool
    fresh: bool
    risk: str
    reason_code: str


def assess_battery(message, now_s, ttl_s, *, required=True,
                   low_percent=22.0, critical_percent=-1.0):
    """배터리 메시지를 검증하고 LOW/CRITICAL 후보를 분류한다.

    `required=False`이면 배터리 기능을 사용하지 않고 기존 B4 결과를
    보존한다. `valid/fresh`가 거짓이면 마지막 정상값을 재사용하지 않고
    B4가 UNKNOWN 안전 차단으로 처리한다.
    """
    if not required:
        return BatteryAssessment(True, True, NORMAL, "NONE")
    if message is None or not isfinite(now_s) or now_s <= 0:
        return BatteryAssessment(False, False, UNKNOWN, "SENSOR_STALE")

    # producer의 측정 시각이 현재보다 미래이거나 너무 오래됐으면,
    # 최신처럼 보이는 오래된 잔량으로 주행 허가를 내리지 않는다.
    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
    if not isfinite(stamp) or stamp <= 0 or stamp > now_s or now_s - stamp > ttl_s:
        return BatteryAssessment(False, False, UNKNOWN, "SENSOR_STALE")
    # present=false는 하드웨어가 없거나 측정할 수 없다는 뜻이다.
    if not bool(message.present):
        return BatteryAssessment(False, True, UNKNOWN, "SENSOR_INVALID")
    # sensor_msgs/BatteryState.percentage는 0.0~1.0 비율이다.
    # 예: 22%는 0.22이며 0.22 이하일 때 LOW 후보가 된다.
    if not isfinite(message.percentage) or not 0.0 <= message.percentage <= 1.0:
        return BatteryAssessment(False, True, UNKNOWN, "SENSOR_INVALID")
    # sensor_msgs/BatteryState uses UNKNOWN=0, CHARGING=1, DISCHARGING=2,
    # NOT_CHARGING=3, FULL=4.  Unknown charge state is not safe evidence.
    # CHARGING(1), DISCHARGING(2), NOT_CHARGING(3), FULL(4)만 허용한다.
    # UNKNOWN(0)은 충전 상태를 신뢰할 수 없으므로 required 모드에서 차단한다.
    if int(message.power_supply_status) not in (1, 2, 3, 4):
        return BatteryAssessment(False, True, UNKNOWN, "SENSOR_INVALID")

    percent = message.percentage * 100.0
    if critical_percent >= 0 and percent <= critical_percent:
        return BatteryAssessment(True, True, CRITICAL, "BATTERY_CRITICAL")
    if low_percent >= 0 and percent <= low_percent:
        return BatteryAssessment(True, True, LOW, "BATTERY_LOW")
    return BatteryAssessment(True, True, NORMAL, "NONE")
