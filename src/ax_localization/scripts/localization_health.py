"""B2 위치 신뢰성·고정 출발 위치·마지막 신뢰 위치·제한된 재위치 판단(ROS 없는 순수 계산).

ROS 연결은 localization_supervisor.py가 하고, 이 모듈은 입력 기록을 받아 판단만 한다(단위 시험 대상).
모든 임계값은 개발용 출발값이며(config/localization_health.yaml) 정확도 기준이 아니다.
상태·사유 코드는 공통 인터페이스 계약 4.2절·D-19 "위치"를 따른다.
"""

from collections import deque
from dataclasses import dataclass, field
import hashlib
import math
from pathlib import Path

import yaml

# LocalizationHealth.state 값(ax_interfaces와 같은 숫자)
UNKNOWN, OK, DEGRADED, LOST = 0, 1, 2, 3
STATE_NAMES = {UNKNOWN: "UNKNOWN", OK: "OK", DEGRADED: "DEGRADED", LOST: "LOST"}

# ActuationState.STOPPED
ACTUATION_STOPPED = 4

# 사유가 여럿일 때 앞에 둘 순서(심각한 순)
REASON_ORDER = [
    "CLOCK_INVALID", "NOT_INITIALIZED",
    "REINIT_IN_PROGRESS", "REINIT_FAILED_CLOCK_INVALID", "REINIT_FAILED_NOT_STOPPED",
    "REINIT_FAILED_AMCL_UNAVAILABLE", "REINIT_FAILED_NOT_RECOVERED", "REINIT_FAILED_TIMEOUT",
    "ESTIMATE_DISCONTINUOUS", "ESTIMATE_INVALID", "ODOM_INVALID", "ODOM_DISCONTINUOUS", "TF_MISSING", "INPUT_LOST",
    "COVARIANCE_LOST",
    "TF_STALE", "SCAN_STALE", "ODOM_STALE", "ESTIMATE_STALE", "COVARIANCE_HIGH", "COVARIANCE_COLLAPSED",
]

# 개발용 출발값. config/localization_health.yaml이 덮어쓴다.
DEFAULTS = {
    "health": {
        "publish_rate_hz": 5.0,
        "scan_stale_s": 0.5, "scan_lost_s": 2.0,
        "odom_stale_s": 0.3, "odom_lost_s": 2.0,
        "tf_stale_s": 0.5, "tf_lost_s": 2.0,
        "tf_future_s": 1.0,
        "estimate_travel_m": 0.5, "estimate_rotation_rad": 0.4,
        "position_std_ok_m": 0.25, "position_std_lost_m": 1.0,
        "yaw_std_ok_rad": 0.15, "yaw_std_lost_rad": 0.5,
        "position_std_min_m": 0.005, "yaw_std_min_rad": 0.005,
        "unconstrained_std_per_m": 0.0, "constrained_std_m": 0.12,
        "clock_stall_wall_s": 2.0,
    },
    # launch initial_poses(AMCL 매개변수) 초기화는 공분산 0으로 필터를 시작하므로, 확인 뒤 감독 노드가 같은 자세를
    # 이 표준편차로 한 번 다시 보내 실제 산포를 만든다(InitialSpread).
    "initial_spread": {
        "position_std_m": 0.10, "yaw_std_rad": 0.10,
        "timeout_s": 15.0,
    },
    "trust": {
        "position_std_m": 0.15, "yaw_std_rad": 0.10,
    },
    "odom_continuity": {
        "max_jump_m": 0.3, "max_jump_rad": 0.5, "speed_margin": 2.0,
    },
    "reinit": {
        "actuation_max_age_s": 0.3,
        "trusted_max_age_s": 60.0,
        "max_travel_m": 1.0, "max_rotation_rad": 0.8,
        "base_std_m": 0.10, "base_yaw_std_rad": 0.10,
        "std_per_m": 0.20, "yaw_std_per_rad": 0.20,
        "timeout_s": 15.0,
        "timeout_wall_s": 60.0,
        "nomotion_period_s": 0.5,
        "min_updates": 3,
        "recover_position_std_m": 0.25, "recover_yaw_std_rad": 0.15,
        "recover_std_ratio": 0.9,
        "consistency_m": 0.5, "consistency_rad": 0.3,
    },
    # 추정 도약 감지(재위치 consistency와 키를 공유하지 않는다)
    "estimate_jump": {
        "position_m": 0.3, "yaw_rad": 0.3,
        "odom_match_s": 0.1, "odom_history_s": 5.0,
        "pair_position_m": 0.02, "pair_yaw_rad": 0.02,
        "intent_position_m": 0.5, "intent_yaw_rad": 0.3,
        "intent_timeout_s": 15.0, "intent_timeout_wall_s": 60.0,
    },
}


# 기본값 위에 사용자 설정을 겹친다. 모르는 키는 거절한다.
def merge_config(user=None):
    config = {section: dict(values) for section, values in DEFAULTS.items()}

    for section, values in (user or {}).items():
        if section not in config:
            raise ValueError(f"알 수 없는 설정 묶음: {section}")

        for key, value in (values or {}).items():
            if key not in config[section]:
                raise ValueError(f"알 수 없는 설정: {section}.{key}")

            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError(f"설정 값은 0 이상 유한한 수: {section}.{key}={value!r}")

            config[section][key] = float(value)

    return config


# 각도를 -π~π로 감는다.
def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# SE(2) 합성 a ∘ b (각 (x, y, yaw))
def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])

    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


# SE(2) 역변환
def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])

    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


# 두 자세 사이 거리·회전 크기
def difference(a, b):
    relative = compose(inverse(a), b)

    return math.hypot(relative[0], relative[1]), abs(relative[2])


# 공분산(6×6 행 우선 36개)에서 위치·방향 표준편차를 꺼낸다. 유한하지 않거나 음수 분산이면 None.
# 판단에 쓰는 대각(xx·yy m², yaw rad²)이 [−ROUNDOFF_VARIANCE, 0)이면 반올림 오차로 보고 내부 계산에서만 0으로 쓴다.
def covariance_std(covariance):
    return covariance_std_detail(covariance)[0]


# 반올림 오차로 볼 대각 분산 음수의 경계(분산 단위). Nav2 Jazzy AMCL은 initial_poses(set_initial_pose) 경로에서
# 초기 공분산 0으로 필터를 시작하고, pf 통계가 yaw 분산을 −2·ln(sqrt((Σw·cosθ)² + (Σw·sinθ)²)), 선형 분산을 E[x²]−평균²로
# 계산한다. 입자 2000개(config/amcl.yaml max_particles) 합의 반올림으로 본 yaw 분산 오차 규모는 설명용 추정으로
# 1e-12 이하이며(설치본 동일 커밋·모든 연산 오차를 증명한 엄밀 상한 아님), 관측값은 −1.09e-13이었다.
# 정확도·수렴 근거가 아니며, 이보다 큰 음수는 계속 무효다. 원점에서 먼 위치의 선형 분산은 상쇄 오차가 이 경계를
# 넘을 수 있으며, 이는 미지원 범위로 두고 보수적으로 무효 처리한다(경계 확대·위치 비례 경계 없음).
ROUNDOFF_VARIANCE = 1e-12


# covariance_std와 같고, 0으로 본 대각 항목 목록 [{"index", "value"}](진단용, 원래 값)을 함께 돌려준다.
# 입력 배열은 바꾸지 않는다.
def covariance_std_detail(covariance):
    if len(covariance) != 36:
        return None, []

    variances, normalized = [], []

    for index in (0, 7, 35):
        value = covariance[index]

        if not math.isfinite(value) or value < -ROUNDOFF_VARIANCE:
            return None, []

        if value < 0:
            normalized.append({"index": index, "value": value})
            value = 0.0

        variances.append(value)

    xx, yy, yaw = variances

    return (math.sqrt(max(xx, yy)), math.sqrt(yaw)), normalized


# 공분산이 붕괴(입자가 한 점에 모임)했는지. std=(위치, 방향) 중 하나라도 하한 미만이면 참, std가 None이면 거짓.
# Nav2 Jazzy AMCL은 launch initial_poses(set_initial_pose)와 lifecycle 재활성 때 공분산 0으로 필터를 다시 시작하고,
# 정지 중에는 운동 잡음이 0이라 무이동 갱신으로도 산포가 생기지 않는다. 이때의 std 0은 불확실성이 작다는 뜻이 아니다.
def covariance_collapsed(std, config):
    if std is None:
        return False

    h = config["health"]

    return std[0] < h["position_std_min_m"] or std[1] < h["yaw_std_min_rad"]


# 자세 값이 모두 유한한지 확인한다.
def finite_pose(pose):
    return pose is not None and all(math.isfinite(value) for value in pose)


# 사유 목록을 심각한 순으로 정렬해 문자열로 만든다.
def join_reasons(reasons):
    unique = list(dict.fromkeys(reasons))

    return ",".join(sorted(unique, key=lambda code: REASON_ORDER.index(code) if code in REASON_ORDER else len(REASON_ORDER)))


# 앞 stamp보다 늦은 유한한 stamp인지(같은·역행·비유한 stamp는 새 입력이 아니다)
def new_stamp(previous, stamp):
    if stamp is None or not math.isfinite(stamp):
        return False

    return previous is None or stamp > previous


# 두 값(수 또는 수의 묶음)이 같은지. NaN끼리는 같다고 본다(같은 메시지 재수신 판정용).
def same_values(a, b, tolerance=1e-9):
    if a is None or b is None:
        return a is b

    a, b = tuple(a), tuple(b)

    if len(a) != len(b):
        return False

    for x, y in zip(a, b):
        if math.isnan(x) and math.isnan(y):
            continue

        if not abs(x - y) <= tolerance:
            return False

    return True


# 입력 stamp 검사. 늦은 stamp만 받아들이고 같은 stamp는 값이 같으면 중복, 다르면 충돌로 나눈다.
# 받아들이지 않은 입력도 판정(verdict)과 개수를 남긴다(원시 기록은 버리지 않고 수용·배제를 구분).
class StampGate:
    VERDICTS = ("ACCEPTED", "DUPLICATE", "CONFLICT", "REVERSED", "INVALID_STAMP")

    # 상태를 비운다.
    def __init__(self):
        self.stamp = None
        self.value = None
        self.counts = {verdict: 0 for verdict in self.VERDICTS}

    # (stamp, 값)을 검사해 판정을 돌려준다. ACCEPTED일 때만 기준 stamp·값을 바꾼다.
    def check(self, stamp, value):
        if stamp is None or not math.isfinite(stamp):
            verdict = "INVALID_STAMP"
        elif self.stamp is None or stamp > self.stamp:
            verdict = "ACCEPTED"
            self.stamp, self.value = stamp, tuple(value)
        elif stamp == self.stamp:
            verdict = "DUPLICATE" if same_values(self.value, value) else "CONFLICT"
        else:
            verdict = "REVERSED"

        self.counts[verdict] += 1

        return verdict


# 한 로봇의 최신 입력 기록. 시각은 sim time(s), wall은 단조 벽시계(s)다.
@dataclass
class Inputs:
    scan_stamp: float = None
    odom_stamp: float = None
    odom_pose: tuple = None
    tf_valid_time: float = None          # map→odom TF stamp − tf_future_s(처리한 스캔 시각)
    map_odom: tuple = None
    tf_chain_ok: bool = False            # map → base_link 조회 성공
    amcl_stamp: float = None
    amcl_pose: tuple = None
    amcl_covariance: tuple = None
    amcl_odom_pose: tuple = None         # amcl_pose 수신 시점의 odom 자세(정지 중 무발행 판단용)
    amcl_updates: int = 0                # 이 세션에서 받은 amcl_pose 수
    odom_break_at: float = None          # 마지막 odom 연속성 상실 시각(없으면 None)
    amcl_after_break: int = 0            # 그 뒤 받은 AMCL 추정 수
    actuation_state: int = None
    actuation_valid: bool = False
    actuation_stamp: float = None


# 위치 신뢰성 판정 결과
@dataclass
class Health:
    state: int
    valid: bool
    reason: str
    position_std_m: float = float("nan")
    yaw_std_rad: float = float("nan")
    pose: tuple = None                   # 현재 map 기준 차체 자세(map→odom · odom)


# sim time 진행을 감시한다. 역행하거나 벽시계로 정해진 시간 동안 멈추면 무효다.
class ClockMonitor:
    # stall_wall_s: sim time이 멈춘 채 허용할 벽시계 시간
    def __init__(self, stall_wall_s):
        self.stall_wall_s = stall_wall_s
        self.last_sim = None
        self.last_change_wall = None
        self.reversed = False

    # 새 표본을 넣고 (유효 여부, 역행 여부)를 돌려준다. 역행은 한 번 일어나면 표시를 남긴다.
    def update(self, sim, wall):
        jumped_back = False

        if self.last_sim is None or sim > self.last_sim:
            self.last_change_wall = wall
        elif sim < self.last_sim - 1e-6:
            self.reversed = True
            jumped_back = True
            self.last_change_wall = wall

        self.last_sim = sim
        stalled = wall - self.last_change_wall > self.stall_wall_s

        return (not stalled and not self.reversed and sim > 0.0), jumped_back


# 이동 거리 비례 위치 std 하한. AMCL 공분산은 odom 보정 오차(바퀴 반경·윤거 비율 오차)를 무작위 잡음으로만 보므로,
# 평행 벽 통로처럼 길이 방향 관측이 없는 구간에서는 계통 오차(이동 거리에 비례)를 작은 공분산으로 낸다.
# 마지막으로 잘 구속된 추정(원래 판정 OK, 위치 std가 position_std_min_m 이상 constrained_std_m 이하) 뒤
# odom 누적 경로 길이에 unconstrained_std_per_m을 곱한 값을 위치 std 하한으로 쓴다(0이면 끔, 기본 0: 통로 시험 기준 미달).
# 정지 중에는 경로가 늘지 않으므로 하한도 그대로다. 진단에는 원래 std와 하한을 따로 남긴다.
class TravelStdFloor:
    # config: 전체 설정
    def __init__(self, config):
        self.config = config
        self.path_m = 0.0            # 마지막 잘 구속된 추정 뒤 odom 누적 경로(m)
        self.last_odom = None

    # odom 표본마다 경로를 더한다. 비유한 표본은 건너뛰고 끊김(odom_problem)이면 기준만 다시 잡는다.
    def on_odom(self, pose, odom_problem=None):
        if not finite_pose(pose):
            return

        if self.last_odom is not None and odom_problem is None:
            self.path_m += difference(self.last_odom, pose)[0]

        self.last_odom = pose

    # 하한을 적용하기 전 판정(raw)이 잘 구속된 추정이면 경로를 0으로 되돌린다.
    def observe(self, raw):
        h = self.config["health"]

        if raw.state == OK and h["position_std_min_m"] <= raw.position_std_m <= h["constrained_std_m"]:
            self.path_m = 0.0

    # 재위치 성공 등으로 추정이 새로 검증되면 경로를 0으로 되돌린다.
    def reset(self):
        self.path_m = 0.0

    # 현재 위치 std 하한(m)
    def floor(self):
        return self.config["health"]["unconstrained_std_per_m"] * self.path_m


# 입력 기록으로 위치 신뢰성 상태를 판정한다(재위치 진행·실패 표시는 호출자가 더한다).
# std_floor: 위치 std 하한(TravelStdFloor.floor). 공분산 std가 이보다 작으면 이 값을 위치 std로 보고 판정·보고한다.
def evaluate(inputs, now, config, clock_valid=True, std_floor=0.0):
    h = config["health"]

    if not clock_valid:
        return Health(UNKNOWN, False, "CLOCK_INVALID")

    if inputs.amcl_updates == 0 and inputs.map_odom is None:
        return Health(UNKNOWN, False, "NOT_INITIALIZED")

    lost, degraded = [], []
    std = covariance_std(inputs.amcl_covariance) if inputs.amcl_covariance is not None else None

    # 추정 수치
    if inputs.amcl_pose is not None and (std is None or not finite_pose(inputs.amcl_pose)):
        lost.append("ESTIMATE_INVALID")

    if inputs.map_odom is not None and not finite_pose(inputs.map_odom):
        lost.append("ESTIMATE_INVALID")

    # odom 수치 오류와 연속성 상실. 끊긴 뒤에는 AMCL이 새 관측으로 min_updates번 갱신하기 전까지 정상으로 보지 않는다.
    if inputs.odom_pose is not None and not finite_pose(inputs.odom_pose):
        lost.append("ODOM_INVALID")

    if inputs.odom_break_at is not None and inputs.amcl_after_break < config["reinit"]["min_updates"]:
        lost.append("ODOM_DISCONTINUOUS")

    # TF: map→odom 나이(처리한 스캔 시각 기준)와 map→base_link 연결
    tf_age = None if inputs.tf_valid_time is None else now - inputs.tf_valid_time

    if tf_age is None or tf_age > h["tf_lost_s"] or not inputs.tf_chain_ok:
        lost.append("TF_MISSING")
    elif tf_age > h["tf_stale_s"]:
        degraded.append("TF_STALE")

    # 센서·odom 최신성
    for stamp, stale, gone, code in ((inputs.scan_stamp, h["scan_stale_s"], h["scan_lost_s"], "SCAN_STALE"),
                                     (inputs.odom_stamp, h["odom_stale_s"], h["odom_lost_s"], "ODOM_STALE")):
        age = None if stamp is None else now - stamp

        if age is None or age > gone:
            lost.append("INPUT_LOST")
        elif age > stale:
            degraded.append(code)

    # 공분산(정확도 확정이 아니라 추정 불확실성의 상한 검사). 붕괴 검사는 하한 적용 전 값으로 한다.
    if std is not None:
        collapsed = covariance_collapsed(std, config)
        position_std, yaw_std = max(std[0], std_floor), std[1]

        if position_std > h["position_std_lost_m"] or yaw_std > h["yaw_std_lost_rad"]:
            lost.append("COVARIANCE_LOST")
        elif position_std > h["position_std_ok_m"] or yaw_std > h["yaw_std_ok_rad"]:
            degraded.append("COVARIANCE_HIGH")
        elif collapsed:
            # 공분산 0(필터 산포 없음)은 정확하다는 근거가 아니므로 OK로 내지 않는다.
            degraded.append("COVARIANCE_COLLAPSED")
    else:
        position_std, yaw_std = float("nan"), float("nan")

    # AMCL은 정지 중 갱신하지 않으므로 시간이 아니라 마지막 갱신 뒤 odom 이동량으로 갱신 정체를 본다.
    # 두 자세가 모두 유한할 때만 이동량을 계산한다. 하나라도 비유한 값이면 이동량을 알 수 없으므로 정체로 본다
    # (현재 odom이 비유한 값이면 위에서 ODOM_INVALID로도 잡힌다).
    if inputs.amcl_odom_pose is not None and inputs.odom_pose is not None:
        if finite_pose(inputs.amcl_odom_pose) and finite_pose(inputs.odom_pose):
            travel, rotation = difference(inputs.amcl_odom_pose, inputs.odom_pose)

            if travel > h["estimate_travel_m"] or rotation > h["estimate_rotation_rad"]:
                degraded.append("ESTIMATE_STALE")
        else:
            degraded.append("ESTIMATE_STALE")
    elif inputs.amcl_updates == 0:
        degraded.append("ESTIMATE_STALE")

    pose = None

    if finite_pose(inputs.map_odom) and finite_pose(inputs.odom_pose):
        pose = compose(inputs.map_odom, inputs.odom_pose)

    # 현재 자세를 계산할 수 없으면 정상이 아니다.
    if pose is None and not lost:
        lost.append("ESTIMATE_INVALID")

    if lost:
        return Health(LOST, True, join_reasons(lost + degraded), position_std, yaw_std, pose)

    if degraded:
        return Health(DEGRADED, True, join_reasons(degraded), position_std, yaw_std, pose)

    return Health(OK, True, "", position_std, yaw_std, pose)


# 갱신 정체 검사의 기준 odom(마지막 AMCL 추정 시점의 odom)을 정한다.
# - AMCL 추정을 받을 때: 그때의 odom이 유한하면 그 값, 아니면 None(다음 유효 odom이 채운다).
# - odom을 받을 때: 추정은 있는데 기준이 비어 있을 때만, 끊김 없는 유한 odom으로 채운다. 이미 있으면 그대로 둔다
#   (odom마다 덮어쓰면 이동량이 계속 0으로 돌아가 정체를 못 잡는다).
def estimate_baseline(current, odom_pose, amcl_updates, odom_problem=None, on_amcl=False):
    if on_amcl:
        return odom_pose if finite_pose(odom_pose) else None

    if current is None and amcl_updates > 0 and odom_problem is None and finite_pose(odom_pose):
        return odom_pose

    return current


# odom 연속성 감시. 시각 역행·비유한 값·속도로 설명되지 않는 자세 도약을 끊김으로 보고, 끊긴 시각과
# 그 뒤 AMCL 추정 수를 남긴다(판정이 이 값으로 ODOM_DISCONTINUOUS를 낸다).
class OdomContinuity:
    # config: odom_continuity 묶음
    def __init__(self, config):
        self.config = config
        self.last = None
        self.broken_at = None
        self.amcl_after_break = 0
        self.last_amcl = None        # 마지막으로 센 AMCL stamp(같은·역행 stamp를 다시 세지 않는다)

    # 새 odom (stamp, pose, linear_speed, angular_speed)을 넣고 끊김 사유(없으면 None)를 돌려준다.
    # 비유한 표본은 기준으로 남기지 않는다(다음 표본을 마지막 정상 표본과 비교).
    def update(self, stamp, pose, linear, angular):
        problem = self.check(stamp, pose, linear, angular)

        if problem is not None:
            self.broken_at = stamp if math.isfinite(stamp) else (self.last[0] if self.last else 0.0)
            self.amcl_after_break = 0

        if finite_pose(pose) and math.isfinite(stamp):
            self.last = (stamp, pose, linear, angular)

        return problem

    # 끊긴 뒤 받은 AMCL 추정을 센다(끊긴 시각보다 늦은 추정만). 비유한·같은·역행 stamp는 새 추정이 아니므로 세지 않는다.
    def note_amcl(self, stamp):
        if not new_stamp(self.last_amcl, stamp):
            return

        self.last_amcl = stamp

        if self.broken_at is not None and stamp > self.broken_at:
            self.amcl_after_break += 1

    # 한 표본의 끊김 사유
    def check(self, stamp, pose, linear, angular):
        if not finite_pose(pose) or not math.isfinite(stamp):
            return "ODOM_INVALID"

        previous = self.last

        if previous is None:
            return None

        dt = stamp - previous[0]

        if dt < 0:
            return "ODOM_TIME_REVERSED"

        travel, rotation = difference(previous[1], pose)
        margin = self.config["speed_margin"]
        allowed = max(abs(linear), abs(previous[2])) * dt * margin + self.config["max_jump_m"]
        allowed_rotation = max(abs(angular), abs(previous[3])) * dt * margin + self.config["max_jump_rad"]

        if travel > allowed or rotation > allowed_rotation:
            return "ODOM_JUMP"

        return None


# 최근 odom 자세 기록. 추정 시각에 가장 가까운 유효 odom을 찾아 이전·새 추정을 같은 odom 자세에서 비교한다.
class OdomHistory:
    # keep_s: 보관 기간(sim s)
    def __init__(self, keep_s):
        self.keep_s = keep_s
        self.samples = deque()

    # 유한한 odom 표본을 넣는다. 시각이 역행하면 기록을 비운다(역행 전 표본과 섞지 않는다).
    def add(self, stamp, pose):
        if not (math.isfinite(stamp) and finite_pose(pose)):
            return

        if self.samples and stamp < self.samples[-1][0]:
            self.samples.clear()

        self.samples.append((stamp, pose))

        while self.samples and self.samples[0][0] < stamp - self.keep_s:
            self.samples.popleft()

    # 시각 t에 가장 가까운 표본의 자세. 차이가 max_age_s를 넘으면 None(대응 없음·지연).
    def lookup(self, t, max_age_s):
        if t is None or not math.isfinite(t) or not self.samples:
            return None

        stamp, pose = min(self.samples, key=lambda sample: abs(sample[0] - t))

        return pose if abs(stamp - t) <= max_age_s else None

    # 기록을 비운다.
    def clear(self):
        self.samples.clear()


# AMCL 재시작 감지. 두 신호 중 하나면 재시작으로 본다.
# - lifecycle 전이: active 상태에서 나가는 전이(deactivate·shutdown). 최초 기동(unconfigured→active)은 해당 없음.
# - 공분산 붕괴: 이 세션에서 산포가 있는 추정을 받은 뒤 다시 공분산 0 추정이 오면(AMCL 프로세스 재시작 포함).
#   최초 초기화의 공분산 0 추정은 산포 있는 추정 전이므로 재시작으로 보지 않는다.
#   감독 노드 sim time이 유효해진 시각(basis_at)보다 이른 stamp의 추정(transient local로 다시 받은 이전 세션 추정)은
#   세지 않는다(launch 초기 위치 확인과 같은 기준).
class AmclResetDetector:
    # config: 전체 설정
    def __init__(self, config):
        self.config = config
        self.spread_seen = False     # 산포 있는(붕괴 아닌) 유효 추정을 받았는지

    # 받아들인 추정(stamp, std)으로 판단한다. 재시작이면 사유 문자열, 아니면 None.
    # basis_at: 감독 노드 sim time이 처음 유효해진 시각(아직 없으면 None → 판단하지 않음)
    def on_estimate(self, std, stamp=None, basis_at=None):
        if std is None or basis_at is None or stamp is None or stamp < basis_at:
            return None

        if not covariance_collapsed(std, self.config):
            self.spread_seen = True
            return None

        return "COVARIANCE_COLLAPSED_AFTER_SPREAD" if self.spread_seen else None

    # lifecycle 전이 사건(시작 상태 label)으로 판단한다. active에서 나가는 전이면 사유, 아니면 None.
    @staticmethod
    def on_transition(start_label, goal_label):
        if start_label == "active":
            return f"LIFECYCLE_{start_label}_TO_{goal_label}".upper()

        return None


# launch initial_poses 최초 초기화의 산포 만들기. launch 초기 위치가 확인(established)됐는데 추정 공분산이 붕괴해 있으면
# 감독 노드가 현재 자세를 initial_spread 표준편차로 한 번 보낸다(이 세션에서 한 번만). 보낸 뒤 health가 OK가 되고
# 새 추정 min_updates개를 받거나 timeout_s가 지나면 무이동 갱신 요청을 멈춘다(정지 중 재표본만 반복하면 다시 붕괴하므로).
class InitialSpread:
    # config: 전체 설정
    def __init__(self, config):
        self.config = config
        self.sent_at = None
        self.updates = 0
        self.done = False
        self.last_nomotion = None
        self.result = None           # None(진행·미송신), "SPREAD", "TIMEOUT"

    # 지금 보내야 하는지: 아직 안 보냈고, 시계 유효, launch 초기 위치 확인됨, 현재 추정 공분산 붕괴, 현재 자세 있음
    def due(self, enabled, established, std, pose, clock_valid):
        return (enabled and self.sent_at is None and clock_valid and established and finite_pose(pose)
                and covariance_collapsed(std, self.config))

    # 보낸 시각을 기록한다.
    def mark_sent(self, now):
        self.sent_at = now

    # 보낸 뒤의 새 추정을 센다(보낸 시각보다 늦은 stamp, 붕괴하지 않은 것만).
    def on_amcl(self, stamp, std):
        if self.sent_at is not None and not self.done and stamp > self.sent_at and std is not None \
                and not covariance_collapsed(std, self.config):
            self.updates += 1

    # 진행 상태를 갱신하고 무이동 갱신 요청이 필요한지 돌려준다.
    def step(self, now, health_ok):
        if self.sent_at is None or self.done:
            return False

        if health_ok and self.updates >= self.config["reinit"]["min_updates"]:
            self.done, self.result = True, "SPREAD"
            return False

        if now - self.sent_at > self.config["initial_spread"]["timeout_s"]:
            self.done, self.result = True, "TIMEOUT"
            return False

        if self.last_nomotion is None or now - self.last_nomotion >= self.config["reinit"]["nomotion_period_s"]:
            self.last_nomotion = now
            return True

        return False


# 추정 연속성 감시: map→odom TF와 amcl_pose를 직전 비교 기준과 같은 odom 자세에서 비교해 도약을 찾는다.
# - 비교 기준(reference)은 마지막으로 검증한 map→odom이다. TF 공백·신뢰 기록 유무와 관계없이 유지한다.
# - 도약이면 latch(LOST·ESTIMATE_DISCONTINUOUS)한다. 해제는 허용된 재위치가 성공했을 때(release)만이다.
# - odom 대응이 없거나 늦으면 도약량을 확정하지 않는다(근거 부족: 비교 기준 유지, DEGRADED ODOM_STALE, 신뢰 기록 보류).
# - 감독 노드가 보낸 초기 위치·재위치 후보(intent)는 송신 뒤 새 입력이 제한시간 안에 후보와 맞을 때 한 번만 허용한다.
# - 신뢰 기록 조건(trust_block): latch 없음·충돌 없음·근거 부족 없음·의도된 초기화 확인(established)·
#   최신 amcl_pose와 현재 TF가 같은 갱신으로 대응(pair).
class EstimateMonitor:
    # config: 전체 설정
    def __init__(self, config):
        self.config = config
        self.j = config["estimate_jump"]
        self.reference = None        # 마지막으로 검증한 map→odom(비교 기준)
        self.current = None          # 마지막으로 받은 map→odom
        self.latched = None          # 도약 기록(dict). None이면 latch 없음
        self.unverified = None       # odom 대응이 없어 비교하지 못한 TF (valid_time, map_odom)
        self.established = False     # 의도된 초기화·재위치로 확인된 추정인지(최초 TF는 근거가 아님)
        self.intent = None           # 감독 노드가 보낸 초기 위치(dict)
        self.amcl = None             # 마지막 amcl_pose (stamp, pose)
        self.amcl_odom = None        # 그 stamp에 대응한 odom 자세(없으면 None)
        self.conflicts = set()       # 같은 stamp·다른 값이 온 입력(다음 새 입력까지 신뢰 기록 보류)
        self.events = []             # 진단 기록용 사건(감독 노드가 비운다)

    # 사건 하나를 진단 목록에 남긴다.
    def note(self, kind, **detail):
        self.events.append({"event": kind, **detail})

    # 이전·새 추정을 같은 odom 자세에서 비교한 (거리, 회전)
    @staticmethod
    def jump(reference, new_map_odom, odom_pose):
        return difference(compose(reference, odom_pose), compose(new_map_odom, odom_pose))

    # 도약 기준을 넘는지
    def exceeds(self, distance, rotation):
        return distance > self.j["position_m"] or rotation > self.j["yaw_rad"]

    # latch를 건다(이미 걸려 있으면 처음 기록을 유지).
    def latch(self, source, stamp, distance, rotation):
        if self.latched is None:
            self.latched = {"source": source, "stamp": stamp, "jump_m": distance, "jump_rad": rotation}
            self.note("LATCHED", source=source, stamp=stamp, jump_m=distance, jump_rad=rotation)

    # AMCL 재시작(lifecycle 재활성·공분산 0 재초기화)을 감지했을 때 latch를 건다. AMCL은 재시작 때 마지막 필터 갱신
    # 자세로 공분산 0 재초기화를 하므로, 차이가 도약 기준보다 작아도 의도된 변경이 아니다. 해제는 재위치 성공 때만이다.
    def reset(self, source, stamp, detail):
        if self.latched is None:
            self.latched = {"source": source, "stamp": stamp, "jump_m": None, "jump_rad": None, "detail": detail}
            self.note("LATCHED", source=source, stamp=stamp, detail=detail)

    # 의도 후보를 기록한다. 확인 기준은 출처마다 다르다.
    # - START_POSE·REINIT: 감독 노드가 initialpose를 보낸 시각(basis "SENT")이 기준이고, 제한시간도 송신부터 잰다.
    # - LAUNCH_INITIAL_POSE: launch가 AMCL 매개변수로 넣은 값이라 송신이 없다. 감독 노드의 sim time이 유효해진
    #   시각(basis "CLOCK_VALID")을 기준으로 그 뒤 stamp의 새 입력만 쓰고(이전 세션·transient local 추정 배제),
    #   제한시간은 AMCL active를 처음 확인한 때(amcl_active)부터 잰다. 그 전에는 만료하지 않는다.
    # 시계가 무효면 기준 시각을 정할 수 없으므로 기록하지 않는다.
    def expect(self, pose, source, now, wall, clock_valid):
        if not clock_valid or not finite_pose(pose):
            self.note("INTENT_REFUSED", source=source, reason="CLOCK_INVALID" if not clock_valid else "POSE_INVALID")
            return False

        launch = source == "LAUNCH_INITIAL_POSE"
        self.intent = {"pose": tuple(pose), "source": source, "basis": "CLOCK_VALID" if launch else "SENT",
                       "basis_at": now, "window_at": None if launch else now, "window_wall": None if launch else wall}
        self.note("INTENT", source=source, pose=list(pose), basis=self.intent["basis"], basis_at=now)

        return True

    # launch 초기 위치 의도의 제한시간을 AMCL active 확인 시각부터 재기 시작한다(한 번만).
    def amcl_active(self, now, wall):
        if self.intent is not None and self.intent["window_at"] is None:
            self.intent["window_at"], self.intent["window_wall"] = now, wall
            self.note("INTENT_WINDOW", source=self.intent["source"], window_at=now)

    # 의도 기록을 지운다(재위치 실패·완료). timeout은 제한시간으로 닫을 때의 두 시계 조건(진단 기록용)이다.
    def cancel_intent(self, reason, timeout=None):
        if self.intent is not None:
            self.note("INTENT_CLOSED", source=self.intent["source"], reason=reason,
                      **({"timeout": timeout} if timeout is not None else {}))
            self.intent = None

    # 제한시간(sim)·벽시계 상한이 지났거나 시계가 무효면 의도 기록을 닫는다.
    def expire(self, now, wall, clock_valid):
        if self.intent is None:
            return

        if not clock_valid:
            self.cancel_intent("CLOCK_INVALID")
            return

        if self.intent["window_at"] is None:
            return

        # 판단 순서·사유 문자열은 그대로 두고(sim 우선), 닫을 때 두 시계 조건을 함께 남긴다.
        sim_elapsed = now - self.intent["window_at"]
        sim_expired = sim_elapsed > self.j["intent_timeout_s"]
        wall_elapsed = (None if wall is None or self.intent["window_wall"] is None
                        else wall - self.intent["window_wall"])
        wall_expired = wall_elapsed is not None and wall_elapsed > self.j["intent_timeout_wall_s"]

        if sim_expired or wall_expired:
            detail = timeout_detail(sim_elapsed, self.j["intent_timeout_s"], sim_expired,
                                    wall_elapsed, self.j["intent_timeout_wall_s"], wall_expired)
            self.cancel_intent("TIMEOUT" if sim_expired else "TIMEOUT_WALL", detail)

    # 의도를 확인할 수 있는 단계인지. launch 초기 위치(LAUNCH_INITIAL_POSE)는 감독 노드가 보낸 것이 아니므로
    # TF만으로 확인하지 않고, 기록 뒤 stamp의 새 amcl_pose가 후보와 맞은 다음에만 TF로 확인한다
    # (재발행 TF·이전 세션의 추정만으로 초기화가 검증됐다고 보지 않는다).
    def intent_ready(self):
        return self.intent["source"] != "LAUNCH_INITIAL_POSE" or self.intent.get("amcl_matched", False)

    # 새 입력(stamp)이 확인 기준 시각(송신 또는 시계 유효) 뒤이고 자세가 의도한 후보와 맞는지
    def matches_intent(self, stamp, pose):
        if self.intent is None or stamp < self.intent["basis_at"]:
            return False

        distance, rotation = difference(self.intent["pose"], pose)

        return distance <= self.j["intent_position_m"] and rotation <= self.j["intent_yaw_rad"]

    # 받아들인(새 stamp) map→odom TF를 판단한다. 결정 코드를 돌려준다.
    def on_tf(self, valid_time, map_odom, history, now=None, wall=None, clock_valid=True):
        if now is not None:
            self.expire(now, wall, clock_valid)

        self.conflicts.discard("tf")

        if not finite_pose(map_odom):
            return "INVALID"

        self.current = tuple(map_odom)

        return self.judge_tf(valid_time, self.current, history)

    # TF 한 건을 비교 기준과 비교한다(on_tf·retry 공통).
    # 값이 비교 기준과 같으면 도약 비교는 생략하지만, 의도 기록이 있으면 의도 확인(odom·송신 뒤 시각·후보 일치)은 한다.
    def judge_tf(self, valid_time, map_odom, history):
        unchanged = self.reference is not None and same_values(map_odom, self.reference)

        if unchanged and self.intent is None:
            self.unverified = None
            return "UNCHANGED"

        if self.reference is None and self.intent is None:
            self.reference = map_odom
            self.note("BASELINE", valid_time=valid_time)
            return "BASELINE"

        odom_pose = history.lookup(valid_time, self.j["odom_match_s"])

        if odom_pose is None:
            # 기준과 같은 값은 도약이 아니므로 근거 부족으로 내리지 않는다(의도 확인은 다음 입력에서).
            if unchanged:
                self.unverified = None
                return "UNCHANGED"

            if self.unverified is None:
                self.note("UNVERIFIED", valid_time=valid_time)

            self.unverified = (valid_time, map_odom)
            return "UNVERIFIED"

        self.unverified = None

        if self.matches_intent(valid_time, compose(map_odom, odom_pose)) and self.intent_ready():
            source = self.intent["source"]
            self.reference = map_odom

            if source in ("START_POSE", "LAUNCH_INITIAL_POSE", "INITIAL_SPREAD"):
                self.established = True

            self.note("INTENDED", source=source, valid_time=valid_time)
            self.cancel_intent("MATCHED")
            return "INTENDED"

        if unchanged:
            return "UNCHANGED"

        if self.reference is None:
            # 의도 기록은 있는데 아직 확인하지 못한 첫 TF: 비교 기준으로만 둔다(초기화 확인 아님).
            self.reference = map_odom
            self.note("BASELINE", valid_time=valid_time, intent_unconfirmed=True)
            return "BASELINE"

        distance, rotation = self.jump(self.reference, map_odom, odom_pose)

        if self.exceeds(distance, rotation):
            self.latch("tf", valid_time, distance, rotation)
            return "JUMP"

        if self.latched is not None:
            return "LATCHED"

        self.reference = map_odom
        return "CONTINUOUS"

    # 받아들인 amcl_pose를 판단한다. TF보다 먼저 와도 비교 기준 · 그 시각 odom과 비교해 도약을 잡는다.
    def on_amcl(self, stamp, pose, history):
        self.conflicts.discard("amcl")
        self.amcl = (stamp, tuple(pose))
        self.amcl_odom = None

        if not finite_pose(pose):
            return "INVALID"

        return self.judge_amcl(history)

    # 마지막 amcl_pose를 비교한다(on_amcl·retry 공통).
    def judge_amcl(self, history):
        stamp, pose = self.amcl
        odom_pose = history.lookup(stamp, self.j["odom_match_s"])

        if odom_pose is None:
            return "UNVERIFIED"

        self.amcl_odom = odom_pose

        if self.matches_intent(stamp, pose):
            self.intent["amcl_matched"] = True
            return "INTENDED_PENDING"

        if self.reference is None:
            return "NO_REFERENCE"

        distance, rotation = difference(compose(self.reference, odom_pose), pose)

        if self.exceeds(distance, rotation):
            self.latch("amcl", stamp, distance, rotation)
            return "JUMP"

        return "LATCHED" if self.latched is not None else "CONTINUOUS"

    # 같은 stamp·다른 값 입력을 기록한다. 그 입력의 다음 새 값이 올 때까지 신뢰 기록을 보류한다.
    def conflict(self, stream, stamp):
        self.conflicts.add(stream)
        self.note("CONFLICT", stream=stream, stamp=stamp)

    # 판정마다: 의도 제한시간을 보고, odom이 늦게 와서 비교하지 못한 입력을 다시 비교한다.
    def retry(self, history, now, wall, clock_valid):
        self.expire(now, wall, clock_valid)
        decisions = []

        if self.unverified is not None:
            decisions.append(("tf", self.judge_tf(*self.unverified, history)))

        if self.amcl is not None and self.amcl_odom is None and finite_pose(self.amcl[1]):
            decisions.append(("amcl", self.judge_amcl(history)))

        return decisions

    # 최신 amcl_pose와 현재 TF가 같은 갱신인지(정확한 stamp 일대일 대응을 가정하지 않고 같은 odom 자세의 값으로 본다).
    def paired(self):
        if self.amcl is None or self.amcl_odom is None or self.current is None or not finite_pose(self.amcl[1]):
            return False

        distance, rotation = difference(compose(self.current, self.amcl_odom), self.amcl[1])

        return distance <= self.j["pair_position_m"] and rotation <= self.j["pair_yaw_rad"]

    # 신뢰 기록을 갱신하지 못하는 추정 쪽 사유(없으면 None)
    def trust_block(self):
        if self.latched is not None:
            return "ESTIMATE_DISCONTINUOUS"

        if self.conflicts:
            return "INPUT_CONFLICT"

        if self.unverified is not None:
            return "ESTIMATE_UNVERIFIED"

        if not self.established:
            return "NOT_ESTABLISHED"

        if not self.paired():
            return "ESTIMATE_UNPAIRED"

        return None

    # 허용된 재위치가 성공했을 때만 latch를 풀고 비교 기준을 검증된 새 추정으로 바꾼다(한 번에 전환).
    def release(self, map_odom):
        self.note("RELEASED", had_latch=self.latched is not None)
        self.latched = None
        self.unverified = None
        self.reference = tuple(map_odom)
        self.current = tuple(map_odom)
        self.established = True
        self.cancel_intent("REINIT_SUCCEEDED")

    # 판정 결과에 latch·근거 부족을 반영한다. 판정 불가(valid=false, 예: CLOCK_INVALID)는 덮지 않고 사유만 덧붙인다.
    def apply(self, health):
        if self.latched is None and self.unverified is None:
            return health

        reasons = [code for code in health.reason.split(",") if code]

        if not health.valid:
            extra = ["ESTIMATE_DISCONTINUOUS"] if self.latched is not None else []
            return Health(health.state, False, join_reasons(reasons + extra), health.position_std_m,
                          health.yaw_std_rad, health.pose)

        if self.latched is not None:
            return Health(LOST, True, join_reasons(["ESTIMATE_DISCONTINUOUS"] + reasons), health.position_std_m,
                          health.yaw_std_rad, health.pose)

        # 근거 부족: 추정 시각의 odom이 없거나 늦다(기존 ODOM_STALE로 알린다). OK로 두지 않는다.
        state = DEGRADED if health.state == OK else health.state
        return Health(state, True, join_reasons(reasons + ["ODOM_STALE"]), health.position_std_m,
                      health.yaw_std_rad, health.pose)

    # 진단 기록용 요약
    def summary(self):
        return {"latched": self.latched, "established": self.established,
                "unverified": self.unverified is not None, "intent": None if self.intent is None else
                {"source": self.intent["source"], "basis": self.intent["basis"], "basis_at": self.intent["basis_at"],
                 "window_at": self.intent["window_at"], "amcl_matched": self.intent.get("amcl_matched", False)},
                "reference": None if self.reference is None else list(self.reference),
                "paired": self.paired(), "conflicts": sorted(self.conflicts)}


# 마지막 신뢰 위치: 신뢰 조건을 만족한 마지막 시각의 map→odom·공분산·odom 자세와 지도 식별값.
@dataclass
class TrustedPose:
    stamp: float
    map_odom: tuple
    odom_pose: tuple
    position_std_m: float
    yaw_std_rad: float
    map_id: str


# 로봇 하나의 마지막 신뢰 위치를 관리한다(같은 실행 세션 안에서만, 디스크에 남기지 않음).
class TrustedPoseTracker:
    # config: 전체 설정
    def __init__(self, config):
        self.config = config
        self.record = None
        self.invalid_reason = None
        self.path_m = 0.0            # 기록 뒤 odom 누적 이동 거리
        self.path_rad = 0.0          # 기록 뒤 odom 누적 회전량
        self.last_odom = None

    # odom 표본마다 기록 뒤의 누적 거리·회전량을 더한다(왕복·반복 회전도 모두 센다). 비유한 표본은 건너뛴다.
    def on_odom(self, pose):
        if not finite_pose(pose):
            return

        if self.record is not None and self.last_odom is not None:
            distance, rotation = difference(self.last_odom, pose)
            self.path_m += distance
            self.path_rad += rotation

        self.last_odom = pose

    # 신뢰 기록을 갱신하지 않는 첫 사유(진단 기록용). 갱신 조건을 모두 만족하면 None.
    # consider()가 이 함수로 판단하므로 두 결과는 항상 같다.
    # estimate_block: EstimateMonitor.trust_block() 결과(도약 latch·근거 부족·초기화 미확인·갱신 대응 실패)
    def rejection(self, health, inputs, map_id, reinit_active, estimate_block=None):
        t = self.config["trust"]

        if reinit_active:
            return "REINIT_ACTIVE"

        if estimate_block is not None:
            return estimate_block

        if health.state != OK:
            return "HEALTH_NOT_OK"

        if map_id is None:
            return "MAP_UNKNOWN"

        if not health.position_std_m <= t["position_std_m"]:
            return "POSITION_STD_HIGH"

        if not health.yaw_std_rad <= t["yaw_std_rad"]:
            return "YAW_STD_HIGH"

        if not (finite_pose(inputs.map_odom) and finite_pose(inputs.odom_pose)):
            return "POSE_INVALID"

        return None

    # 판정 결과가 신뢰 조건을 만족하면 기록을 갱신한다. 재위치 중·입력 무효·연결성 상실 동안은 갱신하지 않는다.
    def consider(self, health, inputs, now, map_id, reinit_active, estimate_block=None):
        if self.rejection(health, inputs, map_id, reinit_active, estimate_block) is not None:
            return False

        self.record = TrustedPose(now, inputs.map_odom, inputs.odom_pose, health.position_std_m,
                                  health.yaw_std_rad, map_id)
        self.invalid_reason = None
        self.path_m = self.path_rad = 0.0
        self.last_odom = inputs.odom_pose

        return True

    # 연결성을 잃으면 기록을 복구 근거에서 뺀다(지도 변경·시계 역행·odom 끊김).
    def invalidate(self, reason):
        if self.record is not None:
            self.record = None
            self.invalid_reason = reason

    # 현재 지도가 기록의 지도와 다르면 무효로 만든다.
    def check_map(self, map_id):
        if self.record is not None and map_id != self.record.map_id:
            self.invalidate("MAP_CHANGED")


# 재위치 후보를 계산한다. 근거가 부족하면 (None, 거절 코드)를 돌려준다.
# 후보 = 마지막 신뢰 map→odom · 현재 odom. 공분산은 이동량에 따라 키운다.
def reinit_candidate(tracker, odom_pose, now, map_id, config):
    r = config["reinit"]

    if tracker.record is None:
        if tracker.invalid_reason == "MAP_CHANGED":
            return None, "REINIT_REJECTED_MAP_CHANGED"

        if tracker.invalid_reason == "CLOCK_REVERSED":
            return None, "REINIT_REJECTED_CLOCK_INVALID"

        if tracker.invalid_reason is not None and tracker.invalid_reason.startswith("ODOM_"):
            return None, "REINIT_REJECTED_ODOM_DISCONTINUOUS"

        return None, "REINIT_REJECTED_NO_TRUSTED_POSE"

    record = tracker.record

    if map_id != record.map_id:
        return None, "REINIT_REJECTED_MAP_CHANGED"

    if not finite_pose(odom_pose):
        return None, "REINIT_REJECTED_ODOM_DISCONTINUOUS"

    if now - record.stamp > r["trusted_max_age_s"] or now < record.stamp:
        return None, "REINIT_REJECTED_TRUSTED_POSE_EXPIRED"

    # 이동 제한·불확실성은 기록 뒤 누적 이동량으로 본다(순변위만 보면 왕복·반복 회전으로 우회된다).
    net_travel, net_rotation = difference(record.odom_pose, odom_pose)
    travel = max(tracker.path_m, net_travel)
    rotation = max(tracker.path_rad, net_rotation)

    if travel > r["max_travel_m"] or rotation > r["max_rotation_rad"]:
        return None, "REINIT_REJECTED_MOTION_TOO_LARGE"

    pose = compose(record.map_odom, odom_pose)
    position_std = max(record.position_std_m, r["base_std_m"]) + r["std_per_m"] * travel
    yaw_std = max(record.yaw_std_rad, r["base_yaw_std_rad"]) + r["yaw_std_per_rad"] * rotation

    return {"pose": pose, "position_std_m": position_std, "yaw_std_rad": yaw_std,
            "travel_m": travel, "rotation_rad": rotation, "net_travel_m": net_travel,
            "net_rotation_rad": net_rotation, "trusted_stamp": record.stamp}, None


# A4 구동 상태가 최신·유효·STOPPED인지 확인한다. 아니면 사유 코드.
def actuation_problem(inputs, now, config):
    if inputs.actuation_stamp is None or not inputs.actuation_valid:
        return "ACTUATION_STALE"

    if now - inputs.actuation_stamp > config["reinit"]["actuation_max_age_s"] or now < inputs.actuation_stamp:
        return "ACTUATION_STALE"

    if inputs.actuation_state != ACTUATION_STOPPED:
        return "NOT_STOPPED"

    return None


# 진행 중인 재위치 한 건. 초기 위치 전달 뒤 새 AMCL 추정·TF·센서가 회복 조건을 만족해야 완료한다.
# 시간초과 한 건의 두 시계 조건(sim time·단조 벽시계 경과와 제한, 초과 여부). 진단 기록용이며 판정에 쓰지 않는다.
def timeout_detail(sim_elapsed, sim_limit, sim_expired, wall_elapsed, wall_limit, wall_expired):
    return {"sim_elapsed_s": sim_elapsed, "sim_limit_s": sim_limit, "sim_expired": bool(sim_expired),
            "wall_elapsed_s": wall_elapsed, "wall_limit_s": wall_limit, "wall_expired": bool(wall_expired),
            "wall_source": "단조 벽시계(time.monotonic)"}


@dataclass
class ReinitAttempt:
    accepted_at: float
    candidate: dict
    accepted_wall: float = None                    # 접수 시각(단조 벽시계). sim time이 멈춰도 끝나게 한다
    sent_at: float = None
    updates: list = field(default_factory=list)   # (stamp, pose, position_std, yaw_std)
    last_nomotion: float = None
    result: str = None                             # None(진행), "SUCCEEDED", "REINIT_FAILED_*"
    timeout: dict = None                           # 시간초과로 끝났을 때 두 시계 조건(진단 기록용, 판정에 쓰지 않음)
    number: int = None                             # 이 감독 세션의 접수 순번(진단 기록의 접수·종료 대응용, 판정에 쓰지 않음)


# 현재 map→odom · odom 자세가 후보와 허용 범위 안인지(재위치 완료 시 TF도 같은 갱신을 따랐는지 확인).
def tf_pose_matches(inputs, pose, position_m, yaw_rad):
    if not (finite_pose(inputs.map_odom) and finite_pose(inputs.odom_pose)):
        return False

    distance, rotation = difference(pose, compose(inputs.map_odom, inputs.odom_pose))

    return distance <= position_m and rotation <= yaw_rad


# 재위치 절차. 요청 접수·진행·완료·실패를 관리하고 결과 사유를 판정에 더한다.
class Reinitializer:
    # config: 전체 설정
    def __init__(self, config):
        self.config = config
        self.attempt = None
        self.failure = None          # 마지막 실패 사유(다음 요청이나 회복 근거가 생기기 전까지 유지)
        self.failure_at = None       # 실패 확정 시각
        self.after_failure = []      # 실패 뒤 받은 AMCL 추정 (stamp, position_std, yaw_std)
        self.last_amcl = None        # 마지막으로 받은 AMCL stamp
        self.accepted_count = 0      # 접수한 재위치 수(진단 기록의 시도 번호, 판정에 쓰지 않음)

    # 진행 중인지
    def active(self):
        return self.attempt is not None and self.attempt.result is None

    # D의 요청을 처리한다. (접수 여부, 응답 코드)를 돌려준다. 진행 중이면 같은 재위치로 본다.
    def request(self, health, inputs, tracker, now, map_id, clock_valid, wall=None):
        if self.active():
            return True, "REINIT_IN_PROGRESS"

        if not clock_valid:
            return False, "REINIT_REJECTED_CLOCK_INVALID"

        if health.state == OK and self.failure is None:
            return False, "REINIT_REJECTED_NOT_NEEDED"

        problem = actuation_problem(inputs, now, self.config)

        if problem == "ACTUATION_STALE":
            return False, "REINIT_REJECTED_ACTUATION_STALE"

        if problem == "NOT_STOPPED":
            return False, "REINIT_REJECTED_NOT_STOPPED"

        candidate, rejection = reinit_candidate(tracker, inputs.odom_pose, now, map_id, self.config)

        if candidate is None:
            return False, rejection

        self.accepted_count += 1
        self.attempt = ReinitAttempt(accepted_at=now, candidate=candidate, accepted_wall=wall,
                                     number=self.accepted_count)
        self.failure = None

        return True, "REINIT_ACCEPTED"

    # 초기 위치를 AMCL에 보냈음을 기록한다(보낸 시각 이후의 추정만 회복 근거로 쓴다).
    def mark_sent(self, now):
        if self.active():
            self.attempt.sent_at = now

    # 초기 위치 전달 뒤의 AMCL 추정을 기록한다. 실패 뒤의 추정은 회복 근거 판단용으로 따로 쌓는다.
    # 비유한·같은·역행 stamp는 새 추정이 아니므로 회복 근거(갱신 수)에 넣지 않는다.
    def on_amcl(self, stamp, pose, covariance):
        std = covariance_std(covariance)

        if not new_stamp(self.last_amcl, stamp):
            return

        self.last_amcl = stamp

        if not self.active():
            if self.failure is not None and stamp > self.failure_at:
                self.after_failure.append((stamp, *(std if std is not None and finite_pose(pose)
                                                    else (float("inf"), float("inf")))))
            return

        if self.attempt.sent_at is None or stamp <= self.attempt.sent_at:
            return

        if std is None or not finite_pose(pose):
            self.attempt.updates.append((stamp, pose, float("inf"), float("inf")))
        else:
            self.attempt.updates.append((stamp, pose, std[0], std[1]))

    # 무이동 갱신 요청이 필요한 시각인지
    def nomotion_due(self, now):
        if not self.active() or self.attempt.sent_at is None:
            return False

        last = self.attempt.last_nomotion

        if last is None or now - last >= self.config["reinit"]["nomotion_period_s"]:
            self.attempt.last_nomotion = now
            return True

        return False

    # 진행 상태를 갱신한다. 끝나면 결과 코드("SUCCEEDED"·"REINIT_FAILED_*")를, 진행 중이면 None을 돌려준다.
    # clock_valid=false면 sim time 기반 판단을 할 수 없으므로 실패로 끝낸다. wall(단조 벽시계)이 주어지면
    # timeout_wall_s로도 끝내 sim time이 멈춰도 무기한 진행하지 않는다.
    def step(self, inputs, now, clock_valid=True, wall=None):
        if not self.active():
            return None

        r = self.config["reinit"]
        h = self.config["health"]
        attempt = self.attempt

        if not clock_valid:
            return self.finish("REINIT_FAILED_CLOCK_INVALID", now)

        wall_expired = (wall is not None and attempt.accepted_wall is not None
                        and wall - attempt.accepted_wall > r["timeout_wall_s"])

        # 정지 조건 상실
        if actuation_problem(inputs, now, self.config) is not None:
            return self.finish("REINIT_FAILED_NOT_STOPPED", now)

        updates = attempt.updates
        candidate = attempt.candidate

        # 후보에서 크게 벗어나면 회복이 아니라 발산이다.
        if len(updates) >= r["min_updates"]:
            distance, rotation = difference(candidate["pose"], updates[-1][1]) if finite_pose(updates[-1][1]) \
                else (float("inf"), float("inf"))

            if distance > 2 * r["consistency_m"] or rotation > 2 * r["consistency_rad"]:
                return self.finish("REINIT_FAILED_NOT_RECOVERED", now)

            last = updates[-1]
            converged = (last[2] <= r["recover_position_std_m"] and last[3] <= r["recover_yaw_std_rad"]
                         and last[2] <= candidate["position_std_m"] * r["recover_std_ratio"]
                         and not covariance_collapsed((last[2], last[3]), self.config))
            consistent = distance <= r["consistency_m"] and rotation <= r["consistency_rad"]
            tf_fresh = (inputs.tf_valid_time is not None and inputs.tf_valid_time > attempt.sent_at
                        and now - inputs.tf_valid_time <= h["tf_stale_s"] and inputs.tf_chain_ok)
            scan_fresh = inputs.scan_stamp is not None and now - inputs.scan_stamp <= h["scan_stale_s"]
            tf_matches = tf_pose_matches(inputs, candidate["pose"], r["consistency_m"], r["consistency_rad"])

            if converged and consistent and tf_fresh and scan_fresh and tf_matches:
                return self.finish("SUCCEEDED", now)

        # 시간초과(sim time 또는 벽시계): 추정이 전혀 없으면 AMCL 무응답, 있으면 조건 미충족
        # 두 시계 조건을 함께 남긴다(LD-3 근거, 결과 코드만으로는 어느 시계인지 알 수 없다).
        sim_expired = now - attempt.accepted_at > r["timeout_s"]

        if sim_expired or wall_expired:
            attempt.timeout = timeout_detail(now - attempt.accepted_at, r["timeout_s"], sim_expired,
                                             None if wall is None or attempt.accepted_wall is None
                                             else wall - attempt.accepted_wall, r["timeout_wall_s"], wall_expired)

            return self.finish("REINIT_FAILED_TIMEOUT" if updates else "REINIT_FAILED_AMCL_UNAVAILABLE", now)

        return None

    # 결과를 확정한다. 실패면 다음 요청이나 회복 근거가 생기기 전까지 사유를 유지한다.
    def finish(self, result, now):
        self.attempt.result = result
        self.failure = None if result == "SUCCEEDED" else result
        self.failure_at = now
        self.after_failure = []

        return result

    # 실패 뒤 AMCL이 스스로 회복했다는 근거: 실패 뒤 새 추정 min_updates개 이상, 마지막 추정이 회복 기준 이하이고
    # 마지막 후보 표준편차 × recover_std_ratio 이하. 전달한 pose가 같은 공분산으로 돌아온 것은 근거가 아니다.
    def recovered_after_failure(self):
        r = self.config["reinit"]

        if len(self.after_failure) < r["min_updates"]:
            return False

        _, position_std, yaw_std = self.after_failure[-1]
        limit = self.attempt.candidate["position_std_m"] * r["recover_std_ratio"] if self.attempt else float("inf")

        return (position_std <= min(r["recover_position_std_m"], limit) and yaw_std <= r["recover_yaw_std_rad"]
                and not covariance_collapsed((position_std, yaw_std), self.config))

    # 판정 결과에 재위치 진행·실패를 반영한다. 진행·실패는 LOST다.
    # 판정 불가(UNKNOWN·valid=false, 예: CLOCK_INVALID)는 덮지 않고 재위치 사유만 덧붙인다.
    def apply(self, health):
        if not health.valid:
            extra = ["REINIT_IN_PROGRESS"] if self.active() else ([self.failure] if self.failure else [])
            reasons = [c for c in health.reason.split(",") if c] + extra

            return Health(health.state, False, join_reasons(reasons), health.position_std_m, health.yaw_std_rad,
                          health.pose)

        if self.active():
            return Health(LOST, True, join_reasons(["REINIT_IN_PROGRESS"] + [c for c in health.reason.split(",") if c]),
                          health.position_std_m, health.yaw_std_rad, health.pose)

        if self.failure is not None:
            if health.state == OK and self.recovered_after_failure():
                # AMCL이 새 관측으로 스스로 회복한 근거가 있으면 실패 표시를 지운다.
                self.failure = None
                return health

            return Health(LOST, True, join_reasons([self.failure] + [c for c in health.reason.split(",") if c]),
                          health.position_std_m, health.yaw_std_rad, health.pose)

        return health


# 재위치 종료 진단 줄의 내용: 결과 코드, 시도 번호(접수 줄과 대응), 시간초과면 두 시계 조건, 실제 적용 제한값과 설정 출처.
def reinit_finished_entry(reinit, result, config_source):
    r = reinit.config["reinit"]
    attempt = reinit.attempt

    return {"result": result, "attempt": None if attempt is None else attempt.number,
            "timeout": None if attempt is None else attempt.timeout,
            "limits": {"timeout_s": r["timeout_s"], "timeout_wall_s": r["timeout_wall_s"]},
            "config_source": config_source}


# 고정 출발 위치 설정을 읽고 검사한다. 지도 revision·이미지 해시가 실제 지도와 다르면 거절한다.
# 형식:
#   map: {revision: 0, image_sha256: <hex>}     # 채택 지도는 map_info.yaml map_revision, 개발 지도는 0
#   robots: {robot_2: {x, y, yaw, position_std_m, yaw_std_rad}}
def load_start_poses(path, map_yaml, robot_ids):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    expected = data.get("map") or {}
    robots = data.get("robots") or {}

    if "revision" not in expected or "image_sha256" not in expected:
        raise ValueError("start_poses: map.revision·map.image_sha256 필요")

    actual_revision, actual_sha = map_identity(map_yaml)

    if int(expected["revision"]) != actual_revision:
        raise ValueError(f"start_poses: 지도 revision 불일치(설정 {expected['revision']}, 지도 {actual_revision})")

    expected_sha = str(expected["image_sha256"])

    if expected_sha != actual_sha:
        raise ValueError(f"start_poses: 지도 이미지 해시 불일치(설정 {expected_sha[:12]}…, 지도 {actual_sha[:12]}…)")

    poses = {}

    for robot_id in robot_ids:
        if robot_id not in robots:
            raise ValueError(f"start_poses: {robot_id} 출발 위치 없음")

        entry = robots[robot_id] or {}
        keys = ("x", "y", "yaw", "position_std_m", "yaw_std_rad")
        missing = [key for key in keys if key not in entry]

        if missing:
            raise ValueError(f"start_poses: {robot_id}에 {', '.join(missing)} 없음")

        values = {}

        for key in keys:
            value = entry[key]

            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"start_poses: {robot_id}.{key}는 유한한 수")

            values[key] = float(value)

        if not (0.0 < values["position_std_m"] <= 1.0 and 0.0 < values["yaw_std_rad"] <= math.pi):
            raise ValueError(f"start_poses: {robot_id} 표준편차 범위 밖(위치 0~1 m, 방향 0~π rad)")

        poses[robot_id] = values

    return {"map_revision": actual_revision, "image_sha256": actual_sha, "robots": poses}


# 지도 yaml의 (revision, 이미지 sha256). revision은 같은 폴더 map_info.yaml의 map_revision, 없으면 0(개발 지도).
def map_identity(map_yaml):
    map_yaml = Path(map_yaml)
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image = map_yaml.parent / meta["image"]
    sha = hashlib.sha256(image.read_bytes()).hexdigest()
    info = map_yaml.parent / "map_info.yaml"
    revision = 0

    if map_yaml.name == "map.yaml" and info.is_file():
        revision = int((yaml.safe_load(info.read_text(encoding="utf-8")) or {}).get("map_revision", 0))

    return revision, sha


# 6×6 공분산 대각을 표준편차로 채운 목록(초기 위치 전달용)
def pose_covariance(position_std_m, yaw_std_rad):
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = position_std_m ** 2
    covariance[35] = yaw_std_rad ** 2

    return covariance
