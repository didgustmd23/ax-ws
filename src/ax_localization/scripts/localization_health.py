"""B2 위치 신뢰성·고정 출발 위치·마지막 신뢰 위치·제한된 재위치 판단(ROS 없는 순수 계산).

ROS 연결은 localization_supervisor.py가 하고, 이 모듈은 입력 기록을 받아 판단만 한다(단위 시험 대상).
모든 임계값은 개발용 출발값이며(config/localization_health.yaml) 정확도 기준이 아니다.
상태·사유 코드는 공통 인터페이스 계약 4.2절·D-19 "위치"를 따른다.
"""

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
    "ESTIMATE_INVALID", "ODOM_INVALID", "ODOM_DISCONTINUOUS", "TF_MISSING", "INPUT_LOST", "COVARIANCE_LOST",
    "TF_STALE", "SCAN_STALE", "ODOM_STALE", "ESTIMATE_STALE", "COVARIANCE_HIGH",
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
        "clock_stall_wall_s": 2.0,
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
def covariance_std(covariance):
    if len(covariance) != 36:
        return None

    xx, yy, yaw = covariance[0], covariance[7], covariance[35]

    if not all(math.isfinite(value) for value in (xx, yy, yaw)) or min(xx, yy, yaw) < 0:
        return None

    return math.sqrt(max(xx, yy)), math.sqrt(yaw)


# 자세 값이 모두 유한한지 확인한다.
def finite_pose(pose):
    return pose is not None and all(math.isfinite(value) for value in pose)


# 사유 목록을 심각한 순으로 정렬해 문자열로 만든다.
def join_reasons(reasons):
    unique = list(dict.fromkeys(reasons))

    return ",".join(sorted(unique, key=lambda code: REASON_ORDER.index(code) if code in REASON_ORDER else len(REASON_ORDER)))


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


# 입력 기록으로 위치 신뢰성 상태를 판정한다(재위치 진행·실패 표시는 호출자가 더한다).
def evaluate(inputs, now, config, clock_valid=True):
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

    # 공분산(정확도 확정이 아니라 추정 불확실성의 상한 검사)
    if std is not None:
        position_std, yaw_std = std

        if position_std > h["position_std_lost_m"] or yaw_std > h["yaw_std_lost_rad"]:
            lost.append("COVARIANCE_LOST")
        elif position_std > h["position_std_ok_m"] or yaw_std > h["yaw_std_ok_rad"]:
            degraded.append("COVARIANCE_HIGH")
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

    # 끊긴 뒤 받은 AMCL 추정을 센다(끊긴 시각보다 늦은 추정만).
    def note_amcl(self, stamp):
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
    def rejection(self, health, inputs, map_id, reinit_active):
        t = self.config["trust"]

        if reinit_active:
            return "REINIT_ACTIVE"

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
    def consider(self, health, inputs, now, map_id, reinit_active):
        if self.rejection(health, inputs, map_id, reinit_active) is not None:
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
@dataclass
class ReinitAttempt:
    accepted_at: float
    candidate: dict
    accepted_wall: float = None                    # 접수 시각(단조 벽시계). sim time이 멈춰도 끝나게 한다
    sent_at: float = None
    updates: list = field(default_factory=list)   # (stamp, pose, position_std, yaw_std)
    last_nomotion: float = None
    result: str = None                             # None(진행), "SUCCEEDED", "REINIT_FAILED_*"


# 재위치 절차. 요청 접수·진행·완료·실패를 관리하고 결과 사유를 판정에 더한다.
class Reinitializer:
    # config: 전체 설정
    def __init__(self, config):
        self.config = config
        self.attempt = None
        self.failure = None          # 마지막 실패 사유(다음 요청이나 회복 근거가 생기기 전까지 유지)
        self.failure_at = None       # 실패 확정 시각
        self.after_failure = []      # 실패 뒤 받은 AMCL 추정 (stamp, position_std, yaw_std)

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

        self.attempt = ReinitAttempt(accepted_at=now, candidate=candidate, accepted_wall=wall)
        self.failure = None

        return True, "REINIT_ACCEPTED"

    # 초기 위치를 AMCL에 보냈음을 기록한다(보낸 시각 이후의 추정만 회복 근거로 쓴다).
    def mark_sent(self, now):
        if self.active():
            self.attempt.sent_at = now

    # 초기 위치 전달 뒤의 AMCL 추정을 기록한다. 실패 뒤의 추정은 회복 근거 판단용으로 따로 쌓는다.
    def on_amcl(self, stamp, pose, covariance):
        std = covariance_std(covariance)

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
                         and last[2] <= candidate["position_std_m"] * r["recover_std_ratio"])
            consistent = distance <= r["consistency_m"] and rotation <= r["consistency_rad"]
            tf_fresh = (inputs.tf_valid_time is not None and inputs.tf_valid_time > attempt.sent_at
                        and now - inputs.tf_valid_time <= h["tf_stale_s"] and inputs.tf_chain_ok)
            scan_fresh = inputs.scan_stamp is not None and now - inputs.scan_stamp <= h["scan_stale_s"]

            if converged and consistent and tf_fresh and scan_fresh:
                return self.finish("SUCCEEDED", now)

        # 시간초과(sim time 또는 벽시계): 추정이 전혀 없으면 AMCL 무응답, 있으면 조건 미충족
        if now - attempt.accepted_at > r["timeout_s"] or wall_expired:
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

        return position_std <= min(r["recover_position_std_m"], limit) and yaw_std <= r["recover_yaw_std_rad"]

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
