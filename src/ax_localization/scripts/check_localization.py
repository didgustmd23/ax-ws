#!/usr/bin/env python3
"""B2 AMCL 위치 추정 개발 검증 기록·평가. 실행 중인 시뮬레이션·B1·localization.launch.py에 붙어 기록만 한다.

  python3 src/ax_localization/scripts/check_localization.py --robot-ids robot_1,robot_2 \
      --duration 60 --world-map 2.4927,-1.4957,-0.0003 --output <결과 폴더>

기록(로봇별): map→<robot>/odom TF(stamp, 위치), odom, amcl_pose, initialpose 입력, 스캔 수신 시각,
Gazebo 실제 자세(검증 전용, 위치 추정 입력으로 쓰지 않음), /tf 발행 노드 목록, 끝날 때 TF 연결 조회 결과.

평가: 같은 시각의 AMCL 추정 차체 자세(map→odom · odom→base_link)와 실제 자세(T_world_map⁻¹ · world 자세)의
위치·방향 오차. T_world_map은 지도 검증 기록(accuracy.json)의 값을 쓰는 검증 전용 변환이다.
AMCL TF stamp는 처리한 스캔 시각 + transform_tolerance라서 그만큼 빼서 유효 시각으로 쓴다.
정확도 기준이 없으므로 judgment는 not_judged다.

status(검증 가능 여부)와 judgment(정확도 판정)는 따로 본다.
- completed: 요청한 기록 시간을 채웠고, 요청한 모든 로봇이 검증 가능하다(아래 필수 입력·연결이 모두 있음).
- incomplete: 기록 시간 부족 또는 한 로봇이라도 필수 입력·연결이 없다. 로봇별 사유를 problems에 남기고 종료 코드 1.
  확보한 기록·계산 결과는 그대로 남긴다.
- 로봇별 필수: 스캔·odom·map→odom 수신, Gazebo 실제 자세 표본, 오차 계산 표본, map→odom→base_link TF 연결.

원시 기록(--raw-dir, 선택): 받은 입력을 그때그때 JSON 줄로 덧붙이고 줄마다 flush한다(중단돼도 받은 만큼 남는다).
- <raw-dir>/conditions.json: 수집 시작에 저장(git·해시·환경·옵션). 실행 중 알게 된 값(URDF 해시·감독 세션)은 갱신한다.
- <raw-dir>/raw/<robot_id>/<stream>.jsonl: tf_map_odom·amcl_pose·initialpose·odom·scan·health·actuation·
  amcl_lifecycle·tf_chain·publishers. raw/common/에는 /tf 발행자 등 로봇 공통 기록.
- 모든 줄에 원래 stamp(있으면)와 수신 시각 recv_sim(sim time)·recv_mono(단조 벽시계)를 따로 둔다.
  벽시계 UTC는 conditions.json의 time_reference(같은 순간의 time.time()·monotonic)로 환산한다.
- Gazebo 실제 자세(truth.jsonl)는 평가 전용이며 판정 입력으로 쓰지 않는다.
- 감독 노드 진단(판정 tick·신뢰 기록 판단)은 localization.launch.py health_diagnostics:=<raw-dir>/supervisor로 받는다.
분석(--analyze <raw-dir>): ROS 없이 원시 기록에서 map→odom 갱신량·TF/amcl_pose 대응·health 사유별 시간·
신뢰 기록 판단 빈도·오차와 공분산을 계산해 <raw-dir>/analysis.json에 쓴다. 정확도 판정은 not_judged.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from map_tools import interpolate_pose, se2, se2_params, wrap_angle, yaw_from_quaternion  # noqa: E402

# 평가 표본 간격(시뮬레이션 s)과 실제 자세 보간 허용 간격
SAMPLE_PERIOD_S = 0.5
TRUTH_MAX_GAP_S = 0.1


# 표본 배열의 평균·95백분위수·최대(절댓값). 비어 있으면 None
def stats(values):
    values = np.abs(np.asarray(values, float))

    if len(values) == 0:
        return None

    return {"mean": float(values.mean()), "p95": float(np.percentile(values, 95)), "max": float(values.max())}


# 한 로봇의 기록으로 AMCL 추정 자세와 실제 자세의 오차 시계열·요약을 만든다.
# map_odom: (유효 시각, x, y, yaw), odom: (시각, x, y, yaw, v, w), truth: (시각, x, y, yaw) world 기준.
def pose_errors(map_odom, odom, truth, world_map, period_s=SAMPLE_PERIOD_S, max_gap=TRUTH_MAX_GAP_S):
    map_odom = np.asarray(map_odom, float).reshape(-1, 4)
    odom = np.asarray(odom, float).reshape(-1, 6)
    truth = np.asarray(truth, float).reshape(-1, 4)
    map_world = np.linalg.inv(world_map)
    rows = []
    last = -math.inf

    for sample in odom:
        if sample[0] - last < period_s:
            continue

        earlier = map_odom[map_odom[:, 0] <= sample[0]]
        actual = interpolate_pose(truth[:, 0], truth[:, 1:4], sample[0], max_gap) if len(truth) else None

        if len(earlier) == 0 or actual is None:
            continue

        last = sample[0]
        estimate = se2(*earlier[-1][1:4]) @ se2(*sample[1:4])
        expected = map_world @ se2(*actual)
        error = se2_params(np.linalg.inv(expected) @ estimate)
        pose = se2_params(estimate)
        rows.append([sample[0], *pose, *se2_params(expected), math.hypot(*error[:2]), error[2],
                     math.hypot(sample[4], 0.0), abs(sample[5])])

    if not rows:
        return None

    rows = np.array(rows)

    return {
        "samples": int(len(rows)),
        "position_error_m": stats(rows[:, 7]),
        "yaw_error_rad": stats(rows[:, 8]),
        "final": {"sim_time_s": float(rows[-1, 0]), "position_error_m": float(rows[-1, 7]),
                  "yaw_error_rad": float(rows[-1, 8])},
        "series": {"columns": ["sim_time_s", "est_x", "est_y", "est_yaw", "true_x", "true_y", "true_yaw",
                               "position_error_m", "yaw_error_rad", "odom_speed_m_s", "odom_yaw_rate_rad_s"],
                   "rows": np.round(rows, 4).tolist()},
    }


# 시각 구간 안에서 map→odom 변환이 얼마나 바뀌었는지(위치·방향 최대 변화)를 구한다.
def map_odom_change(map_odom, start, end):
    map_odom = np.asarray(map_odom, float).reshape(-1, 4)
    before = map_odom[map_odom[:, 0] <= start]
    window = map_odom[(map_odom[:, 0] > start) & (map_odom[:, 0] <= end)]

    if len(before) == 0 or len(window) == 0:
        return None

    base = before[-1]

    return {
        "samples": int(len(window)),
        "max_position_change_m": float(max(math.hypot(row[1] - base[1], row[2] - base[2]) for row in window)),
        "max_yaw_change_rad": float(max(abs(wrap_angle(row[3] - base[3])) for row in window)),
    }


# 초기 위치 입력이 다른 로봇에 영향을 주는지 판단할 자료를 만든다.
# 각 initialpose 수신 시각 앞뒤로 모든 로봇의 map→odom 변화와 amcl_pose 수신 수를 센다.
def initial_pose_effects(records, window_s=5.0):
    effects = []

    for source, record in records.items():
        for event in record["initialpose"]:
            stamp = event[0]
            entry = {"input_robot": source, "sim_time_s": stamp, "pose": event[1:], "robots": {}}

            for robot_id, other in records.items():
                poses = [row for row in other["amcl_pose"] if stamp < row[0] <= stamp + window_s]
                entry["robots"][robot_id] = {
                    "map_odom_change": map_odom_change(other["map_odom"], stamp, stamp + window_s),
                    "amcl_pose_messages": len(poses),
                    "last_amcl_pose": poses[-1][1:4] if poses else None,
                }

            effects.append(entry)

    return effects


# 메시지 수신 시각 목록으로 수신 주기 요약을 만든다.
def rate_summary(stamps):
    stamps = np.asarray(stamps, float)

    if len(stamps) < 2:
        return {"messages": int(len(stamps)), "rate_hz": None, "max_gap_s": None}

    gaps = np.diff(stamps)

    return {"messages": int(len(stamps)), "rate_hz": float((len(stamps) - 1) / (stamps[-1] - stamps[0])),
            "max_gap_s": float(gaps.max())}


# health·A4 상태 구독 깊이. 발행 쪽 STATE QoS(TRANSIENT_LOCAL·RELIABLE, 깊이 1)와 호환되며, 콜백이 밀릴 때
# 이 개수까지 쌓아 두고 넘으면 오래된 것부터 버린다(깊이 1이던 2026-11-09 회차에서 손실 확인).
HEALTH_STREAM_DEPTH = 100

# 원시 기록에서 로봇마다 있어야 할 자료. 없으면 missing으로 남긴다(판정은 바꾸지 않는다).
EXPECTED_STREAMS = ("tf_map_odom", "amcl_pose", "odom", "scan", "health", "tf_chain", "publishers")


# JSON에 쓸 수 있게 비유한 실수를 None으로 바꾼다(NaN·inf를 문자열로 쓰지 않는다).
def json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None

    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}

    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]

    if isinstance(value, np.generic):
        return json_safe(value.item())

    return value


# 6×6 공분산(36개)을 실수 목록으로 바꾼다. 길이가 다르면 받은 그대로 남기고 표시한다.
def covariance_list(covariance):
    values = [float(value) for value in covariance]

    return {"values": values, "complete": len(values) == 36}


# 원시 기록 파일. 로봇·자료별 JSON 줄을 덧붙이고 줄마다 flush해 중단돼도 받은 만큼 남긴다.
class RawLog:
    # root: 회차 폴더. 파일은 쓸 때 처음 연다.
    def __init__(self, root):
        self.root = Path(root)
        self.lock = threading.Lock()
        self.files = {}
        self.counts = {}
        self.closed = False

    # 한 줄 기록. robot_id가 None이면 raw/common/에 쓴다.
    def write(self, robot_id, stream, entry):
        key = (robot_id or "common", stream)
        line = json.dumps(json_safe(entry), ensure_ascii=False, allow_nan=False)

        with self.lock:
            if self.closed:
                return

            handle = self.files.get(key)

            if handle is None:
                path = self.root / "raw" / key[0] / f"{stream}.jsonl"
                path.parent.mkdir(parents=True, exist_ok=True)
                handle = self.files[key] = path.open("a", encoding="utf-8")

            handle.write(line + "\n")
            handle.flush()
            self.counts[key] = self.counts.get(key, 0) + 1

    # 모든 파일을 닫는다. 이후 기록은 버린다.
    def close(self):
        with self.lock:
            self.closed = True

            for handle in self.files.values():
                handle.close()

            self.files.clear()


# 파일을 임시 이름으로 쓴 뒤 바꿔 넣는다(쓰는 중 중단돼도 이전 내용이 남는다).
def write_json_atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(json_safe(data), ensure_ascii=False, indent=1, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


# JSON 줄 파일을 읽는다. 마지막 줄이 잘렸거나 깨진 줄은 건너뛰고 개수를 돌려준다.
def read_jsonl(path):
    rows, broken = [], 0
    path = Path(path)

    if not path.is_file():
        return rows, broken

    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue

        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            broken += 1

    return rows, broken


# 회차 폴더의 원시 기록 목록(로봇·자료별 줄 수·깨진 줄)과 로봇별 빠진 자료
def raw_inventory(root, robot_ids, expected=EXPECTED_STREAMS):
    root = Path(root)
    files = {}

    for path in sorted((root / "raw").glob("*/*.jsonl")):
        rows, broken = read_jsonl(path)
        files[f"{path.parent.name}/{path.name}"] = {"lines": len(rows), "broken_lines": broken}

    missing = {}

    for robot_id in robot_ids:
        absent = [stream for stream in expected if not files.get(f"{robot_id}/{stream}.jsonl", {}).get("lines")]

        if absent:
            missing[robot_id] = absent

    return {"files": files, "missing": missing}


# 파일 sha256(없으면 None)
def file_sha256(path):
    path = Path(path)

    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


# 지도 yaml과 그 이미지의 sha256
def map_hashes(map_yaml):
    import yaml

    map_yaml = Path(map_yaml)

    if not map_yaml.is_file():
        return {"yaml": None, "image": None, "image_path": None}

    image = map_yaml.parent / (yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}).get("image", "")

    return {"yaml": file_sha256(map_yaml), "image": file_sha256(image), "image_path": str(image)}


# git commit·미커밋 파일 목록(실패하면 오류 문자열)
def git_state(workspace):
    def run(*command):
        result = subprocess.run(["git", "-C", str(workspace), *command], capture_output=True, text=True)
        return result.stdout if result.returncode == 0 else None

    commit = run("rev-parse", "HEAD")
    status = run("status", "--porcelain", "--untracked-files=no")

    return {"commit": commit.strip() if commit else None,
            "dirty_files": status.splitlines() if status is not None else None}


# 수집 시작 조건. files: {이름: 경로}, options: 수집 옵션. 같은 순간의 벽시계·단조 시각을 함께 남긴다.
def collect_conditions(workspace, files, map_yaml, robot_ids, options, environment=None):
    environment = os.environ if environment is None else environment

    return {
        "schema": "ax_b2_health_raw/1",
        "time_reference": {"wall_utc_epoch_s": time.time(), "monotonic_s": time.monotonic(),
                           "wall_source": "time.time() UTC epoch, 같은 순간의 time.monotonic()과 짝"},
        "git": git_state(workspace),
        "files_sha256": {name: {"path": str(path), "sha256": file_sha256(path)} for name, path in files.items()},
        "map": {"path": str(map_yaml), **map_hashes(map_yaml)} if map_yaml else None,
        "robot_ids": list(robot_ids),
        "environment": {key: environment.get(key) for key in ("ROS_DOMAIN_ID", "GZ_PARTITION",
                                                               "ROS_AUTOMATIC_DISCOVERY_RANGE", "RMW_IMPLEMENTATION")},
        "options": options,
        "runtime": {},
    }


# 값 목록의 개수·평균·백분위수(p95·p99)·최대(절댓값). 비어 있으면 samples 0
def distribution(values):
    values = np.abs(np.asarray([value for value in values if value is not None], float))

    if len(values) == 0:
        return {"samples": 0}

    return {"samples": int(len(values)), "mean": float(values.mean()), "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)), "max": float(values.max())}


# SE(2) 상대 변환 a⁻¹·b의 (거리, |yaw|)
def relative_change(a, b):
    params = se2_params(np.linalg.inv(se2(*a)) @ se2(*b))

    return math.hypot(params[0], params[1]), abs(params[2])


# 두 TF 기록의 값(x, y, yaw)이 같은지
def same_value(a, b, tolerance=1e-9):
    distance, rotation = relative_change((a["x"], a["y"], a["yaw"]), (b["x"], b["y"], b["yaw"]))

    return distance <= tolerance and rotation <= tolerance


# map→odom TF 기록(수신 순서)을 정리한다. 원래 stamp가 이전보다 크면 새 값, 작으면 역행으로 센다.
# 같은 stamp는 값이 같으면 중복(duplicate_stamp), 다르면 충돌(conflicting_stamp)로 따로 세고 둘 다 비교에서 뺀다.
# 새 값끼리 연속 쌍마다 map→odom 상대 변화와, 뒤 TF 유효 시각 직전 odom에서 본 추정 차체 자세 차이를 구한다.
# 그 odom의 나이(유효 시각 − odom stamp)를 남기고 max_odom_age_s를 넘거나 odom이 없으면 대응 불가로 센다.
# 값이 바뀌지 않은 새 stamp(같은 변환 재발행 추정)와 바뀐 갱신을 나눈다. 원인은 확정하지 않는다.
def map_odom_steps(tf_rows, odom_rows, same_tolerance=1e-9, max_odom_age_s=0.1):
    accepted, duplicates, conflicts, reversed_count = [], 0, [], 0
    by_stamp = {}

    for row in tf_rows:
        if accepted and row["stamp"] < accepted[-1]["stamp"]:
            reversed_count += 1
        elif row["stamp"] in by_stamp:
            if same_value(by_stamp[row["stamp"]], row, same_tolerance):
                duplicates += 1
            else:
                first = by_stamp[row["stamp"]]
                conflicts.append({"stamp": row["stamp"], "first": [first["x"], first["y"], first["yaw"]],
                                  "other": [row["x"], row["y"], row["yaw"]], "recv_sim": row.get("recv_sim")})
        else:
            by_stamp[row["stamp"]] = row
            accepted.append(row)

    odom = sorted((row["stamp"], row["x"], row["y"], row["yaw"]) for row in odom_rows)
    odom_times = [row[0] for row in odom]
    changed, repeated, no_odom = [], 0, 0

    for earlier, later in zip(accepted, accepted[1:]):
        a = (earlier["x"], earlier["y"], earlier["yaw"])
        b = (later["x"], later["y"], later["yaw"])
        distance, rotation = relative_change(a, b)

        if distance <= same_tolerance and rotation <= same_tolerance:
            repeated += 1
            continue

        index = int(np.searchsorted(odom_times, later["valid_time"], side="right")) - 1
        estimate_distance = estimate_rotation = odom_age = None

        if index >= 0:
            odom_age = later["valid_time"] - odom[index][0]

        if odom_age is not None and odom_age <= max_odom_age_s:
            current = odom[index][1:]
            before = se2_params(se2(*a) @ se2(*current))
            after = se2_params(se2(*b) @ se2(*current))
            estimate_distance, estimate_rotation = relative_change(before, after)
        else:
            no_odom += 1

        changed.append({"stamp": later["stamp"], "dt_s": later["stamp"] - earlier["stamp"],
                        "map_odom_m": distance, "map_odom_rad": rotation, "odom_age_s": odom_age,
                        "estimate_m": estimate_distance, "estimate_rad": estimate_rotation})

    return {"received": len(tf_rows), "accepted": len(accepted), "duplicate_stamp": duplicates,
            "conflicting_stamp": len(conflicts), "conflicts": conflicts, "reversed_stamp": reversed_count,
            "repeated_value": repeated, "max_odom_age_s": max_odom_age_s, "estimate_unmatched_odom": no_odom,
            "changed": changed}


# TF와 amcl_pose 대응: TF 유효 시각(stamp − tf_future)과 amcl_pose stamp가 tolerance 안이면 짝으로 본다.
# 일대일 대응을 가정하지 않고 대응 수·무대응 수·시각 차이만 남긴다(무대응 원인은 미확정).
def pair_tf_amcl(tf_rows, amcl_rows, tolerance=0.001):
    tf_valid = sorted({row["valid_time"] for row in tf_rows})
    amcl = sorted({row["stamp"] for row in amcl_rows})
    differences, matched_tf = [], set()

    for stamp in amcl:
        index = int(np.searchsorted(tf_valid, stamp))
        nearest = [tf_valid[i] for i in (index - 1, index) if 0 <= i < len(tf_valid)]

        if not nearest:
            continue

        best = min(nearest, key=lambda value: abs(value - stamp))
        differences.append(best - stamp)

        if abs(best - stamp) <= tolerance:
            matched_tf.add(best)

    matched_amcl = sum(1 for value in differences if abs(value) <= tolerance)

    return {"method": f"amcl_pose stamp와 가장 가까운 TF 유효 시각(stamp − tf_future), |차이| ≤ {tolerance} s",
            "tf_unique_stamps": len(tf_valid), "amcl_unique_stamps": len(amcl),
            "matched_amcl": matched_amcl, "unmatched_amcl": len(amcl) - matched_amcl,
            "unmatched_tf": len(tf_valid) - len(matched_tf),
            "nearest_difference_s": distribution(differences), "cause_of_unmatched": "미확정"}


# health 표본(sim stamp 순)으로 상태·사유별 시간(s)을 센다. 다음 표본까지를 그 상태의 시간으로 보며
# max_gap_s를 넘는 간격은 빼고 따로 센다.
def health_time_shares(rows, max_gap_s=1.0):
    rows = sorted(rows, key=lambda row: row["stamp"])
    states, reasons, gaps = {}, {}, []

    for current, following in zip(rows, rows[1:]):
        dt = following["stamp"] - current["stamp"]

        if dt <= 0:
            continue

        if dt > max_gap_s:
            gaps.append(dt)
            continue

        states[current["state"]] = states.get(current["state"], 0.0) + dt

        for code in (current["reason"] or "-").split(","):
            reasons[code] = reasons.get(code, 0.0) + dt

    total = sum(states.values())

    return {"samples": len(rows), "counted_s": total, "excluded_gaps_s": gaps,
            "state_s": states, "state_share": {key: value / total for key, value in states.items()} if total else {},
            "reason_s": reasons}


# 손실 판정 값의 뜻(기록 품질 필드에 함께 남긴다)
LOSS_MEANING = ("confirmed: 대조 기준의 사건 중 원시 기록에 없는 것이 있다(전송·DDS 원인을 확정한 것이 아니다). "
                "none_in_reference: 비교한 범위의 기준 사건이 모두 있다(기준 밖 메시지의 수신은 보장하지 않는다). "
                "unverified: 대조 기준이 없다")


# 원시 기록 품질(완료 여부 status와 별개). 받은 줄 수·고유 stamp 수·중복·수신 최대 공백(단조 벽시계)을 남긴다.
# reference가 주어지면 [(stamp, mono)] 기준 사건을 stamp 집합으로 대조한다(중복이 누락을 상쇄하지 않는다).
# window(수집 시작·끝 mono)가 있으면 기준 사건을 구간 안·밖으로 나누고 대조는 구간 안 사건으로만 한다.
def record_quality(rows, reference=None, source="", reference_source=None, window=None, round_digits=6):
    monotonic = [row["recv_mono"] for row in rows if row.get("recv_mono") is not None]
    gaps = [later - earlier for earlier, later in zip(monotonic, monotonic[1:])]
    stamps = [round(row["stamp"], round_digits) for row in rows if row.get("stamp") is not None]
    quality = {"source": source, "observed": len(rows), "observed_unique_stamps": len(set(stamps)),
               "observed_duplicate_stamps": len(stamps) - len(set(stamps)),
               "observed_mono": [min(monotonic), max(monotonic)] if monotonic else None,
               "max_recv_gap_mono_s": max(gaps) if gaps else None,
               "subscriber_ready": "unknown", "loss_meaning": LOSS_MEANING}

    if reference is None:
        quality.update(reference=None, loss="unverified")
        return quality

    observed = set(stamps)

    # 대조 한 번: 주어진 기준 사건 집합에서 원시 기록에 없는 수
    def compare(events):
        wanted = {round(stamp, round_digits) for stamp, _ in events}
        missing = wanted - observed

        return {"reference_unique": len(wanted), "matched": len(wanted & observed), "missing_in_record": len(missing),
                "loss": "confirmed" if missing else "none_in_reference"}

    inside = [event for event in reference if window is None or window[0] <= event[1] <= window[1]]
    outside = len(reference) - len(inside)
    quality.update(reference=reference_source, reference_events=len(reference),
                   reference_in_window=len(inside), reference_out_of_window=outside,
                   collection_window=compare(inside),
                   extra_in_record=len(observed - {round(stamp, round_digits) for stamp, _ in reference}))
    quality["loss"] = quality["collection_window"]["loss"]

    if monotonic:
        span = [event for event in inside if monotonic[0] <= event[1] <= monotonic[-1]]
        quality["observed_interval"] = {"mono": [monotonic[0], monotonic[-1]], **compare(span),
                                        "note": "원시 기록이 처음·마지막으로 받은 사이의 기준 사건만 대조"}

    return quality


# 회차 폴더의 로봇별 기록 품질(status와 별개).
# - 수집 구간: 시작은 conditions.json time_reference.monotonic_s(기록 시작), 끝은 end_mono(주어지면) 또는 원시 기록의
#   마지막 수신 시각. 구독 준비 시각을 증명할 자료가 없어 subscriber_ready는 unknown이다.
# - health: 감독 진단 tick(sim, mono)과 stamp로 대조한다. 진단 전체 범위(all_ticks)와 수집 구간·관측 구간을 따로 낸다.
# - initialpose: 감독 노드가 stamp 0으로 보내 개수만 비교한다(coverage count_only). 개수가 같아도 완전 수신이 아니다.
# - 나머지 자료: 기준이 없어 unverified.
def session_record_quality(root, robot_ids, streams=EXPECTED_STREAMS + ("actuation", "initialpose", "truth"),
                           end_mono=None):
    root = Path(root)
    conditions_path = root / "conditions.json"
    start_mono = None

    if conditions_path.is_file():
        start_mono = json.loads(conditions_path.read_text(encoding="utf-8")).get("time_reference", {}).get("monotonic_s")

    all_rows = {robot_id: {stream: read_jsonl(root / "raw" / robot_id / f"{stream}.jsonl")[0] for stream in streams}
                for robot_id in robot_ids}
    last_seen = [row["recv_mono"] for streams_rows in all_rows.values() for rows in streams_rows.values()
                 for row in rows if row.get("recv_mono") is not None]
    end = end_mono if end_mono is not None else (max(last_seen) if last_seen else None)
    window = [start_mono, end] if start_mono is not None and end is not None else None
    result = {"collection_window_mono": {"start": start_mono, "end": end,
                                         "start_source": "conditions.json time_reference.monotonic_s(기록 시작)",
                                         "end_source": "기록 종료 시각" if end_mono is not None
                                         else "원시 기록 마지막 수신 시각"},
              "robots": {}}

    for robot_id in robot_ids:
        diagnostics = root / "supervisor" / f"{robot_id}_supervisor.jsonl"
        diagnostic_rows = read_jsonl(diagnostics)[0] if diagnostics.is_file() else None
        entry = {}

        for stream in streams:
            rows = all_rows[robot_id][stream]
            source = f"raw/{robot_id}/{stream}.jsonl"

            if stream == "health" and diagnostic_rows:
                ticks = [(row["sim"], row.get("mono")) for row in diagnostic_rows if row.get("kind") == "tick"]
                entry[stream] = record_quality(rows, ticks, source, f"supervisor/{robot_id}_supervisor.jsonl tick", window)
                entry[stream]["all_ticks"] = record_quality(rows, ticks, source, "진단 전체(기간 제한 없음)")[
                    "collection_window"]
            elif stream == "initialpose" and diagnostic_rows is not None:
                sent = [row for row in diagnostic_rows if row.get("kind") == "initialpose_sent"]
                inside = [row for row in sent if window is None or window[0] <= row.get("mono", -math.inf) <= window[1]]
                entry[stream] = record_quality(rows, None, source)
                entry[stream].update(
                    reference=f"supervisor/{robot_id}_supervisor.jsonl initialpose_sent", coverage="count_only",
                    reference_events=len(sent), reference_in_window=len(inside),
                    reference_out_of_window=len(sent) - len(inside),
                    unmatched_count=max(0, len(inside) - len(rows)),
                    loss="confirmed" if len(rows) < len(inside) else "count_equal_unproven",
                    note="개수만 비교한다. 같아도 같은 메시지를 받았다는 증명이 아니며 다른 발행자의 initialpose가 섞일 수 있다")
            else:
                entry[stream] = record_quality(rows, None, source)

        result["robots"][robot_id] = entry

    return result


# 감독 노드 진단(tick)에서 신뢰 기록 판단 빈도와 기록 나이·누적 이동, 재위치 요청 결과를 센다.
def trust_summary(rows):
    ticks = [row for row in rows if row.get("kind") == "tick"]
    rejections = {}

    for row in ticks:
        key = "STORED" if row.get("trust_stored") else (row.get("trust_rejection") or "UNKNOWN")
        rejections[key] = rejections.get(key, 0) + 1

    requests = [{key: row.get(key) for key in ("sim", "code", "accepted", "trust_record_age_s", "trust_path_m",
                                               "trust_path_rad", "state", "reason")}
                for row in rows if row.get("kind") == "reinit_request"]
    sent = [{key: row.get(key) for key in ("sim", "source", "pose", "sent_position_std_m", "sent_yaw_std_rad")}
            for row in rows if row.get("kind") == "initialpose_sent"]

    return {"ticks": len(ticks), "decisions": rejections,
            "record_age_s": distribution([row.get("trust_record_age_s") for row in ticks]),
            "record_path_m": distribution([row.get("trust_path_m") for row in ticks]),
            "invalid_reasons": sorted({row["trust_invalid_reason"] for row in ticks if row.get("trust_invalid_reason")}),
            "sessions": sorted({row["session"] for row in rows if row.get("session")}),
            "reinit_requests": requests, "initialpose_sent": sent}


# 공분산에서 위치(max(xx, yy))·yaw 표준편차. 불완전하거나 음수면 None
def covariance_std(values):
    if len(values) != 36:
        return None, None

    xx, yy, yaw = values[0], values[7], values[35]

    if min(xx, yy, yaw) < 0:
        return None, None

    return math.sqrt(max(xx, yy)), math.sqrt(yaw)


# 오차 표본(pose_errors series)마다 그 시각 이전 가장 가까운 amcl_pose 공분산을 묶는다(max_age_s 안).
# 못 묶은 표본 수와 시간 차이를 남긴다. 정확도는 판정하지 않는다.
def error_with_covariance(series_rows, amcl_rows, max_age_s=1.0):
    amcl = sorted(amcl_rows, key=lambda row: row["stamp"])
    stamps = [row["stamp"] for row in amcl]
    joined, unmatched, ages = [], 0, []

    for row in series_rows:
        index = int(np.searchsorted(stamps, row[0], side="right")) - 1

        if index < 0 or row[0] - stamps[index] > max_age_s:
            unmatched += 1
            continue

        position_std, yaw_std = covariance_std(amcl[index]["covariance"]["values"])
        ages.append(row[0] - stamps[index])
        joined.append({"sim": row[0], "position_error_m": row[7], "yaw_error_rad": abs(row[8]),
                       "position_std_m": position_std, "yaw_std_rad": yaw_std, "covariance_age_s": row[0] - stamps[index]})

    return {"method": f"오차 표본 시각 이전의 마지막 amcl_pose(≤ {max_age_s} s)", "joined": len(joined),
            "unmatched": unmatched, "covariance_age_s": distribution(ages), "rows": joined}


# 회차 폴더를 읽어 로봇별 분석 결과를 만든다(ROS 없음).
def analyze_session(root, world_map=None, tf_future_s=1.0):
    root = Path(root)
    conditions = json.loads((root / "conditions.json").read_text(encoding="utf-8"))
    robots = {}

    for robot_id in conditions["robot_ids"]:
        def load(stream):
            return read_jsonl(root / "raw" / robot_id / f"{stream}.jsonl")[0]

        tf_rows, amcl_rows, odom_rows = load("tf_map_odom"), load("amcl_pose"), load("odom")
        steps = map_odom_steps(tf_rows, odom_rows)
        entry = {
            "map_odom_steps": {**{key: value for key, value in steps.items() if key != "changed"},
                               "odom_age_s": distribution([row["odom_age_s"] for row in steps["changed"]]),
                               "changed_count": len(steps["changed"]),
                               "map_odom_m": distribution([row["map_odom_m"] for row in steps["changed"]]),
                               "map_odom_rad": distribution([row["map_odom_rad"] for row in steps["changed"]]),
                               "estimate_m": distribution([row["estimate_m"] for row in steps["changed"]]),
                               "estimate_rad": distribution([row["estimate_rad"] for row in steps["changed"]])},
            "tf_amcl_pairing": pair_tf_amcl(tf_rows, amcl_rows),
            "health": health_time_shares(load("health")),
            "trust": trust_summary(read_jsonl(root / "supervisor" / f"{robot_id}_supervisor.jsonl")[0]),
            "steps_series": steps["changed"],
        }

        if world_map is not None:
            truth = [(row["stamp"], row["x"], row["y"], row["yaw"]) for row in load("truth")]
            errors = pose_errors([(row["valid_time"], row["x"], row["y"], row["yaw"]) for row in tf_rows],
                                 [(row["stamp"], row["x"], row["y"], row["yaw"], row["v"], row["w"])
                                  for row in odom_rows], truth, se2(*world_map))
            entry["errors"] = None if errors is None else {key: value for key, value in errors.items()
                                                            if key != "series"}
            entry["error_covariance"] = (error_with_covariance(errors["series"]["rows"], amcl_rows)
                                         if errors else None)

        robots[robot_id] = entry

    return {"judgment": "not_judged", "robots": robots,
            "inventory": raw_inventory(root, conditions["robot_ids"]),
            "record_quality": session_record_quality(root, conditions["robot_ids"]),
            "note": "단일 회차 값. 개발용 출발값 제안의 근거일 뿐 확정값이 아니다"}


# 여러 로봇의 Gazebo 실제 자세를 기록한다(검증 전용).
class GazeboTruth:
    # 기록할 모델 이름과 월드 이름. raw가 있으면 표본을 원시 기록(truth.jsonl, 평가 전용)에도 덧붙인다.
    def __init__(self, models, world, raw=None):
        self.models = set(models)
        self.raw = raw
        self.topic = f"/world/{world}/dynamic_pose/info"
        self.lock = threading.Lock()
        self.samples = {model: [] for model in models}
        self.node = None

    # 자세 토픽을 구독한다.
    def start(self):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        self.node = GzNode()

        if not self.node.subscribe(Pose_V, self.topic, self.on_pose):
            raise RuntimeError(f"Gazebo 자세 구독 실패: {self.topic}")

    # 대상 모델의 (시뮬레이션 시각, x, y, yaw)를 저장한다.
    def on_pose(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9

        for pose in message.pose:
            if pose.name in self.models:
                q = pose.orientation
                yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

                with self.lock:
                    self.samples[pose.name].append((stamp, pose.position.x, pose.position.y, yaw))

                if self.raw is not None:
                    self.raw.write(pose.name, "truth", {"stamp": stamp, "recv_mono": time.monotonic(),
                                                        "x": pose.position.x, "y": pose.position.y, "yaw": yaw,
                                                        "use": "평가 전용"})

    # 로봇별 기록 사본
    def arrays(self):
        with self.lock:
            return {model: np.array(rows, float).reshape(-1, 4) for model, rows in self.samples.items()}

    # 구독 해제
    def stop(self):
        if self.node is not None:
            self.node.unsubscribe(self.topic)
            self.node = None


# ROS 입력 기록 노드를 만든다(rclpy를 쓰는 부분만 함수 안에서 불러 단위 시험이 ROS 없이 돌게 한다).
def make_recorder(robot_ids, scan, tf_future_s, raw=None, on_runtime=None):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener

    # 원시 기록에만 쓰는 타입(없는 환경에서도 기본 기록은 돌게 한다)
    extra = {}

    if raw is not None:
        from ax_interfaces.msg import ActuationState, LocalizationHealth
        from lifecycle_msgs.msg import TransitionEvent
        from rclpy.clock import Clock, ClockType
        from std_msgs.msg import String
        extra = {"ActuationState": ActuationState, "LocalizationHealth": LocalizationHealth,
                 "TransitionEvent": TransitionEvent, "String": String, "Clock": Clock, "ClockType": ClockType}

    # 로봇별 입력을 시각과 함께 쌓는 노드
    class Recorder(Node):
        # 구독을 만든다. 모든 콜백은 같은 잠금으로 기록한다.
        def __init__(self):
            super().__init__("ax_localization_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.lock = threading.Lock()
            self.records = {robot_id: {"map_odom": [], "odom": [], "amcl_pose": [], "initialpose": [], "scan": []}
                            for robot_id in robot_ids}
            self.odom_children = {f"{robot_id}/odom": robot_id for robot_id in robot_ids}
            self.foreign_map_parents = []
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            suffix = "scan_filtered" if scan == "filtered" else "scan"

            self.last_publishers = {}

            for robot_id in robot_ids:
                namespace = f"/ax/{robot_id}"
                self.create_subscription(Odometry, f"{namespace}/diff_drive_controller/odom",
                                         lambda msg, r=robot_id: self.on_odom(r, msg), 50)
                self.create_subscription(PoseWithCovarianceStamped, f"{namespace}/amcl_pose",
                                         lambda msg, r=robot_id: self.on_pose(r, "amcl_pose", msg), latched)
                self.create_subscription(PoseWithCovarianceStamped, f"{namespace}/initialpose",
                                         lambda msg, r=robot_id: self.on_pose(r, "initialpose", msg), 10)
                self.create_subscription(LaserScan, f"{namespace}/sensors/front_lidar/{suffix}",
                                         lambda msg, r=robot_id: self.on_scan(r, msg), qos_profile_sensor_data)

                if raw is not None:
                    # 5 Hz 전체를 남기려면 깊이 1(latched)로는 콜백이 밀릴 때 메시지가 버려진다(2026-11-09 회차에서 확인).
                    # 발행 쪽 STATE QoS(TRANSIENT_LOCAL·RELIABLE)와 맞추고 깊이만 늘린다.
                    stream = QoSProfile(depth=HEALTH_STREAM_DEPTH, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE)
                    self.create_subscription(extra["LocalizationHealth"], f"{namespace}/localization/health",
                                             lambda msg, r=robot_id: self.on_health(r, msg), stream)
                    self.create_subscription(extra["ActuationState"], f"{namespace}/safety/actuation_state",
                                             lambda msg, r=robot_id: self.on_actuation(r, msg), stream)
                    self.create_subscription(extra["TransitionEvent"], f"{namespace}/amcl/transition_event",
                                             lambda msg, r=robot_id: self.on_lifecycle(r, msg), 10)
                    self.create_subscription(extra["String"], f"{namespace}/robot_description",
                                             lambda msg, r=robot_id: self.on_description(r, msg), latched)

            # 연결 조회·발행자 변화는 sim time이 멈춰도 돌도록 단조 벽시계 타이머로 1 Hz 기록한다.
            if raw is not None:
                self.create_timer(1.0, self.on_watch, clock=extra["Clock"](clock_type=extra["ClockType"].STEADY_TIME))

        # 현재 시뮬레이션 시각(s)
        def now_seconds(self):
            return self.get_clock().now().nanoseconds * 1e-9

        # 원시 기록 한 줄(수신 sim·단조 벽시계 시각을 붙인다)
        def log(self, robot_id, stream, entry):
            if raw is not None:
                raw.write(robot_id, stream, {**entry, "recv_sim": self.now_seconds(), "recv_mono": time.monotonic()})

        # LocalizationHealth 전체(5 Hz 재발행 포함 모든 메시지)
        def on_health(self, robot_id, message):
            meta = message.meta
            self.log(robot_id, "health", {
                "stamp": meta.header.stamp.sec + meta.header.stamp.nanosec * 1e-9, "state_code": message.state,
                "state": {0: "UNKNOWN", 1: "OK", 2: "DEGRADED", 3: "LOST"}.get(message.state, str(message.state)),
                "valid": meta.valid, "reason": meta.validity_reason, "revision": meta.revision,
                "session": meta.source_session_id, "position_std_m": message.position_std_m,
                "yaw_std_rad": message.yaw_std_rad, "robot_id": message.robot_id})

            if on_runtime is not None and meta.source_session_id:
                on_runtime(robot_id, "supervisor_sessions", meta.source_session_id)

        # A4(시험 스텁) 구동 상태
        def on_actuation(self, robot_id, message):
            meta = message.meta
            self.log(robot_id, "actuation", {
                "stamp": meta.header.stamp.sec + meta.header.stamp.nanosec * 1e-9, "state": message.state,
                "valid": meta.valid, "revision": meta.revision, "source_id": meta.source_id,
                "robot_id": message.robot_id})

        # AMCL lifecycle 전이 사건(시각은 사건의 timestamp ns, 수신 시각 따로)
        def on_lifecycle(self, robot_id, message):
            self.log(robot_id, "amcl_lifecycle", {
                "event_timestamp_s": message.timestamp * 1e-9, "transition": message.transition.label,
                "transition_id": message.transition.id, "start_state": message.start_state.label,
                "goal_state": message.goal_state.label})

        # 실행 중 robot_description 해시·바퀴 접촉 조건(조건 파일 갱신용)
        def on_description(self, robot_id, message):
            if on_runtime is None:
                return

            on_runtime(robot_id, "robot_description_sha256", hashlib.sha256(message.data.encode()).hexdigest())

            try:
                from map_tools import describe_model
                on_runtime(robot_id, "wheel_contacts", describe_model(message.data).get("wheel_contacts"))
            except Exception as error:
                on_runtime(robot_id, "wheel_contacts", f"읽기 실패: {type(error).__name__}")

        # 1 Hz: TF 연결 조회 결과, amcl_pose·/tf 발행자 변화(GID 포함, 프로세스·재설정 구분 평가용)
        def on_watch(self):
            chain = self.tf_chain()

            for robot_id in robot_ids:
                self.log(robot_id, "tf_chain", {"links": chain.get(robot_id)})
                info = sorted((f"{item.node_namespace.rstrip('/')}/{item.node_name}", bytes(item.endpoint_gid).hex())
                              for item in self.get_publishers_info_by_topic(f"/ax/{robot_id}/amcl_pose"))

                if self.last_publishers.get(robot_id) != info:
                    self.last_publishers[robot_id] = info
                    self.log(robot_id, "publishers", {"topic": f"/ax/{robot_id}/amcl_pose",
                                                      "publishers": [{"node": n, "gid": g} for n, g in info]})

            tf_info = self.tf_publishers()

            if self.last_publishers.get("/tf") != tf_info:
                self.last_publishers["/tf"] = tf_info
                self.log(None, "tf_publishers", {"publishers": tf_info})

        # map→<robot>/odom만 골라 유효 시각(stamp − transform_tolerance)으로 기록한다.
        # 다른 부모에서 <robot>/odom으로 가는 변환이 있으면 따로 남긴다(발행자 중복 확인).
        def on_tf(self, message):
            for transform in message.transforms:
                robot_id = self.odom_children.get(transform.child_frame_id)

                if robot_id is None:
                    continue

                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                t, q = transform.transform.translation, transform.transform.rotation

                yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

                with self.lock:
                    if transform.header.frame_id == "map":
                        self.records[robot_id]["map_odom"].append((stamp - tf_future_s, t.x, t.y, yaw))
                    else:
                        self.foreign_map_parents.append((transform.header.frame_id, transform.child_frame_id))

                if transform.header.frame_id == "map":
                    self.log(robot_id, "tf_map_odom", {"stamp": stamp, "valid_time": stamp - tf_future_s,
                                                       "tf_future_s": tf_future_s, "x": t.x, "y": t.y, "yaw": yaw})
                else:
                    self.log(robot_id, "tf_foreign_parent", {"stamp": stamp, "parent": transform.header.frame_id})

        # odom 자세·속도
        def on_odom(self, robot_id, message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            p, q = message.pose.pose.position, message.pose.pose.orientation
            twist = message.twist.twist

            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

            with self.lock:
                self.records[robot_id]["odom"].append((stamp, p.x, p.y, yaw, twist.linear.x, twist.angular.z))

            self.log(robot_id, "odom", {"stamp": stamp, "x": p.x, "y": p.y, "yaw": yaw,
                                        "v": twist.linear.x, "w": twist.angular.z})

        # amcl_pose·initialpose(map 기준 자세와 xy 공분산 대각합)
        def on_pose(self, robot_id, kind, message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            p, q = message.pose.pose.position, message.pose.pose.orientation
            covariance = message.pose.covariance

            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

            with self.lock:
                self.records[robot_id][kind].append(
                    [self.now_seconds() if kind == "initialpose" else stamp, p.x, p.y,
                     yaw, covariance[0] + covariance[7], message.header.frame_id])

            self.log(robot_id, kind, {"stamp": stamp, "frame_id": message.header.frame_id, "x": p.x, "y": p.y,
                                      "yaw": yaw, "covariance": covariance_list(covariance)})

        # 스캔 수신 시각(stamp)과 frame
        def on_scan(self, robot_id, message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

            with self.lock:
                self.records[robot_id]["scan"].append((stamp, message.header.frame_id))

            self.log(robot_id, "scan", {"stamp": stamp, "frame_id": message.header.frame_id})

        # /tf 발행 노드 목록
        def tf_publishers(self):
            return sorted(f"{info.node_namespace.rstrip('/')}/{info.node_name}"
                          for info in self.get_publishers_info_by_topic("/tf"))

        # 로봇별 TF 연결(map→odom→base_link) 조회 결과
        def tf_chain(self):
            from rclpy.time import Time

            result = {}

            for robot_id in robot_ids:
                entry = {}

                for parent, child in (("map", f"{robot_id}/odom"), (f"{robot_id}/odom", f"{robot_id}/base_link"),
                                      ("map", f"{robot_id}/base_link")):
                    try:
                        transform = self.tf_buffer.lookup_transform(parent, child, Time())
                        t = transform.transform.translation
                        entry[f"{parent}->{child}"] = {"ok": True, "x": t.x, "y": t.y}
                    except Exception as error:
                        entry[f"{parent}->{child}"] = {"ok": False, "error": type(error).__name__}

                result[robot_id] = entry

            return result

    return Recorder()


# 한 로봇 결과에서 검증에 필요한 입력·연결이 빠진 사유 목록을 만든다. 비어 있으면 검증 가능하다.
def robot_problems(entry):
    problems = []

    for key, label in (("scan", "스캔"), ("odom", "odom"), ("map_odom", "map→odom")):
        if not entry[key]["messages"]:
            problems.append(f"{label} 수신 없음")

    if not entry["truth_samples"]:
        problems.append("Gazebo 실제 자세 표본 없음")

    if entry["errors"] is None:
        problems.append("오차 계산 표본 없음")

    chain = entry.get("tf_chain") or {}

    if not chain:
        problems.append("TF 연결 조회 결과 없음")

    for name, link in chain.items():
        if not link.get("ok"):
            problems.append(f"TF 연결 실패 {name}({link.get('error', '원인 미기록')})")

    return problems


# 기록 시간 충족과 로봇별 검증 가능 여부로 status를 정한다. 판정(judgment)은 바꾸지 않는다.
# start·end: 실제 기록한 시뮬레이션 시각, duration: 요청 시간, wall_limit: 벽시계 상한
def finalize(result, start, end, duration, wall_limit):
    duration_met = end - start >= duration
    result["recording"] = {"requested_sim_s": duration, "recorded_sim_s": round(end - start, 3),
                           "window_sim_s": [start, end], "duration_met": duration_met}
    problems = []

    if not duration_met:
        problems.append(f"기록 시간 부족: 벽시계 상한 {wall_limit} s 안에 시뮬레이션 {duration} s 중 "
                        f"{end - start:.2f} s만 기록")

    for robot_id, entry in result["robots"].items():
        entry["problems"] = robot_problems(entry)
        entry["verifiable"] = not entry["problems"]
        problems.extend(f"{robot_id}: {problem}" for problem in entry["problems"])

    result["verifiable"] = all(entry["verifiable"] for entry in result["robots"].values()) and bool(result["robots"])
    result["status"] = "completed" if duration_met and result["verifiable"] else "incomplete"
    result["problems"] = problems

    if not result["robots"]:
        problems.append("요청한 로봇 없음")

    return result


# 기록을 로봇별 결과로 정리한다. status는 finalize가 정한다.
def summarize(records, truth, world_map, tf_chain, tf_publishers, foreign, arguments):
    robots = {}

    for robot_id, record in records.items():
        scan_frames = sorted({row[1] for row in record["scan"]})
        poses = record["amcl_pose"]
        robots[robot_id] = {
            "scan": {**rate_summary([row[0] for row in record["scan"]]), "frames": scan_frames},
            "map_odom": rate_summary([row[0] for row in record["map_odom"]]),
            "odom": rate_summary([row[0] for row in record["odom"]]),
            "amcl_pose": {"messages": len(poses), "frames": sorted({row[5] for row in poses}),
                          "last": poses[-1][:5] if poses else None},
            "initialpose_inputs": [row[:5] for row in record["initialpose"]],
            "truth_samples": int(len(truth.get(robot_id, []))),
            "tf_chain": tf_chain.get(robot_id),
            "errors": pose_errors(record["map_odom"], record["odom"], truth.get(robot_id, []), world_map),
        }

    return {
        "status": None,
        "judgment": "not_judged",
        "judgment_reason": "위치 추정 정확도 기준 미확정(수치만 기록)",
        "conditions": {
            "robot_ids": list(records), "scan": arguments.scan, "duration_sim_s": arguments.duration,
            "world_map": {"x_m": arguments.world_map[0], "y_m": arguments.world_map[1],
                          "yaw_rad": arguments.world_map[2], "source": arguments.world_map_source,
                          "note": "검증 전용 변환. 위치 추정 입력으로 쓰지 않음"},
            "tf_future_s": arguments.tf_future,
            "label": arguments.label,
        },
        "tf_publishers": tf_publishers,
        "foreign_odom_parents": sorted(set(foreign)),
        "initial_pose_effects": initial_pose_effects(records),
        "robots": robots,
        "method": "AMCL 추정 = map→odom(유효 시각 ≤ odom 시각 중 마지막) · odom. 실제 = T_world_map⁻¹ · Gazebo 자세(보간)",
    }


# x,y,yaw 문자열을 세 실수로 바꾼다.
def triple(text):
    values = [float(value) for value in text.split(",")]

    if len(values) != 3:
        raise argparse.ArgumentTypeError("x,y,yaw 세 값")

    return values


# 지도 검증 기록(accuracy.json)에서 검증 전용 T_world_map을 읽는다(정렬 보정값 우선).
def world_map_from_accuracy(path):
    transforms = json.loads(Path(path).read_text(encoding="utf-8"))["transforms"]
    key = "optimal_T_world_map" if transforms.get("optimal_T_world_map") else "initial_T_world_map"
    value = transforms[key]

    return [value["x_m"], value["y_m"], value["yaw_rad"]], f"{path}:{key}"


# 인자를 읽고 정해진 시뮬레이션 시간 동안 기록한 뒤 결과를 쓴다.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-ids", default="robot_2")
    parser.add_argument("--scan", choices=["filtered", "raw"], default="filtered")
    parser.add_argument("--duration", type=float, default=30.0, help="기록 시간(시뮬레이션 s)")
    parser.add_argument("--wall-limit", type=float, default=900.0, help="벽시계 상한(s)")
    parser.add_argument("--world-map", type=triple, default=None, help="검증 전용 T_world_map x,y,yaw")
    parser.add_argument("--map-accuracy", type=Path, default=None, help="T_world_map을 읽을 지도 accuracy.json")
    parser.add_argument("--tf-future", type=float, default=1.0, help="AMCL transform_tolerance(s)")
    parser.add_argument("--gz-world", default="ax_test", help="Gazebo 월드 내부 이름(검증 기록용)")
    parser.add_argument("--label", default="")
    parser.add_argument("--output", type=Path, default=None, help="결과 폴더(--raw-dir만 주면 <raw-dir>/result)")
    parser.add_argument("--raw-dir", type=Path, default=None, help="원시 기록 회차 폴더(log/b2_health/<KST 시각>_<label>)")
    parser.add_argument("--map", type=Path, default=None, help="조건 기록용 지도 yaml")
    parser.add_argument("--start-poses", type=Path, default=None, help="조건 기록용 start_poses yaml")
    parser.add_argument("--world-file", type=Path, default=None, help="조건 기록용 월드 파일")
    parser.add_argument("--analyze", type=Path, default=None, help="ROS 없이 원시 기록 회차 폴더를 분석")
    parser.add_argument("--stop-file", type=Path, default=None,
                        help="이 파일이 생기면 기록을 정상 종료한다(duration 전이어도 중단으로 보지 않음)")
    parser.add_argument("--hash-file", action="append", default=[], metavar="NAME=PATH",
                        help="조건 파일에 sha256을 남길 추가 파일(여러 번)")
    arguments = parser.parse_args()

    if arguments.analyze is not None:
        world_map = None

        if arguments.map_accuracy is not None:
            world_map = world_map_from_accuracy(arguments.map_accuracy)[0]
        elif arguments.world_map is not None:
            world_map = arguments.world_map

        analysis = analyze_session(arguments.analyze, world_map, arguments.tf_future)
        write_json_atomic(arguments.analyze / "analysis.json", analysis)
        print(f"분석: {arguments.analyze / 'analysis.json'}")
        return 0

    if arguments.output is None:
        if arguments.raw_dir is None:
            parser.error("--output 또는 --raw-dir 필요")

        arguments.output = arguments.raw_dir / "result"

    if arguments.map_accuracy is not None:
        arguments.world_map, arguments.world_map_source = world_map_from_accuracy(arguments.map_accuracy)
    elif arguments.world_map is not None:
        arguments.world_map_source = "--world-map"
    else:
        parser.error("--world-map 또는 --map-accuracy 필요")

    robot_ids = [value.strip() for value in arguments.robot_ids.split(",") if value.strip()]
    arguments.output.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    # 원시 기록: 조건 파일을 수집 시작에 먼저 쓴다. 실행 중 알게 된 값은 runtime에 넣어 다시 쓴다.
    raw = conditions = None
    conditions_lock = threading.Lock()

    if arguments.raw_dir is not None:
        if (arguments.raw_dir / "conditions.json").exists():
            parser.error(f"{arguments.raw_dir}에 이미 회차 기록이 있음(새 폴더를 쓴다)")

        scripts = Path(__file__).resolve().parent
        package = scripts.parent
        workspace = package.parents[1]
        files = {
            "check_localization.py": Path(__file__).resolve(),
            "localization_supervisor.py": scripts / "localization_supervisor.py",
            "localization_health.py": scripts / "localization_health.py",
            "localization_health.yaml": package / "config" / "localization_health.yaml",
            "amcl.yaml": package / "config" / "amcl.yaml",
            "localization.launch.py": package / "launch" / "localization.launch.py",
        }

        for name, path in (("start_poses", arguments.start_poses), ("world", arguments.world_file),
                           ("map_accuracy", arguments.map_accuracy)):
            if path is not None:
                files[name] = path

        for item in arguments.hash_file:
            name, _, path = item.partition("=")

            if not name or not path:
                parser.error(f"--hash-file는 NAME=PATH: {item}")

            files[name] = Path(path)

        options = {key: (str(value) if isinstance(value, Path) else value) for key, value in vars(arguments).items()}
        conditions = collect_conditions(workspace, files, arguments.map, robot_ids, options)
        write_json_atomic(arguments.raw_dir / "conditions.json", conditions)
        raw = RawLog(arguments.raw_dir)

    # 실행 중 값(URDF 해시·바퀴 접촉·감독 세션)을 조건 파일에 반영한다. 바뀔 때만 다시 쓴다.
    def on_runtime(robot_id, key, value):
        with conditions_lock:
            entry = conditions["runtime"].setdefault(robot_id, {})

            if key == "supervisor_sessions":
                sessions = entry.setdefault(key, [])

                if value in sessions:
                    return

                sessions.append(value)
            elif entry.get(key) == value:
                return
            else:
                entry[key] = value

            write_json_atomic(arguments.raw_dir / "conditions.json", conditions)

    # 중단 신호(SIGINT·SIGTERM)는 기록을 멈추고 받은 만큼으로 결과를 쓴다.
    stop = threading.Event()

    def on_signal(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    rclpy.init(args=[])
    node = make_recorder(robot_ids, arguments.scan, arguments.tf_future, raw, on_runtime if raw else None)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    truth = GazeboTruth(robot_ids, arguments.gz_world, raw)
    truth.start()

    # 시뮬레이션 시각이 흐르기 시작한 뒤 duration만큼 기록한다.
    wall_deadline = time.monotonic() + arguments.wall_limit

    while node.now_seconds() == 0.0 and time.monotonic() < wall_deadline and not stop.is_set():
        time.sleep(0.05)

    start = node.now_seconds()
    print(f"[START] 기록 시작 sim {start:.2f} s, {arguments.duration} s 동안", flush=True)

    stopped_by_file = False

    while node.now_seconds() - start < arguments.duration and time.monotonic() < wall_deadline and not stop.is_set():
        if arguments.stop_file is not None and arguments.stop_file.exists():
            stopped_by_file = True
            break

        time.sleep(0.1)

    end = node.now_seconds()
    end_mono = time.monotonic()
    truth.stop()

    with node.lock:
        records = {robot_id: {key: list(rows) for key, rows in record.items()}
                   for robot_id, record in node.records.items()}
        foreign = list(node.foreign_map_parents)

    result = summarize(records, truth.arrays(), se2(*arguments.world_map), node.tf_chain(),
                       node.tf_publishers(), foreign, arguments)
    result["conditions"]["recorded_sim_s"] = [start, end]
    # 종료 파일로 끝냈으면 그때까지가 요청 기록 시간이다(기록 시간 부족으로 보지 않음).
    finalize(result, start, end, end - start if stopped_by_file else arguments.duration, arguments.wall_limit)
    result["recording"]["stopped_by_file"] = stopped_by_file

    if stop.is_set():
        result["status"] = "incomplete"
        result["problems"].append("중단 신호로 기록을 멈춤(받은 만큼 남김)")

    # 원시 기록 목록과 빠진 자료(판정은 바꾸지 않는다)
    if raw is not None:
        raw.close()
        result["raw"] = {"dir": str(arguments.raw_dir), **raw_inventory(arguments.raw_dir, robot_ids)}
        # status(completed·incomplete)는 기존 의미 그대로다. 수집량·손실 여부는 record_quality에 따로 둔다.
        result["record_quality"] = session_record_quality(arguments.raw_dir, robot_ids, end_mono=end_mono)

        for robot_id, streams in result["raw"]["missing"].items():
            result["problems"].append(f"{robot_id}: 원시 기록 없음 {', '.join(streams)}")
            result["status"] = "incomplete"

    path = arguments.output / "localization_result.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"결과: {path} (status={result['status']}, judgment={result['judgment']})")

    for robot_id, entry in result["robots"].items():
        errors = entry["errors"]
        text = (f"위치 오차 평균 {errors['position_error_m']['mean']:.3f}·최대 {errors['position_error_m']['max']:.3f} m, "
                f"방향 오차 최대 {errors['yaw_error_rad']['max']:.3f} rad, 마지막 {errors['final']['position_error_m']:.3f} m"
                if errors else "오차 계산 표본 없음")
        print(f"[INFO] {robot_id}: 스캔 {entry['scan']['messages']}개, map→odom {entry['map_odom']['messages']}개, {text}")

    for problem in result["problems"]:
        print(f"[INCOMPLETE] {problem}")

    # 콜백 스레드가 끝난 뒤 노드를 없앤다(순서가 바뀌면 Destroyable 예외가 남는다).
    executor.shutdown(timeout_sec=2.0)
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()

    return 0 if result["status"] == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
