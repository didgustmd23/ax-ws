#!/usr/bin/env python3
"""B2 AMCL 위치 추정 개발 검증 기록·평가. 실행 중인 시뮬레이션·B1·localization.launch.py에 붙어 기록만 한다.

  python3 src/ax_localization/scripts/check_localization.py --robot-ids robot_1,robot_2 \
      --duration 60 --world-map 2.4927,-1.4957,-0.0003 --output <결과 폴더>

기록(로봇별): map→<robot>/odom TF(stamp, 위치), odom, amcl_pose, initialpose 입력, 스캔 수신 시각,
Gazebo 실제 자세(검증 전용, 위치 추정 입력으로 쓰지 않음), /tf 발행 노드 목록, 끝날 때 TF 연결 조회 결과.

평가: 같은 시각의 AMCL 추정 차체 자세(map→odom · odom→base_link)와 실제 자세(T_world_map⁻¹ · world 자세)의
위치·방향 오차. T_world_map은 지도 검증 기록(accuracy.json)의 값을 쓰는 검증 전용 변환이다.
AMCL TF stamp는 처리한 스캔 시각 + transform_tolerance라서 그만큼 빼서 유효 시각으로 쓴다.
정확도 판정: --criteria(개발용 기준 YAML)를 주면 정지·주행 오차, 스캔 입력, 주행 중 health OK 시간 비율(B2-L3,
원시 기록 필요), 벽시계 제한(LD-3, 종료 원인·감독 진단)을 판정해 criteria_items와 judgment를 남긴다. 주지 않으면
judgment는 not_judged다. --rejudge <결과 JSON> --criteria <YAML> [--raw-dir <원시 폴더>]는 ROS 없이
과거 결과를 다시 판정해 같은 폴더에 <이름>.judged.json을 쓴다(원본·원시 기록은 바꾸지 않음).
--batch <결과 JSON...> --criteria <YAML> --output <JSON>은 여러 회차를 묶어 항목별 충족·반복 수(B2-L8)를 판정한다.
판정 값: PASS·FAIL·not_judged·not_applicable(이 회차 조건에서 대상 아님)·recorded(record_only, 값만 기록).

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
import bisect
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
# --scan → 기록·B2-L9 대상 스캔 토픽(namespace 기준 상대). localization.launch.py SCAN_INPUTS와 같다.
SCAN_TOPICS = {"filtered": "sensors/front_lidar/scan_filtered", "raw": "sensors/front_lidar/scan",
               "merged": "sensors/merged/scan_filtered"}
TRUTH_MAX_GAP_S = 0.1


# 표본 배열의 평균·95백분위수·최대(절댓값). 비어 있으면 None
def stats(values):
    values = np.abs(np.asarray(values, float))

    if len(values) == 0:
        return None

    return {"mean": float(values.mean()), "p95": float(np.percentile(values, 95)), "max": float(values.max())}


# 한 로봇의 기록으로 AMCL 추정 자세와 실제 자세의 오차 시계열·요약을 만든다.
# map_odom: (유효 시각, x, y, yaw), odom: (시각, x, y, yaw, v, w), truth: (시각, x, y, yaw) world 기준.
# round_digits: 시계열 행 반올림 자릿수(보고용). None이면 원시 정밀도를 그대로 둔다(B2-L4 경계 판정용).
def pose_errors(map_odom, odom, truth, world_map, period_s=SAMPLE_PERIOD_S, max_gap=TRUTH_MAX_GAP_S, round_digits=4):
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
                   "rows": (rows if round_digits is None else np.round(rows, round_digits)).tolist()},
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

        changed.append({"stamp": later["stamp"], "valid_time": later.get("valid_time"),
                        "dt_s": later["stamp"] - earlier["stamp"],
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

            self.last_publishers = {}

            for robot_id in robot_ids:
                namespace = f"/ax/{robot_id}"
                self.create_subscription(Odometry, f"{namespace}/diff_drive_controller/odom",
                                         lambda msg, r=robot_id: self.on_odom(r, msg), 50)
                self.create_subscription(PoseWithCovarianceStamped, f"{namespace}/amcl_pose",
                                         lambda msg, r=robot_id: self.on_pose(r, "amcl_pose", msg), latched)
                self.create_subscription(PoseWithCovarianceStamped, f"{namespace}/initialpose",
                                         lambda msg, r=robot_id: self.on_pose(r, "initialpose", msg), 10)
                self.create_subscription(LaserScan, f"{namespace}/{SCAN_TOPICS[scan]}",
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

            # 활성 로봇 집합(/ax/<id>/joint_states 발행자)이 바뀔 때만 남긴다. 측정 중 로봇 추가·누락 확인용 기록이다.
            names = [name for name, _ in self.get_topic_names_and_types()]
            active = active_robots_from_topics(names, self.count_publishers)

            if self.last_publishers.get("active_robots") != active:
                self.last_publishers["active_robots"] = active
                self.log(None, "active_robots", {"active_robot_ids": active, "source": ACTIVE_ROBOT_SOURCE})

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
            "robot_ids": list(records), "scan": arguments.scan, "scan_topic": SCAN_TOPICS[arguments.scan],
            "duration_sim_s": arguments.duration,
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


# 렌더러 관련 환경 변수. 측정 도구 프로세스의 값이며 Gazebo 서버 프로세스 값과 다를 수 있다.
RENDERER_ENV_KEYS = ("LIBGL_ALWAYS_SOFTWARE", "GALLIUM_DRIVER", "MESA_D3D12_DEFAULT_ADAPTER_NAME")
ACTIVE_ROBOT_SOURCE = "ROS 그래프 /ax/<id>/joint_states 발행자 1개 이상(측정 시작 시, 같은 ROS domain)"


# 토픽 이름 목록에서 /ax/<id>/joint_states에 발행자가 있는 로봇 ID를 정렬해 돌려준다.
# 구독자만 있는 토픽(robot_state_publisher가 먼저 뜬 경우 등)은 활성으로 보지 않는다.
# publisher_count: 토픽 이름 → 발행자 수 함수(시험에서는 모의)
def active_robots_from_topics(topic_names, publisher_count):
    robot_ids = set()

    for name in topic_names:
        parts = name.split("/")
        shaped = len(parts) == 4 and parts[0] == "" and parts[1] == "ax" and parts[3] == "joint_states" and parts[2]

        if shaped and publisher_count(name) > 0:
            robot_ids.add(parts[2])

    return sorted(robot_ids)


# 측정 도구 프로세스의 렌더러 환경 값을 기록한다.
def renderer_record(environ):
    return {"env": {key: environ.get(key) for key in RENDERER_ENV_KEYS},
            "source": "측정 도구 프로세스 환경(Gazebo 서버 프로세스 값이 아님)"}


# 같은 GZ_PARTITION의 gz sim 서버 명령줄에서 월드 파일을 찾아 --world-file과 같은 파일인지 본다.
# 확인하지 못하면 method를 "인자 기준"으로 남긴다. proc_root는 시험용(/proc 대신 가짜 폴더).
def gz_world_check(world_sha256, partition, proc_root="/proc"):
    found = []

    for entry in Path(proc_root).iterdir() if Path(proc_root).is_dir() else []:
        if not entry.name.isdigit():
            continue

        try:
            args = (entry / "cmdline").read_bytes().split(b"\0")
            environ = (entry / "environ").read_bytes().split(b"\0")
        except OSError:
            continue

        # gz(ruby)는 프로세스 제목을 한 문자열로 바꾸므로 공백으로도 나눈다.
        args = [word for arg in args if arg for word in arg.decode("utf-8", "replace").split()]

        if "gz" not in args or "sim" not in args:
            continue

        env = dict(item.decode("utf-8", "replace").partition("=")[::2] for item in environ if b"=" in item)

        if partition is not None and env.get("GZ_PARTITION") != partition:
            continue

        found += [arg for arg in args if arg.endswith((".world", ".sdf")) and Path(arg).is_file()]

    if not found:
        return {"method": "인자 기준", "reason": "같은 GZ_PARTITION의 gz sim 서버 월드 인자를 찾지 못함",
                "gz_world_files": [], "matches": None}

    hashes = {path: file_sha256(path) for path in sorted(set(found))}

    return {"method": "gz sim 서버 명령줄", "gz_world_files": hashes,
            "matches": world_sha256 is not None and world_sha256 in hashes.values()}


# 결과 conditions에 넣을 부하 단계 정보를 만든다(필드 추가만, 기존 필드는 바꾸지 않는다).
def load_stage_conditions(world_file, active_robot_ids, environ, world_check=None):
    world_sha = file_sha256(world_file) if world_file is not None else None
    record = {
        "active_robot_ids": sorted(active_robot_ids) if active_robot_ids is not None else None,
        "active_robot_count": len(active_robot_ids) if active_robot_ids is not None else None,
        "active_robot_source": ACTIVE_ROBOT_SOURCE,
        "world_file": str(world_file) if world_file is not None else None,
        "world_sha256": world_sha,
        "world_check": world_check if world_check is not None else {"method": "인자 기준"},
        "renderer": renderer_record(environ),
    }

    if world_file is None:
        record["world_reason"] = "--world-file 미지정"
    elif world_sha is None:
        record["world_reason"] = f"월드 파일 없음: {world_file}"

    return record


# 실제 정지 판정 임계값. 공통 인터페이스 계약 D-06(docs/design/INTERFACE_CONTRACT.md)의 값이며 여기서 정하지 않는다.
# 정지·주행 구간은 평가 표본마다 odom 선속도·각속도가 이 값 이하인지로만 나눈다(D-06의 0.5 s 유지는 보지 않는다).
D06_LINEAR_M_S = 0.01
D06_ANGULAR_RAD_S = 0.02

# 판정 값. not_applicable은 이 회차 조건에서 대상이 아님, recorded는 기준 파일 record_only 항목(값만 기록)이다.
PASS = "PASS"
FAIL = "FAIL"
NOT_JUDGED = "not_judged"
NOT_APPLICABLE = "not_applicable"
RECORDED = "recorded"


# 여러 회차를 모아야 판정하는 항목. 회차 판정에서는 not_applicable이고 --batch에서 센다.
CRITERIA_BATCH_ONLY = (
    ("B2-L8", "repeats", "묶음 판정 항목(--batch에서 회차 수를 셈)"),
    ("LD-2", "load.max_moving_p95_increase_from_l0_m", "묶음 판정 항목(--batch, 부하 단계 비교)"),
)

# 묶음에서 충족 여부를 보는 기준 항목 ID(기준 파일 3·4절 B2 몫)
BATCH_COVERAGE_IDS = ("B2-L1", "B2-L2", "B2-L3", "B2-L4", "B2-L5", "B2-L6", "B2-L7", "B2-L8", "B2-L9", "LD-2", "LD-3")


# 값이 유한한 실수인지 확인한다(None·NaN·inf·bool은 아님).
def finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False

    return math.isfinite(value)


# 기준 항목 하나를 만든다. 비교는 경계를 포함하고, 값이 없거나 비유한이면 not_judged다.
# kind: "max"(값 ≤ 한계) 또는 "min"(값 ≥ 한계)
def criteria_item(item_id, metric, value, limit, kind, robot=None, reason=""):
    item = {"id": item_id, "robot": robot, "metric": metric, "value": value, "limit": limit}

    if not finite_number(limit):
        item.update(result=NOT_JUDGED, reason=f"기준값 없음·비유한: {limit}")
    elif not finite_number(value):
        item.update(result=NOT_JUDGED, reason=reason or f"값 없음·비유한: {value}")
    else:
        passed = value <= limit if kind == "max" else value >= limit
        item.update(result=PASS if passed else FAIL, reason=reason)

    return item


# 값 없이 결과만 정하는 항목(not_judged·not_applicable)을 만든다. 사유는 필수다.
def fixed_item(item_id, metric, result, reason, robot=None, limit=None):
    return {"id": item_id, "robot": robot, "metric": metric, "value": None, "limit": limit,
            "result": result, "reason": reason}


# 오차 시계열을 odom 속도로 정지·주행 구간으로 나눈다. 반환: (정지 행, 주행 행)
def split_by_motion(errors):
    if not errors or not errors.get("series"):
        return [], []

    columns = errors["series"]["columns"]
    speed = columns.index("odom_speed_m_s")
    rate = columns.index("odom_yaw_rate_rad_s")
    stationary = []
    moving = []

    for row in errors["series"]["rows"]:
        still = abs(row[speed]) <= D06_LINEAR_M_S and abs(row[rate]) <= D06_ANGULAR_RAD_S
        (stationary if still else moving).append(row)

    return stationary, moving


# 한 구간 행들의 열 하나를 절댓값 배열로 꺼낸다.
def column_values(errors, rows, name):
    index = errors["series"]["columns"].index(name)

    return np.abs(np.asarray([row[index] for row in rows], float))


# 구간 오차 항목들을 만든다. 오차 표본이 아예 없으면 not_judged, 이 구간 표본만 없으면 not_applicable이다.
# specs: [(metric, limit, 값 계산 함수)]
def segment_items(item_id, robot_id, errors, rows, specs, absent_reason):
    items = []

    for metric, limit, compute in specs:
        if not errors:
            items.append(fixed_item(item_id, metric, NOT_JUDGED, "오차 계산 표본 없음", robot_id, limit))
        elif not rows:
            items.append(fixed_item(item_id, metric, NOT_APPLICABLE, absent_reason, robot_id, limit))
        else:
            items.append(criteria_item(item_id, metric, compute(), limit, "max", robot_id))

    return items


# 한 로봇의 B2-L1·L2(정지·주행 오차)와 B2-L9(스캔 입력) 항목을 만든다.
def robot_criteria_items(robot_id, entry, criteria):
    items = []
    errors = entry.get("errors")
    stationary, moving = split_by_motion(errors)
    still_limits = criteria.get("stationary") or {}
    move_limits = criteria.get("moving") or {}

    # B2-L1 정지 구간 위치·방향 오차 최대
    items += segment_items("B2-L1", robot_id, errors, stationary, [
        ("stationary.max_position_error_m", still_limits.get("max_position_error_m"),
         lambda: float(column_values(errors, stationary, "position_error_m").max())),
        ("stationary.max_yaw_error_rad", still_limits.get("max_yaw_error_rad"),
         lambda: float(column_values(errors, stationary, "yaw_error_rad").max())),
    ], "정지 구간 없음(odom 속도 기준)")

    # B2-L2 주행 구간 위치 오차 p95·최대, 방향 오차 최대
    items += segment_items("B2-L2", robot_id, errors, moving, [
        ("moving.max_position_error_p95_m", move_limits.get("max_position_error_p95_m"),
         lambda: float(np.percentile(column_values(errors, moving, "position_error_m"), 95))),
        ("moving.max_position_error_m", move_limits.get("max_position_error_m"),
         lambda: float(column_values(errors, moving, "position_error_m").max())),
        ("moving.max_yaw_error_rad", move_limits.get("max_yaw_error_rad"),
         lambda: float(column_values(errors, moving, "yaw_error_rad").max())),
    ], "주행 구간 없음(odom 속도 기준)")

    # B2-L9 스캔 수신률(공칭 대비)과 최대 공백. 기록 시각은 LaserScan header stamp(시뮬레이션 시각)다.
    load = criteria.get("load") or {}
    scan = entry.get("scan") or {}
    nominal = load.get("nominal_scan_rate_hz")
    rate_hz = scan.get("rate_hz")
    ratio = rate_hz / nominal if finite_number(rate_hz) and finite_number(nominal) and nominal > 0 else None
    items.append(criteria_item("B2-L9", "load.min_scan_receive_ratio", ratio, load.get("min_scan_receive_ratio"),
                               "min", robot_id, "" if ratio is not None else "스캔 수신률 없음"))
    items.append(criteria_item("B2-L9", "load.max_scan_gap_s", scan.get("max_gap_s"), load.get("max_scan_gap_s"),
                               "max", robot_id, "" if scan.get("max_gap_s") is not None else "스캔 공백 값 없음"))

    return items, {"stationary_samples": len(stationary), "moving_samples": len(moving)}


# ---- B2-L3 정상 주행 중 health OK 시간 비율 ----
# health 유지 상한: 계약 D-05 위치 신뢰성 만료 1.0 s(docs/design/INTERFACE_CONTRACT.md). 다음 갱신까지 유지하되 넘지 않는다.
HEALTH_HOLD_MAX_S = 1.0
# odom 유지 상한: 이 간격을 넘는 odom 공백은 주행 여부를 확인할 수 없는 시간이다(odom 공칭 50 Hz).
ODOM_HOLD_MAX_S = 0.5
# 명시적 제외 구간 표시 파일(원시 기록 폴더 바로 아래). 오류 주입·재위치 구간을 기록하는 쪽이 쓴다.
HEALTH_EXCLUSIONS_FILE = "health_exclusions.json"
HEALTH_EXCLUSIONS_SCHEMA = "ax_b2_health_exclusions/1"
HEALTH_RATIO_METHOD = ("정상 주행 시간 중 state=OK이고 meta.valid=true인 시간 / 정상 주행 시간(sim time). "
                       "주행은 odom 표본 속도가 D-06 경계(선속도 0.01 m/s 또는 각속도 0.02 rad/s)를 넘는 구간"
                       "(다음 odom 표본까지, 공백 0.5 s 초과는 주행 여부 미확인). health는 stamp부터 다음 갱신까지, "
                       "최대 1.0 s(D-05 만료) 유지. 확인 불가 시간은 분모에 남기고 있으면 not_judged. "
                       "명시적 제외 구간(health_exclusions.json)만 분모에서 뺀다. Gazebo 실제 자세는 쓰지 않는다")


# 구간 목록을 정렬·병합한다(맞닿은 구간도 합친다). 길이 0 구간은 버린다.
def merge_intervals(intervals):
    merged = []

    for start, end in sorted((start, end) for start, end in intervals if end > start):
        if merged and start <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], end)
        else:
            merged.append([start, end])

    return [tuple(interval) for interval in merged]


# 병합된 구간 목록의 총 길이
def interval_measure(intervals):
    return float(sum(end - start for start, end in intervals))


# 두 구간 목록의 교집합
def intersect_intervals(first, second):
    first, second = merge_intervals(first), merge_intervals(second)
    result = []
    i = j = 0

    while i < len(first) and j < len(second):
        start = max(first[i][0], second[j][0])
        end = min(first[i][1], second[j][1])

        if end > start:
            result.append((start, end))

        if first[i][1] < second[j][1]:
            i += 1
        else:
            j += 1

    return result


# 구간 목록에서 다른 구간 목록을 뺀다.
def subtract_intervals(base, removed):
    result = []
    removed = merge_intervals(removed)

    for start, end in merge_intervals(base):
        cursor = start

        for cut_start, cut_end in removed:
            if cut_end <= cursor or cut_start >= end:
                continue

            if cut_start > cursor:
                result.append((cursor, cut_start))

            cursor = max(cursor, cut_end)

        if cursor < end:
            result.append((cursor, end))

    return result


# 원시 기록 줄을 수신 순서대로 읽어 stamp 순 표본으로 만든다. 같은 stamp는 마지막 수신을 쓴다(중복·재발행).
# 비유한 stamp·stamp 역행은 이상으로 남긴다(시간축을 확정할 수 없음). 반환: (표본, 중복 수, 이상 목록)
def ordered_samples(rows, parse, label):
    samples, duplicates, anomalies = [], 0, []

    for index, row in enumerate(rows):
        parsed = parse(row)

        if parsed is None:
            anomalies.append(f"{label} {index}번째 줄 값 없음·비유한")
            continue

        if samples and parsed[0] == samples[-1][0]:
            duplicates += 1
            samples[-1] = parsed
            continue

        if samples and parsed[0] < samples[-1][0]:
            anomalies.append(f"{label} stamp 역행: {samples[-1][0]} → {parsed[0]}")
            continue

        samples.append(parsed)

    return samples, duplicates, anomalies


# odom 원시 기록에서 주행·정지·주행 여부 미확인 구간을 만든다(sim stamp, D-06 속도 경계).
def motion_intervals(odom_rows, hold_max_s=ODOM_HOLD_MAX_S):
    def parse(row):
        values = (row.get("stamp"), row.get("v"), row.get("w"))

        return values if all(finite_number(value) for value in values) else None

    samples, duplicates, anomalies = ordered_samples(odom_rows, parse, "odom")
    moving, stationary, unknown = [], [], []

    for (stamp, linear, angular), following in zip(samples, samples[1:]):
        if following[0] - stamp > hold_max_s:
            unknown.append((stamp, following[0]))
        elif abs(linear) > D06_LINEAR_M_S or abs(angular) > D06_ANGULAR_RAD_S:
            moving.append((stamp, following[0]))
        else:
            stationary.append((stamp, following[0]))

    return {"samples": len(samples), "duplicates": duplicates, "anomalies": anomalies,
            "moving": merge_intervals(moving), "stationary": merge_intervals(stationary),
            "unknown": merge_intervals(unknown),
            "range": [samples[0][0], samples[-1][0]] if samples else None}


# health 원시 기록에서 상태 유지 구간(확인됨)과 OK·valid 구간을 만든다(sim stamp). 상태는 다음 갱신 또는
# hold_max_s 중 이른 쪽까지 유지하고, 세션이 바뀌어도 이전 세션 상태를 다음 stamp 너머로 잇지 않는다.
def health_intervals(health_rows, hold_max_s=HEALTH_HOLD_MAX_S):
    def parse(row):
        stamp, state, valid = row.get("stamp"), row.get("state"), row.get("valid")

        if not finite_number(stamp) or not isinstance(state, str):
            return None

        return stamp, state, valid is True, row.get("session")

    samples, duplicates, anomalies = ordered_samples(health_rows, parse, "health")
    covered, ok = [], []
    breaks = 0

    for index, (stamp, state, valid, session) in enumerate(samples):
        end = stamp + hold_max_s

        if index + 1 < len(samples):
            end = min(end, samples[index + 1][0])
            breaks += samples[index + 1][3] != session

        covered.append((stamp, end))

        if state == "OK" and valid:
            ok.append((stamp, end))

    return {"samples": len(samples), "duplicates": duplicates, "anomalies": anomalies, "session_breaks": breaks,
            "sessions": sorted({sample[3] for sample in samples if sample[3]}),
            "covered": merge_intervals(covered), "ok": merge_intervals(ok),
            "range": [samples[0][0], samples[-1][0]] if samples else None}


# 명시적 제외 구간 표시를 검사한다. 각 구간에 유한한 start_sim < end_sim, 사유(reason)·근거(source)가 있어야 한다.
# robot_id가 있으면 그 로봇에만 적용한다. 반환: (구간 목록, 문제 목록)
def health_exclusions(entries, robot_id=None):
    intervals, problems = [], []

    for index, entry in enumerate(entries or []):
        if not isinstance(entry, dict):
            problems.append(f"제외 구간 {index}: 형식 오류")
            continue

        if entry.get("robot_id") not in (None, robot_id):
            continue

        start, end = entry.get("start_sim"), entry.get("end_sim")

        if not (finite_number(start) and finite_number(end) and start < end):
            problems.append(f"제외 구간 {index}: start_sim < end_sim(유한) 아님")
        elif not entry.get("reason") or not entry.get("source"):
            problems.append(f"제외 구간 {index}: 사유(reason)·근거(source) 없음")
        else:
            intervals.append({"start_sim": start, "end_sim": end, "reason": entry["reason"],
                              "source": entry["source"]})

    return intervals, problems


# 한 로봇의 정상 주행 중 health OK 시간 비율(B2-L3 지표). status: measured·not_applicable·not_judged
# exclusions: 명시적 제외 구간 표시(health_exclusions.json intervals), robot_id: 표시를 적용할 로봇
def health_ok_ratio(health_rows, odom_rows, exclusions=None, broken_lines=0, robot_id=None):
    marks, problems = health_exclusions(exclusions, robot_id)
    motion = motion_intervals(odom_rows)
    health = health_intervals(health_rows)
    moving_all = motion["moving"]
    moving = subtract_intervals(moving_all, [(mark["start_sim"], mark["end_sim"]) for mark in marks])
    total = interval_measure(moving)
    ok = interval_measure(intersect_intervals(moving, health["ok"]))
    known = interval_measure(intersect_intervals(moving, health["covered"]))
    record = {
        "method": HEALTH_RATIO_METHOD,
        "time_basis": "sim time(health meta.header.stamp, odom header.stamp)",
        "health_hold_max_s": HEALTH_HOLD_MAX_S, "odom_hold_max_s": ODOM_HOLD_MAX_S,
        "d06_linear_m_s": D06_LINEAR_M_S, "d06_angular_rad_s": D06_ANGULAR_RAD_S,
        "moving_s": total, "ok_s": ok, "unknown_s": max(0.0, total - known), "ratio": ok / total if total else None,
        "excluded": marks, "excluded_moving_s": interval_measure(moving_all) - total,
        "scope": ("명시적 제외 구간을 뺀 관측 주행" if marks
                  else "관측된 주행 전체(명시적 오류 주입·재위치 구간 표시 없음)"),
        "motion_unknown_s": interval_measure(motion["unknown"]),
        "health": {key: health[key] for key in ("samples", "duplicates", "anomalies", "session_breaks",
                                                "sessions", "range")},
        "odom": {key: motion[key] for key in ("samples", "duplicates", "anomalies", "range")},
        "broken_lines": broken_lines,
    }
    reasons = problems + health["anomalies"] + motion["anomalies"]

    if broken_lines:
        reasons.append(f"원시 기록 깨진 줄 {broken_lines}개")

    # 주행 여부를 정할 수 없으면(odom 부족·공백) not_judged, 정지만 했으면 not_applicable
    if motion["samples"] < 2:
        reasons.append("odom 원시 기록 없음·부족(주행 여부 확인 불가)")
    elif motion["unknown"]:
        reasons.append(f"odom 공백 {record['motion_unknown_s']:.3f} s(> {ODOM_HOLD_MAX_S} s, 주행 여부 확인 불가)")

    if not reasons and total == 0:
        record.update(status=NOT_APPLICABLE, reason="정상 주행 구간 없음(odom 속도 기준 정지만 함)")
        return record

    if not health["samples"]:
        reasons.append("health 원시 기록 없음")
    elif record["unknown_s"] > 1e-9:
        reasons.append(f"health 확인 불가 주행 시간 {record['unknown_s']:.3f} s(공백·세션 변경·만료 1.0 s 초과)")

    if reasons:
        record.update(status=NOT_JUDGED, reason="; ".join(reasons))
    else:
        record.update(status="measured", reason="")

    return record


# B2-L3 항목 하나(로봇별). 기록의 status에 따라 비교·not_applicable·not_judged
def health_ratio_item(robot_id, record, limit):
    metric = "moving.min_health_ok_ratio"

    if record.get("status") == "measured":
        item = criteria_item("B2-L3", metric, record["ratio"], limit, "min", robot_id,
                             f"정상 주행 {record['moving_s']:.3f} s 중 OK {record['ok_s']:.3f} s")
    else:
        item = fixed_item("B2-L3", metric, record.get("status") if record.get("status") == NOT_APPLICABLE
                          else NOT_JUDGED, record.get("reason") or "근거 없음", robot_id, limit)

    item["health_ok"] = record

    return item


# ---- LD-3 벽시계 제한 회차 ----
# 대상 제한(실제 값은 기록에서 읽는다): 수집기 --wall-limit, 감독 노드 reinit.timeout_wall_s,
# estimate_jump.intent_timeout_wall_s. sim time 정체 감시(health.clock_stall_wall_s → CLOCK_INVALID)는
# 시간 제한이 아니라 시계 유효성 판정이므로 세지 않는다.
WALL_CLOCK_METHOD = ("벽시계 제한으로 판정·기록이 끝난 회차: 수집기 --wall-limit 도달로 요청 sim 시간을 못 채움, "
                     "감독 노드 재위치 timeout_wall_s·의도 추적 intent_timeout_wall_s 초과(진단 기록). "
                     "sim 시간초과만·Ctrl+C·종료 파일은 세지 않음. 두 시계가 함께 넘으면 둘 다 기록하고 한 번 셈. "
                     "clock_stall_wall_s(CLOCK_INVALID)는 세지 않음")
WALL_TIMEOUT_CODES = ("REINIT_FAILED_TIMEOUT", "REINIT_FAILED_AMCL_UNAVAILABLE")
# 감독 진단 tick 허용 공백(단조 벽시계 s). tick은 STEADY_TIME 5 Hz(0.2 s)이고 11-12·11-14 실제 4회차 최대 간격은
# 0.5 s다. 주기 5배(1.0 s)를 넘는 공백이 수집 구간에 걸치면 그 사이 사건 누락을 배제할 수 없어 무발생을 확정하지 않는다.
SUPERVISOR_TICK_MAX_GAP_S = 1.0


# 수집기 종료 원인(측정 도구가 결과 recording.termination에 남긴다). 시계 값은 인자로 받는다(가짜 시계 시험).
# wall_limited: 벽시계 상한에 닿았고 요청 sim 시간을 못 채웠으며 종료 파일로 끝내지 않은 경우.
def collector_termination(sim_start, sim_end, duration, mono_start, mono_end, wall_limit, stopped_by_file, signalled):
    sim_reached = sim_end - sim_start >= duration
    wall_reached = mono_end - mono_start >= wall_limit
    causes = [name for name, flag in (("SIGNAL", signalled), ("STOP_FILE", stopped_by_file),
                                      ("SIM_DURATION", sim_reached), ("WALL_LIMIT", wall_reached)) if flag]

    return {"schema": 1, "causes": causes,
            "wall_limited": bool(wall_reached and not sim_reached and not stopped_by_file),
            "wall_limit_s": wall_limit, "wall_limit_source": "check_localization.py --wall-limit",
            "wall_elapsed_s": mono_end - mono_start, "mono_start": mono_start, "mono_end": mono_end,
            "wall_source": "단조 벽시계(time.monotonic)",
            "sim_start": sim_start, "sim_end": sim_end, "sim_elapsed_s": sim_end - sim_start,
            "requested_sim_s": duration}


# 결과 파일에서 수집기 벽시계 제한 여부를 읽는다. 옛 기록은 기록 시간 충족만으로 무발생을 확인한다.
def collector_wall_status(result):
    recording = result.get("recording") or {}
    termination = recording.get("termination")

    if isinstance(termination, dict) and isinstance(termination.get("wall_limited"), bool):
        return {"source": "recording.termination", "limited": termination["wall_limited"], "level": "confirmed",
                "detail": termination}

    if recording.get("duration_met") is True:
        return {"source": "recording.duration_met", "limited": False, "level": "confirmed",
                "detail": "요청 기록 시간(또는 종료 파일까지)을 채움: 수집기 벽시계 상한이 기록을 자르지 않음"}

    return {"source": "recording", "limited": None, "level": "unverified",
            "detail": "옛 기록: 기록 시간 부족의 원인(벽시계 상한·중단 신호) 미기록"}


# 감독 노드 진단 줄에서 벽시계 시간초과를 찾는다. window: [수집 시작 mono, 끝 mono](없으면 전체).
# 재위치 종료(reinit_finished)·의도 종료(INTENT_CLOSED)의 timeout 세부로 시계를 가른다. 세부가 없는 옛 기록의
# TIMEOUT·REINIT_FAILED_TIMEOUT·AMCL_UNAVAILABLE은 어느 시계인지 알 수 없어 미확인이다.
# limited: True(확인된 제한)·False(무발생 확인)·None(확인 불가)
# 연속성: 수집 구간 [시작, 끝]에 걸친 tick 간격이 모두 허용 공백 이하, 끝 근처 tick 있음, mono 유한·비역행.
# 시작은 첫 tick이 시작 + 허용 공백 이내이거나, 감독 노드가 늦게 켜졌다는 증명(health_rows: 수집기 health 원시 기록의
# 첫 수신 mono가 첫 tick 이후이고 health 세션이 모두 진단 세션)이 있어야 한다. 두 mono는 같은 호스트 CLOCK_MONOTONIC이다.
def supervisor_wall_status(rows, window=None, broken_lines=0, health_rows=None):
    if rows is None:
        return {"limited": None, "level": "unverified", "reason": "감독 진단 기록 없음", "events": []}

    # 수집 구간 안의 줄인지(구간을 모르면 모두 안으로 본다)
    def inside(row):
        mono = row.get("mono")

        return window is None or (finite_number(mono) and window[0] <= mono <= window[1])

    # 사건 하나를 벽시계·sim만·미확인으로 나눈다.
    def classify(entry, wall_by_reason=False):
        timeout = entry.get("timeout")

        if not inside(entry):
            outside.append(entry)
        elif wall_by_reason or (timeout or {}).get("wall_expired"):
            events.append(entry)
        elif timeout:
            sim_only.append(entry)
        else:
            unverified.append(entry)

    events, sim_only, unverified, outside = [], [], [], []
    finished = onsets = 0
    previous = None

    for row in rows:
        if row.get("kind") == "reinit_finished" and row.get("result") in WALL_TIMEOUT_CODES:
            finished += inside(row)
            classify({"source": "reinit_finished", "sim": row.get("sim"), "mono": row.get("mono"),
                      "result": row.get("result"), "timeout": row.get("timeout")})

        for event in row.get("estimate_events") or []:
            if event.get("event") == "INTENT_CLOSED" and event.get("reason") in ("TIMEOUT", "TIMEOUT_WALL"):
                classify({"source": "INTENT_CLOSED", "sim": row.get("sim"), "mono": row.get("mono"),
                          "reason": event.get("reason"), "timeout": event.get("timeout")},
                         event.get("reason") == "TIMEOUT_WALL")

        # tick의 재위치 실패 시작 수(세부 진단 줄이 없는 옛 기록 대조용)
        if row.get("kind") == "tick":
            failure = row.get("reinit_failure")
            onsets += failure in WALL_TIMEOUT_CODES and failure != previous and inside(row)
            previous = failure

    problems = [f"{entry['source']} {entry.get('result') or entry.get('reason')}(sim {entry.get('sim')}): "
                "어느 시계 제한인지 기록 없음" for entry in unverified]

    if onsets > finished:
        problems.append(f"tick 재위치 시간초과 실패 {onsets}회 중 {onsets - finished}회는 종료 원인 기록 없음(옛 기록)")

    problems += tick_coverage_problems(rows, window, health_rows)

    if broken_lines:
        problems.append(f"감독 진단 깨진 줄 {broken_lines}개")

    limited = True if events else (None if problems else False)

    return {"limited": limited, "level": "confirmed" if limited is not None else "unverified",
            "reason": "; ".join(problems), "events": events, "sim_only_timeouts": sim_only,
            "outside_window": outside, "window_mono": window}


# 감독 진단 tick이 수집 구간을 빈틈없이 덮는지 본다. 반환: 문제 문자열 목록(비면 연속성 확인)
def tick_coverage_problems(rows, window, health_rows=None, max_gap=SUPERVISOR_TICK_MAX_GAP_S):
    monos = [row.get("mono") for row in rows if row.get("kind") == "tick"]

    if not monos:
        return ["감독 진단 tick 없음"]

    if not all(finite_number(mono) for mono in monos):
        return ["감독 진단 tick mono 비유한"]

    if any(later < earlier for earlier, later in zip(monos, monos[1:])):
        return ["감독 진단 tick mono 역행"]

    if window is None or not all(finite_number(value) for value in window):
        return ["수집 구간(mono) 미상: tick 연속성 확인 불가"]

    start, end = window
    problems = []
    gaps = [(earlier, later) for earlier, later in zip(monos, monos[1:])
            if later > start and earlier < end and later - earlier > max_gap]

    if gaps:
        problems.append(f"감독 진단 tick 공백 {len(gaps)}곳(> {max_gap} s, 최대 "
                        f"{max(later - earlier for earlier, later in gaps):.3f} s, 첫 공백 mono {gaps[0][0]:.3f})")

    if not any(end - max_gap <= mono <= end + max_gap for mono in monos):
        problems.append(f"감독 진단이 수집 끝(mono {end:.3f}) 근처에 없음")

    if monos[0] > start + max_gap:
        received = [row.get("recv_mono") for row in health_rows or [] if finite_number(row.get("recv_mono"))]
        sessions = {row.get("session") for row in rows if row.get("session")}
        foreign = {row.get("session") for row in health_rows or []} - sessions

        if not received:
            problems.append(f"수집 시작 뒤 감독 진단 시작(첫 tick mono {monos[0]:.3f}): health 기록이 없어 그 전 미실행 증명 불가")
        elif min(received) < monos[0]:
            problems.append(f"수집 시작 뒤 감독 진단 시작: 첫 tick({monos[0]:.3f}) 전에 health 수신({min(received):.3f})")
        elif foreign:
            problems.append(f"수집 시작 뒤 감독 진단 시작: 진단에 없는 health 세션 {sorted(map(str, foreign))}")

    return problems


# 수집기·로봇별 감독 진단을 합쳐 회차 하나의 LD-3 근거를 만든다. 한 회차에 사건이 여럿이어도 회차는 1이다.
def wall_clock_summary(collector, supervisors):
    parts = [("collector", collector)]
    parts += [(f"supervisor:{robot}", status) for robot, status in sorted(supervisors.items())]
    events = []

    if collector.get("limited"):
        events.append({"source": "collector", "detail": collector.get("detail")})

    for name, status in parts[1:]:
        events += [{**event, "robot": name.split(":", 1)[1]} for event in status.get("events") or []]

    unverified = [f"{name}: {status.get('reason') or status.get('detail')}" for name, status in parts
                  if status.get("limited") is None]
    limited = True if events else (None if unverified else False)

    return {"limited": limited, "level": "confirmed" if limited is not None else "unverified",
            "events": events, "unverified": unverified, "collector": collector, "supervisors": supervisors,
            "method": WALL_CLOCK_METHOD}


# 회차 LD-3 항목: 벽시계 제한 사건 유무(0·1)를 기준 상한과 비교한다. 확인 불가면 not_judged
def wall_clock_item(summary, limit):
    metric = "load.max_wall_clock_limited_runs"

    if summary["limited"] is None:
        item = fixed_item("LD-3", metric, NOT_JUDGED, "벽시계 제한 확인 불가: " + "; ".join(summary["unverified"]),
                          limit=limit)
    else:
        count = 1 if summary["limited"] else 0
        item = criteria_item("LD-3", metric, count, limit, "max",
                             reason=f"이 회차 벽시계 제한 사건 {len(summary['events'])}건(회차 수 {count})")

    item["wall_clock"] = {key: summary[key] for key in ("limited", "level", "events", "unverified")}

    return item


# ---- B2-L4~L7 사건 지표(원시 기록·감독 진단). 실시간·재판정·묶음이 같은 순수 함수를 쓴다 ----
# 사건 시험 항목: 정상 주행에서 사건이 없으면 not_applicable이고, 회차 전체 PASS를 만들지 않는다(묶음 coverage에서 미충족).
EVENT_TEST_IDS = ("B2-L5", "B2-L6", "B2-L7")
# 기준 파일 결과 이름 → 감독 노드 실제 응답 코드(명시적·제한된 대응표). 부분 문자열로 맞추지 않는다.
REINIT_RESULT_CODES = {"REJECTED_NOT_STOPPED": "REINIT_REJECTED_NOT_STOPPED",
                       "NOT_NEEDED": "REINIT_REJECTED_NOT_NEEDED"}
# 거절이지만 시험 전제가 성립하지 않은 응답(이동 거절·불필요 거절 성공으로 세지 않는다)
REINIT_PREMISE_CODES = {"REINIT_REJECTED_ACTUATION_STALE": "A4 구동 상태 미수신·만료",
                        "REINIT_REJECTED_CLOCK_INVALID": "sim time 무효"}
# 재위치 종료 중 시험 전제가 깨진 중단(정상 완료 실패와 구분한다)
REINIT_ABORT_RESULTS = ("REINIT_FAILED_NOT_STOPPED", "REINIT_FAILED_CLOCK_INVALID")
ACTUATION_STOPPED = 4
STUB_ACTUATION_SOURCE = "stub_actuation_state"
# 오차 시각과 odom 표본 대응 허용(sim s)
POSE_ODOM_MATCH_S = 0.1
# 같은 감독 tick 허용 오차(sim s). health stamp는 ns 절삭(Time(nanoseconds=int(now·1e9)))이라 같은 tick의 진단 sim보다
# 최대 1 ns 작을 수 있다. 재위치 성공 tick에 발행한 OK를 "성공 전"으로 세지 않게 한다.
SAME_TICK_S = 1e-6
OVERCONFIDENCE_METHOD = ("오차 표본: pose_errors와 같은 정의(odom 시각 기준 0.5 s 간격, map→odom 유효 시각 ≤ odom 시각 중 마지막, "
                         "truth 보간 최대 공백 0.1 s, T_world_map 평가 전용), 원시 정밀도(반올림 없음). "
                         "health 연결: 표본 시각 이하 마지막 health(같은 stamp 재발행은 한 표본), 만료 1.0 s(D-05). "
                         "과신 = 오차 > 기준(초과)이고 state=OK·meta.valid=true")
DISCONTINUITY_METHOD = ("사건: AMCL lifecycle active 이탈(수집기 amcl_lifecycle recv_sim·감독 amcl_transition 수신 sim), "
                        "감독 LATCHED(입력 stamp), 원시 TF로 같은 odom 자세에서 합성한 추정 도약(감독 INTENDED 제외). "
                        "health 저하 시각은 사건 시작으로 쓰지 않는다. 한 사건이 열려 있는 동안(재위치 성공 전) 생긴 사건은 같은 사건. "
                        "지연 = 사건 시작 → 첫 비(OK·valid) health stamp(sim). 조기 복귀 = OK 이탈 뒤 같은 세션 재위치 SUCCEEDED 전 "
                        "유효 OK health 표본 수")
RELOCALIZATION_METHOD = ("시도: 감독 reinit_request REINIT_ACCEPTED ↔ 같은 세션 reinit_finished(attempt 번호, 없으면 순서 대응). "
                         "완료 시간 = 종료 sim − 접수 sim. 정확도 = SUCCEEDED 뒤 같은 세션 첫 유효 OK health 시각의 "
                         "평가용 위치 오차(그 시점 값, 이후 주행 정확도 보장 아님)")
REJECTION_METHOD = ("요청 조건: 요청 시각 odom(≤ 0.5 s 전 표본) 속도가 D-06 초과면 이동, 감독이 판단에 쓴 health가 OK·valid면 OK. "
                    "OK이면서 이동이면 서버 우선순위(시계 → NOT_NEEDED → A4 만료 → NOT_STOPPED)대로 NOT_NEEDED를 기대하고 "
                    "두 조건 어느 쪽 충족에도 세지 않는다")


# health 원시 기록을 (stamp, state, valid, session, reason) 표본으로 만든다. 다른 로봇 줄은 뺀다.
def health_samples(rows, robot_id=None):
    def parse(row):
        stamp, state = row.get("stamp"), row.get("state")

        if not finite_number(stamp) or not isinstance(state, str):
            return None

        return stamp, state, row.get("valid") is True, row.get("session"), row.get("reason")

    own = [row for row in rows if robot_id is None or row.get("robot_id") in (None, robot_id)]

    return ordered_samples(own, parse, "health")


# 시각 t의 health(그 시각 이하 마지막 표본, 만료 전). 없거나 만료면 None
def health_at(samples, stamps, t, hold_max_s=HEALTH_HOLD_MAX_S):
    index = bisect.bisect_right(stamps, t) - 1

    if index < 0 or t - samples[index][0] >= hold_max_s:
        return None

    stamp, state, valid, session, reason = samples[index]

    return {"index": index, "stamp": stamp, "state": state, "valid": valid, "session": session, "reason": reason,
            "ok": state == "OK" and valid}


# 시각 t의 평가용 위치 오차. odom은 t 이하 POSE_ODOM_MATCH_S 안의 마지막 표본, 그 odom 시각으로 추정·truth를 맞춘다.
# 반환: (오차 dict 또는 None, 사유)
def pose_error_at(t, tf_rows, odom_rows, truth_rows, world_map):
    if world_map is None:
        return None, "T_world_map 기록 없음"

    odom = sorted((row["stamp"], row["x"], row["y"], row["yaw"]) for row in odom_rows
                  if all(finite_number(row.get(key)) for key in ("stamp", "x", "y", "yaw")))
    index = bisect.bisect_right([row[0] for row in odom], t) - 1

    if index < 0 or t - odom[index][0] > POSE_ODOM_MATCH_S:
        return None, f"시각 {t:.3f} 근처(≤ {POSE_ODOM_MATCH_S} s) odom 표본 없음"

    sample = odom[index]
    earlier = [row for row in tf_rows if finite_number(row.get("valid_time")) and row["valid_time"] <= sample[0]]

    if not earlier:
        return None, "그 시각 이전 map→odom 없음"

    truth = sorted((row["stamp"], row["x"], row["y"], row["yaw"]) for row in truth_rows)
    actual = interpolate_pose([row[0] for row in truth], [row[1:] for row in truth], sample[0],
                              TRUTH_MAX_GAP_S) if truth else None

    if actual is None:
        return None, f"truth 보간 불가(공백 > {TRUTH_MAX_GAP_S} s 또는 범위 밖)"

    last = max(earlier, key=lambda row: row["valid_time"])
    estimate = se2(last["x"], last["y"], last["yaw"]) @ se2(*sample[1:4])
    error = se2_params(np.linalg.inv(np.linalg.inv(world_map) @ se2(*actual)) @ estimate)

    return {"odom_sim": sample[0], "requested_sim": t, "map_odom_valid_time": last["valid_time"],
            "position_error_m": math.hypot(*error[:2]), "yaw_error_rad": abs(error[2])}, ""


# 결과 conditions.world_map(x_m, y_m, yaw_rad)을 행렬로 읽는다. 없으면 None
def recorded_world_map(result):
    value = (result.get("conditions") or {}).get("world_map") or {}
    triple_value = (value.get("x_m"), value.get("y_m"), value.get("yaw_rad"))

    return se2(*triple_value) if all(finite_number(item) for item in triple_value) else None


# B2-L4 과신: 오차가 기준을 넘는 표본에서 유효 OK health 수. status: measured·not_judged
def overconfidence_record(health_rows, tf_rows, odom_rows, truth_rows, world_map, threshold, robot_id=None):
    record = {"method": OVERCONFIDENCE_METHOD, "threshold_m": threshold, "period_s": SAMPLE_PERIOD_S,
              "truth_max_gap_s": TRUTH_MAX_GAP_S, "health_hold_max_s": HEALTH_HOLD_MAX_S,
              "truth_use": "오프라인 정확도 판정 전용(런타임 health 입력 아님)"}

    if world_map is None:
        return {**record, "status": NOT_JUDGED, "reason": "T_world_map 기록 없음(오차 계산 불가)"}

    if not finite_number(threshold):
        return {**record, "status": NOT_JUDGED, "reason": f"기준값 없음·비유한: {threshold}"}

    errors = pose_errors([(row["valid_time"], row["x"], row["y"], row["yaw"]) for row in tf_rows],
                         [(row["stamp"], row["x"], row["y"], row["yaw"], row["v"], row["w"]) for row in odom_rows],
                         [(row["stamp"], row["x"], row["y"], row["yaw"]) for row in truth_rows], world_map,
                         round_digits=None)

    if errors is None:
        return {**record, "status": NOT_JUDGED, "reason": "오차 표본 없음(map→odom·odom·truth 원시 기록 부족)"}

    samples, _, anomalies = health_samples(health_rows, robot_id)
    stamps = [sample[0] for sample in samples]
    exceeded = []

    for row in errors["series"]["rows"]:
        if row[7] > threshold:
            health = health_at(samples, stamps, row[0])
            keys = ("stamp", "state", "valid", "session", "reason")
            exceeded.append({"sim": row[0], "position_error_m": row[7], "yaw_error_rad": abs(row[8]),
                             "health": None if health is None else {key: health[key] for key in keys}})

    def count(test):
        return sum(1 for entry in exceeded if test(entry["health"]))

    record.update(samples=errors["samples"], exceeded=len(exceeded),
                  overconfident=count(lambda health: health is not None and health["state"] == "OK" and health["valid"]),
                  invalid_ok=count(lambda health: health is not None and health["state"] == "OK" and not health["valid"]),
                  unknown_health=count(lambda health: health is None), exceeded_samples=exceeded[:50],
                  max_position_error_m=errors["position_error_m"]["max"], health_anomalies=anomalies)

    if not samples:
        record.update(status=NOT_JUDGED, reason="health 원시 기록 없음")
    elif record["overconfident"]:
        record.update(status="measured", reason="")
    elif record["unknown_health"]:
        record.update(status=NOT_JUDGED, reason=f"초과 표본 중 health 확인 불가 {record['unknown_health']}개(만료·없음)")
    else:
        record.update(status="measured", reason="")

    return record


# B2-L4 항목 하나(로봇별). 확인된 과신은 다른 자료 부족과 관계없이 FAIL로 남는다.
def overconfidence_item(robot_id, record, limit):
    metric = "overconfidence.max_ok_samples_above_threshold"

    if record.get("status") == "measured":
        exposure = (f"초과 표본 {record['exceeded']}개(위험 조건 노출 {record['exceeded']})" if record["exceeded"]
                    else "초과 표본 0개: 관측 구간 과신 없음(위험 조건 미노출)")
        note = f"; 확인 불가 {record['unknown_health']}개" if record.get("unknown_health") else ""
        item = criteria_item("B2-L4", metric, record["overconfident"], limit, "max", robot_id,
                             f"오차 표본 {record['samples']}개, {exposure}, 과신 {record['overconfident']}개{note}")
    else:
        item = fixed_item("B2-L4", metric, NOT_JUDGED, record.get("reason") or "근거 없음", robot_id, limit)

    item["overconfidence"] = record

    return item


# 원시 TF에서 같은 odom 자세로 합성한 추정 자세 도약을 찾는다(map→odom 병진 자체가 아니라 base 추정 변화).
# intended: 감독 INTENDED valid_time 목록(의도된 초기화·재위치 갱신은 사건이 아니다). 반환: (사건, 대응 odom 없는 큰 변화)
def raw_estimate_jumps(tf_rows, odom_rows, limits, intended):
    if not tf_rows:
        return [], []

    steps = map_odom_steps(tf_rows, odom_rows)
    events, unmatched = [], []

    for change in steps["changed"]:
        valid_time = change.get("valid_time")
        intent = any(finite_number(valid_time) and abs(valid_time - value) < 1e-6 for value in intended)

        if change["estimate_m"] is None:
            if change["map_odom_m"] > limits["position_m"] or change["map_odom_rad"] > limits["yaw_rad"]:
                unmatched.append(change)
        elif not intent and (change["estimate_m"] > limits["position_m"] or change["estimate_rad"] > limits["yaw_rad"]):
            events.append({"source": "tf_estimate_jump(원시 TF·odom 합성)", "start": valid_time,
                           "estimate_m": change["estimate_m"], "estimate_rad": change["estimate_rad"],
                           "map_odom_m": change["map_odom_m"], "map_odom_rad": change["map_odom_rad"]})

    return events, unmatched


# 사건 후보 목록(시간순): lifecycle 이탈·감독 LATCHED·원시 추정 도약. 반환: (사건, 문제)
def discontinuity_candidates(lifecycle_rows, supervisor_rows, tf_rows, odom_rows, limits):
    events, problems = [], []

    for row in lifecycle_rows or []:
        if row.get("start_state") == "active" and row.get("goal_state") != "active":
            events.append({"source": "amcl_lifecycle(수집기)", "start": row.get("recv_sim"),
                           "detail": f"{row.get('start_state')}→{row.get('goal_state')}"})

    intended = []

    for row in supervisor_rows or []:
        if row.get("kind") == "amcl_transition" and row.get("start") == "active" and row.get("goal") != "active":
            events.append({"source": "amcl_transition(감독 수신)", "start": row.get("sim"), "session": row.get("session"),
                           "detail": f"{row.get('start')}→{row.get('goal')}"})

        for event in row.get("estimate_events") or []:
            if event.get("event") == "LATCHED":
                events.append({"source": f"LATCHED:{event.get('source')}(입력 stamp)", "start": event.get("stamp"),
                               "session": row.get("session"), "detail": {key: event.get(key) for key in
                                                                         ("jump_m", "jump_rad", "detail")}})
            elif event.get("event") == "INTENDED" and finite_number(event.get("valid_time")):
                intended.append(event["valid_time"])

    jumps, unmatched = raw_estimate_jumps(tf_rows, odom_rows, limits, intended) if limits else ([], [])
    events += jumps

    if unmatched:
        problems.append(f"odom 대응 없는 큰 map→odom 변화 {len(unmatched)}건(추정 도약 여부 확인 불가)")

    missing = [event for event in events if not finite_number(event.get("start"))]

    if missing:
        problems.append(f"사건 시각 누락 {len(missing)}건({', '.join(sorted({event['source'] for event in missing}))})")

    return sorted((event for event in events if finite_number(event.get("start"))), key=lambda event: event["start"]), \
        problems


# 감독 진단의 재위치 성공 목록 [(sim, session, attempt, 출처)]. 종료 진단이 없는 옛 기록은 RELEASED를 쓴다.
def reinit_successes(supervisor_rows):
    rows = supervisor_rows or []
    finished = [row for row in rows if row.get("kind") == "reinit_finished"]

    if finished:
        return [(row.get("sim"), row.get("session"), row.get("attempt"), "reinit_finished SUCCEEDED")
                for row in finished if row.get("result") == "SUCCEEDED" and finite_number(row.get("sim"))]

    return [(row.get("sim"), row.get("session"), None, "RELEASED(옛 기록, 종료 진단 없음)") for row in rows
            if finite_number(row.get("sim")) and any(event.get("event") == "RELEASED"
                                                     for event in row.get("estimate_events") or [])]


# 사건 하나의 OK 이탈 지연·재위치 성공 전 OK 복귀를 구한다.
def discontinuity_incident(incident, samples, stamps, successes, limit_s):
    start = incident["start"]
    at_start = health_at(samples, stamps, start)

    if at_start is None:
        return {**incident, "status": NOT_JUDGED, "reason": "사건 시각 health 확인 불가(없음·만료)"}

    session = at_start["session"]
    incident.update(state_at_start={key: at_start[key] for key in ("stamp", "state", "valid", "reason", "session")})
    exit_at = exit_state = None
    covered_until = start

    if not at_start["ok"]:
        exit_at, exit_state = start, at_start["state"]
        incident["already_not_ok"] = f"사건 시각에 이미 OK·valid 아님(state {at_start['state']}): 지연 0은 감지 지연이 아님"
    else:
        previous = samples[at_start["index"]]
        covered_until = previous[0] + HEALTH_HOLD_MAX_S

        for sample in samples[at_start["index"] + 1:]:
            if sample[0] - previous[0] >= HEALTH_HOLD_MAX_S or sample[3] != previous[3]:
                covered_until = previous[0] + HEALTH_HOLD_MAX_S if sample[3] == previous[3] else sample[0]
                break

            if not (sample[1] == "OK" and sample[2]):
                exit_at, exit_state = sample[0], ("OK_INVALID" if sample[1] == "OK" else sample[1])
                break

            previous = sample
            covered_until = sample[0] + HEALTH_HOLD_MAX_S

    if exit_at is not None:
        incident.update(exit_sim=exit_at, exit_state=exit_state, delay_s=exit_at - start, delay_level="confirmed")
    elif covered_until - start > limit_s:
        incident.update(delay_s=covered_until - start, delay_level="lower_bound",
                        delay_note="OK가 기준 시간 넘게 이어진 뒤 기록 공백·끝(하한)")
    else:
        incident.update(delay_s=None, delay_level="unverified",
                        delay_note="기준 시간 안에 health 공백·세션 변경·기록 끝(이탈 확인 불가)")

    # 재위치 성공 전 OK 복귀: 이탈 뒤 같은 세션 첫 성공까지 유효 OK 표본 수(다른 세션 성공은 쓰지 않는다)
    others = [entry for entry in successes if entry[0] > start and entry[1] != session]
    closing = min((entry for entry in successes if entry[0] > start and entry[1] == session), default=None)
    incident.update(success=None if closing is None else {"sim": closing[0], "attempt": closing[2],
                                                          "source": closing[3]},
                    ignored_other_session_successes=len(others))

    if exit_at is None:
        incident.update(early_ok=None, early_level="unverified", early_note="OK 이탈 미확인")
        return incident

    end = closing[0] - SAME_TICK_S if closing else math.inf
    window = [sample for sample in samples if exit_at < sample[0] < end]
    early = [sample[0] for sample in window if sample[3] == session and sample[1] == "OK" and sample[2]]
    incident.update(early_ok=len(early), early_ok_first=early[0] if early else None,
                    invalid_ok=sum(1 for sample in window if sample[1] == "OK" and not sample[2]),
                    foreign_session_samples=sum(1 for sample in window if sample[3] != session))

    if early:
        incident["early_level"] = "confirmed"
    elif closing is None:
        incident.update(early_level="truncated", early_note="같은 세션 재위치 성공 전 기록 끝(감시 구간 잘림)")
    else:
        incident["early_level"] = "confirmed"

    return incident


# B2-L5 근거(로봇별). source_ok: 사건 출처가 연속으로 기록됐는지(감독 tick 연속성 또는 수집기 lifecycle 기록)
def discontinuity_record(health_rows, lifecycle_rows, supervisor_rows, tf_rows, odom_rows, limits, limit_s,
                         source_problems, robot_id=None):
    candidates, problems = discontinuity_candidates(lifecycle_rows, supervisor_rows, tf_rows, odom_rows, limits)
    samples, _, anomalies = health_samples(health_rows, robot_id)
    stamps = [sample[0] for sample in samples]
    successes = reinit_successes(supervisor_rows)
    incidents = []

    for event in candidates:
        current = incidents[-1] if incidents else None
        closed = current is not None and current.get("success") is not None and event["start"] > current["success"]["sim"]

        if current is not None and not closed and event["start"] >= current["start"]:
            current["related"].append({"source": event["source"], "start": event["start"]})
            continue

        incident = {"start": event["start"], "source": event["source"], "detail": event.get("detail"), "related": []}
        incidents.append(discontinuity_incident(incident, samples, stamps, successes, limit_s) if finite_number(limit_s)
                         else {**incident, "status": NOT_JUDGED, "reason": f"기준값 없음: {limit_s}"})

    return {"method": DISCONTINUITY_METHOD, "incidents": incidents, "problems": problems + list(source_problems),
            "health_anomalies": anomalies, "jump_limits": limits,
            "sources": {"amcl_lifecycle": lifecycle_rows is not None, "supervisor": supervisor_rows is not None,
                        "tf": bool(tf_rows)}}


# B2-L5 항목 두 개(로봇별): OK 이탈 지연 최대, 재위치 성공 전 OK 복귀 수 최대
def discontinuity_items(robot_id, record, limits):
    incidents = record.get("incidents") or []
    problems = record.get("problems") or []
    specs = (("discontinuity.max_ok_exit_delay_s", limits.get("max_ok_exit_delay_s"), "delay_s", "delay_level"),
             ("discontinuity.max_ok_before_relocalization", limits.get("max_ok_before_relocalization"),
              "early_ok", "early_level"))
    items = []

    for metric, limit, key, level in specs:
        known = [entry[key] for entry in incidents if entry.get(level) in ("confirmed", "lower_bound")]
        unknown = [entry for entry in incidents if entry.get(level) not in ("confirmed", "lower_bound")]

        if not incidents:
            item = (fixed_item("B2-L5", metric, NOT_JUDGED, "사건 출처 확인 불가: " + "; ".join(problems), robot_id, limit)
                    if problems else
                    fixed_item("B2-L5", metric, NOT_APPLICABLE, "사건 없음(lifecycle 이탈·도약 미노출, 도약 감지 시험 아님)",
                               robot_id, limit))
        else:
            item = criteria_item("B2-L5", metric, max(known) if known else None, limit, "max", robot_id,
                                 f"사건 {len(incidents)}건 중 확인 {len(known)}건")

            if item["result"] == PASS and (unknown or problems):
                notes = [entry.get("reason") or entry.get("delay_note") or entry.get("early_note") or "확인 불가"
                         for entry in unknown]
                item.update(result=NOT_JUDGED, reason="확인 불가 사건·출처: " + "; ".join(notes + problems))
            elif item["result"] == NOT_JUDGED and unknown:
                notes = [entry.get("reason") or entry.get("delay_note") or entry.get("early_note") or "확인 불가"
                         for entry in unknown]
                item["reason"] = "확인 불가: " + "; ".join(notes)

        item["discontinuity"] = {"incidents": incidents, "problems": problems, "method": record.get("method")}
        items.append(item)

    return items


# 재위치 시도 목록: 접수 ↔ 종료(같은 세션, attempt 번호가 있으면 번호, 없으면 순서). 옛 기록은 RELEASED로 성공만 본다.
def relocalization_attempts(supervisor_rows):
    rows = supervisor_rows or []
    attempts = []
    finished = [row for row in rows if row.get("kind") == "reinit_finished"]

    for row in rows:
        if row.get("kind") == "reinit_request" and row.get("accepted") is True and row.get("code") == "REINIT_ACCEPTED":
            attempts.append({"accepted_sim": row.get("sim"), "accepted_mono": row.get("mono"),
                             "session": row.get("session"), "attempt": row.get("attempt"), "finish": None})

    used = set()

    for attempt in attempts:
        for index, row in enumerate(finished):
            if index in used or row.get("session") != attempt["session"]:
                continue

            if attempt["attempt"] is not None and row.get("attempt") is not None:
                matched, basis = row.get("attempt") == attempt["attempt"], "attempt 번호"
            else:
                matched, basis = finite_number(row.get("sim")) and row["sim"] >= attempt["accepted_sim"], "순서 대응"

            if matched:
                used.add(index)
                attempt["finish"] = {"sim": row.get("sim"), "result": row.get("result"), "timeout": row.get("timeout"),
                                     "basis": basis}
                break

    if not finished:
        for attempt in attempts:
            released = [row.get("sim") for row in rows if row.get("session") == attempt["session"]
                        and finite_number(row.get("sim")) and row["sim"] >= attempt["accepted_sim"]
                        and any(event.get("event") == "RELEASED" for event in row.get("estimate_events") or [])]

            if released:
                attempt["finish"] = {"sim": released[0], "result": "SUCCEEDED", "timeout": None,
                                     "basis": "RELEASED(옛 기록, 종료 진단 없음)"}

    return attempts


# B2-L6 근거(로봇별): 시도별 완료 시간·결과와 성공 뒤 첫 유효 OK의 위치 오차
def relocalization_record(supervisor_rows, health_rows, tf_rows, odom_rows, truth_rows, world_map, record_end,
                          robot_id=None):
    samples, _, _ = health_samples(health_rows, robot_id)
    attempts = relocalization_attempts(supervisor_rows)

    for attempt in attempts:
        finish = attempt["finish"]

        if finish is None:
            attempt.update(outcome="TRUNCATED", elapsed_s=(record_end - attempt["accepted_sim"])
                           if finite_number(record_end) else None)
            continue

        attempt["elapsed_s"] = finish["sim"] - attempt["accepted_sim"]
        result = finish["result"]
        attempt["outcome"] = ("SUCCEEDED" if result == "SUCCEEDED" else "ABORTED" if result in REINIT_ABORT_RESULTS
                              else "FAILED")

        if result != "SUCCEEDED":
            continue

        first_ok = next((sample for sample in samples if sample[0] >= finish["sim"] - SAME_TICK_S
                         and sample[3] == attempt["session"]
                         and sample[1] == "OK" and sample[2]), None)

        if first_ok is None:
            attempt["accuracy"] = {"status": NOT_JUDGED, "reason": "성공 뒤 같은 세션 유효 OK 없음(기록 끝)"}
            continue

        error, reason = pose_error_at(first_ok[0], tf_rows, odom_rows, truth_rows, world_map)
        attempt["accuracy"] = ({"status": "measured", "ok_sim": first_ok[0], **error} if error
                               else {"status": NOT_JUDGED, "ok_sim": first_ok[0], "reason": reason})

    return {"method": RELOCALIZATION_METHOD, "attempts": attempts, "record_end_sim": record_end,
            "supervisor": supervisor_rows is not None}


# B2-L6 항목 두 개(로봇별): 완료 시간 최대, 성공 뒤 위치 오차 최대. 실패·시간초과는 FAIL, 중단·절단은 not_judged
def relocalization_items(robot_id, record, limits, a4_path):
    attempts = record.get("attempts") or []
    completion_limit = limits.get("max_completion_s")
    accuracy_limit = limits.get("max_position_error_after_ok_m")
    detail = {"attempts": attempts, "method": record.get("method"), "a4_path": a4_path}

    if not record.get("supervisor"):
        items = [fixed_item("B2-L6", metric, NOT_JUDGED, "감독 진단 기록 없음", robot_id, limit)
                 for metric, limit in (("relocalization.max_completion_s", completion_limit),
                                       ("relocalization.max_position_error_after_ok_m", accuracy_limit))]
    elif not attempts:
        items = [fixed_item("B2-L6", metric, NOT_APPLICABLE, "재위치 접수 시도 없음", robot_id, limit)
                 for metric, limit in (("relocalization.max_completion_s", completion_limit),
                                       ("relocalization.max_position_error_after_ok_m", accuracy_limit))]
    else:
        failed = [entry for entry in attempts if entry["outcome"] == "FAILED"
                  or (entry["outcome"] == "TRUNCATED" and finite_number(entry.get("elapsed_s"))
                      and finite_number(completion_limit) and entry["elapsed_s"] > completion_limit)]
        pending = [entry for entry in attempts if entry not in failed and entry["outcome"] in ("ABORTED", "TRUNCATED")]
        done = [entry["elapsed_s"] for entry in attempts if entry["outcome"] == "SUCCEEDED"]
        values = done + [entry["elapsed_s"] for entry in failed if finite_number(entry.get("elapsed_s"))]
        completion = criteria_item("B2-L6", "relocalization.max_completion_s", max(values) if values else None,
                                   completion_limit, "max", robot_id,
                                   f"시도 {len(attempts)}건(성공 {len(done)}, 실패·시간초과 {len(failed)}, "
                                   f"중단·절단 {len(pending)})")

        if failed and finite_number(completion_limit):
            completion.update(result=FAIL, reason=completion["reason"] + ": 정상 완료 못한 시도 "
                              + ", ".join(f"{entry['outcome']}({(entry['finish'] or {}).get('result')})" for entry in failed))
        elif completion["result"] == PASS and pending:
            completion.update(result=NOT_JUDGED, reason=completion["reason"] + ": 중단·기록 절단 시도가 있어 확정 안 함")

        measured = [entry["accuracy"] for entry in attempts if (entry.get("accuracy") or {}).get("status") == "measured"]
        unknown = [entry for entry in attempts if entry["outcome"] == "SUCCEEDED"
                   and (entry.get("accuracy") or {}).get("status") != "measured"]
        worst = max((entry["position_error_m"] for entry in measured), default=None)
        accuracy = criteria_item("B2-L6", "relocalization.max_position_error_after_ok_m", worst, accuracy_limit, "max",
                                 robot_id, f"성공 {len(done)}건 중 오차 확인 {len(measured)}건(성공 뒤 첫 유효 OK 시점 값)")

        if not done:
            accuracy.update(result=NOT_JUDGED, reason="성공한 재위치 시도 없음(정확도 판정 불가)")
        elif accuracy["result"] == PASS and unknown:
            accuracy.update(result=NOT_JUDGED, reason=accuracy["reason"] + ": 확인 불가 "
                            + "; ".join(entry["accuracy"].get("reason", "") for entry in unknown))

        items = [completion, accuracy]

    for item in items:
        item["relocalization"] = detail

    return items


# 시각 t 이하 마지막 행(rows는 key 시각 오름차순). 없으면 None
def last_row_at(rows, key, t):
    index = bisect.bisect_right([row[key] for row in rows], t) - 1

    return rows[index] if index >= 0 else None


# 구동 상태 원시 기록의 출처: stub·실제 A4·없음, 유효 STOPPED 제공 여부
def actuation_path(actuation_rows):
    if actuation_rows is None:
        return {"basis": "none", "stopped_provided": False, "real_a4_path": "검증 불가(구동 상태 원시 기록 없음)"}

    sources = sorted({str(row.get("source_id")) for row in actuation_rows})
    stopped = any(row.get("state") == ACTUATION_STOPPED and row.get("valid") is True for row in actuation_rows)
    basis = ("stub" if sources == [STUB_ACTUATION_SOURCE] else "real" if STUB_ACTUATION_SOURCE not in sources
             else "mixed")
    path = ("stub 근거(실제 A4 경로 아님)" if basis == "stub" else
            "실제 A4 경로" if basis == "real" and stopped else
            "검증 불가(실제 A4가 유효 STOPPED를 제공하지 않음)" if basis == "real" else "검증 불가(stub·A4 혼재)")

    return {"basis": basis, "sources": sources, "stopped_provided": stopped, "real_a4_path": path}


# B2-L7 근거(로봇별): 재위치 요청마다 요청 조건(이동·OK)과 응답을 같은 로봇·세션 기록으로 대응한다.
def rejection_record(supervisor_rows, odom_rows, actuation_rows, criteria_relocalization):
    odom = sorted((row for row in odom_rows if all(finite_number(row.get(key)) for key in ("stamp", "v", "w"))),
                  key=lambda row: row["stamp"])
    actuation = sorted((row for row in actuation_rows or [] if finite_number(row.get("stamp"))),
                       key=lambda row: row["stamp"])
    expected = {"moving": REINIT_RESULT_CODES.get(criteria_relocalization.get("moving_request_result")),
                "ok": REINIT_RESULT_CODES.get(criteria_relocalization.get("ok_request_result"))}
    expected["ok_moving"] = expected["ok"]
    requests = []

    for row in supervisor_rows or []:
        if row.get("kind") != "reinit_request":
            continue

        t = row.get("sim")
        sample = last_row_at(odom, "stamp", t) if finite_number(t) else None
        moving = None

        if sample is not None and t - sample["stamp"] <= ODOM_HOLD_MAX_S:
            moving = abs(sample["v"]) > D06_LINEAR_M_S or abs(sample["w"]) > D06_ANGULAR_RAD_S

        ok = row.get("state") == "OK" and row.get("valid") is True
        drive = last_row_at(actuation, "stamp", t) if finite_number(t) and actuation else None
        category = ("unknown" if moving is None or not isinstance(row.get("state"), str) else
                    "ok_moving" if ok and moving else "moving" if moving else "ok" if ok else "other")
        code = row.get("code")
        want = expected.get(category)

        if category in ("unknown", "other"):
            outcome = "not_classified"
        elif row.get("accepted") is True:
            outcome = "wrong_accept"
        elif code == want:
            outcome = "correct"
        elif code in REINIT_PREMISE_CODES:
            outcome = "premise_unmet"
        else:
            outcome = "wrong_reason"

        requests.append({"sim": t, "session": row.get("session"), "accepted": row.get("accepted"), "code": code,
                         "health": {"state": row.get("state"), "valid": row.get("valid")},
                         "odom": None if sample is None else {"stamp": sample["stamp"], "v": sample["v"], "w": sample["w"]},
                         "actuation": None if drive is None else {key: drive.get(key) for key in
                                                                  ("stamp", "state", "valid", "source_id")},
                         "category": category, "expected": want, "outcome": outcome})

    return {"method": REJECTION_METHOD, "requests": requests, "expected": expected,
            "name_map": REINIT_RESULT_CODES, "supervisor": supervisor_rows is not None}


# B2-L7 항목 세 개(로봇별): 이동 중 요청 결과, OK 중 요청 결과, 오판 접수 수
def rejection_items(robot_id, record, limits, a4_path):
    requests = record.get("requests") or []
    expected = record.get("expected") or {}
    detail = {"requests": requests, "method": record.get("method"), "name_map": record.get("name_map"),
              "a4_path": a4_path}
    basis = f" / A4: {a4_path['real_a4_path']}" if a4_path else ""
    items = []

    for condition, metric in (("moving", "relocalization.moving_request_result"),
                              ("ok", "relocalization.ok_request_result")):
        limit = limits.get(metric.split(".")[1])
        chosen = [entry for entry in requests if entry["category"] == condition]
        counts = {name: sum(1 for entry in chosen if entry["outcome"] == name)
                  for name in ("correct", "wrong_accept", "wrong_reason", "premise_unmet")}

        if not record.get("supervisor"):
            item = fixed_item("B2-L7", metric, NOT_JUDGED, "감독 진단 기록 없음", robot_id, limit)
        elif expected.get(condition) is None:
            item = fixed_item("B2-L7", metric, NOT_JUDGED, f"기준 결과 이름 대응 없음: {limit}", robot_id, limit)
        elif not chosen:
            item = fixed_item("B2-L7", metric, NOT_APPLICABLE, f"이 조건({condition}) 요청 없음", robot_id, limit)
        else:
            wrong = counts["wrong_accept"] + counts["wrong_reason"]
            result = FAIL if wrong else PASS if counts["correct"] else NOT_JUDGED
            reason = (f"요청 {len(chosen)}건: 정상 거절 {counts['correct']}, 오판 접수 {counts['wrong_accept']}, "
                      f"잘못된 거절 사유 {counts['wrong_reason']}, 전제 불충족 {counts['premise_unmet']}{basis}")
            item = {"id": "B2-L7", "robot": robot_id, "metric": metric, "value": counts, "limit": limit,
                    "result": result, "reason": reason if result != NOT_JUDGED else "전제 불충족 요청만 있음: " + reason}

        items.append(item)

    metric = "relocalization.max_wrong_accept_count"
    limit = limits.get("max_wrong_accept_count")
    relevant = [entry for entry in requests if entry["category"] in ("moving", "ok", "ok_moving")]
    unknown = [entry for entry in requests if entry["category"] == "unknown"]

    if not record.get("supervisor"):
        item = fixed_item("B2-L7", metric, NOT_JUDGED, "감독 진단 기록 없음", robot_id, limit)
    elif not relevant:
        item = (fixed_item("B2-L7", metric, NOT_JUDGED, f"요청 {len(unknown)}건 상태 시각 부족(odom·health)", robot_id, limit)
                if unknown else fixed_item("B2-L7", metric, NOT_APPLICABLE, "이동·OK 조건 요청 없음", robot_id, limit))
    else:
        wrong = sum(1 for entry in relevant if entry["outcome"] == "wrong_accept")
        item = criteria_item("B2-L7", metric, wrong, limit, "max", robot_id,
                             f"이동·OK 조건 요청 {len(relevant)}건 중 오판 접수 {wrong}{basis}")

        if item["result"] == PASS and unknown:
            item.update(result=NOT_JUDGED, reason=item["reason"] + f": 상태 시각 부족 요청 {len(unknown)}건")

    items.append(item)

    for item in items:
        item["rejection"] = detail

    return items


# 한 로봇의 B2-L4~L7 근거를 원시 기록 dict에서 만든다(입출력 없음). streams: health·odom·tf_map_odom·truth·
# amcl_lifecycle(없으면 None)·actuation(없으면 None)·supervisor(없으면 None)
def event_evidence(streams, world_map, criteria, jump_limits, source_problems, record_end, robot_id=None):
    health, odom, tf, truth = (streams.get(name) or [] for name in ("health", "odom", "tf_map_odom", "truth"))
    supervisor = streams.get("supervisor")
    a4_path = actuation_path(streams.get("actuation"))

    return {
        "overconfidence": overconfidence_record(health, tf, odom, truth, world_map,
                                                (criteria.get("overconfidence") or {}).get("error_threshold_m"),
                                                robot_id),
        "discontinuity": discontinuity_record(health, streams.get("amcl_lifecycle"), supervisor, tf, odom, jump_limits,
                                              (criteria.get("discontinuity") or {}).get("max_ok_exit_delay_s"),
                                              source_problems, robot_id),
        "relocalization": relocalization_record(supervisor, health, tf, odom, truth, world_map, record_end, robot_id),
        "rejection": rejection_record(supervisor, odom, streams.get("actuation"), criteria.get("relocalization") or {}),
        "a4_path": a4_path,
    }


# 한 로봇의 B2-L4~L7 항목. evidence가 없으면 모두 not_judged
def event_items(robot_id, evidence, criteria):
    over = criteria.get("overconfidence") or {}
    jump = criteria.get("discontinuity") or {}
    relocalization = criteria.get("relocalization") or {}

    if not evidence:
        reason = "원시 기록·감독 진단 근거 없음"
        return [fixed_item(item_id, metric, NOT_JUDGED, reason, robot_id)
                for item_id, metric in (("B2-L4", "overconfidence.max_ok_samples_above_threshold"),
                                        ("B2-L5", "discontinuity.max_ok_exit_delay_s"),
                                        ("B2-L5", "discontinuity.max_ok_before_relocalization"),
                                        ("B2-L6", "relocalization.max_completion_s"),
                                        ("B2-L6", "relocalization.max_position_error_after_ok_m"),
                                        ("B2-L7", "relocalization.moving_request_result"),
                                        ("B2-L7", "relocalization.ok_request_result"),
                                        ("B2-L7", "relocalization.max_wrong_accept_count"))]

    items = [overconfidence_item(robot_id, evidence["overconfidence"], over.get("max_ok_samples_above_threshold"))]
    items += discontinuity_items(robot_id, evidence["discontinuity"], jump)
    items += relocalization_items(robot_id, evidence["relocalization"], relocalization, evidence["a4_path"])
    items += rejection_items(robot_id, evidence["rejection"], relocalization, evidence["a4_path"])

    return items


# 도약 사건 임계값(감독 estimate_jump)을 설정 파일에서 읽는다. 반환: (limits 또는 None, 출처 dict)
def jump_limits_from_config(path):
    path = Path(path)

    if not path.is_file():
        return None, {"path": str(path), "sha256": None, "reason": "설정 파일 없음"}

    import yaml

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    jump = data.get("estimate_jump") or {}
    limits = {key: jump.get(key) for key in ("position_m", "yaw_rad")}

    if not all(finite_number(value) for value in limits.values()):
        return None, {"path": str(path), "sha256": file_sha256(path), "reason": "estimate_jump 값 없음"}

    return limits, {"path": str(path), "sha256": file_sha256(path)}


HEALTH_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "localization_health.yaml"


# ---- 회차 근거(B2-L8 반복·LD-2 부하 비교): 경로 완주, 활성 로봇 연속성, 동작 설정 서명 ----
# 동작 설정 서명: 회차 conditions.files_sha256에서 위치 추정 동작에 영향을 주는 파일의 sha256. 평가 도구(check_localization.py)는
# 넣지 않는다. 하나라도 없으면 서명 없음(같은 설정 확인 불가)이다.
OPERATING_FILES = ("amcl.yaml", "localization_health.yaml", "localization_health.py", "localization_supervisor.py",
                   "localization.launch.py", "mapping_drive.py", "route", "world")
ROUTE_METHOD = ("경로 근거: 기준 repeats.min_route 파일(현재 sha256)과 회차 실행 전 경로 해시(conditions.files_sha256.route)·"
                "주행 결과(drive/drive_result.json)의 route sha256이 모두 같고, 주행 결과가 completed·partial false·"
                "모든 경유점 도달·관문 정지 확인·같은 월드·같은 로봇일 때만 검증. 경로 길이·이름으로 '이상'을 추측하지 않는다")
ACTIVE_CONTINUITY_METHOD = ("측정 중 활성 로봇: 수집기 1 Hz 감시가 /ax/<id>/joint_states 발행자 집합이 바뀔 때 "
                            "raw/common/active_robots.jsonl에 남긴다. 감시 연속성은 tf_chain 1 Hz 기록 간격(≤ 3 s, mono)으로 본다")
WATCH_MAX_GAP_S = 3.0
WORKSPACE = Path(__file__).resolve().parents[3]


# 기준 min_route 경로 파일을 작업공간 기준으로 찾는다(src/<패키지>/… 형식). 반환: (경로, sha256 또는 None)
def criteria_route(criteria):
    relative = (criteria.get("repeats") or {}).get("min_route")

    if not relative:
        return None, None

    path = Path(relative) if Path(relative).is_absolute() else WORKSPACE / relative

    return path, file_sha256(path)


# 회차의 경로 완주 근거(robot_id: 주행 로봇). 파일은 읽기만 한다.
def route_evidence(raw_dir, conditions, criteria):
    path, expected = criteria_route(criteria)
    record = {"method": ROUTE_METHOD, "required_route": str(path) if path else None, "required_sha256": expected,
              "verified": False}
    drive_path = Path(raw_dir) / "drive" / "drive_result.json" if raw_dir is not None else None

    if expected is None:
        return {**record, "reason": "기준 경로 파일 없음"}

    if drive_path is None or not drive_path.is_file():
        return {**record, "reason": "주행 결과(drive/drive_result.json) 없음"}

    drive = json.loads(drive_path.read_text(encoding="utf-8"))
    route = drive.get("route") or {}
    pre = ((conditions.get("files_sha256") or {}).get("route") or {}).get("sha256")
    target_world = (criteria.get("target") or {}).get("world")
    gates = [stop for stop in drive.get("stops") or [] if str(stop.get("reason", "")).startswith("관측 관문")]
    waypoints = route.get("waypoints") or []
    confirmed = [stop.get("confirmed") for stop in gates]
    record.update(drive_result=str(drive_path), drive_result_sha256=file_sha256(drive_path),
                  drive_route_sha256=route.get("sha256"), pre_run_route_sha256=pre, robot_id=drive.get("robot_id"),
                  status=drive.get("status"), partial=route.get("partial"),
                  waypoints_reached=drive.get("waypoints_reached"), waypoints=len(waypoints),
                  world=(drive.get("world") or {}).get("file"), gate_stops_confirmed=confirmed)
    problems = []

    if route.get("sha256") != expected:
        problems.append("주행 결과 경로 sha256이 기준 경로와 다름")

    if pre != expected:
        problems.append("실행 전 경로 해시 기록 없음·기준과 다름")

    if drive.get("status") != "completed" or drive.get("error"):
        problems.append(f"주행 미완료(status {drive.get('status')}, error {drive.get('error')})")

    if route.get("partial") is not False or drive.get("waypoints_reached") != len(waypoints) or not waypoints:
        problems.append(f"경유점 {drive.get('waypoints_reached')}/{len(waypoints)}(partial {route.get('partial')})")

    if (drive.get("world") or {}).get("file") != target_world:
        problems.append(f"주행 월드 {(drive.get('world') or {}).get('file')} ≠ 기준 {target_world}")

    if not gates or not all(stop.get("confirmed") is True for stop in gates):
        problems.append("관문 정지 확인 없음")

    return {**record, "verified": not problems, "reason": "; ".join(problems)}


# 측정 중 활성 로봇 집합이 변하지 않았는지(수집기 기록). status: constant·changed·unverified
def active_continuity(active_rows, watch_rows, window_mono):
    record = {"method": ACTIVE_CONTINUITY_METHOD}

    if not active_rows:
        return {**record, "status": "unverified", "reason": "활성 로봇 기록(raw/common/active_robots.jsonl) 없음"}

    sets = [sorted(row.get("active_robot_ids") or []) for row in active_rows]
    record.update(sets=[{"mono": row.get("recv_mono"), "sim": row.get("recv_sim"), "active_robot_ids": ids}
                        for row, ids in zip(active_rows, sets)])
    # 수집기 시작 직후 ROS 그래프 탐색 전의 빈 집합(첫 기록 뒤 WATCH_MAX_GAP_S 안)은 활성 변화로 세지 않고 따로 남긴다
    # (2026-11-14 l8 single1: 첫 감시 [] → 1 s 뒤 [robot_2]). 비어 있지 않은 다른 집합은 그대로 변화다.
    first = active_rows[0].get("recv_mono")
    warmup = 0

    while (warmup < len(sets) - 1 and not sets[warmup] and finite_number(first)
           and finite_number(active_rows[warmup + 1].get("recv_mono"))
           and active_rows[warmup + 1]["recv_mono"] - first <= WATCH_MAX_GAP_S):
        warmup += 1

    record["discovery_warmup_records"] = warmup
    sets = sets[warmup:]
    monos = sorted(row["recv_mono"] for row in watch_rows if finite_number(row.get("recv_mono")))
    gaps = [later - earlier for earlier, later in zip(monos, monos[1:])]

    if not monos or (gaps and max(gaps) > WATCH_MAX_GAP_S):
        return {**record, "status": "unverified",
                "reason": f"감시 기록 공백(최대 {max(gaps) if gaps else None} s > {WATCH_MAX_GAP_S} s) 또는 없음"}

    if window_mono and all(finite_number(value) for value in window_mono):
        if monos[0] > window_mono[0] + WATCH_MAX_GAP_S or monos[-1] < window_mono[1] - WATCH_MAX_GAP_S:
            return {**record, "status": "unverified", "reason": "감시 기록이 수집 구간 시작·끝을 덮지 않음"}

    changed = any(ids != sets[0] for ids in sets)

    return {**record, "status": "changed" if changed else "constant", "active_robot_ids": sets[0] if not changed
            else None, "reason": "측정 중 활성 로봇 변화" if changed else ""}


# 동작 설정 서명(파일 이름 → sha256). 하나라도 없으면 None
def operating_signature(conditions):
    files = (conditions or {}).get("files_sha256") or {}
    signature = {name: (files.get(name) or {}).get("sha256") for name in OPERATING_FILES}

    return signature if all(signature.values()) else None


# ---- 원시 기록 근거(B2-L3·LD-3 공통, 실시간·재판정·묶음이 같은 함수를 쓴다) ----
# 결과 파일 위치·명시 폴더·결과 raw.dir 기록 순으로 원시 기록 폴더를 찾는다(읽기 전용). 반환: (폴더 또는 None, 출처)
def locate_raw_dir(result_path=None, result=None, explicit=None):
    recorded = ((result or {}).get("raw") or {}).get("dir")
    source = {"recorded_raw_dir": recorded}

    if explicit is not None:
        return Path(explicit), {**source, "how": "--raw-dir 지정", "raw_dir": str(explicit)}

    if result_path is not None:
        for folder in (Path(result_path).parent, Path(result_path).parent.parent):
            if (folder / "raw").is_dir() and (folder / "conditions.json").is_file():
                differs = recorded is not None and Path(recorded).resolve() != folder.resolve()

                return folder, {**source, "how": "결과 옆 회차 폴더", "raw_dir": str(folder),
                                "differs_from_recorded": differs}

    if recorded and (Path(recorded) / "raw").is_dir():
        return Path(recorded), {**source, "how": "결과 raw.dir 기록", "raw_dir": recorded}

    return None, {**source, "how": "없음", "raw_dir": None}


# 원시 기록 폴더에서 로봇별 B2-L3 지표와 LD-3 근거를 만든다. 파일은 읽기만 한다.
# criteria가 있으면 B2-L4~L7 근거(events)도 만든다(없으면 events None).
def collect_raw_evidence(raw_dir, robot_ids, result, source, criteria=None):
    robot_ids = list(robot_ids or [])

    if raw_dir is None:
        reason = "원시 기록 폴더 없음(health·odom 원시 기록이 있어야 B2-L3를 계산한다)"
        robots = {robot: {"health_ok": {"status": NOT_JUDGED, "reason": reason}, "events": None} for robot in robot_ids}
        supervisors = {robot: supervisor_wall_status(None) for robot in robot_ids}

        run = {"route": {"verified": False, "reason": "원시 기록 폴더 없음"},
               "active_continuity": {"status": "unverified", "reason": "원시 기록 폴더 없음"},
               "operating_signature": None}

        return {"source": source, "robots": robots, "run": run,
                "wall_clock": wall_clock_summary(collector_wall_status(result), supervisors)}

    raw_dir = Path(raw_dir)
    conditions_path = raw_dir / "conditions.json"
    conditions = json.loads(conditions_path.read_text(encoding="utf-8")) if conditions_path.is_file() else {}
    exclusions_path = raw_dir / HEALTH_EXCLUSIONS_FILE
    exclusions, exclusion_problem = [], None

    if exclusions_path.is_file():
        try:
            data = json.loads(exclusions_path.read_text(encoding="utf-8"))
            exclusions = data.get("intervals") if data.get("schema") == HEALTH_EXCLUSIONS_SCHEMA else None
            exclusion_problem = None if exclusions is not None else f"{HEALTH_EXCLUSIONS_FILE} schema 불일치"
        except (json.JSONDecodeError, AttributeError) as error:
            exclusion_problem = f"{HEALTH_EXCLUSIONS_FILE} 읽기 실패: {error}"

    termination = (result.get("recording") or {}).get("termination") or {}
    window_end = termination.get("mono_end")

    if window_end is None:
        window_end = ((result.get("record_quality") or {}).get("collection_window_mono") or {}).get("end")

    window_start = (conditions.get("time_reference") or {}).get("monotonic_s")
    window = [window_start, window_end] if finite_number(window_start) and finite_number(window_end) else None
    robots, supervisors, files = {}, {}, {}
    jump_limits, jump_source = jump_limits_from_config(HEALTH_CONFIG_PATH)
    recorded_config = ((conditions.get("files_sha256") or {}).get("localization_health.yaml") or {}).get("sha256")
    jump_source.update(recorded_run_sha256=recorded_config,
                       same_as_run=None if recorded_config is None else recorded_config == jump_source.get("sha256"))
    world_map = recorded_world_map(result)
    record_end = ((conditions.get("recorded_sim_s") or (result.get("conditions") or {}).get("recorded_sim_s"))
                  or [None, None])[-1]
    for robot in robot_ids:
        paths = {stream: raw_dir / "raw" / robot / f"{stream}.jsonl"
                 for stream in ("health", "odom", "tf_map_odom", "truth", "amcl_lifecycle", "actuation")}
        (health, health_broken), (odom, odom_broken) = read_jsonl(paths["health"]), read_jsonl(paths["odom"])
        record = health_ok_ratio(health, odom, exclusions or [], health_broken + odom_broken, robot)

        if exclusion_problem:
            record.update(status=NOT_JUDGED, reason="; ".join(filter(None, [record.get("reason"), exclusion_problem])))

        diagnostics = raw_dir / "supervisor" / f"{robot}_supervisor.jsonl"
        rows, broken = read_jsonl(diagnostics) if diagnostics.is_file() else (None, 0)
        supervisors[robot] = supervisor_wall_status(rows, window, broken, health)
        streams = {"health": health, "odom": odom, "supervisor": rows}
        broken_streams = {"health": health_broken, "odom": odom_broken, "supervisor": broken}

        for name in ("tf_map_odom", "truth", "amcl_lifecycle", "actuation"):
            if paths[name].is_file():
                streams[name], broken_streams[name] = read_jsonl(paths[name])
            else:
                streams[name] = None if name in ("amcl_lifecycle", "actuation") else []

        # 사건 출처 연속성: 감독 tick이 수집 구간을 덮지 않으면 lifecycle·LATCHED 사건 누락을 배제할 수 없다.
        source_problems = [] if rows is None else tick_coverage_problems(rows, window, health)

        if rows is None and streams["amcl_lifecycle"] is None:
            source_problems.append("감독 진단·수집기 lifecycle 기록 모두 없음")

        if jump_limits is None:
            source_problems.append(f"도약 임계값 출처 없음({jump_source.get('reason')}): 원시 TF 도약 미검사")

        source_problems += [f"{name} 깨진 줄 {count}개" for name, count in broken_streams.items() if count]
        robots[robot] = {"health_ok": record,
                         "events": event_evidence(streams, world_map, criteria or {}, jump_limits, source_problems,
                                                  record_end, robot) if criteria is not None else None}
        files[robot] = {name: {"path": str(path), "sha256": file_sha256(path)}
                        for name, path in (*paths.items(), ("supervisor", diagnostics))}

    # 회차 근거(B2-L8·LD-2): 경로 완주, 측정 중 활성 로봇 연속성, 동작 설정 서명
    active_rows, _ = read_jsonl(raw_dir / "raw" / "common" / "active_robots.jsonl") \
        if (raw_dir / "raw" / "common" / "active_robots.jsonl").is_file() else ([], 0)
    watch_rows = read_jsonl(raw_dir / "raw" / robot_ids[0] / "tf_chain.jsonl")[0] \
        if robot_ids and (raw_dir / "raw" / robot_ids[0] / "tf_chain.jsonl").is_file() else []
    run = {"route": route_evidence(raw_dir, conditions, criteria or {}),
           "active_continuity": active_continuity(active_rows, watch_rows, window),
           "operating_signature": operating_signature(conditions)}

    return {"source": {**source, "files": files, "jump_limits_source": jump_source,
                       "world_map_source": (result.get("conditions") or {}).get("world_map"),
                       "exclusions": str(exclusions_path) if exclusions_path.is_file() else None},
            "robots": robots, "run": run,
            "wall_clock": wall_clock_summary(collector_wall_status(result), supervisors)}


# 결과의 대상(지도 revision·파일 해시, 월드 파일 해시)을 기준 파일 target과 대조한다.
# target_context: {"map_info": map_info.yaml 내용, "recorded_map": 회차 지도 해시, "recorded_world_sha256": 회차 월드 해시}
# 반환: (불일치 사유 목록, 대조하지 못한 사유 목록)
def check_target(target, target_context):
    mismatches = []
    unchecked = []
    context = target_context or {}
    map_info = context.get("map_info")
    recorded_map = context.get("recorded_map")
    applicable = context.get("applicable_worlds")
    target_world = None

    if applicable is not None and target:
        entries = applicable.get("applicable_worlds") or []
        target_world = next((entry for entry in entries if entry.get("launch_name") == target.get("world")), None)

        if applicable.get("map_revision") != target.get("map_revision"):
            mismatches.append(f"applicable_worlds revision {applicable.get('map_revision')} ≠ "
                              f"기준 {target.get('map_revision')}")

        if target_world is None:
            mismatches.append(f"기준 월드 {target.get('world')}가 지도 적용 월드 목록에 없음")

    if not target:
        return mismatches, ["기준 파일에 target 없음"]

    if map_info is None:
        unchecked.append("기준 대상 지도의 map_info.yaml을 읽지 못함")
    else:
        if map_info.get("map_revision") != target.get("map_revision"):
            mismatches.append(f"map_info revision {map_info.get('map_revision')} ≠ 기준 {target.get('map_revision')}")

        world_name = (map_info.get("world") or {}).get("launch_name")

        # 지도 적용 월드 목록이 있으면 그 목록으로, 없으면 map_info의 작성 월드 이름으로 대상 월드를 확인한다.
        if applicable is None and world_name != target.get("world"):
            mismatches.append(f"map_info 월드 {world_name} ≠ 기준 {target.get('world')}")

    # 회차가 쓴 지도 파일 해시를 채택 지도 해시와 비교한다.
    if not recorded_map:
        unchecked.append("회차 지도 해시 기록 없음(조건 파일 없음)")
    elif map_info is not None:
        files = map_info.get("files") or {}

        for name, key in (("map.yaml", "yaml"), ("map.pgm", "image")):
            if recorded_map.get(key) is None:
                unchecked.append(f"회차 {name} 해시 기록 없음")
            elif recorded_map.get(key) != files.get(name):
                mismatches.append(f"회차 {name} 해시가 채택 지도 revision {map_info.get('map_revision')}와 다름")

    # 월드는 회차 조건에 월드 파일 해시(--world-file)가 있을 때만 비교한다. 지도 적용 월드 목록이 있으면
    # 목록에 등록된 월드(revision 일치, 단색 작성 월드·질감 기준 월드) 중 하나의 sha256과 같으면 대상으로 인정한다
    # (2026-11-14 사용자 결정). 실제로 쓴 월드는 matched_world()로 결과에 기록한다. 목록이 없으면 map_info 작성 월드와 비교한다.
    world_sha = context.get("recorded_world_sha256")

    if world_sha is None and context.get("world_hash_problem"):
        unchecked.append(context["world_hash_problem"])
    elif world_sha is None:
        unchecked.append("회차 월드 파일 해시 기록 없음(--world-file 미지정)")
    elif applicable is not None:
        if matched_world(context) is None:
            mismatches.append("회차 월드 해시가 지도 적용 월드 목록(applicable_worlds.yaml)에 없음")
    elif map_info is not None and world_sha != (map_info.get("world") or {}).get("world_sha256"):
        mismatches.append("회차 월드 파일 해시가 채택 지도의 월드 해시와 다름")

    return mismatches, unchecked


# 회차 월드 해시와 같은 지도 적용 월드(applicable_worlds.yaml) 항목. 없거나 대조할 수 없으면 None
def matched_world_entry(target_context):
    context = target_context or {}
    world_sha = context.get("recorded_world_sha256")
    applicable = context.get("applicable_worlds")

    if world_sha is None or applicable is None:
        return None

    entries = applicable.get("applicable_worlds") or []

    return next((entry for entry in entries if entry.get("sha256") == world_sha), None)


# 회차 월드 해시와 같은 지도 적용 월드의 launch_name. 없으면 None
def matched_world(target_context):
    entry = matched_world_entry(target_context)

    return entry.get("launch_name") if entry else None


# 회차가 실제로 쓴 월드의 출처 기록: 등록 항목(이름·해시·근거)과 목록 revision, 회차 해시를 어디서 읽었는지
def world_source_record(target_context):
    context = target_context or {}
    entry = matched_world_entry(context)
    applicable = context.get("applicable_worlds") or {}

    return {
        "recorded_sha256": context.get("recorded_world_sha256"),
        "applicable_revision": applicable.get("map_revision"),
        "registered": {key: entry.get(key) for key in ("launch_name", "sha256", "basis")} if entry else None,
        "authored_world": (applicable.get("authored_world") or {}).get("launch_name"),
    }


# 기준 파일 record_only에 지정한 항목 ID는 판정하지 않고 값만 남긴다(recorded).
def apply_record_only(items, record_only):
    for item in items:
        if item["id"] in record_only and item["result"] in (PASS, FAIL):
            item.update(result=RECORDED, reason="기록 전용(record_only), 판정하지 않음")

    return items


# 결과 dict와 기준 dict로 기준 항목과 회차 판정을 만든다(입출력 없는 순수 함수).
# criteria_ref: {"path", "sha256"}. raw_evidence: collect_raw_evidence()가 원시 기록에서 만든 B2-L3·LD-3 근거
# (없으면 B2-L3 not_judged, LD-3은 결과 파일의 수집기 기록만 본다). 반환 dict를 결과에 합친다.
def judge_localization(result, criteria, criteria_ref, target_context=None, raw_evidence=None):
    if raw_evidence is None:
        raw_evidence = collect_raw_evidence(None, run_robot_ids(result) or sorted(result.get("robots") or {}),
                                            result, {"how": "없음", "raw_dir": None}, criteria)

    items = []
    segments = {}
    gap_problems = []
    load = criteria.get("load") or {}
    max_interval = load.get("max_sample_interval_s")

    # 로봇별 판정 항목. 표본 간격은 오차 표본 시각을 정하는 odom 기록의 최대 간격(sim)으로 본다.
    # 기준 파일에 load.max_sample_interval_s가 없으면 표본 간격 검사(SAMPLE_GAP)를 하지 않는다.
    for robot_id, entry in sorted((result.get("robots") or {}).items()):
        robot_items, segments[robot_id] = robot_criteria_items(robot_id, entry, criteria)
        items.extend(robot_items)
        # B2-L3 정상 주행 중 health OK 시간 비율(원시 health·odom)
        record = ((raw_evidence.get("robots") or {}).get(robot_id) or {}).get("health_ok") or \
            {"status": NOT_JUDGED, "reason": "이 로봇의 원시 근거 없음"}
        items.append(health_ratio_item(robot_id, record, (criteria.get("moving") or {}).get("min_health_ok_ratio")))
        # B2-L4~L7 과신·도약 감지·재위치(원시 기록·감독 진단)
        items.extend(event_items(robot_id, ((raw_evidence.get("robots") or {}).get(robot_id) or {}).get("events"),
                                 criteria))
        odom_gap = (entry.get("odom") or {}).get("max_gap_s")

        if finite_number(max_interval) and finite_number(odom_gap) and odom_gap > max_interval:
            gap_problems.append(f"{robot_id} odom 최대 간격 {odom_gap:.3f} s > {max_interval} s")

    # LD-3 이 회차의 벽시계 제한 사건 유무(묶음에서 고유 회차 수를 센다)
    wall_clock = raw_evidence["wall_clock"]
    items.append(wall_clock_item(wall_clock, load.get("max_wall_clock_limited_runs")))

    for item_id, metric, reason in CRITERIA_BATCH_ONLY:
        items.append(fixed_item(item_id, metric, NOT_APPLICABLE, reason))

    apply_record_only(items, set(criteria.get("record_only") or ()))
    mismatches, unchecked = check_target(criteria.get("target"), target_context)
    failed = [item for item in items if item["result"] == FAIL]
    unjudged = [item for item in items if item["result"] == NOT_JUDGED]
    unexposed = [item_id for item_id in EVENT_TEST_IDS
                 if all(item["result"] == NOT_APPLICABLE for item in items if item["id"] == item_id)]
    subset = None

    # 회차 판정 순서: incomplete → 벽시계 제한(LD-3, incomplete로 취급) → 대상 불일치 → SAMPLE_GAP(키가 있을 때만)
    # → FAIL → not_judged 없음이면 PASS(not_applicable·recorded는 PASS를 막지 않는다) → 그 밖 not_judged + judged_subset
    if result.get("status") != "completed":
        judgment, reason = NOT_JUDGED, "status incomplete"
    elif wall_clock["limited"]:
        judgment, reason = NOT_JUDGED, "벽시계 제한 회차(LD-3, incomplete로 취급): " + ", ".join(
            event.get("source", "") for event in wall_clock["events"])
    elif mismatches:
        judgment, reason = NOT_JUDGED, "대상 불일치: " + "; ".join(mismatches)
    elif gap_problems:
        judgment, reason = NOT_JUDGED, "SAMPLE_GAP: " + "; ".join(gap_problems)
    elif failed:
        judgment = FAIL
        reason = "FAIL 항목: " + ", ".join(f"{item['id']}({item['robot']}) {item['metric']}" for item in failed)
    elif not unjudged and unexposed:
        # 사건 시험이 이 회차에 노출되지 않았다(대상 아님). 회차 전체 PASS로 쓰지 않고 묶음 coverage에서 충족을 본다.
        judgment = NOT_JUDGED
        reason = "사건 시험 미노출(not_applicable, 회차 전체 PASS 아님): " + ", ".join(unexposed)
        subset = PASS
    elif not unjudged:
        judgment, reason = PASS, "not_judged 항목 없음(not_applicable·recorded 제외 모두 PASS)"
    else:
        judgment = NOT_JUDGED
        reason = "미판정 항목: " + ", ".join(sorted({item["id"] for item in unjudged}))
        subset = PASS if any(item["result"] == PASS for item in items) else None

    if unchecked:
        reason += " / 대상 대조 불가: " + "; ".join(unchecked)

    return {
        "criteria": dict(criteria_ref),
        "criteria_items": items,
        "judgment": judgment,
        "judgment_reason": reason,
        **({"judged_subset": subset} if subset else {}),
        "criteria_method": {
            "segments": "평가 표본마다 odom 선속도 ≤ D-06 선속도이고 각속도 ≤ D-06 각속도이면 정지, 아니면 주행"
                        "(D-06 0.5 s 유지는 보지 않음). 한 구간 표본이 없으면 그 구간 항목은 not_applicable",
            "d06_linear_m_s": D06_LINEAR_M_S,
            "d06_angular_rad_s": D06_ANGULAR_RAD_S,
            "d06_source": "docs/design/INTERFACE_CONTRACT.md D-06",
            "segment_samples": segments,
            "scan_time_basis": "LaserScan header stamp(시뮬레이션 시각) 기준 rate_summary",
            "sample_gap_basis": "로봇별 odom 기록 최대 간격(sim)과 load.max_sample_interval_s 비교"
            if max_interval is not None else "표본 간격 조건: 기준 파일에 없음(적용 안 함)",
            "rtf": "rtf_min이 null이면 RTF로 판정·무효 처리하지 않음(기록만)"
            if load.get("rtf_min") is None else f"rtf_min {load.get('rtf_min')}: 미적용",
            "target_unchecked": unchecked,
            "target_world_used": matched_world(target_context),
            "target_world_source": world_source_record(target_context),
            "target_world_reference": (criteria.get("target") or {}).get("world"),
            "target_mismatch": mismatches,
            "sample_gap": gap_problems,
            "health_ok_ratio": HEALTH_RATIO_METHOD,
            "event_tests": {"overconfidence": OVERCONFIDENCE_METHOD, "discontinuity": DISCONTINUITY_METHOD,
                            "relocalization": RELOCALIZATION_METHOD, "rejection": REJECTION_METHOD,
                            "unexposed": unexposed},
            "raw_evidence_source": raw_evidence.get("source"),
            "run_evidence": raw_evidence.get("run"),
            "wall_clock": wall_clock,
            "wall_clock_limited": wall_clock["limited"] is True,
        },
    }


# 기준 파일을 읽고 경로·sha256을 함께 돌려준다.
def load_criteria(path):
    import yaml

    path = Path(path)
    data = path.read_bytes()

    return yaml.safe_load(data.decode("utf-8")), {"path": str(path), "sha256": hashlib.sha256(data).hexdigest()}


# 대상 대조 자료를 모은다: 기준 target의 지도 옆 map_info.yaml, 회차 조건 파일(conditions.json)의 지도·월드 해시.
# target.map은 저장소 기준 상대 경로라 기준 파일 위 폴더들에서 그 경로가 있는 곳을 저장소 루트로 본다.
# 월드 해시는 결과 conditions.world_sha256(부하 단계 기록)을 먼저 쓰고, 없으면 조건 파일 files_sha256.world를 쓴다.
def target_context(criteria, criteria_path, conditions, result=None):
    import yaml

    context = {"map_info": None, "applicable_worlds": None, "recorded_map": None, "recorded_world_sha256": None,
               "world_hash_problem": None}
    target = criteria.get("target") or {}

    if target.get("map"):
        for root in Path(criteria_path).resolve().parents:
            info_path = root / Path(target["map"]).parent / "map_info.yaml"

            if (root / target["map"]).is_file() and info_path.is_file():
                context["map_info"] = yaml.safe_load(info_path.read_text(encoding="utf-8"))
                worlds_path = info_path.with_name("applicable_worlds.yaml")

                if worlds_path.is_file():
                    context["applicable_worlds"] = yaml.safe_load(worlds_path.read_text(encoding="utf-8"))

                break

    if conditions:
        context["recorded_map"] = conditions.get("map")
        digest, problem = world_hash((conditions.get("files_sha256") or {}).get("world"))
        context["recorded_world_sha256"], context["world_hash_problem"] = digest, problem

    digest, problem = world_hash(((result or {}).get("conditions") or {}).get("world_sha256"))

    if digest is not None or problem is not None:
        context["recorded_world_sha256"], context["world_hash_problem"] = digest, problem

    return context


# 월드 해시 기록을 문자열 sha256으로 바꾼다. 조건 파일 files_sha256 값은 {"path", "sha256"} 사전이고(collect_conditions),
# 옛 형식·결과 conditions.world_sha256은 문자열이다. 그 밖의 형식은 (None, 사유)다. 반환: (sha256 또는 None, 사유)
def world_hash(value):
    if value is None:
        return None, None

    if isinstance(value, dict):
        digest = value.get("sha256")

        return (digest, None) if isinstance(digest, str) and digest else (None, f"월드 해시 사전에 sha256 없음: {value}")

    if isinstance(value, str) and value:
        return value, None

    return None, f"월드 해시 형식을 알 수 없음: {type(value).__name__}"


# 결과 파일 위치에서 회차 조건 파일을 찾는다(<raw-dir>/result/localization_result.json → <raw-dir>/conditions.json).
def find_conditions(result_path):
    for folder in (Path(result_path).parent, Path(result_path).parent.parent):
        candidate = folder / "conditions.json"

        if candidate.is_file():
            return json.loads(candidate.read_text(encoding="utf-8"))

    return None


# 대상 대조·원시 근거·판정을 한 번에 한다. 실시간 측정(main)과 재판정·묶음(judge_file)이 같은 경로를 쓴다.
def criteria_judgment(result, criteria, criteria_ref, criteria_path, conditions, raw_dir, raw_source):
    context = target_context(criteria, criteria_path, conditions, result)
    robot_ids = run_robot_ids(result) or sorted(result.get("robots") or {})
    evidence = collect_raw_evidence(raw_dir, robot_ids, result, raw_source, criteria)

    return judge_localization(result, criteria, criteria_ref, context, evidence)


# 결과 파일 하나를 읽어 기준으로 판정한다. raw_dir: 명시한 원시 기록 폴더(없으면 결과 옆 폴더·raw.dir 기록).
# 반환: (원본 결과, 판정 dict, 원본 sha256)
def judge_file(result_path, criteria, criteria_ref, criteria_path, raw_dir=None):
    result_path = Path(result_path)
    data = result_path.read_bytes()
    result = json.loads(data.decode("utf-8"))
    folder, source = locate_raw_dir(result_path, result, raw_dir)
    judgment = criteria_judgment(result, criteria, criteria_ref, criteria_path, find_conditions(result_path),
                                 folder, source)

    return result, judgment, hashlib.sha256(data).hexdigest()


# 과거 결과 JSON을 기준 파일로 다시 판정해 같은 폴더의 <이름>.judged.json에 쓴다. 원본은 바꾸지 않는다.
def rejudge(result_path, criteria_path, raw_dir=None):
    result_path = Path(result_path)
    criteria, reference = load_criteria(criteria_path)
    result, judgment, digest = judge_file(result_path, criteria, reference, criteria_path, raw_dir)
    judged = dict(result)
    judged.update(judgment)
    judged["rejudged_from"] = {"path": str(result_path), "sha256": digest,
                               "original_judgment": result.get("judgment"),
                               "original_judgment_reason": result.get("judgment_reason")}
    output = result_path.with_name(f"{result_path.stem}.judged.json")
    write_json_atomic(output, judged)

    return output, judged


# 회차 결과에서 로봇 목록을 읽는다. 읽을 수 없으면 None
def run_robot_ids(result):
    robot_ids = (result.get("conditions") or {}).get("robot_ids")

    if robot_ids is None and isinstance(result.get("robots"), dict):
        robot_ids = list(result["robots"])

    return list(robot_ids) if robot_ids else None


# 묶음 판정용 회차 요약: 경로·해시·회차 판정·로봇 수·월드 해시 유무.
# 활성 로봇 수와 월드 해시는 결과 conditions의 부하 단계 기록(active_robot_count·world_sha256)을 쓴다.
# 활성 로봇 수가 없으면(옛 기록) 측정 대상 robot_ids 길이로 대신하고 그 사실을 active_robots_source에 남긴다.
def run_summary(path, result, judgment, digest, conditions):
    robot_ids = run_robot_ids(result)
    recorded = result.get("conditions") or {}
    world_sha = world_hash(recorded.get("world_sha256"))[0] or \
        world_hash(((conditions or {}).get("files_sha256") or {}).get("world"))[0]
    active = recorded.get("active_robot_count")
    source = "active_robot_count"

    if active is None:
        active = len(robot_ids) if robot_ids else None
        source = "active_robot_count 없음, 측정 대상 robot_ids 길이 사용" if robot_ids else "로봇 정보 없음"

    return {
        "path": str(path),
        "sha256": digest,
        "status": result.get("status"),
        "judgment": judgment["judgment"],
        "judgment_reason": judgment["judgment_reason"],
        "judged_subset": judgment.get("judged_subset"),
        "robot_ids": robot_ids,
        "active_robots": active,
        "active_robots_source": source,
        "load_stage_recorded": recorded.get("active_robot_count") is not None,
        "active_robot_ids": recorded.get("active_robot_ids"),
        "world_sha256": world_sha,
        "label": (result.get("conditions") or {}).get("label"),
        "target_mismatch": judgment["criteria_method"]["target_mismatch"],
        "moving_judged": any(item["id"] == "B2-L2" and item["result"] in (PASS, FAIL)
                             for item in judgment["criteria_items"]),
        "moving_p95_m": {item["robot"]: item["value"] for item in judgment["criteria_items"]
                         if item["metric"] == "moving.max_position_error_p95_m" and item["result"] in (PASS, FAIL)},
        "valid": run_valid(result.get("status"), judgment),
        "wall_clock_limited": judgment["criteria_method"]["wall_clock"]["limited"],
        "wall_clock_unverified": judgment["criteria_method"]["wall_clock"]["unverified"],
        "record_key": record_key(judgment, digest),
        **run_evidence_summary(judgment),
    }


# 회차 판정의 회차 근거(경로·활성 연속성·동작 설정 서명)와 감독 세션을 요약한다. 근거가 없으면 확인 불가로 둔다.
def run_evidence_summary(judgment):
    evidence = judgment["criteria_method"].get("run_evidence") or {}
    route = evidence.get("route") or {}
    continuity = evidence.get("active_continuity") or {}
    sessions = sorted({session for item in judgment["criteria_items"] if item["id"] == "B2-L3"
                       for session in ((item.get("health_ok") or {}).get("health") or {}).get("sessions") or []})

    return {"route_verified": route.get("verified") is True, "route_reason": route.get("reason", "회차 근거 없음"),
            "route_robot": route.get("robot_id"), "active_continuity": continuity.get("status", "unverified"),
            "operating_signature": evidence.get("operating_signature"), "sessions": sessions,
            "session_key": "sessions:" + ",".join(sessions) if sessions else None,
            "robots_with_errors": sorted({item["robot"] for item in judgment["criteria_items"]
                                          if item["id"] in ("B2-L1", "B2-L2") and item["result"] in (PASS, FAIL)})}


# 같은 회차를 가리키는 키: 원시 기록 폴더(있으면, 결과·재판정 사본을 같은 회차로 본다), 없으면 결과 sha256
def record_key(judgment, digest):
    raw_dir = (judgment["criteria_method"].get("raw_evidence_source") or {}).get("raw_dir")

    return f"raw:{Path(raw_dir).resolve()}" if raw_dir else f"sha256:{digest}"


# 묶음 근거로 쓸 수 있는 회차인지 본다: status completed이고 대상 불일치·SAMPLE_GAP·벽시계 제한이 없는 회차.
# 반복 수(B2-L8)·부하 비교(LD-2)·충족 근거(coverage)가 모두 이 기준을 쓴다.
def run_valid(status, judgment):
    method = judgment.get("criteria_method") or {}

    problems = (method.get("target_mismatch"), method.get("sample_gap"), method.get("wall_clock_limited"))

    return status == "completed" and not any(problems)


# 반복·부하 근거로 쓸 수 있는 회차를 고른다(B2-L8·LD-2 공통). 같은 감독 세션(복사본·재판정·중복 제출)은 한 번만 쓴다.
# 조건: 유효 회차, 주행 표본, 경로 완주 검증, 측정 중 활성 로봇 불변, 부하 단계 기록(활성 로봇 수 실측), 동작 설정 서명,
# 감독 세션 기록. 옛 기록의 robot_ids 길이로 활성 수를 대신해 확정하지 않는다. 반환: (쓸 회차, 제외 [(경로, 사유)])
def evidence_runs(summaries):
    chosen, excluded, seen = [], [], {}

    for run in summaries:
        checks = (("무효 회차(incomplete·대상 불일치·SAMPLE_GAP·벽시계 제한)", run["valid"]),
                  ("주행 표본 없음", run["moving_judged"]),
                  (f"경로 근거 없음: {run['route_reason']}", run["route_verified"]),
                  (f"측정 중 활성 로봇 {run['active_continuity']}", run["active_continuity"] == "constant"),
                  ("활성 로봇 수 실측 기록 없음", run["load_stage_recorded"]),
                  ("동작 설정 서명 없음", run["operating_signature"] is not None),
                  ("감독 세션 기록 없음", run["session_key"] is not None))
        failed = [reason for reason, ok in checks if not ok]

        if failed:
            excluded.append((run["path"], "; ".join(failed)))
        elif run["session_key"] in seen:
            excluded.append((run["path"], f"같은 세션 중복({seen[run['session_key']]})"))
        else:
            seen[run["session_key"]] = run["path"]
            chosen.append(run)

    return chosen, excluded


# 동작 설정·월드가 같은 회차 묶음 키
def condition_key(run):
    return json.dumps({"world": run["world_sha256"], "operating": run["operating_signature"]}, sort_keys=True)


# B2-L8 반복 요구를 센다. 단독: 측정 대상 robot_2뿐, 활성 1대, 경로 주행 robot_2. 동시 활성: 활성 2대 이상이고
# 활성 로봇이 모두 측정 대상이며 모두 오차 자료가 있고 경로 주행 로봇이 측정 대상에 있다(두 대 동시 주행 요구가 아니다).
# 경로는 기준 min_route와 같은 경로의 완주 근거로만 인정한다. 동작 설정·월드가 섞이면 확정하지 않는다.
def repeat_items(summaries, criteria):
    repeats = criteria.get("repeats") or {}
    moving = [run for run in summaries if run["valid"] and run["moving_judged"]]
    unknown = [run["path"] for run in moving if run["robot_ids"] is None or run["active_robots"] is None]
    metrics = ("repeats.robot_2_single_runs", "repeats.two_robot_simultaneous_runs_min")

    if unknown:
        reason = "로봇 목록을 읽을 수 없는 회차: " + ", ".join(unknown)
        items = [fixed_item("B2-L8", metric, NOT_JUDGED, reason) for metric in metrics]
        items.append(fixed_item("B2-L8", "repeats.min_route", NOT_JUDGED, reason, limit=repeats.get("min_route")))
        return items

    chosen, excluded = evidence_runs(summaries)
    groups = sorted({condition_key(run) for run in chosen})
    note = f" / 제외 {len(excluded)}개" if excluded else ""
    items = []

    if len(groups) > 1:
        reason = f"동작 설정·월드가 다른 회차 {len(groups)}묶음(같은 조건 반복 아님)"

        for metric in metrics:
            item = fixed_item("B2-L8", metric, NOT_JUDGED, reason)
            item.update(runs=[run["path"] for run in chosen], excluded=excluded)
            items.append(item)
    else:
        single = [run["path"] for run in chosen if run["robot_ids"] == ["robot_2"] and run["active_robots"] == 1
                  and run["route_robot"] == "robot_2"]
        together = [run["path"] for run in chosen if run["active_robots"] >= 2
                    and set(run["active_robot_ids"] or []) <= set(run["robot_ids"])
                    and set(run["robot_ids"]) <= set(run["robots_with_errors"]) and run["route_robot"] in run["robot_ids"]]

        for metric, runs in zip(metrics, (single, together)):
            limit = repeats.get(metric.split(".")[1])
            item = criteria_item("B2-L8", metric, len(runs), limit, "min",
                                 reason=f"경로 완주·활성 불변·같은 조건 고유 회차 {len(runs)}개{note}")

            # 회차 수 부족은 성능 실패가 아니라 미충족이다.
            if item["result"] == FAIL:
                item.update(result=NOT_JUDGED, reason=f"회차 수 부족 {len(runs)}/{limit}(근거 있는 고유 회차만 셈){note}")

            item.update(runs=runs, excluded=excluded)
            items.append(item)

    counted = sorted({path for item in items for path in item.get("runs") or []})
    route = items[0].get("runs") is not None and len(groups) == 1 and counted
    item = (criteria_item("B2-L8", "repeats.min_route", len(counted), 1, "min",
                          reason=f"기준 경로({repeats.get('min_route')}) 완주가 검증된 고유 회차 {len(counted)}개")
            if route else fixed_item("B2-L8", "repeats.min_route", NOT_JUDGED,
                                     "기준 경로 완주가 검증된 회차 없음" + (f"(제외: {excluded[0][1]} 등)" if excluded else "")))
    item.update(limit=repeats.get("min_route"), runs=counted, excluded=excluded, method=ROUTE_METHOD)
    items.append(item)

    return items


# LD-2 부하 불변성: 같은 월드·동작 설정 묶음에서 L0(robot_2 단독, 활성 1대) 회차 주행 p95 평균 대비, 활성 2대 이상 회차의
# 같은 로봇 주행 p95 증가량 최대가 한계 이하인지 본다. 근거 회차는 B2-L8과 같은 조건(evidence_runs)이다.
# 부하 회차는 활성 로봇이 모두 측정 대상이고 모두 오차 자료가 있어야 한다(일부 자료만 있으면 비교하지 않음).
def load_invariance_item(summaries, criteria):
    metric = "load.max_moving_p95_increase_from_l0_m"
    limit = (criteria.get("load") or {}).get("max_moving_p95_increase_from_l0_m")
    chosen, excluded = evidence_runs(summaries)
    usable = [run for run in chosen if run["world_sha256"] and run["moving_p95_m"]]

    if not usable:
        item = fixed_item("LD-2", metric, NOT_JUDGED,
                          "부하 단계 정보 없음(활성 로봇 실측·월드 sha256·경로 완주·동작 설정 근거와 주행 p95가 있는 회차 없음)",
                          limit=limit)
        item["excluded"] = excluded
        return item

    comparisons, incomplete = [], []

    for key in sorted({condition_key(run) for run in usable}):
        same = [run for run in usable if condition_key(run) == key]
        baseline = {}

        for run in same:
            if run["active_robots"] == 1 and run["robot_ids"] == ["robot_2"]:
                for robot, value in run["moving_p95_m"].items():
                    baseline.setdefault(robot, []).append((run["path"], value))

        for run in same:
            if run["active_robots"] < 2:
                continue

            if not (set(run["active_robot_ids"] or []) <= set(run["robot_ids"])
                    and set(run["robot_ids"]) <= set(run["robots_with_errors"])):
                incomplete.append(run["path"])
                continue

            for robot, value in run["moving_p95_m"].items():
                if robot in baseline:
                    values = [entry[1] for entry in baseline[robot]]
                    reference = float(np.mean(values))
                    comparisons.append({"world_sha256": run["world_sha256"], "run": run["path"], "robot": robot,
                                        "p95_m": value, "l0_mean_p95_m": reference, "increase_m": value - reference,
                                        "l0_runs": len(values), "l0_values": baseline[robot]})

    if not comparisons:
        reason = "같은 조건의 L0(robot_2 단독)·다대 회차 짝 없음"
        item = fixed_item("LD-2", metric, NOT_JUDGED,
                          reason + (f"(로봇 자료 부족 다대 회차: {', '.join(incomplete)})" if incomplete else ""),
                          limit=limit)
        item["excluded"] = excluded
        return item

    worst = max(comparisons, key=lambda entry: entry["increase_m"])
    item = criteria_item("LD-2", metric, worst["increase_m"], limit, "max",
                         reason=f"비교 {len(comparisons)}건 중 최대 증가({worst['run']}, {worst['robot']})")
    item.update(comparisons=comparisons, runs=sorted({entry["run"] for entry in comparisons}), excluded=excluded,
                incomplete_runs=incomplete)

    return item


# LD-3 묶음: 벽시계 제한이 확인된 고유 회차 수를 상한과 비교한다. 무효(incomplete 등) 회차도 센다.
# 같은 회차(record_key)의 중복 제출은 한 번만 센다. 확인된 초과는 미확인 회차가 있어도 FAIL로 남긴다.
def wall_clock_runs_item(summaries, criteria):
    metric = "load.max_wall_clock_limited_runs"
    limit = (criteria.get("load") or {}).get("max_wall_clock_limited_runs")
    groups = {}

    for run in summaries:
        groups.setdefault(run["record_key"], []).append(run)

    limited = [runs[0]["path"] for runs in groups.values() if any(run["wall_clock_limited"] is True for run in runs)]
    unverified = [runs[0]["path"] for runs in groups.values()
                  if runs[0]["path"] not in limited and any(run["wall_clock_limited"] is None for run in runs)]
    note = f" / 확인 불가 회차 {len(unverified)}개: {', '.join(unverified)}" if unverified else ""
    item = criteria_item("LD-3", metric, len(limited), limit, "max",
                         reason=f"고유 회차 {len(groups)}개 중 벽시계 제한 {len(limited)}개{note}")

    if item["result"] == PASS and unverified:
        listed = ", ".join(unverified)
        item.update(result=NOT_JUDGED, reason=f"벽시계 제한 확인 불가 회차 {len(unverified)}개(0으로 메우지 않음): {listed}")

    item.update(runs=sorted(run["path"] for run in summaries), limited_runs=limited, unverified_runs=unverified,
                unique_runs=len(groups))

    return item


# 여러 회차 결과를 묶어 판정한다(순수 함수). runs: [(경로, 원본 결과, 회차 판정, 원본 sha256, 조건 dict)]
def judge_batch(runs, criteria, criteria_ref):
    summaries = [run_summary(path, result, judgment, digest, conditions)
                 for path, result, judgment, digest, conditions in runs]
    batch_items = repeat_items(summaries, criteria)

    batch_items.append(load_invariance_item(summaries, criteria))
    batch_items.append(wall_clock_runs_item(summaries, criteria))

    # 항목 ID마다 PASS·FAIL로 판정된 근거를 모은다. 한 ID의 모든 지표가 어느 유효 회차에서든 판정돼야 충족이다.
    # 무효 회차(incomplete·대상 불일치·SAMPLE_GAP)의 항목은 근거에서 뺀다.
    coverage = []
    run_failures = [run for run in summaries if run["judgment"] == FAIL]
    batch_failures = [item for item in batch_items if item["result"] == FAIL]

    for item_id in BATCH_COVERAGE_IDS:
        metrics = {}

        for (path, _, judgment, _, _), summary in zip(runs, summaries):
            if not summary["valid"]:
                continue

            for item in judgment["criteria_items"]:
                # 사건 시험(B2-L5~L7)은 회차에서 대상 아님(not_applicable)이어도 묶음 필수 지표로 둔다.
                # 한 조건(예: OK 중 요청)만 시험한 묶음이 충족으로 보이지 않게 한다.
                required = item_id in EVENT_TEST_IDS

                if item["id"] == item_id and (item["result"] != NOT_APPLICABLE or required):
                    metrics.setdefault(item["metric"], set())

                    if item["result"] in (PASS, FAIL):
                        metrics[item["metric"]].add(str(path))

        for item in batch_items:
            if item["id"] == item_id:
                metrics[item["metric"]] = set(item.get("runs") or []) if item["result"] in (PASS, FAIL) else set()

                if item["result"] in (PASS, FAIL) and not metrics[item["metric"]]:
                    metrics[item["metric"]] = {"(묶음)"}

        missing = sorted(metric for metric, evidence in metrics.items() if not evidence)
        satisfied = bool(metrics) and not missing
        reasons = [item["reason"] for item in batch_items if item["id"] == item_id and item["result"] == NOT_JUDGED]
        coverage.append({
            "id": item_id,
            "satisfied": satisfied,
            "runs": sorted(set().union(*metrics.values())) if metrics else [],
            "missing_metrics": missing,
            "reason": "; ".join(reasons) if reasons else ("" if satisfied else
                                                          "판정된 회차 없음" if metrics else "어느 회차에도 항목 없음"),
        })

    unmet = [entry["id"] for entry in coverage if not entry["satisfied"]]
    subset = None

    # 묶음 판정: FAIL 회차 → FAIL / 미충족 없음 → PASS / 그 밖 not_judged + 미충족 목록 + judged_subset
    if run_failures or batch_failures:
        judgment = FAIL
        parts = [f"{run['path']} ({run['judgment_reason']})" for run in run_failures]
        parts += [f"묶음 {item['id']} {item['metric']} {item['value']} > {item['limit']}" for item in batch_failures]
        reason = "FAIL: " + "; ".join(parts)
    elif not unmet:
        judgment, reason = PASS, "모든 기준 항목 충족"
    else:
        judgment = NOT_JUDGED
        reason = "미충족 항목: " + ", ".join(unmet)
        subset = PASS if any(entry["satisfied"] for entry in coverage) else None

    return {
        "criteria": dict(criteria_ref),
        "runs": summaries,
        "batch_items": batch_items,
        "coverage": coverage,
        "judgment": judgment,
        "judgment_reason": reason,
        **({"judged_subset": subset} if subset else {}),
        "unmet": unmet,
    }


# 묶음 출력 경로가 입력 결과 중 하나이거나 이미 있는 파일이면 쓰지 않는다(원본 덮어쓰기 방지). 문제 문자열 또는 None
def batch_output_problem(result_paths, output):
    output = Path(output).resolve()

    if output in {Path(path).resolve() for path in result_paths}:
        return f"--output이 입력 결과와 같음: {output}"

    if output.exists():
        return f"--output이 이미 있음(덮어쓰지 않음): {output}"

    return None


# 결과 파일 여러 개를 읽어 묶음 판정하고 output에 쓴다. 입력 원본은 읽기만 한다.
def batch(result_paths, criteria_path, output):
    problem = batch_output_problem(result_paths, output)

    if problem is not None:
        raise ValueError(problem)

    criteria, reference = load_criteria(criteria_path)
    runs = []

    for path in result_paths:
        result, judgment, digest = judge_file(path, criteria, reference, criteria_path)
        runs.append((str(path), result, judgment, digest, find_conditions(path)))

    judged = judge_batch(runs, criteria, reference)
    write_json_atomic(output, judged)

    return judged


# 인자를 읽고 정해진 시뮬레이션 시간 동안 기록한 뒤 결과를 쓴다.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-ids", default="robot_2")
    parser.add_argument("--scan", choices=sorted(SCAN_TOPICS), default="filtered",
                        help="기록·L9 수신률 대상 스캔(실제 AMCL 입력과 같게): filtered·raw(앞), merged(시험용 앞·뒤 병합)")
    parser.add_argument("--duration", type=float, default=30.0, help="기록 시간(시뮬레이션 s)")
    parser.add_argument("--wall-limit", type=float, default=900.0, help="벽시계 상한(s)")
    parser.add_argument("--world-map", type=triple, default=None, help="검증 전용 T_world_map x,y,yaw")
    parser.add_argument("--map-accuracy", type=Path, default=None, help="T_world_map을 읽을 지도 accuracy.json")
    parser.add_argument("--tf-future", type=float, default=1.0, help="AMCL transform_tolerance(s)")
    parser.add_argument("--gz-world", default="ax_test", help="Gazebo 월드 내부 이름(검증 기록용)")
    parser.add_argument("--label", default="")
    parser.add_argument("--output", type=Path, default=None, help="결과 폴더(--raw-dir만 주면 <raw-dir>/result)")
    parser.add_argument("--raw-dir", type=Path, default=None,
                        help="원시 기록 회차 폴더(log/b2_health/<KST 시각>_<label>). --rejudge와 함께 쓰면 읽기 전용 근거 폴더")
    parser.add_argument("--map", type=Path, default=None, help="조건 기록용 지도 yaml")
    parser.add_argument("--start-poses", type=Path, default=None, help="조건 기록용 start_poses yaml")
    parser.add_argument("--world-file", type=Path, default=None, help="조건 기록용 월드 파일")
    parser.add_argument("--analyze", type=Path, default=None, help="ROS 없이 원시 기록 회차 폴더를 분석")
    parser.add_argument("--stop-file", type=Path, default=None,
                        help="이 파일이 생기면 기록을 정상 종료한다(duration 전이어도 중단으로 보지 않음)")
    parser.add_argument("--hash-file", action="append", default=[], metavar="NAME=PATH",
                        help="조건 파일에 sha256을 남길 추가 파일(여러 번)")
    parser.add_argument("--criteria", type=Path, default=None,
                        help="개발용 판정 기준 YAML(localization_criteria_dev.yaml). 없으면 판정하지 않음(not_judged)")
    parser.add_argument("--rejudge", type=Path, default=None,
                        help="ROS 없이 과거 localization_result.json을 --criteria로 다시 판정해 같은 폴더에 *.judged.json을 쓴다")
    parser.add_argument("--batch", type=Path, nargs="+", default=None, metavar="RESULT",
                        help="ROS 없이 여러 localization_result.json을 --criteria로 묶음 판정해 --output(JSON 파일)에 쓴다")
    arguments = parser.parse_args()
    criteria = criteria_ref = None

    # 묶음 판정: 입력 결과는 읽기만 하고 --output 파일 하나를 쓴다.
    if arguments.batch is not None:
        if arguments.criteria is None or arguments.output is None:
            parser.error("--batch에는 --criteria와 --output(JSON 파일) 필요")

        if arguments.raw_dir is not None:
            parser.error("--batch에는 --raw-dir를 쓰지 않는다(회차마다 결과 옆 회차 폴더 또는 raw.dir 기록을 읽음)")

        problem = batch_output_problem(arguments.batch, arguments.output)

        if problem is not None:
            parser.error(problem)

        judged = batch(arguments.batch, arguments.criteria, arguments.output)
        print(f"묶음 판정: {arguments.output} (judgment={judged['judgment']}, {judged['judgment_reason']})")
        return 0

    # 과거 결과 재판정: 원본은 바꾸지 않고 새 파일만 쓴다.
    if arguments.rejudge is not None:
        if arguments.criteria is None:
            parser.error("--rejudge에는 --criteria 필요")

        # --raw-dir가 있으면 그 원시 기록 폴더를 읽기 전용으로 쓴다(B2-L3·LD-3 근거).
        output, judged = rejudge(arguments.rejudge, arguments.criteria, arguments.raw_dir)
        print(f"재판정: {output} (judgment={judged['judgment']}, {judged['judgment_reason']})")
        return 0

    if arguments.criteria is not None:
        criteria, criteria_ref = load_criteria(arguments.criteria)

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

    # 시뮬레이션 시각이 흐르기 시작한 뒤 duration만큼 기록한다. 벽시계 상한은 sim 시각 대기부터 잰다.
    wall_start = time.monotonic()
    wall_deadline = wall_start + arguments.wall_limit

    while node.now_seconds() == 0.0 and time.monotonic() < wall_deadline and not stop.is_set():
        time.sleep(0.05)

    start = node.now_seconds()
    print(f"[START] 기록 시작 sim {start:.2f} s, {arguments.duration} s 동안", flush=True)

    # 부하 단계 기록: 측정 시작 시 같은 domain의 활성 로봇(그래프 발견 지연을 고려해 약 2 s 동안 합침)
    active_ids = set()

    for _ in range(5):
        names = [name for name, types in node.get_topic_names_and_types()]
        active_ids |= set(active_robots_from_topics(names, node.count_publishers))
        time.sleep(0.4)

    world_sha = file_sha256(arguments.world_file) if arguments.world_file is not None else None
    load_stage = load_stage_conditions(arguments.world_file, sorted(active_ids), os.environ,
                                       gz_world_check(world_sha, os.environ.get("GZ_PARTITION")))

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
    result["conditions"].update(load_stage)
    # 종료 파일로 끝냈으면 그때까지가 요청 기록 시간이다(기록 시간 부족으로 보지 않음).
    finalize(result, start, end, end - start if stopped_by_file else arguments.duration, arguments.wall_limit)
    result["recording"]["stopped_by_file"] = stopped_by_file
    # 종료 원인(LD-3 근거): sim 기록 시간 충족·종료 파일·중단 신호·벽시계 상한을 구분해 남긴다.
    result["recording"]["termination"] = collector_termination(start, end, arguments.duration, wall_start, end_mono,
                                                               arguments.wall_limit, stopped_by_file, stop.is_set())

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

    # 기준 파일이 있으면 기준 항목과 전체 판정을 더한다(없으면 기존 not_judged 그대로).
    # 재판정(--rejudge)·묶음(--batch)과 같은 함수로 대상 대조·원시 근거(B2-L3·LD-3)·판정을 한다.
    if criteria is not None:
        result.update(criteria_judgment(result, criteria, criteria_ref, arguments.criteria, conditions,
                                        arguments.raw_dir, {"how": "측정 중 --raw-dir" if arguments.raw_dir
                                                            else "없음", "raw_dir": str(arguments.raw_dir)
                                                            if arguments.raw_dir else None}))

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
