#!/usr/bin/env python3
"""B2 지도 정확도 비교. SLAM 지도를 "Gazebo 실제 자세 기반 참조 지도"와 정렬·형상 비교한다.

  python3 src/ax_localization/scripts/check_map_accuracy.py <run 폴더> [--criteria 기준.yaml] [--adopt --map-dir <폴더>]

참조 지도: mapping_drive.py가 기록한 같은 앞 LiDAR 스캔을, 스캔 측정 시각에 보간한 Gazebo 차체 자세와
base_link→LiDAR 장착 변환으로 world 좌표에 누적한 점유 격자다. 자세를 보간할 수 없는 스캔은 제외하고 수를 남긴다.
두 지도는 같은 센서를 쓰므로 센서 오차·가림·LiDAR 높이(바닥 위 약 0.3 m)에서 보이지 않는 장애물은 함께 놓친다(검증 한계).

지표: 초기 기준 변환(T_world_map, 시작 시각의 실제 자세·odom·map→odom으로 계산)과 최적 정렬 변환(ICP)을 구분해 기록하고,
최적 정렬 기준으로 장애물 검출 비율·역방향 일치 비율·벽 위치 오차(평균·95백분위수·최대)·관측 범위를 계산한다.
기준 파일(--criteria)이 없으면 판정하지 않는다(judgment: not_judged). 큰 정렬량만으로 실패 판정하지 않는다.

--adopt: 저장 파일을 검사한 뒤 --map-dir 폴더에 map.yaml·map.pgm·pose graph·map_info.yaml을 교체한다.
  판정(--criteria)이 PASS일 때만 채택하고, FAIL·not_judged면 파일을 바꾸지 않고 종료 코드 1로 끝낸다.
  map_info.yaml의 경로는 작업공간 기준 상대 경로(밖이면 "~/" 기준)로 기록한다. 대조는 해시로 한다.
교체 전 검사가 실패하면 기존 지도를 그대로 둔다. 새 월드 회차는 --map-dir를 명시해야 하며, 옛 AWS 형식 회차만
생략 시 maps/aws_warehouse를 쓴다.

월드·실행 조건: 새 월드 회차는 session.json의 월드 경로(world)·Gazebo 내부 이름(gz_world)·월드 파일 해시와
회차 사본(session/world.world·route.yaml·config/*)을 대조한다. 설정 해시는 회차 기록(config_files)만 쓰고 현재 파일로
채우지 않는다. 불일치·누락은 메타정보 문제로 남겨 채택하지 않는다. 부분 경로(--waypoints) 결과는 단계별 결과를
보존하되 전체 status를 incomplete(종료 코드 1)로 둔다.
"""

import argparse
from datetime import datetime
import copy
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from zoneinfo import ZoneInfo

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import map_tools as mt  # noqa: E402

# 스캔 시각과 앞뒤 실제 자세 표본 간격이 이보다 크면 보간하지 않는다(s).
DEFAULT_MAX_GAP_S = 0.1

# 옛 AWS 월드 이름. session.json에 world가 없거나 이 값이면 옛 AWS 형식 회차로 읽는다.
AWS_WORLD = "aws_warehouse.world"
AWS_MAP_DIR = "src/ax_localization/maps/aws_warehouse"

# run_mapping_session.sh가 실행 전에 사본·해시를 남기는 설정(session.json config_files). 제어기 설정은 로봇별로 다르다
# (robot_test.launch.py CONTROLLER_FILES·simulation.launch.py MANIPULATORS).
COMMON_CONFIG_NAMES = ("slam_mapping.yaml", "common_controllers.yaml", "sensor_mounts.yaml", "a1_validation.yaml")
CONTROLLER_FILES = {"robot_1": "ax1_controllers.yaml", "robot_3": "ax1_controllers.yaml",
                    "robot_2": "ax2_controllers.yaml", "robot_4": "ax2_controllers.yaml"}


# 로봇에 필요한 설정 파일 이름 목록(제어기 파일 포함). 모르는 로봇이면 제어기 없이 돌려준다.
def required_config_names(robot_id):
    controller = CONTROLLER_FILES.get(robot_id)
    return (COMMON_CONFIG_NAMES[0], *((controller,) if controller else ()), *COMMON_CONFIG_NAMES[1:])


# 저장소 루트: src/ax_localization이 있는 상위 폴더. 설치본에서 실행하면 현재 폴더.
def workspace_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "src/ax_localization").is_dir():
            return parent

    return Path.cwd()


WORKSPACE = workspace_root()


# 메타정보에 남길 경로를 작업 PC에 묶이지 않는 형태로 바꾼다(해시는 따로 기록하므로 경로는 표기용).
# - 작업공간 안: 작업공간 기준 상대 경로(예: install/ax_localization/..., log/...)
# - 홈 폴더 안(작업공간 밖): "~/" 기준 경로
# - 그 밖의 절대 경로·상대 경로·빈 값: 그대로
def portable_path(value, workspace=None, home=None):
    if not value:
        return value

    path = Path(value)

    if not path.is_absolute():
        return str(value)

    workspace = Path(workspace or WORKSPACE).resolve()
    home = Path(home or Path.home()).resolve()

    if path.is_relative_to(workspace):
        return path.relative_to(workspace).as_posix()

    if path.is_relative_to(home):
        return "~/" + path.relative_to(home).as_posix()

    return str(value)


# 메타정보의 경로 필드(세션·주행 기록에서 옮긴 값)를 portable_path로 바꾼 사본을 돌려준다. 해시 값은 바꾸지 않는다.
def portable_metadata(info, workspace=None, home=None):
    info = copy.deepcopy(info)

    def convert(container, key):
        if isinstance(container, dict) and isinstance(container.get(key), str):
            container[key] = portable_path(container[key], workspace, home)

    convert(info.get("world") or {}, "world_file")
    convert(((info.get("robot") or {}).get("sensor") or {}).get("scan_merge") or {}, "file")
    convert(info.get("slam") or {}, "config")
    convert(info.get("route") or {}, "file")
    convert(info.get("wheel_friction") or {}, "override_file")

    for record in (info.get("config_files") or {}).values():
        convert(record, "source")

    for key in ("result", "drive_result"):
        convert(info.get("verification") or {}, key)

    return info


# 임시 파일에 쓴 뒤 바꿔 넣는다.
def write_json_atomic(path, data):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=float) + "\n", encoding="utf-8")
    os.replace(temporary, path)


# 기록 npz를 읽는다.
def load_recording(run):
    data = np.load(run / "recording.npz")

    return {key: data[key] for key in data.files}


# 실제 자세로 앞 LiDAR 스캔을 world 좌표에 누적해 참조 점유 격자를 만든다.
def build_reference(recording, max_gap, occupied_ratio, resolution=0.05):
    truth = recording["truth"]
    angle_min, increment, range_min, range_max = recording["scan_layout"]
    mount = mt.se2(*recording["front_mount"])

    if len(truth) < 2:
        raise ValueError("Gazebo 실제 자세 기록이 부족함")

    # 시각 순서로 정렬하고 같은 시각 중복을 없앤다.
    order = np.argsort(truth[:, 0], kind="stable")
    truth = truth[order]
    keep = np.concatenate([[True], np.diff(truth[:, 0]) > 0])
    truth = truth[keep]

    # 격자 범위: 실제 경로 ± 최대 거리 + 1 m
    span = range_max + 1.0
    low = truth[:, 1:3].min(axis=0) - span
    high = truth[:, 1:3].max(axis=0) + span
    size = np.ceil((high - low) / resolution).astype(int)
    geometry = mt.GridGeometry(low[0], low[1], resolution, size[0], size[1])
    accumulator = mt.OccupancyAccumulator(geometry)

    stats = {"scans": int(len(recording["scan_stamps"])), "used": 0, "excluded_outside_truth": 0, "excluded_gap": 0}

    for stamp, ranges in zip(recording["scan_stamps"], recording["scan_ranges"]):
        if stamp < truth[0, 0] or stamp > truth[-1, 0]:
            stats["excluded_outside_truth"] += 1
            continue

        pose = mt.interpolate_pose(truth[:, 0], truth[:, 1:4], stamp, max_gap)

        if pose is None:
            stats["excluded_gap"] += 1
            continue

        angles, distances, hit = mt.scan_rays(ranges, angle_min, increment, range_min, range_max)
        accumulator.add_scan(mt.se2(*pose) @ mount, angles, distances, hit)
        stats["used"] += 1

    return accumulator.classify(occupied_ratio), geometry, stats, truth


# 시작 시각의 실제 자세·odom·map→odom으로 초기 기준 변환 T_world_map을 구한다.
# T_world_odom = T_world_base · T_odom_base⁻¹, T_world_map = T_world_odom · T_map_odom⁻¹
def initial_world_map(recording, truth, max_gap):
    odom = recording["odom"]
    map_odom = recording["map_odom"]

    if len(map_odom) == 0:
        raise ValueError("map→odom 기록 없음")

    for sample in odom:
        pose = mt.interpolate_pose(truth[:, 0], truth[:, 1:4], sample[0], max_gap)

        if pose is None:
            continue

        world_odom = mt.se2(*pose) @ np.linalg.inv(mt.se2(*sample[1:4]))

        # 그 시각 이후 첫 map→odom 표본을 쓴다(SLAM 시작 직후 값).
        later = map_odom[map_odom[:, 0] >= sample[0]]
        reference = later[0] if len(later) else map_odom[0]

        return world_odom @ np.linalg.inv(mt.se2(*reference[1:4])), world_odom, {
            "odom_time_s": float(sample[0]),
            "map_odom_time_s": float(reference[0]),
            "method": "T_world_odom(실제 자세·odom 같은 시각) · T_map_odom⁻¹(그 시각 이후 첫 map→odom)",
        }

    raise ValueError("실제 자세와 겹치는 odom 표본 없음")


# 같은 시각의 SLAM 차체 자세·odom 자세를 Gazebo 실제 자세와 비교한다(world 기준).
# SLAM 자세 = T_world_map(초기 기준) · T_map_odom(그 시각 이전 마지막 SLAM 갱신) · T_odom_base(odom)
# odom 자세 = T_world_odom(시작 시각) · T_odom_base. period_s 간격으로 표본을 고른다.
def trajectory_errors(recording, truth, world_map, world_odom, max_gap, period_s=0.5):
    odom = recording["odom"]
    map_odom = recording["map_odom"]
    rows = []
    last = -math.inf

    for sample in odom:
        if sample[0] - last < period_s:
            continue

        pose = mt.interpolate_pose(truth[:, 0], truth[:, 1:4], sample[0], max_gap)
        earlier = map_odom[map_odom[:, 0] <= sample[0]]

        if pose is None or len(earlier) == 0:
            continue

        last = sample[0]
        actual = mt.se2(*pose)
        base = mt.se2(*sample[1:4])
        slam = world_map @ mt.se2(*earlier[-1][1:4]) @ base
        dead = world_odom @ base

        slam_error = mt.se2_params(np.linalg.inv(actual) @ slam)
        odom_error = mt.se2_params(np.linalg.inv(actual) @ dead)
        rows.append([sample[0], math.hypot(*slam_error[:2]), slam_error[2], math.hypot(*odom_error[:2]), odom_error[2]])

    if not rows:
        return None

    rows = np.array(rows)

    # 평균·95백분위수·최대 요약(각도는 절댓값)
    def stats(values):
        values = np.abs(values)

        return {"mean": float(values.mean()), "p95": float(np.percentile(values, 95)), "max": float(values.max())}

    return {
        "method": "같은 odom 표본 시각의 SLAM·odom 차체 자세와 보간한 Gazebo 실제 자세 비교. SLAM 자세는 초기 T_world_map 기준",
        "samples": int(len(rows)),
        "slam_position_error_m": stats(rows[:, 1]),
        "slam_yaw_error_rad": stats(rows[:, 2]),
        "odom_position_error_m": stats(rows[:, 3]),
        "odom_yaw_error_rad": stats(rows[:, 4]),
        "series": {"columns": ["sim_time_s", "slam_position_error_m", "slam_yaw_error_rad",
                               "odom_position_error_m", "odom_yaw_error_rad"],
                   "rows": np.round(rows, 4).tolist()},
    }


# 변환을 기록용 사전으로 만든다.
def transform_record(transform):
    x, y, yaw = mt.se2_params(transform)

    return {"x_m": x, "y_m": y, "yaw_rad": yaw}


# 기준 파일(임계값)로 판정한다. 기준이 없으면 not_judged.
def judge(metrics, criteria):
    if criteria is None:
        return {"judgment": "not_judged", "reason": "정확도 기준 미확정. 지표 수집만 했으며 PASS로 표시하지 않는다"}

    checks = {
        "obstacle_detection_ratio": metrics["obstacle_detection_ratio"] is not None
        and metrics["obstacle_detection_ratio"] >= criteria["min_obstacle_detection_ratio"],
        "reverse_match_ratio": metrics["reverse_match_ratio"] is not None
        and metrics["reverse_match_ratio"] >= criteria["min_reverse_match_ratio"],
        "wall_error_p95_m": metrics["wall_error_reference_to_compared"] is not None
        and metrics["wall_error_reference_to_compared"]["p95_m"] <= criteria["max_wall_error_p95_m"],
        "comparable_area_m2": metrics["coverage"]["comparable_area_m2"] >= criteria["min_comparable_area_m2"],
    }

    return {"judgment": "PASS" if all(checks.values()) else "FAIL", "checks": checks, "criteria": criteria}


# 참조·SLAM 점유와 실제 경로를 겹친 그림을 저장한다. 참조 격자 기준이며 위쪽이 +y다.
def draw_overlay(path, reference, compared, geometry, truth):
    import cv2

    image = np.full(reference.shape + (3,), 160, np.uint8)
    image[(reference == mt.FREE) | (compared == mt.FREE)] = (255, 255, 255)

    both = (reference == mt.OCCUPIED) & mt.dilate(compared == mt.OCCUPIED, 1)
    image[reference == mt.OCCUPIED] = (255, 120, 0)          # 참조만: 파랑
    image[compared == mt.OCCUPIED] = (0, 0, 230)             # SLAM만: 빨강
    image[both] = (0, 0, 0)                                  # 일치: 검정

    rows, columns = geometry.cells(truth[:, 1:3])
    inside = geometry.inside(rows, columns)
    image[rows[inside], columns[inside]] = (0, 170, 0)       # 실제 경로: 초록

    image = cv2.resize(image[::-1], None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    cv2.putText(image, "blue=reference only  red=SLAM only  black=match  green=truth path", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.imwrite(str(path), image)


# 저장된 SLAM 지도·pose graph를 검사한다. 문제 목록을 돌려준다.
def check_saved_files(run):
    problems = []

    for name in ("slam_map.yaml", "slam_map.pgm", "pose_graph.posegraph", "pose_graph.data"):
        path = run / name

        if not path.is_file() or path.stat().st_size == 0:
            problems.append(f"{name} 없음 또는 빈 파일")

    if not problems:
        try:
            grid, _, _ = mt.read_map(run / "slam_map.yaml")

            if not (grid == mt.OCCUPIED).any():
                problems.append("slam_map에 점유 셀 없음")
        except (OSError, ValueError, KeyError) as error:
            problems.append(f"slam_map 읽기 실패: {error}")

    return problems


# 주행 기록의 저장 직전·종료 odom 기준 정지 확인과 저장 지도 사용 가능 표시를 검사해 문제 목록을 돌려준다.
# 정지 확인 기록이 없는 이전 형식 결과도 채택하지 않는다.
def stop_gate_problems(drive):
    gate = drive.get("stop_gate")
    problems = []

    if not isinstance(gate, dict):
        return ["정지 확인 기록(stop_gate) 없음"]

    for key, label in (("save", "저장 전"), ("completion", "종료")):
        if not (gate.get(key) or {}).get("confirmed"):
            problems.append(f"{label} odom 기준 정지 미확인")

    if drive.get("saved_map_usable") is not True:
        problems.append("저장 지도 사용 불가 표시")

    return problems


# 저장 지도(slam_map)와 채택용 지도(map.yaml)를 각각 다시 읽어 격자·원점·해상도·크기가 같은지 확인한다.
# 지도 파일 수준의 재로딩 확인이며 slam_toolbox pose graph 재로딩은 포함하지 않는다.
def reload_check(saved_yaml, adopted_yaml):
    saved_grid, saved_geometry, saved_yaw = mt.read_map(saved_yaml)
    adopted_grid, adopted_geometry, adopted_yaw = mt.read_map(adopted_yaml)
    same_grid = saved_grid.shape == adopted_grid.shape and bool((saved_grid == adopted_grid).all())
    same_geometry = saved_geometry.as_dict() == adopted_geometry.as_dict() and saved_yaw == adopted_yaw

    return {"match": same_grid and same_geometry, "grid": same_grid, "origin_resolution_size": same_geometry,
            "scope": "map.yaml·map.pgm 파일 재로딩. pose graph 재로딩은 포함하지 않음"}


# run 폴더의 지도를 target에 채택한다. 검사 실패 시 기존 지도를 건드리지 않는다.
# 새 파일을 같은 폴더의 임시 폴더에 모두 만든 뒤 하나씩 바꿔 넣는다.
# numpy 값(정수·실수·배열)을 YAML로 쓸 수 있는 기본 타입으로 바꾼다. 다른 값은 그대로 둔다.
def plain_types(value):
    if isinstance(value, dict):
        return {key: plain_types(item) for key, item in value.items()}

    if isinstance(value, (list, tuple)):
        return [plain_types(item) for item in value]

    if isinstance(value, np.ndarray):
        return [plain_types(item) for item in value.tolist()]

    if isinstance(value, np.generic):
        return value.item()

    return value


def adopt_map(run, target, info):
    problems = check_saved_files(run)

    if problems:
        raise RuntimeError("채택 불가: " + ", ".join(problems))

    target.mkdir(parents=True, exist_ok=True)
    previous = target / "map_info.yaml"
    old = yaml.safe_load(previous.read_text(encoding="utf-8")) if previous.is_file() else {}
    info = dict(info, map_revision=int(old.get("map_revision", 0)) + 1)

    with tempfile.TemporaryDirectory(dir=target, prefix=".adopt_") as staging:
        staging = Path(staging)

        # 지도 YAML은 이미지 이름만 바꿔 다시 쓴다.
        slam_yaml = yaml.safe_load((run / "slam_map.yaml").read_text(encoding="utf-8"))
        slam_yaml["image"] = "map.pgm"
        (staging / "map.yaml").write_text(yaml.safe_dump(slam_yaml, sort_keys=False), encoding="utf-8")
        shutil.copyfile(run / "slam_map.pgm", staging / "map.pgm")
        shutil.copyfile(run / "pose_graph.posegraph", staging / "pose_graph.posegraph")
        shutil.copyfile(run / "pose_graph.data", staging / "pose_graph.data")

        info["files"] = {name: mt.file_sha256(staging / name)
                         for name in ("map.yaml", "map.pgm", "pose_graph.posegraph", "pose_graph.data")}
        info["reload_check"] = reload_check(run / "slam_map.yaml", staging / "map.yaml")

        if not info["reload_check"]["match"]:
            raise RuntimeError("채택 불가: 채택용 지도를 다시 읽은 결과가 저장 지도와 다름 " + str(info["reload_check"]))

        (staging / "map_info.yaml").write_text(
            "# B2 지도 메타데이터. 지도가 바뀌면 map_revision이 오르며 등록 지점·재고 좌표는 revision을 대조해 재검증한다.\n"
            + yaml.safe_dump(plain_types(info), sort_keys=False, allow_unicode=True), encoding="utf-8")

        # map_info.yaml은 마지막에 바꿔 revision과 파일이 어긋나는 시간을 줄인다.
        for name in ("map.yaml", "map.pgm", "pose_graph.posegraph", "pose_graph.data", "map_info.yaml"):
            os.replace(staging / name, target / name)

    return info


# 실행 조건 기록(run_mapping_session.sh의 session.json)을 읽는다. 없으면 None.
def read_session(run):
    path = run.parent / "session" / "session.json"

    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None


# 옛 AWS 형식 회차인지(session.json이 없거나 world가 없거나 AWS 월드). 새 월드 회차는 session.json에 world가 있다.
def is_legacy_aws(session):
    return session is None or session.get("world") in (None, AWS_WORLD)


# 회차 session 폴더 안 사본 파일의 SHA-256. session.json의 상대 경로(session/…)를 run 폴더 기준으로 푼다.
def session_copy_sha(run, relative):
    if not relative:
        return None

    path = run.parent / relative
    return mt.file_sha256(path) if path.is_file() else None


# 기준 월드 기록과 새 월드의 일치 문제. 새 월드는 회차 session.json의 실행 월드 경로·내부 이름·파일 해시를 쓰고
# 월드 사본(session/world.world)·경로 사본(session/route.yaml)의 world·주행 기록의 world와 대조한다.
# 옛 AWS 형식은 기존처럼 AWS 출처(SOURCE.json)를 붙인다. AWS 출처는 AWS 기록에만 붙인다.
def world_record(run, session, drive):
    problems, missing = [], []

    if is_legacy_aws(session):
        source_file = WORKSPACE / "world_assets/aws_small_warehouse/SOURCE.json"
        source = json.loads(source_file.read_text(encoding="utf-8")) if source_file.is_file() else {}
        world = {"launch_name": AWS_WORLD, "format": "legacy_aws",
                 "reference_assets": {"repository": source.get("repository"), "commit": source.get("commit"),
                                      "archive_sha256": source.get("archive_sha256")}}

        if session and session.get("world_sha256"):
            world.update({key: session.get(key) for key in ("world_condition", "world_file", "world_sha256")})
        else:
            world.update({"world_condition": (session or {}).get("world_condition"), "world_file": None, "world_sha256": None})
            missing.append("world: session.json에 실행 월드 파일 해시 없음")

        return world, missing, problems

    name = session["world"]
    world = {"launch_name": name, "gz_world": session.get("gz_world"), "format": "session_world",
             "world_condition": session.get("world_condition"), "world_file": session.get("world_file"),
             "world_sha256": session.get("world_sha256"), "reference_assets": None,
             "source_note": f"src/ax_description/{name if '/' not in name else 'worlds/' + name} (프로젝트 월드, 외부 출처 없음)"}

    if not world["world_sha256"]:
        missing.append("world: session.json에 실행 월드 파일 해시 없음")

    if not world["gz_world"]:
        problems.append("world: session.json에 Gazebo 내부 이름(gz_world) 없음")

    copy_sha = session_copy_sha(run, "session/world.world")
    world["world_copy"] = {"file": "session/world.world", "sha256": copy_sha,
                           "matches": copy_sha is not None and copy_sha == world["world_sha256"]}

    if not world["world_copy"]["matches"]:
        problems.append(f"world: 회차 월드 사본 해시({copy_sha})가 기록 해시({world['world_sha256']})와 다름 또는 사본 없음")

    route_copy = run.parent / (session.get("route_copy") or "session/route.yaml")

    try:
        route_world = (yaml.safe_load(route_copy.read_text(encoding="utf-8")) or {}).get("world")
    except (OSError, yaml.YAMLError) as error:
        route_world = None
        problems.append(f"route: 회차 경로 사본을 읽지 못함({type(error).__name__})")

    world["route_world"] = route_world

    # 재질만 다른 월드(같은 structure, config/worlds.yaml)는 같은 경로를 쓴다.
    if not mt.same_world_structure(route_world, name):
        problems.append(f"route: 경로 world {route_world!r} ≠ 실행 월드 {name!r}(구조가 다른 월드)")

    route_sha = session_copy_sha(run, session.get("route_copy") or "session/route.yaml")

    if not session.get("route_sha256") or route_sha != session.get("route_sha256") or drive["route"].get("sha256") != route_sha:
        problems.append(f"route: 경로 해시 불일치(session {session.get('route_sha256')}, 사본 {route_sha}, "
                        f"주행 기록 {drive['route'].get('sha256')})")

    drive_world = drive.get("world") or {}

    if drive_world.get("file") != name or drive_world.get("gz_world") != world["gz_world"]:
        problems.append(f"world: 주행 기록 world {drive_world} ≠ 실행 월드 {name!r}·{world['gz_world']!r}")

    return world, missing, problems


# 실행 시점 설정(session.json config_files)의 기록 해시와 회차 사본 해시를 대조한다. 현재 파일로 채우지 않는다.
# 기록 형식: {"robot_id", "controller_file", "files": {이름: {package, source, copy, sha256}}}.
# 주행 로봇(robot_id)에 맞는 제어기 파일이 기록돼 있어야 하며, 다른 로봇용 제어기 기록은 문제로 남긴다.
def config_record(run, session, robot_id):
    recorded = (session or {}).get("config_files")
    files, problems = {}, []

    if not isinstance(recorded, dict) or not isinstance(recorded.get("files"), dict):
        return {}, ["config_files: session.json에 실행 시점 설정 사본·해시 기록 없음(현재 파일로 채우지 않음)"]

    expected_controller = CONTROLLER_FILES.get(robot_id)

    if expected_controller is None:
        problems.append(f"config_files: 제어기 설정을 정할 수 없는 로봇 {robot_id!r}")

    if recorded.get("robot_id") != robot_id or recorded.get("controller_file") != expected_controller:
        problems.append(f"config_files: 기록 로봇·제어기({recorded.get('robot_id')}, {recorded.get('controller_file')})가 "
                        f"주행 로봇({robot_id}, {expected_controller})과 다름")

    for name in required_config_names(robot_id):
        item = recorded["files"].get(name)

        if not isinstance(item, dict) or not item.get("sha256"):
            problems.append(f"config_files: {name} 기록 없음")
            continue

        copy_sha = session_copy_sha(run, item.get("copy"))
        files[name] = {"package": item.get("package"), "source": item.get("source"), "copy": item.get("copy"),
                       "recorded_sha256": item["sha256"], "copy_sha256": copy_sha, "copy_matches": copy_sha == item["sha256"]}

        if copy_sha != item["sha256"]:
            problems.append(f"config_files: {name} 사본 해시({copy_sha})가 기록 해시와 다름 또는 사본 없음")

    unexpected = sorted(set(recorded["files"]) - set(required_config_names(robot_id)))

    if unexpected:
        problems.append(f"config_files: 주행 로봇에 맞지 않는 설정 기록 {unexpected}")

    return files, problems


# SLAM 입력 스캔 종류(session.json scan, 주행 기록 slam_scan_input)와 병합 노드 기록을 대조한다.
# 돌려주는 값: (입력 종류 또는 None, {"file", "sha256"} 또는 None, 문제 목록). 옛 기록(둘 다 없음)은 문제로 보지 않는다.
def slam_input_record(session, drive):
    recorded = (session or {}).get("scan")
    driven = drive.get("slam_scan_input")
    scan_merge = (session or {}).get("scan_merge")
    problems = []

    if recorded and driven and recorded != driven:
        problems.append(f"slam_input: session.json scan({recorded})과 주행 기록 slam_scan_input({driven})이 다름")

    if (recorded or driven) == "merged" and not (isinstance(scan_merge, dict) and scan_merge.get("sha256")):
        problems.append("slam_input: 병합 입력인데 session.json에 scan_merge 파일·해시 기록 없음")

    return recorded or driven, scan_merge, problems


# 채택 메타데이터를 만든다. 확보하지 못한 항목은 값을 채우지 않고 missing에 사유를 남긴다.
# 실행 조건 불일치(새 월드 월드·경로·설정)는 consistency_problems에 남겨 채택 전에 거절한다.
def map_metadata(run, drive, accuracy, slam_yaml):
    route_path = Path(drive["route"]["path"])
    session = read_session(run)
    model = drive.get("model") or {}
    world, missing, consistency = world_record(run, session, drive)
    configs, config_problems = config_record(run, session, drive["robot_id"])

    # 옛 AWS 형식 회차는 설정 기록이 없던 형식이라 누락으로 남기고, 새 월드 회차는 채택 전 문제로 본다.
    if is_legacy_aws(session):
        missing += config_problems
    else:
        consistency += config_problems

    # 바퀴 조건(운영/임시 사본)은 실행 조건 기록에서, 실제 마찰 값은 spawn robot_description에서 읽는다.
    friction = {"condition": (session or {}).get("wheel_condition"), "wheel_contacts": None,
                "override_file": (session or {}).get("wheel_override_file") or None,
                "override_sha256": (session or {}).get("wheel_override_sha256") or None}

    # 모델·바퀴 마찰·센서: mapping_drive가 받은 spawn robot_description
    if model.get("sha256") and "missing_reason" not in model:
        robot_model = {key: model.get(key) for key in ("source", "file", "sha256", "robot_name")}
        friction["wheel_contacts"] = model.get("wheel_contacts")
        lidars = model.get("lidars")
    else:
        robot_model, lidars = None, None
        missing.append("model: " + (model.get("missing_reason") or "drive_result.json에 robot_description 기록 없음"))

    if session is None:
        missing.append("session: session.json 없음(실행 조건·코드 버전 미확인)")

    # SLAM 입력 스캔 종류와 병합 노드(scan_merge.py) 해시. 병합 입력인데 기록이 없거나 주행 기록과 다르면 채택 전 문제로 본다.
    slam_input, scan_merge, input_problems = slam_input_record(session, drive)
    (missing if is_legacy_aws(session) else consistency).extend(input_problems)

    metadata = {
        "created": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(timespec="seconds"),
        "map": mt.map_summary(run / "slam_map.yaml"),
        "world": world,
        "robot": {"id": drive["robot_id"], "model": robot_model,
                  "sensor": {"input": drive["scan_input"], "slam_input": slam_input, "scan_merge": scan_merge,
                             "frame": accuracy["scan_frame"],
                             "mount_base_link": accuracy["front_mount"], "lidars": lidars}},
        "wheel_friction": friction,
        "code": {key: (session or {}).get(key) for key in ("git_commit", "git_dirty")},
        "slam": {"package": "slam_toolbox (async, mapping)",
                 "config": (configs.get("slam_mapping.yaml") or {}).get("source"),
                 "config_copy": (configs.get("slam_mapping.yaml") or {}).get("copy"),
                 # 회차 기록 해시만 쓴다. 기록이 없으면 None(현재 파일 해시로 채우지 않는다).
                 "config_sha256": (configs.get("slam_mapping.yaml") or {}).get("recorded_sha256"),
                 "resolution_m": slam_yaml["resolution"]},
        "config_files": configs,
        "route": {"file": str(route_path), "sha256": drive["route"]["sha256"]},
        "pose_graph": {"files": ["pose_graph.posegraph", "pose_graph.data"],
                       "resume": "slam_toolbox deserialize_map 또는 map_file_name 파라미터로 이어 작성·위치 추정"},
        "stop_gate": drive.get("stop_gate"),
        "verification": {
            "judgment": accuracy["judgment"],
            "judgment_reason": accuracy.get("reason"),
            "metrics": accuracy["metrics_optimal"],
            "result": str(run / "accuracy.json"),
            "drive_result": str(run / "drive_result.json"),
        },
        "T_world_map": {
            "use": "검증 전용. 운영 위치 추정·주행에 쓰지 않는다",
            "initial": accuracy["transforms"]["initial_T_world_map"],
            "optimal": accuracy["transforms"]["optimal_T_world_map"],
            "method": accuracy["transforms"]["initial_method"] + " → 참조 지도와 ICP 정렬",
        },
        "missing": missing,
        "consistency_problems": consistency,
    }

    # 세션·주행 기록에서 옮긴 경로는 작업 PC 절대 경로가 남지 않게 바꾼다(해시는 그대로).
    return portable_metadata(metadata)


# 채택 조건 중 메타데이터 문제(누락·개발용 임시 사본 조건)를 돌려준다.
def metadata_problems(metadata):
    problems = list(metadata["missing"]) + list(metadata.get("consistency_problems", []))
    world_condition = metadata["world"].get("world_condition")
    wheel_condition = metadata["wheel_friction"]["condition"]

    if world_condition != "original":
        problems.append(f"월드 조건 {world_condition}은 개발용 임시 사본")

    if wheel_condition != "original":
        problems.append(f"바퀴 조건 {wheel_condition}은 개발용 임시 사본")

    return problems


# 전체 평가 상태와 사유. 세 단계(주행·궤적·지도)가 모두 completed이고 전체 경로일 때만 completed다.
# 부분 경로(--waypoints)는 지도 비교까지 끝났어도 incomplete이며, 세부 결과는 그대로 둔다.
def overall_status(result):
    stages = ("drive_status", "trajectory_comparison", "map_comparison")
    pending = [key for key in stages if result[key]["status"] != "completed"]

    if pending:
        return "incomplete", "미완료 단계: " + ", ".join(pending)

    if result["drive_status"].get("partial_route"):
        return "incomplete", "부분 경로(--waypoints): 단계별 결과는 보존하지만 전체 경로 평가가 아님"

    return "completed", None


# 채택 대상 폴더와 문제 목록. 새 월드 회차는 --map-dir를 명시해야 한다(기본 폴더로 추정하지 않음).
# 옛 AWS 형식 회차만 기존 기본 폴더(maps/aws_warehouse)를 쓴다.
def adoption_target(map_dir, metadata):
    if map_dir is not None:
        return Path(map_dir), []

    if metadata["world"].get("format") == "legacy_aws":
        return WORKSPACE / AWS_MAP_DIR, []

    return None, [f"채택 대상 폴더 미지정: 새 월드({metadata['world'].get('launch_name')})는 --map-dir를 명시해야 함"]


# 명령행 인자를 읽어 비교하고 결과를 저장한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path, help="mapping_drive.py 결과 폴더")
    parser.add_argument("--max-gap", type=float, default=DEFAULT_MAX_GAP_S, help="자세 보간 최대 표본 간격(s)")
    parser.add_argument("--occupied-ratio", type=float, default=0.25, help="참조 지도 점유 판정 비율")
    parser.add_argument("--tolerance-cells", type=int, default=1, help="일치 판정 허용 셀(±)")
    parser.add_argument("--criteria", type=Path, default=None, help="확정 임계값 YAML. 없으면 판정하지 않음")
    parser.add_argument("--adopt", action="store_true",
                        help="검사 후 --map-dir에 채택(판정 PASS일 때만, 새 월드는 --map-dir 필수)")
    parser.add_argument("--map-dir", type=Path, default=None,
                        help="채택 대상 폴더. 생략하면 옛 AWS 형식 회차만 maps/aws_warehouse를 쓰고 새 월드는 거절")
    arguments = parser.parse_args()

    run = arguments.run.resolve()

    # 세 상태를 따로 기록한다. 주행이 실패해도 궤적 비교는 끝날 수 있고, 지도 미생성·비교 미실행은 PASS가 아니다.
    result = {
        "run": str(run),
        "drive_status": {"status": "unknown"},
        "trajectory_comparison": {"status": "not_run"},
        "map_comparison": {"status": "not_run"},
        "judgment": "not_judged",
    }

    # 1) 주행 상태(주행 도구 기록을 그대로 옮긴다)
    try:
        drive = json.loads((run / "drive_result.json").read_text(encoding="utf-8"))
        result["drive_status"] = {"status": drive.get("status"), "error": drive.get("error"),
                                  "waypoints_reached": drive.get("waypoints_reached"),
                                  "partial_route": drive.get("route", {}).get("partial", False),
                                  "stop_gate": drive.get("stop_gate"), "saved_map_usable": drive.get("saved_map_usable")}
    except (OSError, ValueError) as error:
        drive = None
        result["drive_status"] = {"status": "missing", "error": f"{type(error).__name__}: {error}"}

    # 2) 궤적 비교: 기록(recording.npz)만 있으면 지도 저장 여부와 무관하게 계산한다.
    recording = truth = initial = None

    try:
        recording = load_recording(run)
        reference, geometry, stats, truth = build_reference(recording, arguments.max_gap, arguments.occupied_ratio)
        result["reference"] = {"name": "Gazebo 실제 자세 기반 참조 지도", "geometry": geometry.as_dict(),
                               "samples": stats, "occupied_ratio": arguments.occupied_ratio,
                               "max_gap_s": arguments.max_gap}
        result["scan_frame"] = str(recording["scan_frame"])
        result["front_mount"] = [float(v) for v in recording["front_mount"]]

        initial, world_odom, method = initial_world_map(recording, truth, arguments.max_gap)
        result["transforms"] = {"initial_T_world_map": transform_record(initial), "initial_method": method["method"],
                                "initial_samples": method}
        trajectory = trajectory_errors(recording, truth, initial, world_odom, arguments.max_gap)

        if trajectory is None:
            raise ValueError("실제 자세와 겹치는 SLAM·odom 표본 없음")

        result["trajectory_comparison"] = dict(trajectory, status="completed")
    except Exception as error:
        result["trajectory_comparison"] = {"status": "failed", "error": f"{type(error).__name__}: {error}"}

    write_json_atomic(run / "accuracy.json", result)

    # 3) 지도 비교: 지도 저장 파일이 있어야 한다. 부분 경로·저장 생략이면 실행하지 않는다.
    problems = check_saved_files(run)

    if recording is None or initial is None:
        result["map_comparison"] = {"status": "not_run", "reason": "참조 지도·초기 기준 변환을 만들지 못함"}
    elif problems:
        reason = "부분 경로(지도 저장 생략)" if result["drive_status"].get("partial_route") else "SLAM 저장 파일 문제"
        result["map_comparison"] = {"status": "not_run", "reason": reason, "problems": problems}
    else:
        try:
            result["map_comparison"] = compare_map(run, drive, result, reference, geometry, truth, initial, arguments)
        except Exception as error:
            result["map_comparison"] = {"status": "failed", "error": f"{type(error).__name__}: {error}"}

    result["status"], reason = overall_status(result)

    if reason:
        result["incomplete_reason"] = reason

    # 채택을 요청했는데 채택하지 못하면 정상 완료가 아니다.
    adoption = result["map_comparison"].get("adoption")

    if adoption is not None:
        result["adoption"] = adoption

        if adoption["status"] != "adopted":
            result["status"] = "adoption_refused"

    write_json_atomic(run / "accuracy.json", result)

    print(f"결과: {run / 'accuracy.json'}", flush=True)
    print(f"[INFO] drive_status={result['drive_status']['status']} "
          f"trajectory_comparison={result['trajectory_comparison']['status']} "
          f"map_comparison={result['map_comparison']['status']} judgment={result['judgment']}", flush=True)

    trajectory = result["trajectory_comparison"]

    if trajectory["status"] == "completed":
        print(f"[INFO] 차체 자세 오차(실제 대비): SLAM 위치 최대 {trajectory['slam_position_error_m']['max']:.3f} m·"
              f"방향 최대 {trajectory['slam_yaw_error_rad']['max']:.3f} rad, odom 위치 최대 "
              f"{trajectory['odom_position_error_m']['max']:.3f} m·방향 최대 {trajectory['odom_yaw_error_rad']['max']:.3f} rad",
              flush=True)

    for key in ("drive_status", "trajectory_comparison", "map_comparison"):
        if result[key]["status"] != "completed":
            print(f"[INFO] {key}: {result[key].get('error') or result[key].get('reason') or result[key]['status']}", flush=True)

    metrics = result["map_comparison"].get("metrics_optimal") or {}

    if metrics:
        wall = metrics.get("wall_error_reference_to_compared") or {}
        print(f"[INFO] 장애물 검출 {metrics.get('obstacle_detection_ratio')}, 역방향 일치 {metrics.get('reverse_match_ratio')}, "
              f"벽 오차 평균 {wall.get('mean_m')} m·p95 {wall.get('p95_m')} m·최대 {wall.get('max_m')} m", flush=True)

    if "adoption" in result:
        print(f"[INFO] adoption={result['adoption']['status']} {result['adoption'].get('problems') or result['adoption'].get('map_revision')}",
              flush=True)

    sys.exit(0 if result["status"] == "completed" else 1)


# SLAM 지도를 읽어 정렬·형상 비교·그림·채택을 수행하고 map_comparison 결과를 돌려준다.
def compare_map(run, drive, result, reference, geometry, truth, initial, arguments):
    comparison = {"status": "running"}
    slam_grid, slam_geometry, slam_yaw = mt.read_map(run / "slam_map.yaml")

    if abs(slam_yaw) > 1e-9:
        raise ValueError("회전된 지도 원점은 지원하지 않음")

    slam_yaml = yaml.safe_load((run / "slam_map.yaml").read_text(encoding="utf-8"))
    comparison["slam_map"] = {"geometry": slam_geometry.as_dict(),
                              "thresholds": {"occupied": slam_yaml["occupied_thresh"], "free": slam_yaml["free_thresh"]}}

    # 최적 정렬 변환(초기 기준에서 시작하는 ICP)
    optimal, icp = mt.align_icp(mt.occupied_points(slam_grid, slam_geometry),
                                mt.occupied_points(reference, geometry), initial)
    correction = np.linalg.inv(initial) @ optimal
    result["transforms"].update({
        "optimal_T_world_map": transform_record(optimal),
        "alignment_correction": transform_record(correction),
        "icp": icp,
        "note": "정렬 보정량은 기록만 한다. 큰 정렬량만으로 실패 판정하지 않는다",
    })

    # 형상 지표(초기·최적 정렬 둘 다)
    compared_initial = mt.resample(slam_grid, slam_geometry, geometry, initial)
    compared = mt.resample(slam_grid, slam_geometry, geometry, optimal)
    comparison["metrics_initial"] = mt.compare_grids(reference, compared_initial, geometry.resolution, arguments.tolerance_cells)
    comparison["metrics_optimal"] = mt.compare_grids(reference, compared, geometry.resolution, arguments.tolerance_cells)
    comparison["limits"] = [
        "참조와 SLAM이 같은 앞 LiDAR 스캔을 써서 센서 오차·가림·LiDAR 높이 밖 장애물을 함께 놓친다",
        ("LiDAR는 렌더링 형상을 보므로 물리 collision과 다를 수 있다(AWS 메시)"
         if is_legacy_aws(read_session(run)) else
         "새 월드는 collision·visual이 같은 상자다. LiDAR 평면(약 0.41 m) 밖 선반판·낮은 상자는 참조·SLAM 모두에 없다"),
        "참조 지도는 스캔 측정 시각 보간 자세를 쓰며 보간 불가 스캔은 제외했다",
    ]

    criteria = yaml.safe_load(arguments.criteria.read_text(encoding="utf-8")) if arguments.criteria else None
    verdict = judge(comparison["metrics_optimal"], criteria)
    result.update(verdict)

    draw_overlay(run / "overlay.png", reference, compared, geometry, truth)
    mt.write_map(run / "reference_map.yaml", reference, geometry)
    comparison["files"] = ["overlay.png", "reference_map.yaml", "reference_map.pgm"]

    # 채택 여부와 무관하게 메타정보 초안과 누락·개발 조건을 결과에 남긴다.
    draft = None

    try:
        draft = map_metadata(run, drive, dict(result, metrics_optimal=comparison["metrics_optimal"]), slam_yaml)
        comparison["map"] = draft["map"]
        comparison["metadata_problems"] = metadata_problems(draft)
        comparison["world"] = draft["world"]
    except (KeyError, OSError, ValueError, TypeError) as error:
        comparison["metadata_problems"] = [f"메타정보 생성 실패: {type(error).__name__}: {error}"]

    # 채택은 주행·궤적 비교 완료, 저장 전·종료 odom 기준 정지 확인, 메타정보 완비·운영 조건일 때만 한다.
    # 거부는 지도 비교 결과와 따로 adoption에 기록하며 기존 채택 지도는 건드리지 않는다.
    if arguments.adopt:
        problems = []

        if result["drive_status"]["status"] != "completed" or result["trajectory_comparison"]["status"] != "completed":
            problems.append("주행·궤적 비교 미완료")

        if result["drive_status"].get("partial_route"):
            problems.append("부분 경로(--waypoints) 지도")

        # 판정 기준(--criteria)으로 PASS일 때만 채택한다. FAIL·not_judged(기준 미지정)는 파일을 바꾸지 않고 거절한다.
        if result.get("judgment") != "PASS":
            problems.append(f"판정 {result.get('judgment')}: PASS가 아니면 채택하지 않음(--criteria 지정 필요)")

        problems += stop_gate_problems(drive)
        problems += comparison.get("metadata_problems", [])
        target = None

        if draft is None:
            problems.append("메타정보 초안 없음")
        else:
            target, target_problems = adoption_target(arguments.map_dir, draft)
            problems += target_problems

        if problems:
            comparison["adoption"] = {"status": "refused", "dir": str(target) if target else None, "problems": problems}
        else:
            try:
                metadata = map_metadata(run, drive, dict(result, metrics_optimal=comparison["metrics_optimal"]), slam_yaml)
                info = adopt_map(run, target.resolve(), metadata)
                comparison["adoption"] = {"status": "adopted", "dir": str(target),
                                          "map_revision": info["map_revision"], "reload_check": info["reload_check"]}
            except (RuntimeError, OSError, ValueError, KeyError) as error:
                comparison["adoption"] = {"status": "refused", "dir": str(target),
                                          "problems": [f"{type(error).__name__}: {error}"]}

    comparison["status"] = "completed"

    return comparison


if __name__ == "__main__":
    main()
