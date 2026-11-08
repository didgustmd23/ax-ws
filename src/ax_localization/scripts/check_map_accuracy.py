#!/usr/bin/env python3
"""B2 지도 정확도 비교. SLAM 지도를 "Gazebo 실제 자세 기반 참조 지도"와 정렬·형상 비교한다.

  python3 src/ax_localization/scripts/check_map_accuracy.py <run 폴더> [--criteria 기준.yaml] [--adopt]

참조 지도: mapping_drive.py가 기록한 같은 앞 LiDAR 스캔을, 스캔 측정 시각에 보간한 Gazebo 차체 자세와
base_link→LiDAR 장착 변환으로 world 좌표에 누적한 점유 격자다. 자세를 보간할 수 없는 스캔은 제외하고 수를 남긴다.
두 지도는 같은 센서를 쓰므로 센서 오차·가림·LiDAR 높이(바닥 위 약 0.3 m)에서 보이지 않는 장애물은 함께 놓친다(검증 한계).

지표: 초기 기준 변환(T_world_map, 시작 시각의 실제 자세·odom·map→odom으로 계산)과 최적 정렬 변환(ICP)을 구분해 기록하고,
최적 정렬 기준으로 장애물 검출 비율·역방향 일치 비율·벽 위치 오차(평균·95백분위수·최대)·관측 범위를 계산한다.
기준 파일(--criteria)이 없으면 판정하지 않는다(judgment: not_judged). 큰 정렬량만으로 실패 판정하지 않는다.

--adopt: 저장 파일을 검사한 뒤 src/ax_localization/maps/<이름>/에 map.yaml·map.pgm·pose graph·map_info.yaml을 교체한다.
교체 전 검사가 실패하면 기존 지도를 그대로 둔다.
"""

import argparse
from datetime import datetime
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


# 저장소 루트: src/ax_localization이 있는 상위 폴더. 설치본에서 실행하면 현재 폴더.
def workspace_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "src/ax_localization").is_dir():
            return parent

    return Path.cwd()


WORKSPACE = workspace_root()


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


# run 폴더의 지도를 target에 채택한다. 검사 실패 시 기존 지도를 건드리지 않는다.
# 새 파일을 같은 폴더의 임시 폴더에 모두 만든 뒤 하나씩 바꿔 넣는다.
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
        (staging / "map_info.yaml").write_text(
            "# B2 지도 메타데이터. 지도가 바뀌면 map_revision이 오르며 등록 지점·재고 좌표는 revision을 대조해 재검증한다.\n"
            + yaml.safe_dump(info, sort_keys=False, allow_unicode=True), encoding="utf-8")

        # map_info.yaml은 마지막에 바꿔 revision과 파일이 어긋나는 시간을 줄인다.
        for name in ("map.yaml", "map.pgm", "pose_graph.posegraph", "pose_graph.data", "map_info.yaml"):
            os.replace(staging / name, target / name)

    return info


# 채택 메타데이터를 만든다.
def map_metadata(run, drive, accuracy, slam_yaml):
    route_path = Path(drive["route"]["path"])
    world_file = WORKSPACE / "world_assets/aws_small_warehouse/aws_warehouse_harmonic.world"
    source_file = WORKSPACE / "world_assets/aws_small_warehouse/SOURCE.json"
    slam_config = WORKSPACE / "src/ax_localization/config/slam_mapping.yaml"
    source = json.loads(source_file.read_text(encoding="utf-8")) if source_file.is_file() else {}

    return {
        "created": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(timespec="seconds"),
        "frame": "map",
        "resolution_m": slam_yaml["resolution"],
        "world": {
            "launch_name": "aws_warehouse.world",
            "file": str(world_file.relative_to(WORKSPACE)) if world_file.is_file() else None,
            "sha256": mt.file_sha256(world_file) if world_file.is_file() else None,
            "reference_assets": {"repository": source.get("repository"), "commit": source.get("commit"),
                                 "archive_sha256": source.get("archive_sha256")},
        },
        "robot": {"id": drive["robot_id"], "tool": "none (운송 로봇, 공구 없음)",
                  "sensor": {"input": drive["scan_input"], "frame": accuracy["scan_frame"],
                             "mount_base_link": accuracy["front_mount"]}},
        "slam": {"package": "slam_toolbox (async, mapping)", "config": str(slam_config.relative_to(WORKSPACE)),
                 "config_sha256": mt.file_sha256(slam_config)},
        "route": {"file": str(route_path), "sha256": drive["route"]["sha256"]},
        "pose_graph": {"files": ["pose_graph.posegraph", "pose_graph.data"],
                       "resume": "slam_toolbox deserialize_map 또는 map_file_name 파라미터로 이어 작성·위치 추정"},
        "verification": {
            "judgment": accuracy["judgment"],
            "metrics": accuracy["metrics_optimal"],
            "result": str((run / "accuracy.json").relative_to(WORKSPACE)) if run.is_relative_to(WORKSPACE) else str(run / "accuracy.json"),
            "drive_result": str((run / "drive_result.json").relative_to(WORKSPACE)) if run.is_relative_to(WORKSPACE) else str(run / "drive_result.json"),
        },
        "T_world_map": {
            "use": "검증 전용. 운영 위치 추정·주행에 쓰지 않는다",
            "initial": accuracy["transforms"]["initial_T_world_map"],
            "optimal": accuracy["transforms"]["optimal_T_world_map"],
            "method": accuracy["transforms"]["initial_method"] + " → 참조 지도와 ICP 정렬",
        },
    }


# 명령행 인자를 읽어 비교하고 결과를 저장한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path, help="mapping_drive.py 결과 폴더")
    parser.add_argument("--max-gap", type=float, default=DEFAULT_MAX_GAP_S, help="자세 보간 최대 표본 간격(s)")
    parser.add_argument("--occupied-ratio", type=float, default=0.25, help="참조 지도 점유 판정 비율")
    parser.add_argument("--tolerance-cells", type=int, default=1, help="일치 판정 허용 셀(±)")
    parser.add_argument("--criteria", type=Path, default=None, help="확정 임계값 YAML. 없으면 판정하지 않음")
    parser.add_argument("--adopt", action="store_true", help="검사 후 maps/aws_warehouse에 채택")
    parser.add_argument("--map-dir", type=Path, default=WORKSPACE / "src/ax_localization/maps/aws_warehouse")
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
                                  "partial_route": drive.get("route", {}).get("partial", False)}
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

    result["status"] = ("completed" if all(result[key]["status"] == "completed"
                                           for key in ("drive_status", "trajectory_comparison", "map_comparison"))
                        else "incomplete")
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
        "LiDAR는 렌더링 형상을 보므로 물리 collision과 다를 수 있다(AWS 메시)",
        "참조 지도는 스캔 측정 시각 보간 자세를 쓰며 보간 불가 스캔은 제외했다",
    ]

    criteria = yaml.safe_load(arguments.criteria.read_text(encoding="utf-8")) if arguments.criteria else None
    verdict = judge(comparison["metrics_optimal"], criteria)
    result.update(verdict)

    draw_overlay(run / "overlay.png", reference, compared, geometry, truth)
    mt.write_map(run / "reference_map.yaml", reference, geometry)
    comparison["files"] = ["overlay.png", "reference_map.yaml", "reference_map.pgm"]

    # 채택은 주행 완료·궤적 비교 완료일 때만 한다.
    if arguments.adopt:
        if result["drive_status"]["status"] != "completed" or result["trajectory_comparison"]["status"] != "completed":
            raise RuntimeError("주행·궤적 비교가 완료되지 않아 채택하지 않음")

        metadata = map_metadata(run, drive, dict(result, metrics_optimal=comparison["metrics_optimal"]), slam_yaml)
        info = adopt_map(run, arguments.map_dir.resolve(), metadata)
        comparison["adopted"] = {"dir": str(arguments.map_dir), "map_revision": info["map_revision"]}

    comparison["status"] = "completed"

    return comparison


if __name__ == "__main__":
    main()
