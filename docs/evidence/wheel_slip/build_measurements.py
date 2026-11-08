#!/usr/bin/env python3
"""A1 전달용 measurements.csv와 validity_summary.json을 이 폴더의 첨부(attachments/)만으로 다시 만든다. 값은 반올림하지 않는다.

python3 build_measurements.py  (위치와 무관, 이 파일이 있는 폴더 기준)
- 입력은 아래 INPUTS에 고정한 첨부 파일뿐이다. 폴더 밖 원본 로그나 나중에 생긴 실행 결과를 찾지 않는다.
- source 열은 원본 출처 기록(INPUTS 값)이며 입력 경로로 쓰지 않는다.
- 입력 누락·형식 오류·대체 관계 불일치가 있으면 기존 결과 파일을 덮어쓰지 않고 종료 코드 1로 끝난다.
- 대체 관계(replaced_by)는 대상 행의 정확한 trial_id다. 대상이 없거나 여러 개이거나 자기 자신·순환이면 오류다.
"""

import csv
from collections import Counter
import json
import os
from pathlib import Path
import sys
import tempfile

H = Path(__file__).resolve().parent

# 고정 입력: 첨부 경로 → 원본 출처(기록용)
RAW = "../../a2_rotation_slip/20261106_224038"
R1RAW = "../../a2_robot1_rotation/20261108_134133/trials"
DRAW = "../../a2_drive_candidate/20261108_142156"
CAUSE_FILES = ["rotation_base_6x.json", "friction_no_fdir1.json", "friction_mu1_1.0.json", "com_moved_to_axle_mid.json",
               "com_moved_rear.json", "contact_base.json"]
CANDIDATE_FILES = ["base.json", "S005.json", "S020.json"]
B2_RUNS = ["20261106_214348", "20261106_221246", "20261106_230400", "20261106_230603", "20261106_230838", "20261106_231056",
           "20261106_231320", "20261108_125024", "20261108_125921", "20261108_130812", "20261108_131127", "20261108_131510", "run1"]
B2_SESSION = [r for r in B2_RUNS if r != "run1"]
B2_TRAJECTORY = [r for r in B2_RUNS if r not in ("20261106_231320", "run1")]
R1_FILES = ["S005_neutral_0kg.json", "S005_reachside_0kg.json", "S005_reachside_1kg.json", "S005_reachside_2kg.json",
            "base_neutral_0kg.json", "base_reachside_0kg.json", "base_reachside_1kg.json", "base_reachside_2kg.json", "contact_diag.json"]
DRIVE_FILES = [f"{folder}/{name}" for folder in ("baseline", "s005")
               for name in ("robot_1.json", "robot_2.json", "robot_2.run1_stop_anomaly.json", "robot_2_stops.json")]
INPUTS = {}
INPUTS.update({f"attachments/cause_analysis/{n}": f"{RAW}/cause_analysis/{n}" for n in CAUSE_FILES})
INPUTS.update({f"attachments/candidate_tests/{n}": f"{RAW}/{n}" for n in CANDIDATE_FILES})
INPUTS.update({f"attachments/b2_runs/{r}/drive_result.json": f"../../b2_mapping/{r}" for r in B2_RUNS})
INPUTS.update({f"attachments/b2_runs/{r}/session.json": f"../../b2_mapping/{r}" for r in B2_SESSION})
INPUTS.update({f"attachments/b2_runs/{r}/trajectory_recomputed.json": f"../../b2_mapping/{r}" for r in B2_TRAJECTORY})
INPUTS.update({f"attachments/robot1/trials/{n}": f"{R1RAW}/{n}" for n in R1_FILES})
INPUTS.update({f"attachments/drive_candidate/results/{n}": f"{DRAW}/{n}" for n in DRIVE_FILES})


class BuildError(Exception):
    """입력·대체 관계 오류. 결과 파일을 쓰지 않는다."""


# BuildError는 사유만 출력하고 종료 코드 1로 끝낸다(결과 파일은 쓰기 전이라 그대로 남는다).
def report_error(kind, error, trace):
    if issubclass(kind, BuildError):
        print(f"[FAIL] {error}. 기존 결과 파일을 덮어쓰지 않음", file=sys.stderr)
        return

    sys.__excepthook__(kind, error, trace)


sys.excepthook = report_error


# 고정 목록의 첨부 JSON을 읽는다. 목록 밖·누락·형식 오류는 BuildError
def load(path):
    key = Path(path).as_posix()

    if key not in INPUTS:
        raise BuildError(f"고정 입력 목록에 없는 파일: {key}")

    try:
        data = json.loads((H / key).read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise BuildError(f"입력 누락: {key}") from None
    except (ValueError, UnicodeDecodeError) as error:
        raise BuildError(f"입력 형식 오류: {key}: {error}") from None

    if not isinstance(data, dict):
        raise BuildError(f"입력 형식 오류(객체 아님): {key}")

    return data
COLUMNS = ["trial_id", "tool_version", "validity", "replaced_by", "replacement_reason", "robot", "group", "condition", "world", "wheel_friction", "world_collision", "arm_pose", "load", "contact_sensor_overlay",
           "command", "omega_rad_s", "repeat", "direction", "status", "truth_turn_rad", "odom_turn_rad", "turn_diff_rad",
           "truth_translation_m", "odom_translation_m", "translation_error_m", "path_length_m", "centre_forward_m", "centre_left_m",
           "rotation_radius_m", "fit_residual_m", "odom_position_error_max_m", "odom_yaw_error_max_rad",
           "truth_distance_m", "odom_distance_m", "lateral_m", "odom_stop_s", "truth_stop_s", "slide_after_stop_m",
           "tilt_max_rad", "contact_observation", "note", "source", "attachment"]
FRICTION = {"base": "mu1 0.1 / mu2 1.0 / fdir1 축", "S005": "기본 + slip1 0.05 m/s/N", "S020": "기본 + slip1 0.20 m/s/N"}
rows = []


# 첨부 경로의 원본 출처 기록
def rel(path):
    return INPUTS[Path(path).as_posix()]


# 한 행 추가(없는 열은 빈칸)
def add(**values):
    rows.append({c: ("" if values.get(c) is None else values.get(c)) for c in COLUMNS})


# ── robot_2 원인 조사 ───────────────────────────────────────────────
CA = Path("attachments/cause_analysis")
flat = dict(world="ax_test", world_collision="평면 바닥", load="0")

for i, t in enumerate(load(CA / "rotation_base_6x.json")["trials"], 1):
    add(robot="robot_2", group="cause_rotation", condition="base", wheel_friction=FRICTION["base"], contact_sensor_overlay="no",
        command="회전 ±0.3 rad/s 10 s", omega_rad_s=0.3, repeat=i, direction=t["direction"], status="completed",
        truth_turn_rad=t["true_turn_rad"], odom_turn_rad=t["odom_turn_rad"], turn_diff_rad=t["odom_turn_rad"] - t["true_turn_rad"],
        truth_translation_m=t["true_translation_m"], odom_translation_m=t["odom_translation_m"],
        centre_forward_m=t["centre_in_start_body_m"][0], centre_left_m=t["centre_in_start_body_m"][1], rotation_radius_m=t["centre_radius_m"],
        contact_observation="바퀴 속도 " + "/".join(str(v) for v in t["wheel_speed_rad_s"].values()) + " rad/s, 바퀴 중심 z " +
        "/".join(str(v[0]) for v in t["wheel_centre_z_world_m"].values()) + " m (FL/FR/RL/RR)",
        note="모델 원점=base_link 확인", source=rel(CA / "rotation_base_6x.json"), **flat)

variants = [("friction_no_fdir1.json", "fdir1 제거", "mu1 0.1 / mu2 1.0 / fdir1 없음", "no", "원인 분리 전용"),
            ("friction_mu1_1.0.json", "mu1 1.0", "mu1 1.0 / mu2 1.0 / fdir1 축", "no", "원인 분리 전용"),
            ("com_moved_to_axle_mid.json", "COM 바퀴 중간", FRICTION["base"], "yes", "base_link inertial x 0.0128→-0.0337. 원인 분리 전용"),
            ("com_moved_rear.json", "COM 뒤쪽", FRICTION["base"], "yes", "base_link inertial x 0.0128→-0.0800. 원인 분리 전용")]

for name, condition, friction, contact, note in variants:
    for i, t in enumerate(load(CA / name)["trials"], 1):
        turned = abs(t["true_turn_rad"]) > 0.5
        add(robot="robot_2", group="cause_variant", condition=condition, wheel_friction=friction, contact_sensor_overlay=contact,
            command="회전 ±0.3 rad/s 10 s", omega_rad_s=0.3, repeat=i, direction=t["direction"], status="completed",
            truth_turn_rad=t["true_turn_rad"], odom_turn_rad=t["odom_turn_rad"], turn_diff_rad=t["odom_turn_rad"] - t["true_turn_rad"],
            truth_translation_m=t["true_translation_m"], odom_translation_m=t["odom_translation_m"],
            centre_forward_m=t["centre_in_start_body_m"][0] if turned else None, centre_left_m=t["centre_in_start_body_m"][1] if turned else None,
            rotation_radius_m=t["centre_radius_m"] if turned else None,
            note=note + ("" if turned else "; 차체가 거의 회전하지 않아 회전 중심 무의미"), source=rel(CA / name), **flat)

for phase, wheels in load(CA / "contact_base.json").items():
    for wheel, c in wheels.items():
        add(robot="robot_2", group="cause_contact", condition="base", wheel_friction=FRICTION["base"], contact_sensor_overlay="yes",
            command="정지" if phase == "still" else "왼쪽 회전 0.3 rad/s", direction=wheel, status="completed",
            contact_observation=f"접촉점 {c['contacts_per_message']}/메시지, 수직 하중 합 {c['normal_force_sum_per_step_N']} N, "
            f"구름 방향 {c.get('force_rolling_mean_N')} N/점, 축 방향 {c.get('force_axle_mean_N')} N/점, 위치(차체) {c.get('contact_points_body_mean_m')}",
            note="direction 칸은 바퀴 이름", source=rel(CA / "contact_base.json"), **flat)

# ── robot_2 후보 비교(slip_eval) ─────────────────────────────────────
A = Path("attachments/candidate_tests")

for condition in ("base", "S005", "S020"):
    r = load(A / f"{condition}.json")
    common = dict(robot="robot_2", condition=condition, wheel_friction=FRICTION[condition], contact_sensor_overlay="yes",
                  source=rel(A / f"{condition}.json"), world="ax_test", world_collision="평면 바닥")

    for i, t in enumerate(r["rotation"], 1):
        c = t["rotation_centre_body_m"]
        add(group="candidate_rotation", command="회전 ±0.3 rad/s 10 s", omega_rad_s=0.3, load="0", repeat=i, direction=t["direction"],
            status="completed", truth_turn_rad=t["truth_turn_rad"], odom_turn_rad=t["odom_turn_rad"],
            turn_diff_rad=t["odom_turn_rad"] - t["truth_turn_rad"], truth_translation_m=t["truth_translation_m"],
            odom_translation_m=t["odom_translation_m"], translation_error_m=t["translation_error_m"],
            centre_forward_m=c[0] if c else None, centre_left_m=c[1] if c else None, rotation_radius_m=t["rotation_radius_m"],
            note=f"RTF {r['clock']['rtf']}", **common)

    for i, t in enumerate(r["straight"], 1):
        add(group="candidate_straight", command="직진 ±0.3 m/s 3 s", load="0", repeat=i, direction=t["direction"], status="completed",
            truth_distance_m=t["truth_distance_m"], odom_distance_m=t["odom_distance_m"], lateral_m=t["lateral_m"],
            note=f"미끄러짐 {t['slip']}", **common)

    for key, t in r["stop"].items():
        add(group="candidate_stop", command="직진 0.67 m/s 4 s 뒤 중단" if key.startswith("linear") else "회전 0.5 rad/s 6 s 뒤 중단",
            load="0", repeat=1, status="stop_confirmed" if t["odom_stop_s"] is not None and t["truth_stop_s"] is not None else "stop_not_confirmed",
            odom_stop_s=t["odom_stop_s"], truth_stop_s=t["truth_stop_s"], slide_after_stop_m=t["slide_after_odom_stop_m"],
            note="D-06 값 0.5 s 유지 기준(개발 시험, A4 판정 아님)", **common)

    for t in r["load"]:
        c = t["rotation_centre_body_m"]
        add(group="candidate_load", command="왼쪽 회전 0.3 rad/s 10 s", omega_rad_s=0.3,
            load=f"상자 {t['mass_kg']} kg, 적재면 x {t['offset_m']:+} m(올려놓기만 함, 고정 없음)", repeat=1, direction="left",
            status="completed", truth_turn_rad=t["truth_turn_rad"], odom_turn_rad=t["odom_turn_rad"],
            turn_diff_rad=t["odom_turn_rad"] - t["truth_turn_rad"], truth_translation_m=t["truth_translation_m"],
            odom_translation_m=t["odom_translation_m"], translation_error_m=t["translation_error_m"],
            centre_forward_m=c[0] if c else None, centre_left_m=c[1] if c else None, rotation_radius_m=t["rotation_radius_m"],
            note=f"적재면 위 미끄러짐 {t.get('box_slip_on_cargo_m')} m. 민감도 진단 자료", **common)

    for wheel, c in r["contact_during_first_left_turn"].items():
        add(group="candidate_contact", command="첫 왼쪽 회전 중", load="0", direction=wheel, status="completed",
            contact_observation=f"최소 접촉점 {c['min_points']}, 수직 하중 평균 {c['normal_mean_N']} N·최소 {c['normal_min_N']} N, "
            f"축 방향 힘 평균 {c['axle_force_mean_N']} N(바퀴 합)", note="direction 칸은 바퀴 이름", **common)

# ── robot_2 B2 실행(실패·중단 포함) ─────────────────────────────────
for name in B2_RUNS:
    run = Path("attachments/b2_runs") / name
    d = load(run / "drive_result.json")
    session = load(run / "session.json") if name in B2_SESSION else {}
    traj = load(run / "trajectory_recomputed.json")["trajectory_comparison"] if name in B2_TRAJECTORY else {}
    stops = d.get("stops", [])
    note = f"경유점 {d.get('waypoints_reached')}/{len(d['route']['waypoints'])}" + (" (부분 경로)" if d["route"].get("partial") else "")
    note += f"; 정지 {len(stops)}회(확인 {sum(s['confirmed'] for s in stops)}회): " + " | ".join(s["reason"] for s in stops)
    note += f"; 오류: {d['error']}" if d.get("error") else ""
    add(robot="robot_2", group="b2_run", condition=name, world="aws_warehouse",
        world_collision=session.get("world_condition", "light_collision(세션 기록 이전)"), wheel_friction=session.get("wheel_condition", "original"),
        load="0", contact_sensor_overlay="no", command="지도 작성 경로", status=d.get("status"),
        odom_position_error_max_m=traj.get("odom_position_error_m", {}).get("max") if traj.get("status") == "completed" else None,
        odom_yaw_error_max_rad=traj.get("odom_yaw_error_rad", {}).get("max") if traj.get("status") == "completed" else None,
        note=note + (f"; SLAM 위치 최대 {traj['slam_position_error_m']['max']} m·방향 최대 {traj['slam_yaw_error_rad']['max']} rad"
                     if traj.get("status") == "completed" else "; 궤적 비교 불가(기록 없음)"),
        source=rel(run / "drive_result.json"))

# ── robot_1 회전 기준·S005 ───────────────────────────────────────────
for name in R1_FILES:
    path = Path("attachments/robot1/trials") / name
    r = load(path)
    condition = r["condition"]
    friction = FRICTION["S005"] if condition.startswith("S005") else FRICTION["base"]
    payload = condition.split("_")[-1] if condition != "contact_diag" else "0kg"

    for t in r.get("rotations", []):
        steady = t.get("circle_fit", {}).get("steady", {})
        centre = steady.get("centre_body_m") or [None, None]
        contact = t.get("contact")
        add(robot="robot_1", group="r1_rotation" if condition != "contact_diag" else "r1_contact", condition=condition,
            world="ax_test", world_collision="평면 바닥", wheel_friction=friction, arm_pose=r["pose"],
            load=f"고정 시험 치구 {payload}(payload_mass)" if payload != "0kg" else "0",
            contact_sensor_overlay="yes" if condition == "contact_diag" else "no", command="회전 ±0.3 rad/s, odom 3.0 rad 목표",
            omega_rad_s=0.3, repeat=t["repeat"], direction=t["direction"], status=t["status"],
            truth_turn_rad=t.get("truth_turn_rad"), odom_turn_rad=t.get("odom_turn_rad"),
            turn_diff_rad=(t.get("odom_turn_rad") or 0) - (t.get("truth_turn_rad") or 0),
            truth_translation_m=t.get("truth_translation_m"), odom_translation_m=t.get("odom_translation_m"),
            translation_error_m=t.get("translation_error_m"), path_length_m=t.get("truth_path_length_m"),
            centre_forward_m=centre[0], centre_left_m=centre[1], rotation_radius_m=steady.get("radius_m"),
            fit_residual_m=steady.get("residual_rms_m"), odom_position_error_max_m=t.get("odom_error", {}).get("position_max_m"),
            odom_yaw_error_max_rad=t.get("odom_error", {}).get("yaw_max_rad"), odom_stop_s=t.get("stop", {}).get("odom_stop_s"),
            truth_stop_s=t.get("stop", {}).get("truth_stop_s"), tilt_max_rad=max((t.get("tilt_max_rad") or {"x": None}).values() or [None]),
            contact_observation="; ".join(f"{w}: 접촉점 {c['min_points']}, 하중 {c['normal_mean_N']} N, 옆방향 {c['lateral_mean_N']} N"
                                          for w, c in contact.items()) if contact else None,
            note="회전 중심은 정상 회전 구간 원 맞춤(시작 차체 기준). " + "; ".join(t.get("reasons", [])),
            source=rel(path))

    for i, s in enumerate(r.get("straight", []), 1):
        add(robot="robot_1", group="r1_straight", condition=condition, world="ax_test", world_collision="평면 바닥", wheel_friction=friction,
            arm_pose=r["pose"], load=payload, command=f"직진 {s['command_m_s']} m/s 3 s", repeat=i, status=s["status"],
            truth_distance_m=s["truth_distance_m"], odom_distance_m=s["odom_distance_m"], lateral_m=s["lateral_m"],
            tilt_max_rad=max(s["tilt_max_rad"].values()), note=f"미끄러짐 {s['slip']}", source=rel(path))

    for key, s in (r.get("stop_tests") or {}).items():
        add(robot="robot_1", group="r1_stop", condition=condition, world="ax_test", world_collision="평면 바닥", wheel_friction=friction,
            arm_pose=r["pose"], load=payload, command=key, repeat=1, status=s["status"], odom_stop_s=s["stop"]["odom_stop_s"],
            truth_stop_s=s["stop"]["truth_stop_s"], tilt_max_rad=max(s["tilt_max_rad"].values()),
            note="D-06 값 0.5 s 유지 기준(개발 시험, A4 판정 아님)", source=rel(path))

# ── 주행 허용 자세 후보 비교(있으면) ─────────────────────────────────
for folder in [Path("attachments/drive_candidate/results")]:
    for path in [folder / name for name in DRIVE_FILES]:
        r = load(path)
        robot = r.get("robot", path.stem)
        friction = FRICTION["S005"] if path.parent.name == "s005" else FRICTION["base"]
        common = dict(robot=robot, condition=f"{path.parent.name}/{path.stem}", world="ax_test", world_collision="평면 바닥", wheel_friction=friction,
                      arm_pose="neutral(시험용 주행 자세 후보)" if robot == "robot_1" else "팔 없음", load="0", contact_sensor_overlay="no",
                      source=rel(path))

        if r.get("status") != "completed" and not r.get("rotations"):
            add(group="drive_failed", status=r.get("status"), note=r.get("error"), **common)

        for t in r.get("rotations", []):
            steady = t.get("circle_fit", {}).get("steady", {})
            centre = steady.get("centre_body_m") or [None, None]
            add(group="drive_rotation", command=f"회전 ±{t['omega_rad_s']} rad/s, odom 3.0 rad 목표", omega_rad_s=t["omega_rad_s"],
                repeat=t["repeat"], direction=t["direction"], status=t["status"], truth_turn_rad=t.get("truth_turn_rad"),
                odom_turn_rad=t.get("odom_turn_rad"), turn_diff_rad=(t.get("odom_turn_rad") or 0) - (t.get("truth_turn_rad") or 0),
                truth_translation_m=t.get("truth_translation_m"), odom_translation_m=t.get("odom_translation_m"),
                translation_error_m=t.get("translation_error_m"), path_length_m=t.get("truth_path_length_m"),
                centre_forward_m=centre[0], centre_left_m=centre[1], rotation_radius_m=steady.get("radius_m"),
                fit_residual_m=steady.get("residual_rms_m"), odom_position_error_max_m=t.get("odom_error", {}).get("position_max_m"),
                odom_yaw_error_max_rad=t.get("odom_error", {}).get("yaw_max_rad"), odom_stop_s=t.get("stop", {}).get("odom_stop_s"),
                truth_stop_s=t.get("stop", {}).get("truth_stop_s"), tilt_max_rad=max((t.get("tilt_max_rad") or {"x": None}).values()),
                note="; ".join(t.get("reasons", [])), **common)

        for i, p in enumerate(r.get("paths", []), 1):
            add(group=f"drive_{p['kind']}", command=f"{p['kind']} v {p['command'][0]} m/s, w {p['command'][1]} rad/s, {p['command'][2]} s",
                repeat=i, status=p["status"], truth_turn_rad=p.get("truth_turn_rad"), odom_turn_rad=p.get("odom_turn_rad"),
                truth_distance_m=p["truth_displacement_body_m"][0], odom_distance_m=p["odom_displacement_body_m"][0],
                lateral_m=p["truth_displacement_body_m"][1], odom_position_error_max_m=(p.get("odom_error") or {}).get("position_max_m"),
                odom_yaw_error_max_rad=(p.get("odom_error") or {}).get("yaw_max_rad"), odom_stop_s=p["stop"]["odom_stop_s"],
                truth_stop_s=p["stop"]["truth_stop_s"], tilt_max_rad=max(p["tilt_max_rad"].values()), note=p.get("reason") or "", **common)

        for key, s in (r.get("stop_tests") or {}).items():
            add(group="drive_stop", command=key, repeat=1, status=s["status"], odom_stop_s=s["stop"]["odom_stop_s"],
                truth_stop_s=s["stop"]["truth_stop_s"], slide_after_stop_m=s.get("slide_after_odom_stop_m"),
                tilt_max_rad=max(s["tilt_max_rad"].values()), note="D-06 값 0.5 s 유지 기준(개발 시험, A4 판정 아님)", **common)

# 원본 경로 → 전달 폴더 첨부 경로
def attachment(source):
    path = source.replace("../../", "", 1)
    table = [("a2_rotation_slip/20261106_224038/cause_analysis/", "attachments/cause_analysis/"),
             ("a2_rotation_slip/20261106_224038/", "attachments/candidate_tests/"),
             ("a2_robot1_rotation/20261108_134133/trials/", "attachments/robot1/trials/"),
             ("a2_drive_candidate/20261108_142156/", "attachments/drive_candidate/results/")]

    for old, new in table:
        if path.startswith(old):
            return new + path[len(old):]

    if path.startswith("b2_mapping/"):
        return f"attachments/b2_runs/{Path(path).name}/drive_result.json"

    return ""


# 원본 파일별 측정 도구 버전
def tool_version(source):
    if "cause_analysis" in source:
        return "probe_rotation.py·probe_contact.py(11-06)"
    if "a2_rotation_slip" in source:
        return "slip_eval.py(11-06)"
    if "b2_mapping" in source:
        return "mapping_drive.py(실행 당시)·check_map_accuracy.py(11-08 재계산)"
    if "a2_robot1_rotation" in source:
        return "r1_rotation.py"
    if source.endswith("robot_2_stops.json"):
        return "drive_eval v3"
    if source.endswith("robot_2.json"):
        return "drive_eval v2"
    return "drive_eval v1"


counter = {}
FAILED = {"failed", "aborted", "stop_unconfirmed", "stop_not_confirmed", "target_not_reached", "unexpected_motion", "running"}

for row in rows:
    row["attachment"] = attachment(row["source"])
    row["tool_version"] = tool_version(row["source"])
    key = (row["attachment"] or row["source"], row["group"])
    counter[key] = counter.get(key, 0) + 1
    row["trial_id"] = f"{key[0]}#{row['group']}#{counter[key]}"
    row["validity"] = "failed" if row["status"] in FAILED else "valid"


# 대체 대상을 정확히 하나 찾는다. 로봇·기본/S005 폴더·시험 종류·명령·반복·방향이 같아야 한다.
def find_target(row, target_file, match_repeat):
    folder = Path(row["attachment"]).parent.as_posix()
    target_attachment = f"{folder}/{target_file}"
    matches = [r for r in rows
               if r["attachment"] == target_attachment and r["robot"] == row["robot"] and r["group"] == row["group"]
               and r["command"] == row["command"] and r["direction"] == row["direction"]
               and (not match_repeat or str(r["repeat"]) == str(row["repeat"]))]

    if len(matches) != 1:
        raise BuildError(f"대체 대상 {len(matches)}개(정확히 1개 필요): {row['trial_id']} → {target_attachment}")

    return matches[0]


# 대체 관계: robot_2 첫 실행(v1)은 같은 조건 재실행(v2)으로, 막힌 정지 시험은 출발 위치 재측정(v3)으로 대체
for row in rows:
    if row["source"].endswith("robot_2.run1_stop_anomaly.json"):
        if row["group"] == "drive_stop":
            target = find_target(row, "robot_2_stops.json", match_repeat=False)
            row["validity"] = "invalid_measurement"
            row["replacement_reason"] = ("경로 끝에서 작업대 station_src(8, 3)에 막힌 상태의 정지 시간(0.003~0.007 s). "
                                         "v1에 예상 밖 접촉 감지가 없어 정상으로 기록됨. 대체: 같은 정지 시험(command)의 출발 위치 재측정(v3)")
        else:
            target = find_target(row, "robot_2.json", match_repeat=True)
            row["validity"] = "superseded"
            row["replacement_reason"] = ("같은 조건을 v2(접촉 감지 추가, 계산식 동일)로 다시 실행해 대체. 이 행의 측정 자체는 유효했으나 중복 집계하지 않음. "
                                         "대체: 같은 시험 종류·command·repeat·direction의 v2 회차")
        row["replaced_by"] = target["trial_id"]
    elif row["source"].endswith("robot_2.json") and row["group"] == "drive_stop" and row["status"] == "unexpected_motion":
        target = find_target(row, "robot_2_stops.json", match_repeat=False)
        row["replaced_by"] = target["trial_id"]
        row["replacement_reason"] = ("경로 끝에서 작업대에 막혀 접촉 감지로 중단(실패 유지). "
                                     "대체: 같은 정지 시험(command)의 출발 위치 재측정(v3)")


# 결과 검사: trial_id 중복, 대체 대상 존재·유효·자기 참조·순환
def validate():
    ids = [r["trial_id"] for r in rows]
    duplicated = [i for i, n in Counter(ids).items() if n > 1]

    if duplicated:
        raise BuildError(f"trial_id 중복: {duplicated[:3]}")

    by_id = {r["trial_id"]: r for r in rows}

    for row in rows:
        target = row["replaced_by"]

        if not target:
            continue

        if target == row["trial_id"]:
            raise BuildError(f"자기 자신 참조: {target}")

        if target not in by_id:
            raise BuildError(f"없는 대체 대상: {row['trial_id']} → {target}")

        # 대상은 유효 회차여야 하므로 대체가 이어지거나 돌아올 수 없다(순환 거절).
        if by_id[target]["validity"] != "valid" or by_id[target]["replaced_by"]:
            raise BuildError(f"대체 대상이 유효 회차가 아님(연쇄·순환): {row['trial_id']} → {target}")


# 같은 폴더의 임시 파일에 쓴 뒤 바꿔 넣는다.
def write_atomic(path, writer):
    handle, temporary = tempfile.mkstemp(dir=H, prefix=f".{path.name}.")

    try:
        with os.fdopen(handle, "w", newline="", encoding="utf-8") as stream:
            writer(stream)

        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


# CSV 쓰기
def write_csv(stream):
    writer = csv.DictWriter(stream, fieldnames=COLUMNS)
    writer.writeheader()
    writer.writerows(rows)


validate()

# 유효성 집계: 로봇·묶음·조건별 전체·유효·실패·무효·대체 건수
summary = {}

for row in rows:
    key = f"{row['robot']}|{row['group']}|{row['condition']}"
    entry = summary.setdefault(key, {"total": 0, "valid": 0, "failed": 0, "invalid_measurement": 0, "superseded": 0})
    entry["total"] += 1
    entry[row["validity"]] += 1

write_atomic(H / "measurements.csv", write_csv)
write_atomic(H / "validity_summary.json", lambda stream: stream.write(json.dumps(summary, indent=1, ensure_ascii=False) + "\n"))
print(len(rows), dict(Counter(r["validity"] for r in rows)), "replaced_by", sum(1 for r in rows if r["replaced_by"]))
