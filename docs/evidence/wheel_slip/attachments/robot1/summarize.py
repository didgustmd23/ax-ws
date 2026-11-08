#!/usr/bin/env python3
"""robot_1 회전 시험 결과 정리: summary.json, trajectory.png, manifest.json을 만든다."""

import hashlib
import json
import math
import subprocess
from pathlib import Path

import numpy as np

R = Path(__file__).resolve().parent
WS = Path.home() / "ax_ws"
CONDITIONS = ["base_neutral_0kg", "base_reachside_0kg", "base_reachside_1kg", "base_reachside_2kg",
              "S005_neutral_0kg", "S005_reachside_0kg", "S005_reachside_1kg", "S005_reachside_2kg"]
ARM_JOINTS = ["column_yaw_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "gripper_rotation_joint"]
POSES = {"neutral": [0.0] * 5, "reach_side": [0.0, -0.90, 1.25, -0.40, 0.0]}


# 파일 SHA-256
def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# 평균·표준편차·최댓값(None 제외)
def stats(values):
    values = [abs(v) for v in values if v is not None]

    if not values:
        return None

    return {"count": len(values), "mean": float(np.mean(values)), "std": float(np.std(values)), "max": float(np.max(values))}


# URDF 관절 변환으로 전체 COM을 계산한다(팔 관절각 적용).
def robot_com(urdf_text, joint_angles):
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf_text)

    # 원점 요소 → 4×4
    def origin(element):
        xyz = [float(v) for v in (element.get("xyz") if element is not None and element.get("xyz") else "0 0 0").split()]
        rr, pp, yy = [float(v) for v in (element.get("rpy") if element is not None and element.get("rpy") else "0 0 0").split()]
        cr, sr, cp, sp, cy, sy = math.cos(rr), math.sin(rr), math.cos(pp), math.sin(pp), math.cos(yy), math.sin(yy)
        matrix = np.eye(4)
        matrix[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                          [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]
        matrix[:3, 3] = xyz
        return matrix

    # 축 회전 4×4
    def axis_rotation(axis, angle):
        axis = np.asarray(axis, float) / np.linalg.norm(axis)
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        matrix = np.eye(4)
        matrix[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
        return matrix

    parents = {}

    for joint in root.findall("joint"):
        child, parent = joint.find("child"), joint.find("parent")

        if child is None or parent is None:
            continue

        axis = joint.find("axis")
        motion = np.eye(4)

        if joint.get("name") in joint_angles:
            motion = axis_rotation([float(v) for v in axis.get("xyz").split()], joint_angles[joint.get("name")])

        parents[child.get("link")] = (parent.get("link"), origin(joint.find("origin")) @ motion)

    # 링크 → base_link 변환
    def world(link):
        matrix = np.eye(4)

        while link in parents:
            parent, transform = parents[link]
            matrix = transform @ matrix
            link = parent

        return matrix

    total, moment = 0.0, np.zeros(3)

    for link in root.findall("link"):
        inertial = link.find("inertial")

        if inertial is None:
            continue

        mass = float(inertial.find("mass").get("value"))
        total += mass
        moment += mass * (world(link.get("name")) @ origin(inertial.find("origin")))[:3, 3]

    return total, moment / total


# 조건별 COM(xacro로 payload를 반영한 URDF 생성)
def condition_com(pose, payload):
    command = ["bash", "-c", f"source /opt/ros/jazzy/setup.bash && source {WS}/install/setup.bash && xacro "
               f"{WS}/src/ax_description/urdf/ax.urdf.xacro robot_name:=robot_1 sensor_robot_id:=robot_1 frame_prefix:=robot_1/ "
               f"sensors_enabled:=false ros_namespace:=/ax/robot_1 gripper_model:=epick test_payload_mass:={payload} tool_attach:=true "
               f"ros2_control_params:={WS}/src/ax_control/config/ax1_controllers.yaml "
               f"ros2_control_common_params:={WS}/src/ax_control/config/common_controllers.yaml"]
    urdf = subprocess.run(command, capture_output=True, text=True, check=True).stdout
    mass, com = robot_com(urdf, dict(zip(ARM_JOINTS, POSES[pose])))
    front, rear = 0.437, -0.443

    return {"mass_kg": mass, "com_base_m": com.round(4).tolist(), "com_ahead_of_axle_mid_m": float(com[0] - (front + rear) / 2),
            "front_load_share_static": float((com[0] - rear) / (front - rear))}


# 조건 하나 요약
def summarize(name, data):
    rotations = data.get("rotations", [])
    out = {"status": data.get("status"), "error": data.get("error"), "rtf": data.get("clock", {}).get("rtf"),
           "trials": len(rotations), "trial_status": [r["status"] for r in rotations],
           "failures": [{"repeat": r["repeat"], "direction": r["direction"], "status": r["status"], "reasons": r["reasons"]}
                        for r in rotations if r["status"] != "completed"]}

    for side in ("left", "right", "all"):
        items = [r for r in rotations if side == "all" or r["direction"] == side]
        steady = [r.get("circle_fit", {}).get("steady", {}) for r in items]
        out[side] = {
            "truth_turn_rad": stats([r.get("truth_turn_rad") for r in items]),
            "turn_diff_rad": stats([(r.get("odom_turn_rad") or 0) - (r.get("truth_turn_rad") or 0) for r in items]),
            "truth_translation_m": stats([r.get("truth_translation_m") for r in items]),
            "translation_error_m": stats([r.get("translation_error_m") for r in items]),
            "path_length_m": stats([r.get("truth_path_length_m") for r in items]),
            "odom_position_error_max_m": stats([r.get("odom_error", {}).get("position_max_m") for r in items]),
            "steady_centre_forward_m": stats([s.get("centre_body_m", [None])[0] if s.get("centre_body_m") else None for s in steady]),
            "steady_centre_left_m": stats([s.get("centre_body_m", [None, None])[1] if s.get("centre_body_m") else None for s in steady]),
            "steady_fit_residual_m": stats([s.get("residual_rms_m") for s in steady]),
            "odom_stop_s": stats([r.get("stop", {}).get("odom_stop_s") for r in items]),
            "truth_stop_s": stats([r.get("stop", {}).get("truth_stop_s") for r in items]),
            "tilt_max_rad": stats([max(r.get("tilt_max_rad", {}).values() or [None]) for r in items]),
        }

    if "straight" in data:
        out["straight"] = [{k: s.get(k) for k in ("command_m_s", "status", "truth_distance_m", "odom_distance_m", "lateral_m", "slip")}
                           for s in data["straight"]]
        out["stop_tests"] = data.get("stop_tests")

    return out


# 실행
def main():
    summary = {"judgment": "not_judged", "note": "정량 기준이 확정되지 않아 기록만 한다. 병진 감소만으로 후보를 채택하지 않는다",
               "conditions": {}, "com": {}}

    for name in CONDITIONS + ["contact_diag", "smoke"]:
        path = R / "trials" / f"{name}.json"

        if path.exists():
            summary["conditions"][name] = summarize(name, json.loads(path.read_text()))

    for name in CONDITIONS[:4]:
        _, pose, payload = name.split("_")
        summary["com"][name.replace("base_", "")] = condition_com("neutral" if pose == "neutral" else "reach_side",
                                                                  float(payload.replace("kg", "")))

    contact = json.loads((R / "trials/contact_diag.json").read_text()) if (R / "trials/contact_diag.json").exists() else None

    if contact:
        summary["contact_diag"] = [{"direction": r["direction"], "contact": r.get("contact")} for r in contact.get("rotations", [])]

    (R / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False, default=float))

    # 그림: 조건별 첫 왼쪽·오른쪽 회전의 차체 원점 경로(시작 차체 기준)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 4, figsize=(16, 8.5), sharex=True, sharey=True)

    for axis, name in zip(axes.flat, CONDITIONS):
        path = R / "trials" / f"{name}.json"
        axis.set_title(name, fontsize=9)
        axis.plot([0.437, 0.437, -0.443, -0.443, 0.437], [0.311, -0.311, -0.311, 0.311, 0.311], color="0.7", lw=0.8)
        axis.plot([0], [0], "k+")

        if path.exists():
            for r in json.loads(path.read_text()).get("rotations", [])[:2]:
                points = np.array(r.get("path_body_m") or [[0, 0]])
                axis.plot(points[:, 0], points[:, 1], lw=1.4, label=f"{r['direction']} {r.get('truth_translation_m', 0):.3f} m")

        axis.set_aspect("equal")
        axis.grid(alpha=0.3)
        axis.legend(fontsize=7, loc="lower left")

    fig.suptitle("robot_1 base_link path during ~3 rad in-place turn (start body frame, m). grey = wheel contact rectangle")
    fig.tight_layout()
    fig.savefig(R / "trajectory.png", dpi=110)

    # manifest
    git = lambda *a: subprocess.run(["git", "-C", str(WS), *a], capture_output=True, text=True).stdout.strip()
    pkg = lambda n: subprocess.run(["dpkg-query", "-W", "-f=${Version}", n], capture_output=True, text=True).stdout
    manifest = {
        "purpose": "robot_1 회전 병진 기준 시험·S005 비교(모델·구동 진단, A4·G1 검증 아님)",
        "git": {"commit": git("rev-parse", "HEAD"), "uncommitted": git("status", "--short").splitlines(),
                "model_changed_vs_commit": git("diff", "--stat", "HEAD", "--", "src/ax_description", "src/ax_control/config") or "없음"},
        "versions": {n: pkg(n) for n in ("ros-jazzy-gz-sim-vendor", "ros-jazzy-gz-physics-vendor", "ros-jazzy-gz-dartsim-vendor",
                                          "ros-jazzy-sdformat-vendor", "ros-jazzy-gz-ros2-control", "ros-jazzy-ros2-control",
                                          "ros-jazzy-diff-drive-controller")},
        "versions_note": "gz-sim 8.15.0, gz-physics 7.8.0 dartsim, DART 6.13.2, SDFormat 14",
        "robot": {"id": "robot_1", "tool": "epick(기본, 12컵 비교 모델)", "wheel_separation": 0.622, "wheel_radius": 0.19631,
                  "wheel_separation_multiplier": 1.2315, "wheel_friction": "mu1 0.1 / mu2 1.0 / fdir1 축 (ros2_control.xacro ax_wheel_contact)"},
        "model_sha256": {p: sha(WS / p) for p in ["src/ax_description/urdf/ax.urdf.xacro", "src/ax_description/urdf/ros2_control.xacro",
                                                   "src/ax_description/urdf/physical_properties.xacro", "src/ax_description/config/physical_properties.json",
                                                   "src/ax_description/config/epick.yaml", "src/ax_control/config/ax1_controllers.yaml",
                                                   "src/ax_control/config/common_controllers.yaml", "src/ax_description/worlds/ax_test.world"]},
        "tools_sha256": {p.name: sha(p) for p in [R / "plan.yaml", R / "r1_rotation.py", R / "run_conditions.sh", R / "summarize.py",
                                                  WS / "src/ax_control/scripts/check_wheel_calibration.py"]},
        "model_diffs": {p.name: sha(p) for p in sorted((R / "model_diffs").glob("*.diff"))},
        "trials_sha256": {p.name: sha(p) for p in sorted((R / "trials").glob("*.json"))},
        "environment": {"ROS_DOMAIN_ID": "87", "GZ_PARTITION": "ax-r1-<조건>-<pid>", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
                        "GALLIUM_DRIVER": "d3d12", "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA",
                        "launch": "ros2 launch ax_control simulation.launch.py robot_ids:=robot_1 gui:=false sensors:=false tool:=epick world:=ax_test.world payload_mass:=<kg>"},
        "command": "bash run_conditions.sh <이름>:<자세>:<하중>:<slip1|->:<contact>[:<회차>] ... ; python3 summarize.py",
    }
    (R / "manifest.json").write_text(json.dumps(manifest, indent=1, ensure_ascii=False))
    print("summary·trajectory·manifest 작성")


if __name__ == "__main__":
    main()
