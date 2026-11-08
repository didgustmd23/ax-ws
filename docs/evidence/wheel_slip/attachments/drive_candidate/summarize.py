#!/usr/bin/env python3
"""주행 허용 자세 후보 비교 정리: summary.json, trajectory.png, manifest.json을 만든다."""

import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np

R = Path(__file__).resolve().parent
WS = Path.home() / "ax_ws"
R1 = WS / "log/a2_robot1_rotation/20261108_134133/trials"
CANDIDATES = {"rotation_translation_per_3rad_m": 0.10, "rotation_odom_yaw_error_rad": 0.02,
              "straight_odom_position_error_end_m": 0.02, "curve_odom_position_error_end_m": 0.05,
              "stop_time_increase_vs_baseline_s": 0.3}


# 파일 SHA-256
def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# 평균·표준편차·최댓값·개수(None 제외)
def stats(values):
    values = [abs(v) for v in values if v is not None]

    return None if not values else {"count": len(values), "mean": float(np.mean(values)), "std": float(np.std(values)),
                                    "max": float(np.max(values))}


# 한 로봇·조건 결과 요약. robot_1의 0.30 rad/s 회전은 재사용 결과를 합친다.
def summarize(data, reuse):
    rotations = list(data.get("rotations", []))

    for t in (reuse or {}).get("rotations", []):
        rotations.append(dict(t, omega_rad_s=0.3, reused=True))

    out = {"status": data.get("status"), "error": data.get("error"), "rtf": (data.get("clock") or {}).get("rtf"),
           "controller_limits": data.get("controller_limits"), "rotation": {}, "straight": {}, "curve": {}, "stop": {},
           "failures": []}

    for t in rotations:
        if t["status"] != "completed":
            out["failures"].append({"test": f"rotation {t['omega_rad_s']} {t['direction']} #{t['repeat']}", "status": t["status"],
                                    "reasons": t.get("reasons")})

    for omega in sorted({t["omega_rad_s"] for t in rotations}):
        for side in ("left", "right"):
            items = [t for t in rotations if t["omega_rad_s"] == omega and t["direction"] == side]
            steady = [t.get("circle_fit", {}).get("steady", {}) for t in items]
            out["rotation"][f"{omega:.2f}_{side}"] = {
                "reused": any(t.get("reused") for t in items), "trials": len(items),
                "statuses": [t["status"] for t in items],
                "truth_turn_rad": stats([t.get("truth_turn_rad") for t in items]),
                "turn_diff_rad": stats([(t.get("odom_turn_rad") or 0) - (t.get("truth_turn_rad") or 0) for t in items]),
                "truth_translation_m": stats([t.get("truth_translation_m") for t in items]),
                "odom_yaw_error_max_rad": stats([t.get("odom_error", {}).get("yaw_max_rad") for t in items]),
                "centre_forward_m": stats([s.get("centre_body_m", [None])[0] if s.get("centre_body_m") else None for s in steady]),
                "fit_residual_m": stats([s.get("residual_rms_m") for s in steady]),
                "odom_stop_s": stats([t.get("stop", {}).get("odom_stop_s") for t in items]),
                "truth_stop_s": stats([t.get("stop", {}).get("truth_stop_s") for t in items]),
                "tilt_max_rad": stats([max(t.get("tilt_max_rad", {"x": None}).values()) for t in items]),
            }

    for p in data.get("paths", []):
        group = out[p["kind"]]
        key = f"{p['command'][0]}_{p['command'][1]}"
        group.setdefault(key, []).append({"status": p["status"], "reason": p.get("reason"),
                                          "odom_position_error_max_m": (p.get("odom_error") or {}).get("position_max_m"),
                                          "odom_position_error_end_m": (p.get("odom_error") or {}).get("position_end_m"),
                                          "odom_yaw_error_max_rad": (p.get("odom_error") or {}).get("yaw_max_rad"),
                                          "truth_displacement_body_m": p["truth_displacement_body_m"],
                                          "odom_stop_s": p["stop"]["odom_stop_s"], "truth_stop_s": p["stop"]["truth_stop_s"],
                                          "tilt_max_rad": max(p["tilt_max_rad"].values())})

        if p["status"] != "completed":
            out["failures"].append({"test": f"{p['kind']} {key}", "status": p["status"], "reasons": p.get("reason")})

    for key, s in (data.get("stop_tests") or {}).items():
        out["stop"][key] = {k: s.get(k) for k in ("status", "reason", "stop", "slide_after_odom_stop_m", "tilt_max_rad", "motion")}

        if s["status"] != "completed":
            out["failures"].append({"test": f"stop {key}", "status": s["status"], "reasons": s.get("reason")})

    return out


# 구조 기준(PASS/FAIL)과 성능 후보(not_judged 기록) 정리
def structural(out):
    rows = [v for v in out["rotation"].values()]
    tilt = max([r["tilt_max_rad"]["max"] for r in rows if r["tilt_max_rad"]] or [0.0])
    vehicle = [f for f in out["failures"] if "category" not in f]
    layout = [f for f in out["failures"] if "category" in f]
    spawn_stops = out.get("stop_from_spawn")
    spawn_ok = spawn_stops is None or all(v["status"] == "completed" for v in spawn_stops.values())

    return {"vehicle_failures": len(vehicle), "test_layout_failures": len(layout), "tilt_max_rad": tilt, "tilt_ok": tilt <= 0.15,
            "stop_from_spawn_ok": spawn_ok,
            "verdict": "PASS" if not vehicle and tilt <= 0.15 and spawn_ok else "FAIL",
            "note": "시험 배치 실패(작업대 충돌)는 차량 구조 실패와 구분하되 결과에서 숨기지 않는다"}


# 실행
def main():
    summary = {"structural_criteria": "plan.yaml structural_criteria", "performance_candidates": CANDIDATES,
               "performance_judgment": "not_judged", "results": {}}

    for folder in ("baseline", "s005"):
        for robot in ("robot_1", "robot_2"):
            path = R / folder / f"{robot}.json"
            # robot_2_stops.json은 위에서 함께 읽는다.

            if not path.exists():
                summary["results"][f"{folder}/{robot}"] = {"status": "missing"}
                continue

            reuse = None

            if robot == "robot_1":
                name = "base_neutral_0kg" if folder == "baseline" else "S005_neutral_0kg"
                reuse = json.loads((R1 / f"{name}.json").read_text())

            out = summarize(json.loads(path.read_text()), reuse)

            # robot_2 정지 시험: 경로 끝 위치에서 작업대 station_src(8, 3)에 막힌 회차는 시험 배치 문제로 분류하고,
            # 출발 위치에서 따로 잰 정지 시험(*_stops.json, drive_eval v3)을 함께 싣는다.
            stops = R / folder / f"{robot}_stops.json"

            if stops.exists():
                for failure in out["failures"]:
                    if failure["status"] == "unexpected_motion" and failure["test"].startswith("stop"):
                        failure["category"] = "test_layout: 경로 끝에서 작업대 station_src(8, 3)에 막힘(실제 속도 0, odom 0.67 m/s)"

                separate = json.loads(stops.read_text())
                out["stop_from_spawn"] = {k: {kk: v.get(kk) for kk in ("status", "reason", "stop", "slide_after_odom_stop_m", "tilt_max_rad", "motion")}
                                          for k, v in (separate.get("stop_tests") or {}).items()}
                out["stop_from_spawn_status"] = separate.get("status")

            out["structural"] = structural(out)
            summary["results"][f"{folder}/{robot}"] = out

    summary["superseded"] = {p.name: "정지 시험 실제 정지 시간 이상값(0.003~0.007 s). 예상 밖 접촉 감지 없는 v1 도구 결과. 재실행본으로 대체"
                             for p in sorted(R.glob("*/robot_2.run1_stop_anomaly.json"))}
    (R / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False, default=float))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 4, figsize=(17, 8.5))

    for row, robot in enumerate(("robot_1", "robot_2")):
        for col, folder in enumerate(("baseline", "s005")):
            path = R / folder / f"{robot}.json"
            data = json.loads(path.read_text()) if path.exists() else {}
            ax = axes[row, col]
            ax.set_title(f"{robot} {folder}: 0.5 rad/s turns", fontsize=9)

            for t in [t for t in data.get("rotations", []) if t["omega_rad_s"] == 0.5][:2]:
                points = np.array(t.get("path_body_m") or [[0, 0]])
                ax.plot(points[:, 0], points[:, 1], label=f"{t['direction']} {t.get('truth_translation_m', 0):.3f} m")

            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7)
            ax = axes[row, col + 2]
            ax.set_title(f"{robot} {folder}: curves 0.3 m/s ±0.3 rad/s", fontsize=9)

            for p in [p for p in data.get("paths", []) if p["kind"] == "curve"][:2]:
                points = np.array(p.get("path_body_m") or [[0, 0]])
                ax.plot(points[:, 0], points[:, 1], label=f"w {p['command'][1]:+} odom err {p['odom_error']['position_max_m']:.3f} m")

            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7)

    fig.suptitle("base_link truth path in start body frame (m)")
    fig.tight_layout()
    fig.savefig(R / "trajectory.png", dpi=105)

    git = lambda *a: subprocess.run(["git", "-C", str(WS), *a], capture_output=True, text=True).stdout.strip()
    manifest = {
        "purpose": "주행 허용 자세 후보 기준 기본·S005 구동 비교(개발 진단, A4·G1 검증 아님)",
        "git": {"commit": git("rev-parse", "HEAD"), "uncommitted": git("status", "--short").splitlines(),
                "model_changed_vs_commit": git("diff", "--stat", "HEAD", "--", "src/ax_description", "src/ax_control/config") or "없음"},
        "versions_note": "gz-sim 8.15.0, gz-physics 7.8.0 dartsim, DART 6.13.2, SDFormat 14, gz_ros2_control 1.2.20, ros2_control 4.48.1",
        "model_sha256": {p: sha(WS / p) for p in ["src/ax_description/urdf/ax.urdf.xacro", "src/ax_description/urdf/ax_2.urdf.xacro",
                                                   "src/ax_description/urdf/ros2_control.xacro", "src/ax_description/urdf/physical_properties.xacro",
                                                   "src/ax_description/urdf/ax_2_physical_properties.xacro", "src/ax_control/config/common_controllers.yaml",
                                                   "src/ax_control/config/ax1_controllers.yaml", "src/ax_control/config/ax2_controllers.yaml",
                                                   "src/ax_description/worlds/ax_test.world"]},
        "tools": {
            "drive_eval.v1.py": {"sha256": sha(R / "drive_eval.v1.py"), "used_for": ["baseline/robot_1", "s005/robot_1", "*/robot_2.run1_stop_anomaly"],
                                 "note": "v2에서 예상 밖 접촉 감지 변경분을 역적용해 복원한 사본(원본 바이트 대조 불가)"},
            "drive_eval.v3.py": {"sha256": sha(R / "drive_eval.v3.py"), "used_for": ["baseline/robot_2_stops", "s005/robot_2_stops"],
                                 "change": "v2 + --stops-only(출발 위치에서 정지 시험만). 측정 방식 변경 없음"},
            "drive_eval.v2.py": {"sha256": sha(R / "drive_eval.v2.py"), "used_for": ["baseline/robot_2", "s005/robot_2"],
                                 "change": "odom 선속도 ≥ 0.2 m/s인데 실제 속도 < 절반이 0.5 s 이어지면 unexpected_motion 중단, 정지 시험 시작·끝 위치·이동 거리 기록"},
            "run_conditions.sh": sha(R / "run_conditions.sh"), "plan.yaml": sha(R / "plan.yaml"), "summarize.py": sha(R / "summarize.py"),
        },
        "reused": {"robot_1 0.30 rad/s 회전": {p: sha(R1 / p) for p in ("base_neutral_0kg.json", "S005_neutral_0kg.json")},
                   "source": "../a2_robot1_rotation/20261108_134133/trials (r1_rotation.py, 같은 모델 해시·조건·기록 항목)"},
        "results_sha256": {str(p.relative_to(R)): sha(p) for p in sorted(list((R / "baseline").glob("*.json")) + list((R / "s005").glob("*.json")))},
        "model_diffs": {p.name: sha(p) for p in sorted((R / "model_diffs").glob("*.diff"))},
        "environment": {"ROS_DOMAIN_ID": "86", "GZ_PARTITION": "ax-drive-<조건>-<pid>", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
                        "GALLIUM_DRIVER": "d3d12", "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA",
                        "launch": "ros2 launch ax_control simulation.launch.py robot_ids:=<robot> gui:=false sensors:=false tool:=epick world:=ax_test.world payload_mass:=0.0"},
    }
    (R / "manifest.json").write_text(json.dumps(manifest, indent=1, ensure_ascii=False))
    print("summary·trajectory·manifest 작성")


if __name__ == "__main__":
    main()
