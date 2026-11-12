# B2 지도 평가 도구(check_map_accuracy.py)의 새 창고 월드 메타정보·부분 경로 상태·실행 시점 설정 해시를 검사한다.
# 5단계 준비(log/b2_custom_warehouse/20261111_095146/step5_prep)에서 재현한 결함의 회귀 시험이다. Gazebo·ROS 실행 없음.
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "src/ax_localization/scripts"
PROCS = SCRIPTS / "session_procs.sh"
NEW_WORLD = "planned/ax_warehouse.world"
CONFIG_NAMES = ("slam_mapping.yaml", "ax2_controllers.yaml", "common_controllers.yaml", "sensor_mounts.yaml",
                "a1_validation.yaml")
CONFIG_PACKAGES = {"slam_mapping.yaml": "ax_localization", "ax1_controllers.yaml": "ax_control",
                   "ax2_controllers.yaml": "ax_control", "common_controllers.yaml": "ax_control",
                   "sensor_mounts.yaml": "ax_description", "a1_validation.yaml": "ax_description"}


# 비교 도구를 읽는다(의존 패키지가 없으면 None).
def load_tool():
    spec = importlib.util.spec_from_file_location("check_map_accuracy_under_test", SCRIPTS / "check_map_accuracy.py")
    module = importlib.util.module_from_spec(spec)

    try:
        import sys
        sys.path.insert(0, str(SCRIPTS))
        spec.loader.exec_module(module)
    except ImportError:
        return None

    return module


cma = load_tool()


def sha(data):
    return hashlib.sha256(data).hexdigest()


# 시험용 회차(run·session)를 만든다. world=None이면 옛 AWS 형식 session(world 키 없음).
def make_run(folder, world=NEW_WORLD, gz_world="ax_test", route_world=NEW_WORLD, world_copy_ok=True,
             configs="ok", drive_world=NEW_WORLD, robot_id="robot_2", recorded_robot=None):
    folder = Path(folder)
    run, session_dir = folder / "run", folder / "session"
    run.mkdir()
    session_dir.mkdir()
    grid = np.full((20, 30), 0, np.int8)
    grid[0, :] = 100
    import map_tools as mt
    mt.write_map(run / "slam_map.yaml", grid, mt.GridGeometry(0.0, 0.0, 0.05, 30, 20))
    world_bytes = b"<sdf>world</sdf>"
    route_text = (f"world: {route_world}\n" if route_world else "") + "robot_id: robot_2\n"
    (session_dir / "route.yaml").write_text(route_text)
    session = {"world_condition": "original", "world_sha256": sha(world_bytes),
               "wheel_condition": "original", "wheel_override_file": "", "wheel_override_sha256": "",
               "git_commit": "c" * 40, "git_dirty": True, "route_sha256": sha(route_text.encode()),
               "route_copy": "session/route.yaml"}

    if world is None:
        session["world_file"] = "install/.../aws_warehouse_harmonic.world"
    else:
        session.update({"world": world, "gz_world": gz_world,
                        "world_file": f"install/ax_description/share/ax_description/worlds/{world}"})
        (session_dir / "world.world").write_bytes(world_bytes if world_copy_ok else b"changed")

    if configs is not None:
        (session_dir / "config").mkdir()
        files = {}
        recorded_robot = recorded_robot or robot_id
        controller = "ax1_controllers.yaml" if recorded_robot in ("robot_1", "robot_3") else "ax2_controllers.yaml"

        for name in ("slam_mapping.yaml", controller, *CONFIG_NAMES[2:]):
            data = f"{name}: 1\n".encode()
            edited = configs == "mismatch" and name == "slam_mapping.yaml"
            (session_dir / "config" / name).write_bytes(b"edited\n" if edited else data)
            files[name] = {"package": CONFIG_PACKAGES[name], "source": f"/prefix/share/x/config/{name}",
                           "copy": f"session/config/{name}", "sha256": sha(data)}

        if configs == "partial":
            files.pop("sensor_mounts.yaml")

        session["config_files"] = {"robot_id": recorded_robot, "controller_file": controller, "files": files}

    (session_dir / "session.json").write_text(json.dumps(session))
    drive = {"robot_id": robot_id, "scan_input": "scan_filtered",
             "route": {"path": str(session_dir / "route.yaml"), "sha256": sha(route_text.encode()), "partial": False},
             "world": {"file": drive_world, "gz_world": gz_world},
             "stop_gate": {"save": {"confirmed": True}, "completion": {"confirmed": True}},
             "model": {"source": "t", "file": "f", "sha256": "d" * 64, "robot_name": "robot_2", "wheel_contacts": {},
                       "lidars": {}}}
    transforms = {"initial_T_world_map": {}, "optimal_T_world_map": {}, "initial_method": "m"}
    accuracy = {"judgment": "not_judged", "reason": None, "metrics_optimal": {}, "scan_frame": "robot_2/front_lidar_link",
                "front_mount": [0.61, 0, 0], "transforms": transforms}
    slam_yaml = yaml.safe_load((run / "slam_map.yaml").read_text())

    return run, drive, accuracy, slam_yaml


@unittest.skipIf(cma is None, "의존 패키지 필요")
class NewWorldMetadata(unittest.TestCase):

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    # [재현] 새 월드 회차 메타정보는 session의 월드 경로·내부 이름·해시를 쓰고 AWS 출처를 붙이지 않는다
    def test_new_world_uses_session_world(self):
        metadata = cma.map_metadata(*make_run(self.folder))
        world = metadata["world"]

        self.assertEqual(world["launch_name"], NEW_WORLD)
        self.assertEqual(world["gz_world"], "ax_test")
        self.assertTrue(world["world_file"].endswith(NEW_WORLD))
        self.assertEqual(world["world_sha256"], sha(b"<sdf>world</sdf>"))
        self.assertNotIn("aws", json.dumps(world).lower())
        self.assertIsNone(world.get("reference_assets"))
        self.assertEqual(cma.metadata_problems(metadata), [])

    # 옛 AWS 형식 session(world 키 없음)은 기존처럼 AWS 출처를 붙여 읽는다
    def test_legacy_aws_session_still_readable(self):
        metadata = cma.map_metadata(*make_run(self.folder, world=None, route_world=None, configs=None,
                                              drive_world=None))

        self.assertEqual(metadata["world"]["launch_name"], "aws_warehouse.world")
        self.assertIn("reference_assets", metadata["world"])

    # [재현] 새 월드의 월드 사본·경로 world·주행 기록 world·내부 이름 불일치는 채택 전 문제로 남는다
    def test_new_world_mismatches_are_problems(self):
        cases = {"월드 사본 해시": dict(world_copy_ok=False), "경로 world": dict(route_world="planned/other.world"),
                 "주행 기록 world": dict(drive_world="planned/other.world"), "내부 이름 없음": dict(gz_world=None)}

        for label, options in cases.items():
            with self.subTest(case=label), tempfile.TemporaryDirectory() as folder:
                problems = cma.metadata_problems(cma.map_metadata(*make_run(folder, **options)))
                self.assertTrue(problems, label)

    # [재현] 설정 해시는 회차 기록·사본만 쓴다. 기록이 없거나 사본이 다르면 현재 파일 해시로 채우지 않는다
    def test_config_hashes_from_session_only(self):
        run, drive, accuracy, slam_yaml = make_run(self.folder)
        metadata = cma.map_metadata(run, drive, accuracy, slam_yaml)
        self.assertEqual(metadata["slam"]["config_sha256"], sha(b"slam_mapping.yaml: 1\n"))
        self.assertEqual(set(metadata["config_files"]), set(CONFIG_NAMES))
        self.assertTrue(all(item["copy_matches"] for item in metadata["config_files"].values()))

        for mode, needle in (("mismatch", "slam_mapping.yaml"), ("partial", "sensor_mounts.yaml"), (None, "config_files")):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                metadata = cma.map_metadata(*make_run(folder, configs=mode))
                problems = cma.metadata_problems(metadata)
                self.assertTrue(any(needle in item for item in problems), problems)

                if mode is None:
                    self.assertIsNone(metadata["slam"]["config_sha256"])

    # 주행 로봇에 맞는 제어기 설정만 인정한다: robot_1·3은 ax1, robot_2·4는 ax2. 다른 로봇 기록은 문제
    def test_controller_file_by_robot(self):
        for robot, controller in (("robot_1", "ax1_controllers.yaml"), ("robot_3", "ax1_controllers.yaml"),
                                  ("robot_2", "ax2_controllers.yaml"), ("robot_4", "ax2_controllers.yaml")):
            with self.subTest(robot=robot), tempfile.TemporaryDirectory() as folder:
                metadata = cma.map_metadata(*make_run(folder, robot_id=robot))
                self.assertIn(controller, metadata["config_files"])
                self.assertEqual(cma.metadata_problems(metadata), [])

        for robot, recorded in (("robot_1", "robot_2"), ("robot_2", "robot_1")):
            with self.subTest(robot=robot, recorded=recorded), tempfile.TemporaryDirectory() as folder:
                problems = cma.metadata_problems(cma.map_metadata(*make_run(folder, robot_id=robot, recorded_robot=recorded)))
                self.assertTrue(any("controllers.yaml" in item or "기록 로봇" in item for item in problems), problems)

    # [재현] 채택 대상 폴더: 새 월드는 --map-dir를 명시해야 하며, 옛 AWS 회차만 기존 기본값을 쓴다
    def test_adoption_target_requires_explicit_dir(self):
        new = cma.map_metadata(*make_run(self.folder))

        with tempfile.TemporaryDirectory() as folder:
            legacy = cma.map_metadata(*make_run(folder, world=None, route_world=None, configs=None, drive_world=None))

        target, problems = cma.adoption_target(None, new)
        self.assertIsNone(target)
        self.assertTrue(problems)
        target, problems = cma.adoption_target(Path("/tmp/maps/ax_warehouse"), new)
        self.assertEqual((target, problems), (Path("/tmp/maps/ax_warehouse"), []))
        target, problems = cma.adoption_target(None, legacy)
        self.assertEqual(target.name, "aws_warehouse")
        self.assertEqual(problems, [])


@unittest.skipIf(cma is None, "의존 패키지 필요")
class PartialRouteStatus(unittest.TestCase):

    def base(self, partial, map_status="completed"):
        return {"drive_status": {"status": "completed", "partial_route": partial},
                "trajectory_comparison": {"status": "completed"}, "map_comparison": {"status": map_status},
                "judgment": "not_judged"}

    # [재현] 부분 경로는 세 단계가 모두 completed여도 전체 status가 completed가 아니다(세부 결과는 보존)
    def test_partial_route_not_completed(self):
        result = self.base(True)
        status, reason = cma.overall_status(result)
        self.assertEqual(status, "incomplete")
        self.assertIn("부분 경로", reason)
        self.assertEqual(result["map_comparison"]["status"], "completed")

    # 전체 경로는 기존처럼 completed, 부분 경로 지도 미저장(not_run)도 기존처럼 incomplete
    def test_full_and_no_save_regression(self):
        self.assertEqual(cma.overall_status(self.base(False))[0], "completed")
        self.assertEqual(cma.overall_status(self.base(True, "not_run"))[0], "incomplete")
        self.assertEqual(cma.overall_status(self.base(False, "failed"))[0], "incomplete")


# 가짜 ament prefix(패키지 표식·config 파일)를 만든다. 값은 파일 내용 접두사.
def fake_prefix(root, label, packages):
    for package, names in packages.items():
        (root / "share/ament_index/resource_index/packages").mkdir(parents=True, exist_ok=True)
        (root / "share/ament_index/resource_index/packages" / package).write_text("")
        (root / "share" / package / "config").mkdir(parents=True, exist_ok=True)

        for name in names:
            (root / "share" / package / "config" / name).write_text(f"{label}:{name}\n")

    return root


ALL_PACKAGES = {"ax_localization": ["slam_mapping.yaml"],
                "ax_control": ["ax1_controllers.yaml", "ax2_controllers.yaml", "common_controllers.yaml"],
                "ax_description": ["sensor_mounts.yaml", "a1_validation.yaml"]}


@unittest.skipIf(importlib.util.find_spec("ament_index_python") is None, "ament_index_python 필요(source ROS)")
class SessionConfigRecord(unittest.TestCase):

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)

    def tearDown(self):
        for path in self.folder.rglob("*"):
            if path.is_file():
                path.chmod(0o644)

        self.temp.cleanup()

    # AMENT_PREFIX_PATH를 바꿔 기록 함수를 실행한다. (종료 코드, 기록 또는 None, session 폴더)
    def record(self, prefixes, robot, name="s"):
        session = self.folder / name
        session.mkdir()
        env = dict(__import__("os").environ, AMENT_PREFIX_PATH=":".join(str(p) for p in prefixes))
        result = subprocess.run(["bash", "-c", f"source {PROCS}; session_record_configs {session} {robot}"],
                                capture_output=True, text=True, timeout=60, env=env)
        path = session / "config_files.json"
        return result.returncode, (json.loads(path.read_text()) if path.is_file() else None), session

    # [재현] 로봇별 제어기 파일과 launch와 같은 ament share 경로(overlay 우선)를 기록한다
    def test_record_by_robot_and_overlay(self):
        base = fake_prefix(self.folder / "base", "base", ALL_PACKAGES)
        overlay = fake_prefix(self.folder / "overlay", "overlay",
                              {"ax_description": ["sensor_mounts.yaml", "a1_validation.yaml"]})

        for robot, controller in (("robot_1", "ax1_controllers.yaml"), ("robot_2", "ax2_controllers.yaml")):
            with self.subTest(robot=robot):
                code, record, session = self.record([overlay, base], robot, name=robot)
                self.assertEqual(code, 0)
                self.assertEqual((record["robot_id"], record["controller_file"]), (robot, controller))
                self.assertEqual(set(record["files"]), {"slam_mapping.yaml", controller, "common_controllers.yaml",
                                                        "sensor_mounts.yaml", "a1_validation.yaml"})
                # overlay가 앞이면 ax_description 설정은 overlay에서, 나머지는 base에서 읽는다(launch와 같은 탐색 순서)
                self.assertEqual(record["files"]["sensor_mounts.yaml"]["source"],
                                 str(overlay / "share/ax_description/config/sensor_mounts.yaml"))
                self.assertEqual((session / "config/sensor_mounts.yaml").read_text(), "overlay:sensor_mounts.yaml\n")
                self.assertEqual(record["files"][controller]["source"], str(base / "share/ax_control/config" / controller))

                for name, item in record["files"].items():
                    self.assertEqual(item["sha256"], sha((session / "config" / name).read_bytes()))

    # 복사 실패·패키지 없음·모르는 로봇이면 1을 돌려주고 기록 파일을 쓰지 않는다(시뮬레이션 전 중단 근거)
    def test_failures_leave_no_record(self):
        base = fake_prefix(self.folder / "base", "base", ALL_PACKAGES)
        unreadable = base / "share/ax_control/config/ax2_controllers.yaml"
        unreadable.chmod(0)
        code, record, _ = self.record([base], "robot_2", name="unreadable")
        unreadable.chmod(0o644)
        self.assertNotEqual(code, 0)
        self.assertIsNone(record)

        partial = fake_prefix(self.folder / "partial", "p", {"ax_localization": ["slam_mapping.yaml"]})
        code, record, _ = self.record([partial], "robot_2", name="no_package")
        self.assertNotEqual(code, 0)
        self.assertIsNone(record)

        code, record, _ = self.record([base], "robot_9", name="unknown_robot")
        self.assertNotEqual(code, 0)
        self.assertIsNone(record)

    # 세션 스크립트가 실행 로봇으로 기록 함수를 시뮬레이션 전에 부르고, 실패하면 시뮬레이션 전에 끝낸다
    def test_session_uses_record(self):
        text = (SCRIPTS / "run_mapping_session.sh").read_text()
        call = 'session_record_configs "$SESSION" "$ROBOT" || {'
        self.assertIn(call, text)
        self.assertIn("exit 2; }", text[text.index(call):text.index(call) + 120])
        self.assertIn('"config_files"', text)
        self.assertLess(text.index(call), text.index("start sim "))


if __name__ == "__main__":
    unittest.main()
