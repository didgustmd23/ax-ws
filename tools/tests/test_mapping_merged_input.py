# B2 지도 작성 기본 입력 = 앞·뒤 병합 스캔(S2, 2026-11-12 채택) 시험.
# mapping.launch.py 인자·노드 구성, slam_mapping.yaml, 세션 스크립트 기본값, 주행 기록, 평가 메타데이터 대조를 검사한다.
# 근거: log/b2_custom_warehouse/20261111_212314/compare
import importlib.util
from pathlib import Path
import subprocess
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1].parent
PKG = ROOT / "src/ax_localization"
HAS_LAUNCH = importlib.util.find_spec("launch_ros") is not None
HAS_RCLPY = importlib.util.find_spec("rclpy") is not None


# 파일 경로로 모듈을 불러온다.
def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ml = load("mapping_launch_under_test", PKG / "launch/mapping.launch.py") if HAS_LAUNCH else None


# mapping.launch.py setup()을 그래프 검사 없이 실행해 만들어지는 action 목록을 돌려준다.
def setup_actions(scan, localization=(), merge=()):
    from launch import LaunchContext

    context = LaunchContext()
    context.launch_configurations.update({"robot_id": "robot_2", "scan": scan})
    saved = ml.running_localization, ml.running_scan_merge
    ml.running_localization = lambda namespace: list(localization)
    ml.running_scan_merge = lambda namespace: list(merge)

    try:
        return ml.setup(context)
    finally:
        ml.running_localization, ml.running_scan_merge = saved


# action 목록에서 (package, executable) 쌍을 모은다.
def executables(actions):
    return [(a.node_package, a.node_executable) for a in actions if hasattr(a, "node_executable")]


@unittest.skipIf(ml is None, "launch_ros 없음")
class MappingLaunch(unittest.TestCase):
    # 선언 기본값은 merged
    def test_default_scan_is_merged(self):
        from launch.actions import DeclareLaunchArgument
        declared = {a.name: a.default_value[0].text for a in ml.generate_launch_description().entities
                    if isinstance(a, DeclareLaunchArgument)}
        self.assertEqual(declared["scan"], "merged")

    # 입력 종류 → 토픽·병합 노드 필요 여부, 잘못된 값 거절
    def test_validate(self):
        self.assertEqual(ml.validate("robot_2", "merged"), ("sensors/merged/scan_filtered", True))
        self.assertEqual(ml.validate("robot_2", "filtered"), ("sensors/front_lidar/scan_filtered", False))
        self.assertEqual(ml.validate("robot_2", "raw"), ("sensors/front_lidar/scan", False))
        for bad in (("robot_2", "rear"), ("robot_9", "merged")):
            with self.assertRaises(ValueError):
                ml.validate(*bad)

    # merged: 병합 노드와 slam_toolbox를 함께 띄우고 slam 입력은 병합 토픽
    def test_merged_starts_merge_node(self):
        actions = setup_actions("merged")
        self.assertIn(("ax_localization", "scan_merge.py"), executables(actions))
        self.assertIn(("slam_toolbox", "async_slam_toolbox_node"), executables(actions))
        slam = next(a for a in actions if getattr(a, "node_executable", None) == "async_slam_toolbox_node")
        text = lambda value: "".join(getattr(part, "text", str(part)) for part in value) if isinstance(value, tuple) else str(value)  # noqa: E731
        overrides = [p for p in slam._Node__parameters if isinstance(p, dict)]
        topics = [text(v) for d in overrides for k, v in d.items() if text(k) == "scan_topic"]
        self.assertTrue(any("sensors/merged/scan_filtered" in str(t) for t in topics), topics)

    # filtered·raw: 병합 노드 없음(비교용 경로 유지)
    def test_filtered_and_raw_without_merge(self):
        for scan in ("filtered", "raw"):
            self.assertNotIn(("ax_localization", "scan_merge.py"), executables(setup_actions(scan)), scan)

    # 같은 namespace에 scan_merge가 이미 있으면 merged는 거절, filtered는 영향 없음
    def test_existing_merge_rejected(self):
        with self.assertRaises(RuntimeError):
            setup_actions("merged", merge=["scan_merge"])
        setup_actions("filtered", merge=["scan_merge"])

    # map→odom 발행 후보가 있으면 거절(기존 규칙 유지)
    def test_existing_localization_rejected(self):
        with self.assertRaises(RuntimeError):
            setup_actions("merged", localization=["/ax/robot_2/amcl"])

    # RTAB-Map 선택 모드의 start_merge도 같은 병합 노드 생성 함수를 쓴다
    def test_rtabmap_shares_merge_node(self):
        rl = load("mapping_rtabmap_shared_under_test", PKG / "launch/mapping_rtabmap.launch.py")
        node = rl.mapping_launch().scan_merge_node("/ax/robot_2", "robot_2")
        self.assertEqual((node.node_package, node.node_executable), ("ax_localization", "scan_merge.py"))
        self.assertIn("running_scan_merge", (PKG / "launch/mapping_rtabmap.launch.py").read_text())


class SlamConfig(unittest.TestCase):
    # 기본 설정: 병합 토픽, expansion 끔. 나머지 정합 값은 비교 시험(S2)과 같아야 한다.
    def test_slam_mapping_yaml(self):
        cfg = yaml.safe_load((PKG / "config/slam_mapping.yaml").read_text())["/**/slam_toolbox"]["ros__parameters"]
        self.assertEqual(cfg["scan_topic"], "/ax/robot_2/sensors/merged/scan_filtered")
        self.assertIs(cfg["use_response_expansion"], False)
        self.assertEqual((cfg["minimum_travel_distance"], cfg["minimum_travel_heading"], cfg["resolution"]), (0.5, 0.5, 0.05))

    # AMCL 기본 입력은 바꾸지 않는다(앞 scan_filtered 유지, 별도 결정). 2026-11-14부터 localization.launch.py에
    # 시험용 선택 입력 scan:=merged가 있으나 기본값은 filtered이고 amcl.yaml은 앞 입력 그대로다.
    def test_amcl_untouched(self):
        self.assertNotIn("merged", (PKG / "config/amcl.yaml").read_text())
        self.assertIn("/sensors/front_lidar/scan_filtered", (PKG / "config/amcl.yaml").read_text())
        launch = (PKG / "launch/localization.launch.py").read_text()
        self.assertIn('DeclareLaunchArgument("scan", default_value="filtered",', launch)


class Session(unittest.TestCase):
    SCRIPT = PKG / "scripts/run_mapping_session.sh"

    # 세션 기본 scan은 merged이고 병합 토픽 준비를 기다리며 병합 노드 해시를 session.json에 남긴다
    def test_session_defaults(self):
        text = self.SCRIPT.read_text()
        self.assertIn("SCAN=merged;", text)
        self.assertIn('wait_topic "/ax/$ROBOT/sensors/merged/scan_filtered"', text)
        self.assertIn('"scan_merge": $SCAN_MERGE_RECORD', text)
        self.assertIn('if [ "$SCAN" != raw ]; then', text)     # merged·filtered는 B1을 띄운다

    # 알 수 없는 scan은 시뮬레이션 전에 rc 2로 거절
    def test_session_rejects_unknown_scan(self):
        result = subprocess.run(["bash", str(self.SCRIPT), "--scan", "rear"], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 2)
        self.assertIn("merged, filtered, raw", result.stderr)


@unittest.skipIf(not HAS_RCLPY, "rclpy 없음")
class Drive(unittest.TestCase):
    md = load("mapping_drive_merged_under_test", PKG / "scripts/mapping_drive.py") if HAS_RCLPY else None

    # 장애물 감시는 병합 스캔이 아니라 앞·뒤 원 스캔
    def test_obstacle_suffix(self):
        self.assertEqual(self.md.obstacle_scan_suffix("merged"), "scan_filtered")
        self.assertEqual(self.md.obstacle_scan_suffix("filtered"), "scan_filtered")
        self.assertEqual(self.md.obstacle_scan_suffix("raw"), "scan")
        with self.assertRaises(ValueError):
            self.md.obstacle_scan_suffix("rear")

    # 주행 기록에 SLAM 입력 종류가 남는다
    def test_run_records_slam_input(self):
        import tempfile
        route_file = Path(tempfile.mkdtemp()) / "r.yaml"
        route_file.write_text("x: 1\n")
        route = {"robot_id": "robot_2", "frame": "robot_2/odom", "waypoints": []}
        run = self.md.Run(Path(tempfile.mkdtemp()), route_file, route, "scan_filtered", slam_scan_input="merged")
        self.assertEqual((run.record["scan_input"], run.record["slam_scan_input"]), ("scan_filtered", "merged"))


class Accuracy(unittest.TestCase):
    ca = load("check_map_accuracy_merged_under_test", PKG / "scripts/check_map_accuracy.py") if HAS_RCLPY else None

    # 병합 입력·해시 기록 일치면 문제 없음, 해시 없음·종류 불일치는 문제, 옛 기록(둘 다 없음)은 문제 아님
    @unittest.skipIf(not HAS_RCLPY, "rclpy 없음")
    def test_slam_input_record(self):
        merge = {"file": "/x/scan_merge.py", "sha256": "a" * 64}
        ok = self.ca.slam_input_record({"scan": "merged", "scan_merge": merge}, {"slam_scan_input": "merged"})
        self.assertEqual(ok, ("merged", merge, []))
        self.assertTrue(self.ca.slam_input_record({"scan": "merged", "scan_merge": None}, {"slam_scan_input": "merged"})[2])
        self.assertTrue(self.ca.slam_input_record({"scan": "merged", "scan_merge": merge}, {"slam_scan_input": "filtered"})[2])
        self.assertEqual(self.ca.slam_input_record({"scan": "filtered"}, {}), ("filtered", None, []))
        self.assertEqual(self.ca.slam_input_record(None, {}), (None, None, []))


if __name__ == "__main__":
    unittest.main()
