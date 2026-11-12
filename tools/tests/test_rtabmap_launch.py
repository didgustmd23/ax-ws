# B2 비교용 RTAB-Map 선택 모드(mapping_rtabmap.launch.py) 인자 검증과 기본 동작 불변 시험.
# rtabmap 패키지 설치 없이 launch 파일의 순수 함수와 설정 파일 내용만 검사한다.
import importlib.util
from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1].parent
PKG = ROOT / "src/ax_localization"
LAUNCH = PKG / "launch/mapping_rtabmap.launch.py"


# launch 모듈을 불러온다. launch_ros가 없으면 None.
def load():
    if importlib.util.find_spec("launch_ros") is None:
        return None
    spec = importlib.util.spec_from_file_location("mapping_rtabmap_under_test", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


lm = load()


@unittest.skipIf(lm is None, "launch_ros 없음")
class Validate(unittest.TestCase):
    # 기본값 조합: 앞 LiDAR, 중복 검사 켬, 병합 노드 없음
    def test_defaults(self):
        self.assertEqual(lm.validate("robot_2", "front", "true", "false"),
                         ("robot_2", "sensors/front_lidar/scan_filtered", True, False))

    # 병합 스캔과 병합 노드
    def test_merged(self):
        self.assertEqual(lm.validate("robot_2", "merged", "false", "true")[1:], ("sensors/merged/scan_filtered", False, True))

    # 잘못된 값은 거절
    def test_rejects(self):
        for bad in (("robot_9", "front", "true", "false"), ("robot_2", "rear", "true", "false"),
                    ("robot_2", "front", "yes", "false"), ("robot_2", "front", "true", "true")):
            with self.assertRaises(ValueError, msg=str(bad)):
                lm.validate(*bad)

    # database 비움: 새 임시 폴더 db에만 -d
    def test_database_empty_uses_temp_with_delete(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path, arguments = lm.database_plan("", make_temp=lambda: tmp)
            self.assertEqual((path, arguments), (str(Path(tmp) / "rtabmap.db"), ["-d"]))

    # database 지정: 새 경로면 -d 없이, 기존 파일이면 거절(지우지 않음)
    def test_database_given_never_deletes(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            new = str(Path(tmp) / "new.db")
            self.assertEqual(lm.database_plan(new), (new, []))

            old = Path(tmp) / "old.db"
            old.write_bytes(b"keep")
            with self.assertRaises(ValueError):
                lm.database_plan(str(old))
            self.assertEqual(old.read_bytes(), b"keep")

    # 선언된 기본값이 문서와 같다(preflight true, start_merge false, scan front)
    def test_declared_defaults(self):
        from launch.actions import DeclareLaunchArgument
        declared = {a.name: a.default_value[0].text for a in lm.generate_launch_description().entities
                    if isinstance(a, DeclareLaunchArgument)}
        self.assertEqual(declared["preflight"], "true")
        self.assertEqual(declared["start_merge"], "false")
        self.assertEqual(declared["scan"], "front")


class Config(unittest.TestCase):
    # 초안 설정의 핵심 값(지시 초안: ICP 정합, 3DoF, LiDAR 격자, RGB-D+scan 구독)
    def test_rtabmap_config(self):
        cfg = yaml.safe_load((PKG / "config/rtabmap_mapping.yaml").read_text())["/**/rtabmap"]["ros__parameters"]
        self.assertEqual((cfg["Reg/Strategy"], cfg["Reg/Force3DoF"], cfg["Grid/Sensor"]), ("1", "true", "0"))
        self.assertTrue(cfg["subscribe_rgbd"] and cfg["subscribe_scan"] and cfg["use_sim_time"])

    # 기본 지도 작성(mapping.launch.py)은 RTAB-Map 노드를 띄우지 않고 slam_toolbox·slam_mapping.yaml을 쓴다.
    # (2026-11-12 기본 입력이 병합 스캔으로 바뀌어 주석에 mapping_rtabmap 이름이 나올 수 있으므로 실행 패키지로 검사한다)
    def test_default_mapping_untouched(self):
        text = (PKG / "launch/mapping.launch.py").read_text()
        for package in ("rtabmap_slam", "rtabmap_sync", "rtabmap_util"):
            self.assertNotIn(f'package="{package}"', text)
        self.assertIn('package="slam_toolbox"', text)
        self.assertIn("slam_mapping.yaml", text)

        if lm is not None:
            import test_mapping_merged_input as merged
            packages = {package for package, _ in merged.executables(merged.setup_actions("merged"))}
            self.assertEqual(packages, {"slam_toolbox", "ax_localization"})


if __name__ == "__main__":
    unittest.main()
