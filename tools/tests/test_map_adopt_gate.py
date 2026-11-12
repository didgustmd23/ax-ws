# B2 지도 채택(check_map_accuracy.py --adopt)의 판정 검사와 map_info 경로 표기를 검사한다. Gazebo·ROS 실행 없음.
# - 판정 PASS만 채택하고, FAIL·not_judged는 채택 폴더의 파일을 바꾸지 않고 거절한다.
# - map_info의 경로는 작업공간 기준 상대 경로(밖이면 "~/" 기준)로 기록하고 해시는 바꾸지 않는다.
import argparse
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_map_accuracy_world as base  # noqa: E402

cma = base.cma


# 채택 폴더의 파일 이름별 바이트(채택 전후 비교용)
def snapshot(folder):
    return {p.name: p.read_bytes() for p in sorted(Path(folder).iterdir()) if p.is_file()}


@unittest.skipIf(cma is None, "의존 패키지 필요")
class AdoptJudgmentGate(unittest.TestCase):

    # 시험 회차(지도·pose graph·세션)와 기존 채택 폴더(revision 7)를 만든다.
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        folder = Path(self.temp.name)
        (folder / "case").mkdir()
        self.run, self.drive, accuracy, _ = base.make_run(folder / "case")
        (self.run / "pose_graph.posegraph").write_bytes(b"graph")
        (self.run / "pose_graph.data").write_bytes(b"data")
        self.drive["saved_map_usable"] = True    # 판정 외 채택 조건은 모두 충족시켜 판정 검사만 본다

        self.target = folder / "maps"
        self.target.mkdir()
        (self.target / "map_info.yaml").write_text("map_revision: 7\n", encoding="utf-8")
        (self.target / "map.yaml").write_text("image: old.pgm\n", encoding="utf-8")
        self.before = snapshot(self.target)

        self.criteria = {"PASS": folder / "pass.yaml", "FAIL": folder / "fail.yaml"}
        self.criteria["PASS"].write_text("min_obstacle_detection_ratio: 0.0\nmin_reverse_match_ratio: 0.0\n"
                                         "max_wall_error_p95_m: 100.0\nmin_comparable_area_m2: 0.0\n", encoding="utf-8")
        self.criteria["FAIL"].write_text("min_obstacle_detection_ratio: 2.0\nmin_reverse_match_ratio: 0.0\n"
                                         "max_wall_error_p95_m: 100.0\nmin_comparable_area_m2: 0.0\n", encoding="utf-8")
        self.accuracy = accuracy

    def tearDown(self):
        self.temp.cleanup()

    # compare_map을 --adopt로 실행한다. 참조 지도는 SLAM 지도 자체, 정렬은 단위 행렬.
    def adopt(self, criteria):
        grid, geometry, _ = cma.mt.read_map(self.run / "slam_map.yaml")
        result = {"drive_status": {"status": "completed", "partial_route": False},
                  "trajectory_comparison": {"status": "completed"},
                  "transforms": dict(self.accuracy["transforms"]),
                  "scan_frame": self.accuracy["scan_frame"], "front_mount": self.accuracy["front_mount"],
                  "judgment": "not_judged"}
        truth = np.array([[0.0, 0.5, 0.5, 0.0], [1.0, 0.6, 0.5, 0.0]])
        arguments = argparse.Namespace(criteria=criteria, tolerance_cells=1, adopt=True, map_dir=self.target)
        comparison = cma.compare_map(self.run, self.drive, result, grid, geometry, truth, np.eye(3), arguments)
        return result, comparison

    # PASS면 채택하고 revision이 1 오른다
    def test_pass_adopts(self):
        result, comparison = self.adopt(self.criteria["PASS"])

        self.assertEqual(result["judgment"], "PASS")
        self.assertEqual(comparison["adoption"]["status"], "adopted", comparison["adoption"])
        self.assertEqual(comparison["adoption"]["map_revision"], 8)
        self.assertNotEqual(snapshot(self.target), self.before)

    # FAIL·not_judged(기준 없음)는 거절하고 채택 폴더 파일을 바꾸지 않는다
    def test_fail_and_unjudged_refused_without_change(self):
        for label, criteria in (("FAIL", self.criteria["FAIL"]), ("not_judged", None)):
            with self.subTest(judgment=label):
                result, comparison = self.adopt(criteria)

                self.assertEqual(result["judgment"], label)
                self.assertEqual(comparison["adoption"]["status"], "refused")
                self.assertTrue(any("PASS가 아니면 채택하지 않음" in p for p in comparison["adoption"]["problems"]))
                self.assertEqual(snapshot(self.target), self.before)
                self.assertFalse(any(p.name.startswith(".adopt_") for p in self.target.iterdir()))


@unittest.skipIf(cma is None, "의존 패키지 필요")
class PortablePaths(unittest.TestCase):

    # 작업공간 안은 상대 경로, 홈 안은 "~/", 그 밖·상대·빈 값은 그대로
    def test_portable_path(self):
        workspace, home = Path("/w/ws"), Path("/w")

        cases = {"/w/ws/install/a/b.yaml": "install/a/b.yaml", "/w/other/x.yaml": "~/other/x.yaml",
                 "/opt/ros/jazzy/x": "/opt/ros/jazzy/x", "session/config/x": "session/config/x", "": "", None: None}

        for value, expected in cases.items():
            with self.subTest(value=value):
                self.assertEqual(cma.portable_path(value, workspace, home), expected)

    # 메타정보의 경로 필드만 바뀌고 해시·나머지 값은 그대로다
    def test_portable_metadata_converts_only_paths(self):
        workspace, home = Path("/w/ws"), Path("/w")
        info = {"world": {"world_file": "/w/ws/install/w.world", "world_sha256": "a" * 64},
                "robot": {"sensor": {"scan_merge": {"file": "/w/ws/install/lib/scan_merge.py", "sha256": "b" * 64}}},
                "slam": {"config": "/w/ws/install/share/slam.yaml", "config_sha256": "c" * 64},
                "config_files": {"slam.yaml": {"source": "/w/ws/install/share/slam.yaml", "copy": "session/config/slam.yaml",
                                               "recorded_sha256": "c" * 64}},
                "route": {"file": "/w/ws/log/r/route.yaml", "sha256": "d" * 64},
                "wheel_friction": {"override_file": None},
                "verification": {"result": "/w/ws/log/x/run/accuracy.json", "drive_result": "/w/elsewhere/drive.json"}}

        out = cma.portable_metadata(info, workspace, home)

        self.assertEqual(out["world"], {"world_file": "install/w.world", "world_sha256": "a" * 64})
        self.assertEqual(out["robot"]["sensor"]["scan_merge"], {"file": "install/lib/scan_merge.py", "sha256": "b" * 64})
        self.assertEqual(out["slam"], {"config": "install/share/slam.yaml", "config_sha256": "c" * 64})
        self.assertEqual(out["config_files"]["slam.yaml"]["source"], "install/share/slam.yaml")
        self.assertEqual(out["config_files"]["slam.yaml"]["recorded_sha256"], "c" * 64)
        self.assertEqual(out["route"], {"file": "log/r/route.yaml", "sha256": "d" * 64})
        self.assertEqual(out["verification"], {"result": "log/x/run/accuracy.json", "drive_result": "~/elsewhere/drive.json"})
        self.assertEqual(info["route"]["file"], "/w/ws/log/r/route.yaml")   # 원본은 바꾸지 않는다

    # 실제 채택 지도 map_info.yaml에 작업 PC 홈 절대 경로가 없다
    def test_adopted_map_info_has_no_home_path(self):
        text = (base.ROOT / "src/ax_localization/maps/ax_warehouse/map_info.yaml").read_text(encoding="utf-8")

        # 이식성 검사(tools/check_portability.py)와 같은 방식으로 접두어를 나눠 써서 이 파일 자체가 걸리지 않게 한다.
        prefixes = ("/" + "home/", "/" + "mnt/", "/" + "Users/", str(Path.home()) + "/")
        lines = [line for line in text.splitlines() if any(prefix in line for prefix in prefixes)]

        self.assertEqual(lines, [])


if __name__ == "__main__":
    unittest.main()
