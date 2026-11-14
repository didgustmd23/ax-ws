# mapping_drive.py 출발 조건: sim time(/clock)을 받기 전에는 출발하지 않는다(진척 감시 기준 시각이 0에서 건너뛰는 결함 재현).
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "src/ax_localization/scripts/mapping_drive.py"


# rclpy가 있을 때만 모듈을 읽는다(없으면 None).
def load_module():
    if importlib.util.find_spec("rclpy") is None:
        return None

    spec = importlib.util.spec_from_file_location("mapping_drive_start_under_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


md = load_module()


@unittest.skipIf(md is None, "rclpy 필요(source install/setup.bash)")
class StartProblems(unittest.TestCase):

    # 입력이 모두 있어도 sim time이 0(미수신)이면 출발하지 않는다(2026-11-14 중앙 경로 AMCL 회차 재현).
    def test_clock_not_received(self):
        problems = md.start_problems([], 0.0)

        self.assertEqual(len(problems), 1)
        self.assertIn("sim time", problems[0])

    # 입력 문제는 그대로 남고 sim time 문제와 함께 보고한다.
    def test_inputs_and_clock(self):
        self.assertEqual(md.start_problems(["odom 없음"], 0.0)[0], "odom 없음")
        self.assertEqual(len(md.start_problems(["odom 없음"], 0.0)), 2)

    # sim time을 받았고 입력 문제가 없으면 출발할 수 있다. 비유한 값은 미수신으로 본다.
    def test_ready(self):
        self.assertEqual(md.start_problems([], 12.5), [])
        self.assertEqual(len(md.start_problems([], float("nan"))), 1)


if __name__ == "__main__":
    unittest.main()
