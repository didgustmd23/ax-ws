# B2 지도 작성 주행(mapping_drive.py)의 관측 관문(P1…)·정지 기록을 ROS 그래프·시뮬레이터 없이 검사한다.
# 가짜 노드로 관문 진입·대기·중단 우선·진행 파일 1회 소비·재출발 재검증과 옵션 없음 동작을 본다.
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1].parent
SCRIPT = ROOT / "src/ax_localization/scripts/mapping_drive.py"


# rclpy가 있어야 모듈을 읽을 수 있다(source install/setup.bash).
def load_module():
    if importlib.util.find_spec("rclpy") is None:
        return None

    sys.path.insert(0, str(SCRIPT.parent))
    spec = importlib.util.spec_from_file_location("mapping_drive_under_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


md = load_module()


# 경로 파일 내용(gate 표시 선택)
def route_text(gates=True):
    first = "{xy: [1.0, 0.0], gate: true}" if gates else "[1.0, 0.0]"
    return (
        "robot_id: robot_2\nframe: robot_2/odom\nlimits: {linear_m_s: 0.3, angular_rad_s: 0.3}\n"
        "goal_tolerance_m: 0.15\nheading_tolerance_rad: 0.12\n"
        "footprint: {front_m: 0.66, rear_m: 0.62, half_width_m: 1.03}\n"
        "safety: {processing_latency_s: 0.3, margin_m: 0.25, rotation_margin_m: 0.15}\n"
        "inputs: {scan_max_age_s: 0.5, odom_max_age_s: 0.3, tf_receive_max_age_boottime_s: 4.0, "
        "slam_progress_distance_m: 1.0, slam_progress_heading_rad: 1.0, input_timeout_s: 3.0}\n"
        "progress: {time_factor: 3.0, time_margin_s: 30.0, stall_window_s: 15.0, stall_min_improvement: 0.05}\n"
        f"waypoints:\n  - {first}\n  - [2.0, 0.0]\n  - {{xy: [3.0, 0.0], gate: {'true' if gates else 'false'}}}\n")


# 가짜 노드: sim time은 벽시계로 흐르고, 다른 스레드가 상태를 바꿀 수 있다(콜백 흉내).
class FakeNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.start = time.monotonic()
        self.frozen = None
        self.odom = (0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
        self.scans = {"front": None, "rear": None}
        self.map_record = []
        self.odom_record = []
        self.stops = 0

    def now_seconds(self):
        return self.frozen if self.frozen is not None else 10.0 + time.monotonic() - self.start

    def publish_stop(self):
        self.stops += 1

    def other_publishers(self):
        return []

    def slam_health(self):
        return {}


# 가짜 실행 기록
class FakeRun:
    def __init__(self):
        self.record = {"events": [], "stops": []}

    def event(self, sim_time, kind, detail):
        self.record["events"].append((kind, detail))

    def save(self):
        pass


@unittest.skipIf(md is None, "rclpy 필요(source install/setup.bash)")
class GateParsingAndHelpers(unittest.TestCase):

    # gate: true만 관문(P1, P2 순서). 경유점 튜플은 기존과 같다. 비 bool은 거절
    def test_load_route_gates(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "route.yaml"
            path.write_text(route_text(True))
            route = md.load_route(path)
            self.assertEqual(route["gates"], {0: "P1", 2: "P2"})
            self.assertEqual(route["waypoints"], [(1.0, 0.0, False), (2.0, 0.0, False), (3.0, 0.0, False)])

            path.write_text(route_text(False))
            self.assertEqual(md.load_route(path)["gates"], {})

            data = yaml.safe_load(route_text(True))
            data["waypoints"][0]["gate"] = "yes"
            path.write_text(yaml.safe_dump(data))

            with self.assertRaises(ValueError):
                md.load_route(path)

    # 신호: 진입 때 있던 파일(stale) 무시, 다시 touch하면 신호, 중단 우선, 진행 파일은 소비 뒤 재사용 불가
    def test_gate_signals(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            stale = folder / "T_P1.continue"
            stale.write_text("")
            baseline = md.gate_signal_baseline(folder, "T", "P1")
            self.assertEqual(md.read_gate_signal(folder, "T", "P1", baseline), (None, None))

            os.utime(stale, ns=(time.time_ns(), time.time_ns() + 1))
            kind, path = md.read_gate_signal(folder, "T", "P1", baseline)
            self.assertEqual(kind, "continue")
            (folder / "T.abort").write_text("")
            self.assertEqual(md.read_gate_signal(folder, "T", "P1", baseline)[0], "abort")
            (folder / "T.abort").unlink()

            md.consume_gate_signal(path)
            self.assertEqual(md.read_gate_signal(folder, "T", "P1", baseline), (None, None))
            self.assertEqual(md.read_gate_signal(folder, "T", "P2", baseline), (None, None))

    # [재현 원인] 벽시계가 뒤로 바뀐 뒤 만든 파일(mtime이 진입 시각보다 과거)도 새 신호로 본다(시계 비교 없음)
    def test_signal_after_clock_step_back(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            baseline = md.gate_signal_baseline(folder, "T", "P1")
            past = time.time_ns() - 3_000_000_000

            for name, kind in (("T_P1.continue", "continue"), ("T.abort", "abort")):
                path = folder / name
                path.write_text("")
                os.utime(path, ns=(past, past))
                self.assertEqual(md.read_gate_signal(folder, "T", "P1", baseline)[0], kind)

    # 기록: 자료별 stamp·age, 없으면 missing, 불완전 표시. map 자세 = map→odom · odom
    def test_snapshot_fields(self):
        odom = (10.0, 1.0, 0.0, 0.0, 0.2, 0.0)
        scans = {"front": {"stamp": 9.9, "data": {"ranges": [1.0, "inf"]}}, "rear": None}
        snapshot = md.build_snapshot("gate", "P1", "test", 10.5, 0.0, odom, scans, (10.2, 0.5, 0.0, 0.0), None,
                                     None, {}, {})
        self.assertAlmostEqual(snapshot["odom"]["age_s"], 0.5)
        self.assertTrue(snapshot["scans"]["rear"]["missing"])
        self.assertTrue(snapshot["truth_world_eval_only"]["missing"])
        self.assertEqual(sorted(snapshot["missing"]), ["scan_rear", "truth_world_eval_only"])
        self.assertFalse(snapshot["complete"])
        self.assertAlmostEqual(snapshot["map_pose"][0], 1.5)

    # 물리 걸림은 의심 표시만: odom은 움직였는데 실제는 거의 그대로
    def test_recent_motion_suspicion(self):
        odom = [(t, 0.1 * t, 0.0) for t in range(8, 11)]
        truth = md.np.array([(t, 0.0, 0.0, 0.0) for t in range(8, 11)], float)
        self.assertTrue(md.recent_motion(odom, truth, 10.0)["physical_stuck_suspected"])
        truth_moving = md.np.array([(t, 0.1 * t, 0.0, 0.0) for t in range(8, 11)], float)
        self.assertFalse(md.recent_motion(odom, truth_moving, 10.0)["physical_stuck_suspected"])


# 관문 동작 시험의 공통 준비(가짜 follower·보조 스레드·사건 조건·신호 파일). 시험 메서드는 없다.
class GateTestBase(unittest.TestCase):

    # 시험용 follower(정지 확인·입력·장애물 검사를 조절)
    def make(self, folder, gate_dir=True, stop_ok=True):
        route = md.load_route(folder / "route.yaml")
        node = FakeNode()
        follower = md.RouteFollower(node, route, FakeRun(), threading.Event(), folder / "gates" if gate_dir else None,
                                    "T", None)
        follower.limits = {"linear": 0.3, "angular": 0.3}
        follower.problems = []
        follower.obstacle = None
        follower.driven = []
        follower.stop_and_confirm = lambda reason: stop_ok
        follower.input_problems = lambda: list(follower.problems)
        follower.obstacle_problem = lambda linear, angular: follower.obstacle
        follower.drive_to = lambda index, goal: follower.driven.append(index)

        if gate_dir:
            (folder / "gates").mkdir(exist_ok=True)

        return follower, node

    def setUp(self):
        self.threads = []
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        (self.folder / "route.yaml").write_text(route_text(True))

    # 늦게 파일을 쓰는 보조 스레드가 끝난 뒤 임시 폴더를 지운다.
    def tearDown(self):
        for thread in self.threads:
            thread.join(timeout=20.0)

        self.temp.cleanup()

    # 조건이 참이 되면 동작을 한다(보조 스레드, 고정 대기 대신 관문 사건에 맞춘다).
    def when(self, condition, action, timeout=15.0):
        def work():
            deadline = time.monotonic() + timeout

            while time.monotonic() < deadline:
                if condition():
                    action()
                    return

                time.sleep(0.01)

        thread = threading.Thread(target=work, daemon=True)
        thread.start()
        self.threads.append(thread)
        return thread

    # 사건 조건: 관문 진입·재출발 거부
    @staticmethod
    def entered(follower, name):
        return lambda: any(kind == "GATE" and f"관측 관문 {name} 대기" in detail
                           for kind, detail in list(follower.run.record["events"]))

    @staticmethod
    def blocked(follower, name):
        return lambda: any(kind == "GATE" and f"관측 관문 {name} 재출발 거부" in detail
                           for kind, detail in list(follower.run.record["events"]))

    # 신호 파일 만들기
    def touch(self, *names, folder=None):
        folder = folder or self.folder

        def action():
            for name in names:
                (folder / "gates" / name).write_text("")

        return action


@unittest.skipIf(md is None, "rclpy 필요(source install/setup.bash)")
class GateBehaviour(GateTestBase):

    # 옵션 없음: 관문 표시가 있어도 기다리지 않고 기존처럼 연속 주행
    def test_no_option_same_behaviour(self):
        follower, node = self.make(self.folder, gate_dir=False)
        follower.wait_gate = lambda index: self.fail("관문 대기가 불리면 안 됨")
        follower.stop_and_confirm = lambda reason: True
        follower.follow()
        self.assertEqual(follower.driven, [0, 1, 2])

    # 정지 확인 실패면 관문에 들어가지 않고 실패(다음 경유점으로 가지 않음)
    def test_no_gate_without_stop(self):
        follower, node = self.make(self.folder, stop_ok=False)

        with self.assertRaises(RuntimeError):
            follower.follow()

        self.assertEqual(follower.driven, [0])
        self.assertTrue(any("gate_refused" in name for name in os.listdir(self.folder / "gates")))

    # 대기 중 0 속도 계속 발행, 콜백(다른 스레드의 상태 변경)이 막히지 않음, 진행 파일로 다음 구간
    def test_wait_publishes_stop_and_continues(self):
        follower, node = self.make(self.folder)
        follower.problems = ["front 스캔 만료 1.00 s"]

        # 콜백이 새 스캔을 받은 것처럼 P1 대기 중 입력 문제를 없앤 뒤 진행 파일을 만든다
        def callback_then_continue():
            follower.problems = []
            self.touch("T_P1.continue")()

        self.when(self.entered(follower, "P1"), callback_then_continue)
        self.when(self.entered(follower, "P2"), self.touch("T_P2.continue"))
        follower.follow()
        self.assertEqual(follower.driven, [0, 1, 2])
        self.assertGreater(node.stops, 0)
        names = os.listdir(self.folder / "gates")
        self.assertTrue(any(name.startswith("T_P1.continue.consumed") for name in names))
        self.assertTrue(any(name.startswith("T_P2.continue.consumed") for name in names))

    # 진행과 중단이 같이 있으면 중단 우선(관문 진입 뒤 두 파일을 함께 만든다)
    def test_abort_priority(self):
        follower, node = self.make(self.folder)
        self.when(self.entered(follower, "P1"), self.touch("T.abort", "T_P1.continue"))

        with self.assertRaises(RuntimeError) as context:
            follower.wait_gate(0)

        self.assertIn("중단", str(context.exception))

    # 관문 진입 때 이미 있던 진행 파일(stale)은 쓰지 않는다(진입 뒤 중단 파일로 끝낸다)
    def test_stale_continue_ignored(self):
        follower, node = self.make(self.folder)
        stale = self.folder / "gates" / "T_P1.continue"
        stale.write_text("")

        def abort_after_check():
            stops_before = node.stops
            deadline = time.monotonic() + 5.0

            while node.stops < stops_before + 5 and time.monotonic() < deadline:   # 대기 반복을 몇 번 본 뒤
                time.sleep(0.01)

            self.touch("T.abort")()

        self.when(self.entered(follower, "P1"), abort_after_check)

        with self.assertRaises(RuntimeError) as context:
            follower.wait_gate(0)

        self.assertIn("중단", str(context.exception))
        self.assertTrue(stale.exists())

    # 다음 명령 검사 실패면 진행 파일이 있어도 출발하지 않음(파일은 소비, 새 파일 필요)
    def test_restart_blocked(self):
        follower, node = self.make(self.folder)
        follower.obstacle = "전방 장애물 3점(정지 거리 0.40 m)"
        self.when(self.entered(follower, "P1"), self.touch("T_P1.continue"))

        def clear_and_continue():
            follower.obstacle = None
            self.touch("T_P1.continue")()

        self.when(self.blocked(follower, "P1"), clear_and_continue)
        follower.wait_gate(0)
        self.assertTrue(self.blocked(follower, "P1")())
        self.assertTrue(any("restart_blocked" in name for name in os.listdir(self.folder / "gates")))

    # 대기 중 입력이 제한 시간 넘게 회복되지 않으면 실패(보호 유지)
    def test_input_timeout_during_wait(self):
        follower, node = self.make(self.folder)
        follower.route["inputs"]["input_timeout_s"] = 0.3
        follower.problems = ["odom 만료 1.00 s"]

        with self.assertRaises(RuntimeError) as context:
            follower.wait_gate(0)

        self.assertIn("입력 미회복", str(context.exception))

    # 대기 중 정지 상실(odom 속도)이면 실패
    def test_stop_lost_during_wait(self):
        follower, node = self.make(self.folder)

        def move():
            with node.lock:
                node.odom = (0.0, 1.0, 0.0, 0.0, 0.2, 0.0)

        self.when(self.entered(follower, "P1"), move)

        with self.assertRaises(RuntimeError) as context:
            follower.wait_gate(0)

        self.assertIn("정지 상실", str(context.exception))

    # [검토 재현] 비유한 odom(stamp·자세·선/각속도)은 정지 확인 불가: 재출발 거절, 대기 중이면 실패
    def test_nonfinite_odom_blocks(self):
        nan, inf = float("nan"), float("inf")
        cases = {"속도 NaN": (10.0, 0.0, 0.0, 0.0, nan, nan), "선속도 inf": (10.0, 0.0, 0.0, 0.0, inf, 0.0),
                 "각속도 -inf": (10.0, 0.0, 0.0, 0.0, 0.0, -inf), "자세 NaN": (10.0, nan, 0.0, 0.0, 0.0, 0.0),
                 "yaw inf": (10.0, 0.0, 0.0, inf, 0.0, 0.0), "stamp NaN": (nan, 0.0, 0.0, 0.0, 0.0, 0.0)}

        for label, odom in cases.items():
            with self.subTest(case=label):
                self.assertEqual(md.gate_odom_stopped(odom)[0], False)

                # 검토자 순수 호출과 같은 형태: 입력 문제 없음·다음 경유점 있음 → 반드시 거절 사유가 있어야 한다
                follower = md.RouteFollower.__new__(md.RouteFollower)
                follower.node = FakeNode()
                follower.node.odom = odom
                follower.input_problems = lambda: []
                follower.route = {"waypoints": [(0.0, 0.0, False), (1.0, 0.0, False)], "heading_tolerance_rad": 0.12}
                follower.limits = {"linear": 0.3, "angular": 0.3}
                follower.obstacle_problem = lambda linear, angular: None
                self.assertTrue(follower.restart_blockers(0, time.monotonic()))

                # 대기 중 비유한 odom → 실패(신호로 우회 불가). 경우마다 폴더를 따로 써서 앞 경우의 늦은 파일과 섞이지 않게 한다.
                case_folder = self.folder / f"case_{len(os.listdir(self.folder))}"
                case_folder.mkdir()
                (case_folder / "route.yaml").write_text(route_text(True))
                gate_follower, node = self.make(case_folder)

                # 관문 진입 뒤 odom을 비유한 값으로 바꾸고 이어서 진행 파일을 만든다(진행 신호로 우회되면 안 됨)
                def corrupt_then_continue(node=node, odom=odom, folder=case_folder):
                    with node.lock:
                        node.odom = odom

                    (folder / "gates" / "T_P1.continue").write_text("")

                self.when(self.entered(gate_follower, "P1"), corrupt_then_continue)

                with self.assertRaises(RuntimeError) as context:
                    gate_follower.wait_gate(0)

                self.assertIn("확인 불가", str(context.exception))

        self.assertEqual(md.gate_odom_stopped((10.0, 0.0, 0.0, 0.0, 0.0, 0.0)), (True, None))
        self.assertFalse(md.gate_odom_stopped(None)[0])

    # sim time이 멈춰 있으면(일시정지) 진행 파일이 있어도 출발하지 않음.
    # 진입 뒤 시계 정체 기준(GATE_CLOCK_FRESH_WALL_S)을 넘긴 다음 진행 파일 → 재출발 거부 사건 → 중단 파일 순서로 맞춘다.
    def test_frozen_clock_blocks_restart(self):
        follower, node = self.make(self.folder)
        node.frozen = 50.0
        entered_at = []

        def note_entry():
            entered_at.append(time.monotonic())

        self.when(self.entered(follower, "P1"), note_entry)
        self.when(lambda: entered_at and time.monotonic() - entered_at[0] > md.GATE_CLOCK_FRESH_WALL_S + 0.2,
                  self.touch("T_P1.continue"))
        self.when(self.blocked(follower, "P1"), self.touch("T.abort"))

        with self.assertRaises(RuntimeError):
            follower.wait_gate(0)

        details = [detail for kind, detail in follower.run.record["events"] if kind == "GATE"]
        self.assertTrue(any("sim time 정체" in detail for detail in details), details)


# 정렬 경유점(align)이 있는 경로: 첫 경유점이 관문 P1이고 기준선은 y=0, +x 방향
def aligned_route_text(align="{heading_rad: 0.0, max_lateral_m: 0.04, max_heading_rad: 0.02}"):
    return route_text(True).replace("  - {xy: [1.0, 0.0], gate: true}",
                                    f"  - {{xy: [1.0, 0.0], gate: true, align: {align}}}")


NEW_ROUTE = ROOT / "src/ax_localization/config/routes/ax_warehouse_center.yaml"
AWS_ROUTE = ROOT / "src/ax_localization/config/routes/aws_warehouse.yaml"


@unittest.skipIf(md is None, "rclpy 필요(source install/setup.bash)")
class AlignmentAndWorld(unittest.TestCase):

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    # align 해석: 세 값만, 유한, 허용값 양수. 정렬 없는 경유점은 기록하지 않는다
    def test_parse_alignment(self):
        path = self.folder / "route.yaml"
        path.write_text(aligned_route_text())
        route = md.load_route(path)
        self.assertEqual(route["alignment"], {0: {"heading_rad": 0.0, "max_lateral_m": 0.04, "max_heading_rad": 0.02}})

        for bad in ("{heading_rad: 0.0, max_lateral_m: 0.04}", "{heading_rad: .nan, max_lateral_m: 0.04, max_heading_rad: 0.02}",
                    "{heading_rad: 0.0, max_lateral_m: -0.04, max_heading_rad: 0.02}", "0.04",
                    "{heading_rad: 0.0, max_lateral_m: 0.04, max_heading_rad: 0.0}"):
            with self.subTest(align=bad):
                path.write_text(aligned_route_text(bad))

                with self.assertRaises(ValueError):
                    md.load_route(path)

    # 정렬 오차: 측면(왼쪽 +)·방향(감김 정리)·비유한 자세 거부
    def test_alignment_error(self):
        align = {"heading_rad": 0.0, "max_lateral_m": 0.04, "max_heading_rad": 0.02}
        ok = md.alignment_error((0.9, 0.03, 0.01), (1.0, 0.0), align)
        self.assertTrue(ok["within"])
        self.assertAlmostEqual(ok["lateral_m"], 0.03)
        self.assertAlmostEqual(ok["along_m"], -0.1)
        self.assertFalse(md.alignment_error((1.0, -0.05, 0.0), (1.0, 0.0), align)["within"])
        self.assertFalse(md.alignment_error((1.0, 0.0, 0.03), (1.0, 0.0), align)["within"])
        wrapped = md.alignment_error((1.0, 0.0, 2 * md.math.pi + 0.01), (1.0, 0.0), align)
        self.assertTrue(wrapped["within"])
        self.assertFalse(md.alignment_error((float("nan"), 0.0, 0.0), (1.0, 0.0), align)["within"])
        north = {"heading_rad": md.math.pi / 2, "max_lateral_m": 0.04, "max_heading_rad": 0.02}
        self.assertAlmostEqual(md.alignment_error((0.97, 1.0, md.math.pi / 2), (1.0, 1.0), north)["lateral_m"], 0.03)

    # 월드·경로 불일치: AWS 경로, 다른 world, gz_world, robot_id, frame 접두사
    def test_route_world_problems(self):
        new = md.load_route(NEW_ROUTE)
        self.assertEqual(md.route_world_problems(new, "planned/ax_warehouse.world", "ax_test", "robot_2"), [])
        self.assertTrue(md.route_world_problems(new, "planned/other.world", "ax_test", "robot_2"))
        self.assertTrue(md.route_world_problems(new, "planned/ax_warehouse.world", "aws", "robot_2"))
        self.assertTrue(md.route_world_problems(new, "planned/ax_warehouse.world", "ax_test", "robot_1"))
        aws = md.load_route(AWS_ROUTE)
        self.assertTrue(any("AWS" in item for item in md.route_world_problems(aws)))
        self.assertTrue(len(md.route_world_problems(aws, "planned/ax_warehouse.world")) >= 2)
        broken = dict(new, frame="robot_1/odom")
        self.assertTrue(md.route_world_problems(broken))

    # 새 중앙 통로 경로: 외곽은 A1 기준과 같고, 두 관문은 정지 허용을 넣어도 통로(x -4.2~4.2 world) 밖이며,
    # 경유점은 통로 중심선(y=0) 위라 통로 안 회전이 필요 없다. 정렬은 서쪽 관문에만 있다
    def test_new_route_geometry(self):
        route = md.load_route(NEW_ROUTE)
        a1 = yaml.safe_load((ROOT / "src/ax_description/config/a1_validation.yaml").read_text())
        corners = a1["mobility"]["robot_2"]["footprint_xy_m"]
        self.assertEqual(route["footprint"], {"front_m": max(c[0] for c in corners), "rear_m": -min(c[0] for c in corners),
                                              "half_width_m": max(c[1] for c in corners)})
        self.assertEqual(route["safety"], {"processing_latency_s": 0.3, "margin_m": 0.25, "rotation_margin_m": 0.15})
        self.assertEqual(route["gates"], {0: "P1", 1: "P2"})
        self.assertEqual(set(route["alignment"]), {0})
        start_x = route["start_world_reference"][0]
        tolerance, fp = route["goal_tolerance_m"], route["footprint"]
        west, east = (start_x + route["waypoints"][0][0], start_x + route["waypoints"][1][0])
        self.assertLessEqual(west + tolerance + fp["front_m"], -4.2)
        self.assertGreaterEqual(east - tolerance - fp["rear_m"], 4.2)
        self.assertTrue(all(point[1] == 0.0 and not point[2] for point in route["waypoints"]))
        rotation = md.math.hypot(max(fp["front_m"], fp["rear_m"]), fp["half_width_m"]) + route["safety"]["rotation_margin_m"]
        self.assertGreater(rotation, 0.9)   # 통로 반폭보다 커서 통로 안 회전 불가 → 경로에 넣지 않는다

    # --check-only: 일치하면 0과 요약, 불일치·AWS 경로는 2(ROS 연결 없음)
    def test_check_only_cli(self):
        base = [sys.executable, str(SCRIPT), "--check-only", "--gz-world", "ax_test", "--robot-id", "robot_2"]
        ok = subprocess.run(base + ["--route", str(NEW_ROUTE), "--world", "planned/ax_warehouse.world"],
                            capture_output=True, text=True, timeout=60)
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertEqual(json.loads(ok.stdout)["gates"], {"1": "P1", "2": "P2"})

        for args in (["--route", str(AWS_ROUTE), "--world", "planned/ax_warehouse.world"],
                     ["--route", str(NEW_ROUTE), "--world", "planned/other.world"],
                     ["--route", str(self.folder / "missing.yaml"), "--world", "planned/ax_warehouse.world"]):
            with self.subTest(args=args):
                result = subprocess.run(base + args, capture_output=True, text=True, timeout=60)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)

        result = subprocess.run([sys.executable, str(SCRIPT), "--check-only", "--route", str(NEW_ROUTE),
                                 "--world", "planned/ax_warehouse.world", "--robot-id", "robot_1"],
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 2)


@unittest.skipIf(md is None, "rclpy 필요(source install/setup.bash)")
class AlignmentBehaviour(GateTestBase):
    # 관문 공통 준비를 쓰되 정렬 경로로 바꾼다.
    def setUp(self):
        super().setUp()
        (self.folder / "route.yaml").write_text(aligned_route_text())

    # 관문 재출발: 정렬 허용 밖이면 진행 파일이 있어도 거부, 정렬 회복 뒤 새 진행 파일로 출발. 기록이 남는다
    def test_align_restart_blocked_until_aligned(self):
        follower, node = self.make(self.folder)
        node.odom = (0.0, 1.0, 0.06, 0.0, 0.0, 0.0)
        self.when(self.entered(follower, "P1"), self.touch("T_P1.continue"))

        def realign_and_continue():
            with node.lock:
                node.odom = (0.0, 1.0, 0.01, 0.005, 0.0, 0.0)

            self.touch("T_P1.continue")()

        self.when(self.blocked(follower, "P1"), realign_and_continue)
        follower.wait_gate(0)
        details = [detail for kind, detail in follower.run.record["events"] if kind == "GATE"]
        self.assertTrue(any("정렬 허용 초과" in detail for detail in details), details)
        stages = [(item["stage"], item["within"]) for item in follower.run.record["alignment"]]
        self.assertEqual(stages[0], ("관문 도달", False))
        self.assertIn(("재출발 전", True), stages)

    # 비유한 odom이면 정렬도 확인 불가(재출발 사유에 정지 확인 불가가 들어가고 다음 명령 검사 전에 끝난다)
    def test_align_nonfinite_odom(self):
        follower, node = self.make(self.folder)
        node.odom = (0.0, float("nan"), 0.0, 0.0, 0.0, 0.0)
        blockers = follower.restart_blockers(0, time.monotonic())
        self.assertTrue(any("비유한" in item for item in blockers), blockers)

    # 관문 폴더 없이 실행해도 정렬 경유점에서는 멈춰 정렬을 확인하고, 허용 밖이면 다음 구간으로 가지 않는다
    def test_align_without_gate_dir(self):
        follower, node = self.make(self.folder, gate_dir=False)
        node.odom = (0.0, 1.0, 0.0, 0.05, 0.0, 0.0)

        with self.assertRaises(RuntimeError) as context:
            follower.follow()

        self.assertIn("출발 거부", str(context.exception))
        self.assertEqual(follower.driven, [0])

        node.odom = (0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
        follower.driven = []
        follower.follow()
        self.assertEqual(follower.driven, [0, 1, 2])

    # 정렬 경유점에서 정지 확인이 안 되면 정렬 검사 없이 실패
    def test_align_without_gate_dir_needs_stop(self):
        follower, node = self.make(self.folder, gate_dir=False, stop_ok=False)

        with self.assertRaises(RuntimeError) as context:
            follower.follow()

        self.assertIn("정지를 확인하지 못해", str(context.exception))
        self.assertNotIn("alignment", follower.run.record)


if __name__ == "__main__":
    unittest.main()
