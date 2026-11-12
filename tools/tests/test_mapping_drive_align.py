# B2 지도 작성 주행의 회전 정렬 선택 모드(route rotation_align) 시험. ROS 그래프·시뮬레이터 없이
# 가짜 기구학 노드(가감속 제한 직선 0.67 m/s², 회전 0.2 rad/s²)로 상태 전환을 검사한다.
# 근거: log/b2_custom_warehouse/20261111_112945/c_align(C 조건 시험).
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1].parent
SCRIPTS = ROOT / "src/ax_localization/scripts"
sys.path.insert(0, str(SCRIPTS))


def load(name):
    if importlib.util.find_spec("rclpy") is None and name == "mapping_drive":
        return None

    spec = importlib.util.spec_from_file_location(name + "_under_test", SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


md = load("mapping_drive")
ra = load("rotation_align") if (SCRIPTS / "rotation_align.py").is_file() else None

LAW = {"gain_per_s": 0.5, "omega_min_rad_s": 0.005, "stop_band_rad": 0.002, "final_tolerance_rad": 0.002,
       "max_realign": 3, "limit_sim_s": 60.0, "limit_wall_s": 300.0}


def route_text(law=LAW, waypoints="  - [0.0, 1.0]\n"):
    text = ("robot_id: robot_2\nframe: robot_2/odom\nlimits: {linear_m_s: 0.3, angular_rad_s: 0.3}\n"
            "goal_tolerance_m: 0.15\nheading_tolerance_rad: 0.12\n"
            "footprint: {front_m: 0.67, rear_m: 0.63, half_width_m: 0.53}\n"
            "safety: {processing_latency_s: 0.3, margin_m: 0.25, rotation_margin_m: 0.15}\n"
            "inputs: {scan_max_age_s: 0.5, odom_max_age_s: 0.3, tf_receive_max_age_boottime_s: 4.0, "
            "slam_progress_distance_m: 1.0, slam_progress_heading_rad: 1.0, input_timeout_s: 3.0}\n"
            "progress: {time_factor: 3.0, time_margin_s: 30.0, stall_window_s: 15.0, stall_min_improvement: 0.05}\n")

    if law is not None:
        text += "rotation_align: " + json.dumps(law) + "\n"

    return text + "waypoints:\n  - [0.0, 0.0]\n" + waypoints


class PlantNode:
    """가짜 노드+기구학. publish·publish_stop마다 dt만큼 진행한다. kick: 정지 확인 성공 뒤 방향 변화,
    drift: 직진 중 m당 odom 방향 변화, scale: 회전 효율, v0: 처음 남은 전진 속도."""

    def __init__(self, v0=0.0, kick=0.0, drift=0.0, scale=1.0, yaw=0.0, dt=0.02):
        self.lock = threading.Lock()
        self.t, self.x, self.y, self.yaw, self.v, self.w = 10.0, 0.0, 0.0, yaw, v0, 0.0
        self.kick, self.drift, self.scale, self.dt = kick, drift, scale, dt
        self.commands = []
        self.first_rotation_v = None
        self.scans = {"front": None, "rear": None}
        self.map_record, self.odom_record = [], []
        self.time_offset = 0.0
        self.kicked_stops = 0
        self.missing = False

    @property
    def odom(self):
        if self.missing:
            return None

        return (self.t, self.x, self.y, self.yaw, self.v, self.w)

    @odom.setter
    def odom(self, value):
        self.t, self.x, self.y, self.yaw, self.v, self.w = value

    def now_seconds(self):
        return self.t + self.time_offset

    def step(self, cv, cw):
        self.v += max(-0.67 * self.dt, min(0.67 * self.dt, cv - self.v))
        self.w += max(-0.2 * self.dt, min(0.2 * self.dt, cw - self.w))
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.w * self.dt * self.scale + self.drift * abs(self.v) * self.dt
        self.t += self.dt

    def publish(self, linear, angular):
        self.commands.append((self.t, linear, angular))

        if angular != 0.0 and self.first_rotation_v is None:
            self.first_rotation_v = abs(self.v)

        self.step(linear, angular)

    def publish_stop(self):
        was_moving = abs(self.v) > 1e-9 or abs(self.w) > 1e-9
        self.step(0.0, 0.0)

        if was_moving and abs(self.v) < 1e-9 and abs(self.w) < 1e-9 and self.kick:
            self.yaw += self.kick
            self.kicked_stops += 1

    def other_publishers(self):
        return []

    def slam_health(self):
        return {}


class FakeRun:
    def __init__(self):
        self.record = {"events": [], "stops": []}

    def event(self, sim_time, kind, detail):
        self.record["events"].append((kind, detail))

    def save(self):
        pass


@unittest.skipIf(ra is None, "rotation_align.py 필요")
class RotationAlignModule(unittest.TestCase):

    # 경로 항목 검사: 일곱 키, 유한 양수, 대역 ≤ 허용, 정수 재시도, 실행 상한 비교
    def test_parse(self):
        self.assertEqual(ra.parse(dict(LAW))["max_realign"], 3)
        bad = [dict(LAW, gain_per_s=-1), dict(LAW, limit_sim_s=float("nan")), dict(LAW, stop_band_rad=0.003),
               dict(LAW, max_realign=1.5), dict(LAW, max_realign=True), {k: v for k, v in LAW.items() if k != "gain_per_s"},
               dict(LAW, extra=1), dict(LAW, omega_min_rad_s="0.1")]

        for item in bad:
            with self.subTest(item=item), self.assertRaises(ValueError):
                ra.parse(item)

        with self.assertRaises(ValueError):
            ra.parse(dict(LAW, omega_min_rad_s=0.4), omega_max=0.3)

    # 각속도 명령은 실행 상한을 넘지 않는다(상한 0.2이면 0.2)
    def test_command_uses_execution_limit(self):
        self.assertAlmostEqual(ra.omega_command(2.0, LAW, 0.2), 0.2)
        self.assertAlmostEqual(ra.omega_command(-0.1, LAW, 0.3), -0.05)
        self.assertAlmostEqual(ra.omega_command(0.003, LAW, 0.3), 0.005)
        self.assertEqual(ra.omega_command(0.0019, LAW, 0.3), 0.0)


@unittest.skipIf(md is None, "rclpy 필요(source install/setup.bash)")
class RotationAlignMode(unittest.TestCase):

    def setUp(self):
        self.saved = {name: getattr(md, name) for name in ("CONTROL_PERIOD_S", "ODOM_FIRST_WAIT_S")}
        md.CONTROL_PERIOD_S = 0.0
        md.ODOM_FIRST_WAIT_S = 0.2
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)

    def tearDown(self):
        for name, value in self.saved.items():
            setattr(md, name, value)

        self.temp.cleanup()

    def make(self, node, law=LAW, waypoints="  - [0.0, 1.0]\n"):
        path = self.folder / "route.yaml"
        path.write_text(route_text(law, waypoints))
        route = md.load_route(path)
        follower = md.RouteFollower(node, route, FakeRun(), threading.Event())
        follower.limits = {"linear": 0.3, "angular": 0.3}
        follower.deceleration = {"linear": 0.67, "angular": 0.2}
        follower.input_problems = lambda: []
        follower.obstacle_calls = []

        def obstacle(linear, angular):
            follower.obstacle_calls.append((linear, angular))
            return None

        follower.obstacle_problem = obstacle
        return follower

    # [재현] 경로 키 해석: 있으면 검사된 dict, 없으면 None. 잘못된 값은 ValueError
    def test_load_route_key(self):
        path = self.folder / "r.yaml"
        path.write_text(route_text())
        self.assertEqual(md.load_route(path)["rotation_align"], LAW)
        path.write_text(route_text(None))
        self.assertIsNone(md.load_route(path)["rotation_align"])
        path.write_text(route_text(dict(LAW, stop_band_rad=0.01)))

        with self.assertRaises(ValueError):
            md.load_route(path)

    # [재현] --check-only도 같은 검사 경로: 잘못된 rotation_align은 2, 정상은 출력에 포함
    def test_check_only(self):
        path = self.folder / "r.yaml"
        path.write_text(route_text())
        ok = subprocess.run([sys.executable, str(SCRIPTS / "mapping_drive.py"), "--check-only", "--route", str(path)],
                            capture_output=True, text=True, timeout=60)
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertEqual(json.loads(ok.stdout)["rotation_align"], LAW)
        path.write_text(route_text(dict(LAW, max_realign=-1)))
        bad = subprocess.run([sys.executable, str(SCRIPTS / "mapping_drive.py"), "--check-only", "--route", str(path)],
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(bad.returncode, 2)

    # 키가 없으면 기존 waypoint_command 경로(혼합 명령 허용)를 그대로 쓴다
    def test_mode_absent_keeps_existing(self):
        node = PlantNode()
        follower = self.make(node, law=None)
        calls = []
        original = md.waypoint_command

        def spy(*args, **kwargs):
            calls.append(1)
            return original(*args, **kwargs)

        md.waypoint_command = spy

        try:
            follower.drive_to(1, follower.route["waypoints"][1])
        finally:
            md.waypoint_command = original

        self.assertTrue(calls)
        self.assertNotIn("rotation_align", follower.run.record)

    # [재현] 모드에서는 모든 명령이 회전(v=0) 또는 직진(ω=0)이고, 정지 뒤 0.002 안일 때만 직진한다
    def test_commands_separated_and_aligned(self):
        node = PlantNode()
        follower = self.make(node)
        follower.drive_to(1, follower.route["waypoints"][1])
        self.assertTrue(all(v == 0.0 or w == 0.0 for _, v, w in node.commands))
        self.assertTrue(any(v > 0 for _, v, _ in node.commands))
        entries = follower.run.record["rotation_align"]
        self.assertEqual(entries[0]["trigger"], "start")
        self.assertTrue(entries[0]["aligned"])
        self.assertLessEqual(abs(entries[0]["final_error_rad"]), 0.002)
        first_drive = next(t for t, v, _ in node.commands if v > 0)
        self.assertGreater(first_drive, entries[0]["passes"][-1]["sim_t"] - 1e-9)
        self.assertLessEqual(max(abs(w) for _, _, w in node.commands), 0.3 + 1e-12)

    # [재현] 감속 중 전진 속도가 남아 있으면 정지 확인 전에는 회전 명령을 내지 않는다
    def test_no_rotation_while_moving(self):
        node = PlantNode(v0=0.2)
        follower = self.make(node)
        follower.drive_to(1, follower.route["waypoints"][1])
        self.assertIsNotNone(node.first_rotation_v)
        self.assertLess(node.first_rotation_v, md.STOP_LINEAR_M_S)

    # [재현] 정지 확인 실패는 정렬 실패이며 직진하지 않는다
    def test_stop_failure(self):
        node = PlantNode()
        follower = self.make(node)
        follower.stop_and_confirm = lambda reason: False

        with self.assertRaises(RuntimeError):
            follower.drive_to(1, follower.route["waypoints"][1])

        self.assertFalse(any(v > 0 for _, v, _ in node.commands))

    # [재현] 정지 뒤 허용 밖이면 재정렬(1회 뒤 성공), 매번 밀리면 3회 뒤 실패하고 직진하지 않음
    def test_realign_after_stop(self):
        node = PlantNode(kick=0.004)
        follower = self.make(node)
        original = node.publish_stop
        state = {"kicks": 0}

        def once():
            before = node.kicked_stops
            original()
            if node.kicked_stops > before:
                state["kicks"] += 1
                if state["kicks"] >= 1:
                    node.kick = 0.0

        node.publish_stop = once
        follower.drive_to(1, follower.route["waypoints"][1])
        entry = follower.run.record["rotation_align"][0]
        self.assertTrue(entry["aligned"])
        self.assertGreaterEqual(entry["realigns"], 1)

        node2 = PlantNode(kick=0.004)
        follower2 = self.make(node2)

        with self.assertRaises(RuntimeError) as ctx:
            follower2.drive_to(1, follower2.route["waypoints"][1])

        self.assertIn("재정렬", str(ctx.exception))
        self.assertFalse(any(v > 0 for _, v, _ in node2.commands))
        self.assertFalse(follower2.run.record["rotation_align"][-1]["aligned"])

    # [재현] sim·벽시계 제한
    def test_time_limits(self):
        node = PlantNode(scale=0.001)
        follower = self.make(node, law=dict(LAW, limit_sim_s=5.0))

        with self.assertRaises(RuntimeError) as ctx:
            follower.drive_to(1, follower.route["waypoints"][1])

        self.assertIn("sim 시간", str(ctx.exception))
        node = PlantNode()
        follower = self.make(node, law=dict(LAW, limit_wall_s=0.0001))   # YAML은 1e-9 같은 지수 표기를 문자열로 읽는다

        with self.assertRaises(RuntimeError) as ctx:
            follower.drive_to(1, follower.route["waypoints"][1])

        self.assertIn("벽시계", str(ctx.exception))

    # [재현] 직진 중 방향 오차가 heading_tolerance를 넘으면 정지 뒤 같은 정렬(trigger heading_drift).
    # 경유점마다 정렬 절차는 1 + max_realign회까지이며 넘으면 실패(무한 반복 없음)
    def test_straight_drift_realign_and_cap(self):
        node = PlantNode(drift=0.05)
        follower = self.make(node, waypoints="  - [0.0, 4.0]\n")
        follower.drive_to(1, follower.route["waypoints"][1])
        triggers = [e["trigger"] for e in follower.run.record["rotation_align"]]
        self.assertEqual(triggers[0], "start")
        self.assertIn("heading_drift", triggers)
        self.assertTrue(all(v == 0.0 or w == 0.0 for _, v, w in node.commands))

        node = PlantNode(drift=1.0)
        follower = self.make(node, waypoints="  - [0.0, 8.0]\n")

        with self.assertRaises(RuntimeError) as ctx:
            follower.drive_to(1, follower.route["waypoints"][1])

        self.assertIn("정렬 절차", str(ctx.exception))
        self.assertLessEqual(len(follower.run.record["rotation_align"]), 1 + LAW["max_realign"])

    # [재현] 비유한 odom·sim 시계 역행·중단 신호
    def test_nonfinite_clock_interrupt(self):
        node = PlantNode()
        follower = self.make(node)
        node.yaw = float("nan")

        with self.assertRaises(RuntimeError):
            follower.drive_to(1, follower.route["waypoints"][1])

        node = PlantNode()
        follower = self.make(node)
        original = node.publish
        count = {"n": 0}

        def rewind(linear, angular):
            count["n"] += 1
            if count["n"] == 20:
                node.time_offset -= 5.0
            original(linear, angular)

        node.publish = rewind

        with self.assertRaises(RuntimeError) as ctx:
            follower.drive_to(1, follower.route["waypoints"][1])

        self.assertIn("역행", str(ctx.exception))
        node = PlantNode()
        follower = self.make(node)
        follower.stop_flag.set()

        with self.assertRaises(KeyboardInterrupt):
            follower.drive_to(1, follower.route["waypoints"][1])

    # [재현] 회전 단계 보호: 회전 명령마다 회전 원 검사(obstacle_problem(0, ω))를 거친다
    def test_rotation_obstacle_checked(self):
        node = PlantNode()
        follower = self.make(node)
        follower.drive_to(1, follower.route["waypoints"][1])
        self.assertTrue(any(v == 0.0 and w != 0.0 for v, w in follower.obstacle_calls))
        self.assertTrue(any(v > 0.0 and w == 0.0 for v, w in follower.obstacle_calls))

    # [재현] 관문 재출발 검사의 다음 명령: 모드에서는 방향 오차가 크면 회전(v=0) 명령으로 검사한다
    def test_restart_first_command(self):
        node = PlantNode()
        follower = self.make(node)
        linear, angular = follower.first_command((0.0, 0.0, 0.0), (0.0, 1.0, False))
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)
        linear, angular = follower.first_command((0.0, 0.0, math.pi / 2), (0.0, 1.0, False))
        self.assertGreater(linear, 0.0)
        self.assertEqual(angular, 0.0)



# 정렬 경유점 1(odom (1, 0), heading 0, 측면 0.04 m·방향 0.02 rad) 뒤에 다음 구간이 있는 경로
ALIGN_WAYPOINTS = ("  - xy: [1.0, 0.0]\n    align: {heading_rad: 0.0, max_lateral_m: 0.04, max_heading_rad: 0.02}\n"
                   "  - [3.0, 0.0]\n")
GATE_WAYPOINTS = ("  - xy: [1.0, 0.0]\n    gate: true\n"
                  "    align: {heading_rad: 0.0, max_lateral_m: 0.04, max_heading_rad: 0.02}\n  - [3.0, 0.0]\n")
# 정적 재평가(20261111_120933 rotation_align_mode)의 P1 도착 자세: 측면 +0.0207 m, 방향 −0.10766 rad
P1_POSE = (0.92, 0.0207, -0.10766)


@unittest.skipIf(md is None, "rclpy 필요(source install/setup.bash)")
class GateFinalAlign(unittest.TestCase):
    """회전 정렬 모드의 정렬 경유점 최종 방향 정렬(도착 → 정지 확인 → 입력·측면 검사 → heading_rad 회전 →
    정지 뒤 방향·측면 재검사 → 관문/출발 검사). 경유점 주행은 도착 자세를 놓는 대역으로 바꾼다."""

    setUp = RotationAlignMode.setUp
    tearDown = RotationAlignMode.tearDown
    make = RotationAlignMode.make

    # 도착 자세를 놓고 follow()를 실행할 준비. gate=True면 관문 폴더를 주고 관문 대기를 재출발 검사 기록으로 바꾼다.
    def prepare(self, node, pose, law=LAW, gate=False, waypoints=None):
        follower = self.make(node, law=law, waypoints=waypoints or (GATE_WAYPOINTS if gate else ALIGN_WAYPOINTS))

        def drive(index, goal):
            if index == 1:
                node.x, node.y, node.yaw = pose
                node.v = node.w = 0.0

        follower.drive_to = drive
        follower.entered = []

        if gate:
            follower.gate_dir = self.folder
            follower.gate_token = "t"

            def wait_gate(index):
                follower.entered.append((index, follower.restart_blockers(index, md.time.monotonic())))

            follower.wait_gate = wait_gate

        return follower

    def final(self, follower):
        return follower.run.record.get("final_align", [])

    # [재현] P1 자세: 관문 없이 실행하면 방향 −0.10766 rad로 출발 거부였다 → 최종 회전 뒤 통과
    def test_p1_reproduction_without_gate(self):
        node = PlantNode()
        follower = self.prepare(node, P1_POSE)
        follower.follow()
        entry = self.final(follower)[0]
        self.assertEqual(entry["outcome"], "aligned")
        self.assertAlmostEqual(entry["before"]["heading_error_rad"], -0.10766, places=5)
        self.assertLessEqual(abs(entry["after"]["heading_error_rad"]), LAW["final_tolerance_rad"])
        self.assertLessEqual(abs(entry["after"]["lateral_m"]), 0.04)
        self.assertTrue(all(v == 0.0 and w > 0.0 for _, v, w in node.commands))
        stages = [item["stage"] for item in follower.run.record["alignment"]]
        self.assertEqual(stages[-1], "관문 없이 출발 전")
        self.assertTrue(follower.run.record["alignment"][-1]["within"])

    # [재현] P1 자세: 관문 폴더가 있으면 최종 정렬 뒤 관문에 들어가고 재출발 검사 사유가 없다
    def test_p1_reproduction_with_gate(self):
        node = PlantNode()
        follower = self.prepare(node, P1_POSE, gate=True)
        follower.follow()
        self.assertEqual(self.final(follower)[0]["outcome"], "aligned")
        self.assertEqual(self.final(follower)[0]["gate"], "P1")
        self.assertEqual(follower.entered, [(1, [])])

    # 측면 초과(방향 정렬 여부와 무관): 회전·직진 없이 기존 거부(관문 없으면 출발 거부, 관문이면 재출발 거부)
    def test_lateral_excess_no_motion(self):
        for heading in (-0.10766, 0.0):
            with self.subTest(heading=heading):
                node = PlantNode()
                follower = self.prepare(node, (0.92, 0.06, heading))

                with self.assertRaises(RuntimeError) as ctx:
                    follower.follow()

                self.assertIn("출발 거부", str(ctx.exception))
                self.assertEqual(node.commands, [])
                entry = self.final(follower)[0]
                self.assertEqual(entry["outcome"], "refused")
                self.assertIn("측면", entry["reason"])

                node = PlantNode()
                follower = self.prepare(node, (0.92, 0.06, heading), gate=True)
                follower.follow()
                self.assertEqual(node.commands, [])
                index, blockers = follower.entered[0]
                self.assertTrue(any("측면 +0.060" in item for item in blockers), blockers)

    # 회전 중 odom 측면이 변해 허용을 넘으면 다시 돌리지 않고 거부(직진 없음)
    def test_lateral_after_rotation(self):
        node = PlantNode()
        follower = self.prepare(node, (0.92, 0.035, -0.10766))
        original = node.publish

        def slip(linear, angular):
            original(linear, angular)
            node.y += 0.0005 if angular else 0.0

        node.publish = slip

        with self.assertRaises(RuntimeError) as ctx:
            follower.follow()

        self.assertIn("출발 거부", str(ctx.exception))
        entry = self.final(follower)[0]
        self.assertEqual(entry["outcome"], "refused")
        self.assertIn("회전 뒤", entry["reason"])
        self.assertGreater(abs(entry["after"]["lateral_m"]), 0.04)
        self.assertFalse(any(v != 0.0 for _, v, _ in node.commands))
        self.assertEqual(len(follower.run.record["rotation_align"]), 1)

    # 정지 확인 실패·odom 미수신은 회전하지 않고 실패
    def test_stop_and_missing_odom(self):
        node = PlantNode()
        follower = self.prepare(node, P1_POSE)
        follower.stop_and_confirm = lambda reason: False

        with self.assertRaises(RuntimeError) as ctx:
            follower.follow()

        self.assertIn("정지", str(ctx.exception))
        self.assertEqual(node.commands, [])
        self.assertEqual(self.final(follower)[0]["outcome"], "failed")

        node = PlantNode()
        follower = self.prepare(node, P1_POSE)
        original = follower.drive_to

        def lose(index, goal):
            original(index, goal)
            node.missing = index == 1

        follower.drive_to = lose

        with self.assertRaises(RuntimeError):
            follower.follow()

        self.assertEqual(node.commands, [])

    # 비유한 odom·입력 문제는 회전하지 않고 기존 거부로 넘긴다
    def test_nonfinite_and_input_refused(self):
        node = PlantNode()
        follower = self.prepare(node, (0.92, 0.0207, float("nan")))

        with self.assertRaises(RuntimeError):
            follower.follow()

        self.assertEqual(node.commands, [])
        self.assertEqual(self.final(follower)[0]["outcome"], "refused")

        node = PlantNode()
        follower = self.prepare(node, P1_POSE, gate=True)
        follower.input_problems = lambda: ["front 스캔 만료 0.90 s"]
        follower.follow()
        self.assertEqual(node.commands, [])
        self.assertEqual(self.final(follower)[0]["outcome"], "refused")
        self.assertTrue(follower.entered[0][1])

    # 회전 원 장애물·sim 시계 역행·다른 발행자·중단 신호는 회전 정렬 실패로 끝나고 직진하지 않는다
    def test_rotation_guards(self):
        md_wait = md.OBSTACLE_WAIT_S
        md.OBSTACLE_WAIT_S = 1.0

        try:
            node = PlantNode()
            follower = self.prepare(node, P1_POSE)
            follower.obstacle_problem = lambda linear, angular: "회전 반경 안 장애물 3점" if angular else None

            with self.assertRaises(RuntimeError) as ctx:
                follower.follow()

            self.assertIn("장애물", str(ctx.exception))
            self.assertEqual(node.commands, [])
            self.assertEqual(self.final(follower)[0]["outcome"], "failed")
        finally:
            md.OBSTACLE_WAIT_S = md_wait

        node = PlantNode()
        follower = self.prepare(node, P1_POSE)
        original = node.publish
        count = {"n": 0}

        def rewind(linear, angular):
            count["n"] += 1
            node.time_offset -= 5.0 if count["n"] == 10 else 0.0
            original(linear, angular)

        node.publish = rewind

        with self.assertRaises(RuntimeError) as ctx:
            follower.follow()

        self.assertIn("역행", str(ctx.exception))
        self.assertFalse(any(v != 0.0 for _, v, _ in node.commands))

        node = PlantNode()
        follower = self.prepare(node, P1_POSE)
        node.other_publishers = lambda: ["/teleop"]

        with self.assertRaises(RuntimeError) as ctx:
            follower.follow()

        self.assertIn("다른 cmd_vel", str(ctx.exception))
        self.assertEqual(node.commands, [])

        node = PlantNode()
        follower = self.prepare(node, P1_POSE)
        follower.stop_flag.set()

        with self.assertRaises(KeyboardInterrupt):
            follower.follow()

        self.assertEqual(node.commands, [])

    # 최종 정렬 절차는 경유점마다 1회이고 내부 재정렬 상한(max_realign)·시간 제한을 그대로 쓴다(초기화 반복 없음)
    def test_bounded_single_procedure(self):
        node = PlantNode(kick=0.004)
        follower = self.prepare(node, P1_POSE)

        with self.assertRaises(RuntimeError) as ctx:
            follower.follow()

        self.assertIn("재정렬", str(ctx.exception))
        self.assertEqual(len(follower.run.record["rotation_align"]), 1)
        self.assertFalse(follower.run.record["rotation_align"][0]["aligned"])
        self.assertEqual(self.final(follower)[0]["outcome"], "failed")

        node = PlantNode(scale=0.001)
        follower = self.prepare(node, P1_POSE, law=dict(LAW, limit_sim_s=5.0))

        with self.assertRaises(RuntimeError) as ctx:
            follower.follow()

        self.assertIn("sim 시간", str(ctx.exception))
        self.assertEqual(len(follower.run.record["rotation_align"]), 1)

    # 이미 최종 허용 안이면 회전하지 않고 통과(already_within)
    def test_already_within(self):
        node = PlantNode()
        follower = self.prepare(node, (0.92, 0.01, 0.001))
        follower.follow()
        self.assertEqual(node.commands, [])
        self.assertEqual(self.final(follower)[0]["outcome"], "already_within")

    # 방향 감김: 목표 π, yaw −π+0.05이면 짧은 쪽(−)으로 0.05 rad만 돈다
    def test_heading_wrap(self):
        node = PlantNode()
        waypoints = ("  - xy: [1.0, 0.0]\n    align: {heading_rad: 3.141592653589793, max_lateral_m: 0.04, "
                     "max_heading_rad: 0.02}\n  - [-1.0, 0.0]\n")
        follower = self.prepare(node, (1.0, 0.0, -math.pi + 0.05), waypoints=waypoints)
        follower.follow()
        entry = self.final(follower)[0]
        self.assertEqual(entry["outcome"], "aligned")
        self.assertAlmostEqual(entry["before"]["heading_error_rad"], 0.05, places=6)
        self.assertTrue(all(w < 0.0 for _, _, w in node.commands))
        self.assertLess(sum(abs(w) for _, _, w in node.commands) * node.dt, 0.08)

    # 모드가 없거나 align이 없으면 최종 정렬을 하지 않고 기존 동작(모드 없음: 기존 출발 거부) 그대로
    def test_mode_or_align_absent(self):
        node = PlantNode()
        follower = self.prepare(node, P1_POSE, law=None)

        with self.assertRaises(RuntimeError) as ctx:
            follower.follow()

        self.assertIn("출발 거부", str(ctx.exception))
        self.assertEqual(node.commands, [])
        self.assertNotIn("final_align", follower.run.record)

        node = PlantNode()
        follower = self.prepare(node, P1_POSE, waypoints="  - xy: [1.0, 0.0]\n    gate: true\n  - [3.0, 0.0]\n")
        follower.follow()
        self.assertNotIn("final_align", follower.run.record)
        self.assertEqual(node.commands, [])


    # [재현 reviewer] 회전 뒤 방향 0.005 rad(final 0.002 < 오차 ≤ max_heading 0.02)이면 final_align은 refused이고,
    # 그 거부가 관문 없는 출발 검사·관문 재출발 검사까지 그대로 적용된다(직진 없음)
    def test_refused_after_rotation_blocks_departure(self):
        for gate in (False, True):
            with self.subTest(gate=gate):
                node = PlantNode()
                follower = self.prepare(node, P1_POSE, gate=gate)
                original = follower.align_to

                def align_then_turn(*args, **kwargs):
                    original(*args, **kwargs)
                    node.yaw = 0.005

                follower.align_to = align_then_turn

                if gate:
                    follower.follow()
                    self.assertTrue(follower.entered[0][1])
                else:
                    with self.assertRaises(RuntimeError) as ctx:
                        follower.follow()

                    self.assertIn("출발 거부", str(ctx.exception))

                self.assertEqual(self.final(follower)[0]["outcome"], "refused")
                self.assertFalse(any(v != 0.0 for _, v, _ in node.commands))

    # [재현 reviewer] 정렬 성공 뒤 대기 중 방향이 0.005 rad로 변하면 출발·재출발 검사가 final_tolerance로 막는다
    def test_heading_change_after_alignment(self):
        node = PlantNode()
        follower = self.prepare(node, P1_POSE, gate=True)
        spy = follower.wait_gate

        def drift_then_check(index):
            node.yaw = 0.005
            spy(index)

        follower.wait_gate = drift_then_check
        follower.follow()
        self.assertEqual(self.final(follower)[0]["outcome"], "aligned")
        blockers = follower.entered[0][1]
        self.assertTrue(any("방향 +0.0050" in item for item in blockers), blockers)

        node = PlantNode()
        follower = self.prepare(node, P1_POSE)
        stop = follower.stop_and_confirm

        def drift_on_departure_check(reason):
            if reason.startswith("정렬 확인"):
                node.yaw = 0.005

            return stop(reason)

        follower.stop_and_confirm = drift_on_departure_check

        with self.assertRaises(RuntimeError) as ctx:
            follower.follow()

        self.assertIn("출발 거부", str(ctx.exception))
        self.assertFalse(any(v != 0.0 for _, v, _ in node.commands))

    # [재현 reviewer] 진입 전 입력 문제로 정렬하지 못한(refused) 상태는 나중 입력 회복만으로 성공이 되지 않고,
    # 다시 회전하지도 않는다(방향 0.005 rad: 기존 0.02 검사로는 통과하던 값)
    def test_input_refusal_not_auto_cleared(self):
        for gate in (False, True):
            with self.subTest(gate=gate):
                node = PlantNode()
                follower = self.prepare(node, (0.92, 0.01, 0.005), gate=gate)
                state = {"bad": True}
                follower.input_problems = lambda: ["front 스캔 만료 0.90 s"] if state["bad"] else []
                original = follower.final_align

                def final_then_recover(*args, **kwargs):
                    result = original(*args, **kwargs)
                    state["bad"] = False
                    return result

                follower.final_align = final_then_recover

                if gate:
                    follower.follow()
                    blockers = follower.entered[0][1]
                    self.assertTrue(any("최종 방향 정렬" in item for item in blockers), blockers)
                else:
                    with self.assertRaises(RuntimeError) as ctx:
                        follower.follow()

                    self.assertIn("최종 방향 정렬", str(ctx.exception))

                self.assertEqual(self.final(follower)[0]["outcome"], "refused")
                self.assertEqual(node.commands, [])


if __name__ == "__main__":
    unittest.main()
