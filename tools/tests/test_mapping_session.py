# B2 지도 작성 세션(run_mapping_session.sh)의 기동 전 거절과 프로세스 정리(session_procs.sh)를 Gazebo 없이 검사한다.
# - 월드·경로 사전검사: 시뮬레이션·로그 폴더를 만들기 전에 exit 2로 거절하는지
# - 정리: setsid 그룹·SIGINT 무시·그룹 밖으로 빠진 소유 프로세스까지 정리하고, 표식 없는 프로세스는 남기는지
# - 중단 우선: 주행 그룹에 SIGINT를 보내 정지 절차가 끝날 때까지 기다리는지
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[1].parent
sys.path.insert(0, str(Path(__file__).resolve().parent))

import ros_test_domain  # noqa: E402

SESSION = ROOT / "src/ax_localization/scripts/run_mapping_session.sh"
PROCS = ROOT / "src/ax_localization/scripts/session_procs.sh"
ROUTES = ROOT / "src/ax_localization/config/routes"
WORLD = "planned/ax_warehouse.world"
ROS_READY = Path("/opt/ros/jazzy/setup.bash").is_file() and (ROOT / "install/setup.bash").is_file()


# bash 스크립트를 실행해 (종료 코드, 출력)을 돌려준다.
def bash(script, timeout=60, env=None):
    result = subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=timeout,
                            env=env, stdin=subprocess.DEVNULL)
    return result.returncode, result.stdout + result.stderr


# 살아 있는(좀비 아닌) 프로세스인지
def alive(pid):
    try:
        state = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except (FileNotFoundError, IndexError):
        return False

    return state != "Z"


@unittest.skipUnless(ROS_READY, "ROS Jazzy·install/setup.bash 필요")
class SessionPrecheck(unittest.TestCase):

    # 각 잘못된 조합은 exit 2이고 시뮬레이션 로그 폴더(log/b2_mapping/<시각>)를 만들지 않는다
    def test_rejected_before_launch(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            center = ROUTES / "ax_warehouse_center.yaml"
            mismatch = folder / "mismatch.yaml"
            mismatch.write_text(center.read_text().replace(f"world: {WORLD}", "world: planned/other.world"))
            gz = folder / "gz.yaml"
            gz.write_text(center.read_text().replace("gz_world: ax_test", "gz_world: aws"))
            cases = {
                "월드 없음(옛 AWS 시험기)": ["--route", str(center)],
                "경로 없음": ["--world", WORLD],
                "지원하지 않는 월드": ["--world", "other.world", "--route", str(center)],
                "AWS 경로": ["--world", WORLD, "--route", str(ROUTES / "aws_warehouse.yaml")],
                "경로 world 불일치": ["--world", WORLD, "--route", str(mismatch)],
                "gz_world 불일치": ["--world", WORLD, "--route", str(gz)],
                "로봇 불일치": ["--world", WORLD, "--route", str(center), "--robot-id", "robot_1"],
                "--light": ["--world", WORLD, "--route", str(center), "--light"],
                "--wheel-slip1": ["--world", WORLD, "--route", str(center), "--wheel-slip1", "0.05"],
                "없는 경로 파일": ["--world", WORLD, "--route", str(folder / "none.yaml")],
                "터미널 없는 대기": ["--world", WORLD, "--route", str(center), "--wait-before-drive"],
            }
            logs = ROOT / "log/b2_mapping"
            before = set(os.listdir(logs)) if logs.is_dir() else set()

            for label, args in cases.items():
                with self.subTest(case=label):
                    result = subprocess.run(["bash", str(SESSION), *args], capture_output=True, text=True, timeout=120,
                                            stdin=subprocess.DEVNULL)
                    self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                    self.assertNotIn("시뮬레이션 시작", result.stdout)

            after = set(os.listdir(logs)) if logs.is_dir() else set()
            self.assertEqual(after - before, set())

    # 예방: wait_topic과 같은 명령(ros2 topic echo --once --no-daemon)은 메시지를 받고 ros2-daemon을 띄우지 않는다.
    # 전용 ROS domain을 써서 사용자 세션과 섞지 않고, 이 시험이 띄운 발행자만 정리한다.
    def test_wait_topic_no_daemon(self):
        self.assertIn('ros2 topic echo --once --no-daemon "$topic"', SESSION.read_text())
        domain, lock = ros_test_domain.reserve_domain(ros_test_domain.HEALTH_AND_SESSION_DOMAINS)
        self.addCleanup(ros_test_domain.release, lock)
        domain = str(domain)
        env = dict(os.environ, ROS_DOMAIN_ID=domain, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
        publisher = subprocess.Popen(
            ["python3", "-c", "import rclpy, time\nfrom std_msgs.msg import String\nrclpy.init()\n"
             "n = rclpy.create_node('ax_test_pub')\np = n.create_publisher(String, '/ax_wait_topic_test', 10)\n"
             "while True:\n    p.publish(String(data='x'))\n    time.sleep(0.1)\n"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        try:
            result = subprocess.run(["timeout", "30", "ros2", "topic", "echo", "--once", "--no-daemon",
                                     "/ax_wait_topic_test", "std_msgs/msg/String"],
                                    env=env, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            daemons = subprocess.run(["pgrep", "-f", f"ros2-daemon --ros-domain-id {domain} "],
                                     capture_output=True, text=True).stdout.split()
            self.assertEqual(daemons, [], f"domain {domain}에 ros2-daemon 생성")
        finally:
            publisher.kill()
            publisher.wait()

    # 도움말은 새 옵션을 설명하고 0으로 끝난다
    def test_help(self):
        result = subprocess.run(["bash", str(SESSION), "--help"], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0)
        self.assertIn("--world FILE", result.stdout)
        self.assertIn("ax_warehouse_center.yaml", result.stdout)


@unittest.skipUnless(shutil.which("setsid"), "setsid 필요")
class SessionProcesses(unittest.TestCase):

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.bystander = subprocess.Popen(["sleep", "120"])   # 소유 표식 없는 무관한 프로세스

    def tearDown(self):
        self.bystander.kill()
        self.bystander.wait()
        self.temp.cleanup()

    # 정리 뒤 소유 잔여 0: SIGINT를 무시하는 그룹, 그룹 밖(setsid)으로 빠진 손자까지 정리. 표식 없는 프로세스는 남는다
    def test_cleanup_owned_only(self):
        pids = self.folder / "pids"
        script = textwrap.dedent(f"""
            source {PROCS}
            export AX_SESSION_OWNER=test-owner-$$
            session_start stubborn {self.folder}/a.log bash -c 'trap "" INT; sleep 300 & echo $! >> {pids}; wait'
            session_start escaped {self.folder}/b.log bash -c 'setsid sleep 300 & echo $! >> {pids}; sleep 300'
            session_start plain {self.folder}/c.log sleep 300
            sleep 1
            echo "before $(session_owned | wc -l)"
            session_cleanup 2
            echo "remaining $SESSION_REMAINING"
        """)
        code, output = bash(script, timeout=60)
        self.assertEqual(code, 0, output)
        before = int(output.split("before ")[1].split()[0])
        self.assertGreaterEqual(before, 5, output)
        self.assertIn("remaining 0", output)

        for pid in pids.read_text().split():
            self.assertFalse(alive(int(pid)), f"남은 프로세스 {pid}")

        self.assertIsNone(self.bystander.poll())

    # 중단 우선: 주행 그룹에 SIGINT → 정지 절차(1 s)가 끝나 표시 파일을 남길 때까지 기다린 뒤 돌아온다
    def test_stop_group_waits_for_stop_procedure(self):
        marker = self.folder / "stopped"
        drive = textwrap.dedent(f"""
            trap 'sleep 1; echo done > {marker}; exit 130' INT
            while true; do sleep 0.1; done
        """)
        (self.folder / "drive.sh").write_text(drive)
        script = textwrap.dedent(f"""
            source {PROCS}
            export AX_SESSION_OWNER=test-owner-$$
            session_start drive {self.folder}/drive.log bash {self.folder}/drive.sh
            sleep 0.5
            start=$SECONDS
            session_stop_group "$SESSION_PID_drive" 20; code=$?
            echo "code $code waited $((SECONDS - start))"
            [ -f {marker} ] && echo marker-ok
            session_cleanup 1
            echo "remaining $SESSION_REMAINING"
        """)
        code, output = bash(script, timeout=60)
        self.assertEqual(code, 0, output)
        self.assertIn("code 0", output)
        self.assertIn("marker-ok", output)
        self.assertIn("remaining 0", output)

    # 정지 절차가 시간 안에 끝나지 않으면 1을 돌려주고, 정리 단계가 강제로 끝낸다
    def test_stop_group_timeout_then_cleanup(self):
        script = textwrap.dedent(f"""
            source {PROCS}
            export AX_SESSION_OWNER=test-owner-$$
            session_start drive {self.folder}/drive.log bash -c 'trap "" INT; while true; do sleep 0.1; done'
            sleep 0.5
            session_stop_group "$SESSION_PID_drive" 1; echo "code $?"
            session_cleanup 1
            echo "remaining $SESSION_REMAINING"
        """)
        code, output = bash(script, timeout=60)
        self.assertIn("code 1", output)
        self.assertIn("remaining 0", output)

    # [4단계 재현] 세션 셸과 같은 SID에 남은 소유 표식 고아(이중 fork, ros2 daemon과 같은 형태)를 놓치지 않는다.
    # 정리 뒤 잔여 수는 실제 프로세스 생존과 일치해야 하며, 다른 회차 표식·표식 없는 프로세스는 남는다.
    def test_same_sid_orphan_counted_and_cleaned(self):
        orphan, other = self.folder / "orphan.pid", self.folder / "other.pid"
        script = textwrap.dedent(f"""
            source {PROCS}
            ( AX_SESSION_OWNER=other-run setsid sleep 300 > /dev/null 2>&1 < /dev/null & echo $! > {other} )
            export AX_SESSION_OWNER=test-owner-$$
            ( sleep 300 > /dev/null 2>&1 < /dev/null & echo $! > {orphan} )
            sleep 0.5
            echo "sid_shell $(session_sid $$) sid_orphan $(session_sid $(cat {orphan}))"
            echo "listed $(session_owned | tr '\\n' ' ')"
            session_cleanup 1
            echo "remaining $SESSION_REMAINING"
        """)
        code, output = bash(script, timeout=60)
        orphan_pid, other_pid = int(orphan.read_text()), int(other.read_text())

        try:
            sids = output.split("sid_shell ")[1].split()
            self.assertEqual(sids[0], sids[2], output)                 # 같은 SID 고아 조건이 실제로 만들어졌는지
            self.assertIn(str(orphan_pid), output.split("listed ")[1].splitlines()[0], output)
            self.assertNotIn(str(other_pid), output.split("listed ")[1].splitlines()[0], output)
            self.assertFalse(alive(orphan_pid), "같은 SID 소유 고아가 남음")
            self.assertIn("remaining 0", output)
            self.assertTrue(alive(other_pid), "다른 회차 표식 프로세스를 정리함")
        finally:
            for pid in (orphan_pid, other_pid):
                try:
                    os.kill(pid, 9)
                except ProcessLookupError:
                    pass

    # 정리 보조 명령(tr·grep·wc·명령 치환 subshell)은 소유 프로세스로 세지 않는다
    def test_helpers_not_listed(self):
        script = textwrap.dedent(f"""
            source {PROCS}
            export AX_SESSION_OWNER=test-owner-$$
            echo "n=$(session_owned | wc -l)"
            session_cleanup 1
            echo "remaining $SESSION_REMAINING"
        """)
        code, output = bash(script, timeout=60)
        self.assertIn("n=0", output)
        self.assertIn("remaining 0", output)

    # 잔여가 실제로 남으면 0으로 보고하지 않는다(종료를 막을 수 없는 프로세스는 없으므로 정리 함수를 가로채 확인)
    def test_remaining_reflects_actual(self):
        pid_file = self.folder / "kept.pid"
        script = textwrap.dedent(f"""
            source {PROCS}
            export AX_SESSION_OWNER=test-owner-$$
            session_start kept {self.folder}/k.log sleep 300
            echo $SESSION_PID_kept > {pid_file}
            sleep 0.5
            kill() {{ :; }}          # 신호 전달을 막아 정리가 실패하는 상황을 만든다
            session_cleanup 1
            unset -f kill
            echo "remaining $SESSION_REMAINING"
            command kill -9 $(cat {pid_file})
        """)
        code, output = bash(script, timeout=60)
        self.assertNotIn("remaining 0", output, output)
        self.assertRegex(output, r"remaining [1-9]")

    # 표식이 없으면 아무것도 소유로 보지 않는다(다른 세션 보호)
    def test_no_owner_no_match(self):
        code, output = bash(f"source {PROCS}; unset AX_SESSION_OWNER; echo \"n=$(session_owned | wc -l)\"")
        self.assertIn("n=0", output)


if __name__ == "__main__":
    unittest.main()
