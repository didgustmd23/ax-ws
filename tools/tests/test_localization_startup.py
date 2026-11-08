# localization.launch.py 시작 감시(StartupMonitor)를 가짜 lifecycle 노드로 실제 ros2 launch에서 검사한다.
# Gazebo 없이 격리 ROS_DOMAIN_ID에서 실행하고, 실행마다 환경 표시로 남은 프로세스를 찾는다.
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[2]
LAUNCH_FILE = ROOT / 'src/ax_localization/launch/localization.launch.py'
STUB = Path(__file__).resolve().parent / 'stub_lifecycle_node.py'

# 시험용 launch: 감시 코드는 localization.launch.py의 startup_actions를 그대로 쓰고 노드만 가짜로 바꾼다.
TEST_LAUNCH = '''
import importlib.util
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

spec = importlib.util.spec_from_file_location("localization_launch", {launch!r})
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def generate_launch_description():
    nodes = []

    for name, namespace, behavior in {nodes!r}:
        node = LifecycleNode(executable={stub!r}, name=name, namespace=namespace, output="screen",
                             parameters=[{{"behavior": behavior, "exit_delay_s": 1.0}}])
        nodes.append((node, (namespace.rstrip("/") + "/" + name) if namespace else "/" + name))

    actions, _ = module.startup_actions(nodes, {timeout!r}, {report!r})

    return LaunchDescription(actions)
'''


# ROS 2 launch를 쓸 수 있는지 확인한다(source install/setup.bash 필요).
def ros_available():
    if shutil.which('ros2') is None:
        return False

    return importlib.util.find_spec('launch_ros') is not None and importlib.util.find_spec('rclpy') is not None


# 표시 환경변수를 가진 살아 있는 프로세스 pid 목록
def marked_processes(marker):
    found = []

    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue

        try:
            environ = (entry / 'environ').read_bytes().split(b'\0')
        except OSError:
            continue

        if f'AX_STARTUP_TEST={marker}'.encode() in environ:
            found.append(int(entry.name))

    return found


@unittest.skipUnless(ros_available(), 'ROS 2 launch 필요(source install/setup.bash)')
class StartupMonitorLaunch(unittest.TestCase):

    # 실행마다 임시 폴더와 격리 domain을 쓴다.
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp(prefix='ax_startup_'))
        self.marker = uuid.uuid4().hex
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(150 + os.getpid() % 50), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                        ROS_LOG_DIR=str(self.folder / 'roslog'), AX_STARTUP_TEST=self.marker)
        self.extra = []

    # 남은 표시 프로세스를 정리하고 임시 폴더를 지운다.
    def tearDown(self):
        for process in self.extra:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)

        for pid in marked_processes(self.marker):
            os.kill(pid, signal.SIGKILL)

        shutil.rmtree(self.folder, ignore_errors=True)

    # 가짜 노드 구성으로 launch를 실행한다. wait_active면 모두 active가 된 뒤 SIGINT로 끝낸다.
    def run_launch(self, nodes, timeout_s=8.0, wait_active=False, after_active_s=0.0, limit_s=60.0):
        report = self.folder / 'startup.json'
        launch = self.folder / 'stub.launch.py'
        launch.write_text(TEST_LAUNCH.format(launch=str(LAUNCH_FILE), nodes=nodes, stub=str(STUB),
                                             timeout=timeout_s, report=str(report)))
        log = open(self.folder / 'launch.log', 'w')
        started = time.monotonic()
        process = subprocess.Popen(['ros2', 'launch', str(launch)], env=self.env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)

        try:
            if wait_active:
                while time.monotonic() - started < limit_s and process.poll() is None:
                    if report.is_file() and json.loads(report.read_text())['status'] == 'active':
                        break

                    time.sleep(0.2)

                time.sleep(after_active_s)
                still_running = process.poll() is None
                process.send_signal(signal.SIGINT)
            else:
                still_running = None

            code = process.wait(timeout=limit_s)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)

            log.close()

        # 종료 뒤 잠시 기다려 자식 프로세스 종료를 확인한다.
        time.sleep(1.0)

        return {
            'code': code,
            'elapsed_s': time.monotonic() - started,
            'report': json.loads(report.read_text()) if report.is_file() else None,
            'log': (self.folder / 'launch.log').read_text(),
            'left': marked_processes(self.marker),
            'running_after_active': still_running,
        }

    # 세 노드가 모두 active가 되면 status active, Ctrl+C로 정상 종료하고 남는 프로세스가 없다.
    def test_all_active(self):
        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_1', 'ok'),
                                  ('amcl', '/ax/robot_2', 'ok')], wait_active=True)

        self.assertEqual(result['report']['status'], 'active', result['log'][-2000:])
        self.assertIsNone(result['report']['failure'])
        self.assertTrue(all(node['state'] == 'active' for node in result['report']['nodes'].values()))
        self.assertIn('모든 노드 활성화', result['log'])
        self.assertEqual(result['code'], 0)
        self.assertEqual(result['left'], [])

    # 실패 결과 공통 검사: 종료 코드 1, 기록된 실패 노드·단계, 남은 프로세스 없음, 무기한 대기 없음
    def assert_failed(self, result, node, stage, reason):
        report = result['report']

        self.assertEqual(result['code'], 1, result['log'][-2000:])
        self.assertEqual(report['status'], 'failed')
        self.assertIn(node, report['failure']['node'])
        self.assertEqual(report['failure']['stage'], stage)
        self.assertIn(reason, report['failure']['reason'])
        self.assertIn('시작 실패', result['log'])
        self.assertEqual(result['left'], [], result['log'][-2000:])

    # configure 거절
    def test_configure_rejected(self):
        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_2', 'reject_configure')])

        self.assert_failed(result, '/ax/robot_2/amcl', 'configure', 'configure 거절')

    # configure 무응답: 제한시간에 실패하고, 응답 없는 프로세스도 정리된다.
    def test_configure_hang(self):
        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_2', 'hang_configure')], timeout_s=5.0)

        self.assert_failed(result, '/ax/robot_2/amcl', 'timeout', '5.0 s 안에 active가 되지 않음')
        self.assertNotIn('/map_server', result['report']['failure']['node'])
        self.assertLess(result['elapsed_s'], 40.0)

    # activate 실패
    def test_activate_failed(self):
        result = self.run_launch([('map_server', '', 'fail_activate'), ('amcl', '/ax/robot_2', 'ok')])

        self.assert_failed(result, '/map_server', 'activate', 'activate 실패')

    # 시작 중 노드 종료: 실패 처리·정리·종료 코드 1
    def test_exit_during_startup(self):
        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_1', 'exit_configure'),
                                  ('amcl', '/ax/robot_2', 'ok')])

        self.assert_failed(result, '/ax/robot_1/amcl', 'configure', '시작 중 프로세스 종료(종료 코드 3)')

    # 활성화 완료 후 노드 종료: 기록만 수행. 실패 처리하지 않고 launch와 다른 노드는 계속 실행한다.
    # 끝낼 때(Ctrl+C)의 종료 코드는 정상 종료와 같다.
    def test_exit_after_active(self):
        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_2', 'exit_active')],
                                 wait_active=True, after_active_s=3.0)
        report = result['report']

        self.assertEqual(report['status'], 'active')
        self.assertIsNone(report['failure'])
        self.assertEqual(report['nodes']['/ax/robot_2/amcl']['exit_code'], 4)
        self.assertEqual(report['nodes']['/map_server']['state'], 'active')
        self.assertIsNone(report['nodes']['/map_server']['exit_code'])
        self.assertIn('시작 뒤 /ax/robot_2/amcl 프로세스 종료', result['log'])
        self.assertNotIn('시작 실패', result['log'])
        self.assertTrue(result['running_after_active'])
        self.assertEqual(result['code'], 0)
        self.assertEqual(result['left'], [])

    # 실패로 끝나도 이 launch가 실행하지 않은 노드(다른 사용자·시뮬레이션 대용)는 종료하지 않는다.
    def test_failure_keeps_other_nodes(self):
        other_marker = uuid.uuid4().hex
        other_env = dict(self.env, AX_STARTUP_TEST=other_marker)
        other = subprocess.Popen([str(STUB), '--ros-args', '-r', '__node:=other_user_node'], env=other_env,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.extra.append(other)
        time.sleep(1.0)

        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_2', 'reject_configure')])

        self.assert_failed(result, '/ax/robot_2/amcl', 'configure', 'configure 거절')
        self.assertIsNone(other.poll())
        self.assertEqual(marked_processes(other_marker), [other.pid])


if __name__ == '__main__':
    unittest.main()
