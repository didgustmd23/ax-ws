# localization.launch.py 시작 감시(StartupMonitor)를 가짜 lifecycle 노드로 실제 ros2 launch에서 검사한다.
# Gazebo 없이 격리 ROS_DOMAIN_ID에서 실행하고, 실행마다 환경 표시로 남은 프로세스를 찾는다.
# domain은 시험 전용 범위(101~110, ros_test_domain.py)에서 다른 프로세스가 쓰지 않는 값을 잠금 파일로 잡아 클래스 전체에 쓴다.
# 팀 격리 시뮬레이션(141~231)의 같은 이름 노드(/map_server, /ax/<robot>/amcl)와 섞이면 결과가 뒤틀린다.
#
# 환경변수
#   AX_STARTUP_TEST_DOMAIN    domain을 직접 지정한다(재현용). 프로세스 사용 검사는 하지 않고 시작 전 그래프 검사만 한다.
#   AX_STARTUP_TEST_KEEP_DIR  실패한 시험의 launch.log·startup.json·stub.launch.py·domain.txt를 이 폴더에 보존한다.
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))

import ros_test_domain  # noqa: E402

LAUNCH_FILE = ROOT / 'src/ax_localization/launch/localization.launch.py'
STUB = Path(__file__).resolve().parent / 'stub_lifecycle_node.py'

# 이 시험의 가짜 노드와 같은 이름. 선택한 domain에 이미 있으면 다른 실행과 섞인다.
CONFLICT_NAMES = r'^/map_server$|^/ax/[^/]+/amcl$'

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
    return ros_test_domain.processes_with(f'AX_STARTUP_TEST={marker}')


# 기한(s) 안에서 표시 프로세스가 모두 끝날 때까지 기다리고 남은 목록을 돌려준다.
def wait_marked_gone(marker, deadline_s):
    end = time.monotonic() + deadline_s
    left = marked_processes(marker)

    while left and time.monotonic() < end:
        time.sleep(0.1)
        left = marked_processes(marker)

    return left


# 폴링용 결과 상태 읽기. 파일이 없거나 쓰는 중이라 해석할 수 없으면 None(다음 폴링에서 다시 읽음).
def read_status(report):
    try:
        return json.loads(report.read_text())['status']
    except (OSError, ValueError, KeyError):
        return None


@unittest.skipUnless(ros_available(), 'ROS 2 launch 필요(source install/setup.bash)')
class StartupMonitorLaunch(unittest.TestCase):

    # 클래스 전체에 시험 전용 domain 하나를 잡고(잠금 유지), 시작 전에 같은 이름 노드가 있는지 한 번 확인한다.
    # 지정 domain(AX_STARTUP_TEST_DOMAIN)은 프로세스 사용 검사 없이 그대로 쓰고 그래프 검사만 한다.
    @classmethod
    def setUpClass(cls):
        fixed = os.environ.get('AX_STARTUP_TEST_DOMAIN')

        if fixed:
            cls.domain, cls.lock = int(fixed), None
        else:
            cls.domain, cls.lock = ros_test_domain.reserve_domain(ros_test_domain.STARTUP_DOMAINS)

        cls.conflict = ros_test_domain.conflicting_nodes(cls.domain, CONFLICT_NAMES)

    # domain 잠금을 푼다.
    @classmethod
    def tearDownClass(cls):
        ros_test_domain.release(cls.lock)

    # 실행마다 임시 폴더를 쓴다. 클래스 시작 때 같은 이름 노드가 보였으면 원인이 보이는 실패로 즉시 끝낸다.
    # setUp이 실패하면 tearDown이 불리지 않으므로 임시 폴더를 만들기 전에 확인한다.
    def setUp(self):
        if self.conflict:
            self.fail(ros_test_domain.conflict_message(self.domain, self.conflict))

        self.folder = Path(tempfile.mkdtemp(prefix='ax_startup_'))
        self.marker = uuid.uuid4().hex
        (self.folder / 'domain.txt').write_text(f'{self.domain}\n')
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                        ROS_LOG_DIR=str(self.folder / 'roslog'), AX_STARTUP_TEST=self.marker)
        self.extra = []

    # 남은 표시 프로세스를 정리하고, 실패했으면 기록을 보존한 뒤 임시 폴더를 지운다.
    def tearDown(self):
        for process in self.extra:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)

        for pid in marked_processes(self.marker):
            os.kill(pid, signal.SIGKILL)

        self.keep_if_failed()
        shutil.rmtree(self.folder, ignore_errors=True)

    # 이 시험이 실패·오류로 기록됐는지(unittest 결과와 pytest 결과 모두)
    def failed(self):
        result = getattr(getattr(self, '_outcome', None), 'result', None)

        for name in ('failures', 'errors'):
            if any(case is self for case, _ in getattr(result, name, None) or []):
                return True

        return bool(getattr(result, '_excinfo', None))

    # 실패한 시험의 launch.log·startup.json·stub.launch.py·domain.txt를 AX_STARTUP_TEST_KEEP_DIR에 복사한다.
    def keep_if_failed(self):
        keep = os.environ.get('AX_STARTUP_TEST_KEEP_DIR')

        if not keep or not self.failed():
            return

        target = Path(keep) / f'{datetime.now():%Y%m%d_%H%M%S}_{self._testMethodName}_domain{self.domain}'
        target.mkdir(parents=True, exist_ok=True)

        for name in ('launch.log', 'startup.json', 'stub.launch.py', 'domain.txt'):
            if (self.folder / name).is_file():
                shutil.copy2(self.folder / name, target / name)

    # 실패 메시지 앞머리: 사용 domain과 launch 로그 끝부분
    def note(self, result=None):
        tail = result['log'][-2000:] if result else ''

        return f'[domain {self.domain}] {tail}'

    # launch 직전, 이 시험이 띄운 것(표시 프로세스·extra) 말고 이 domain을 쓰는 프로세스가 생겼으면 즉시 실패한다.
    # 지정 domain 모드는 재현용이라 검사하지 않는다.
    def assert_domain_free(self):
        if os.environ.get('AX_STARTUP_TEST_DOMAIN'):
            return

        own = set(marked_processes(self.marker)) | {process.pid for process in self.extra}
        others = ros_test_domain.other_domain_users(self.domain, own)

        if others:
            self.fail(f'domain {self.domain} 사용 중: 다른 프로세스 {others}가 같은 ROS_DOMAIN_ID를 씀. '
                      f'다른 실행과 섞이므로 시험하지 않음')

    # 가짜 노드 구성으로 launch를 실행한다. wait_active면 모두 active가 된 뒤 SIGINT로 끝낸다.
    def run_launch(self, nodes, timeout_s=8.0, wait_active=False, after_active_s=0.0, limit_s=60.0):
        report = self.folder / 'startup.json'
        launch = self.folder / 'stub.launch.py'
        launch.write_text(TEST_LAUNCH.format(launch=str(LAUNCH_FILE), nodes=nodes, stub=str(STUB),
                                             timeout=timeout_s, report=str(report)))
        self.assert_domain_free()
        log = open(self.folder / 'launch.log', 'w')
        started = time.monotonic()
        process = subprocess.Popen(['ros2', 'launch', str(launch)], env=self.env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)

        try:
            if wait_active:
                while time.monotonic() - started < limit_s and process.poll() is None:
                    if read_status(report) == 'active':
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

        # 종료 뒤 최대 1.0 s 안에서 자식 프로세스 종료를 확인한다.
        left = wait_marked_gone(self.marker, 1.0)

        return {
            'code': code,
            'elapsed_s': time.monotonic() - started,
            'report': json.loads(report.read_text()) if report.is_file() else None,
            'log': (self.folder / 'launch.log').read_text(),
            'left': left,
            'running_after_active': still_running,
        }

    # 세 노드가 모두 active가 되면 status active, Ctrl+C로 정상 종료하고 남는 프로세스가 없다.
    def test_all_active(self):
        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_1', 'ok'),
                                  ('amcl', '/ax/robot_2', 'ok')], wait_active=True)

        self.assertEqual(result['report']['status'], 'active', self.note(result))
        self.assertIsNone(result['report']['failure'], self.note(result))
        self.assertTrue(all(node['state'] == 'active' for node in result['report']['nodes'].values()),
                        self.note(result))
        self.assertIn('모든 노드 활성화', result['log'], self.note(result))
        self.assertEqual(result['code'], 0, self.note(result))
        self.assertEqual(result['left'], [], self.note(result))

    # 실패 결과 공통 검사: 종료 코드 1, 기록된 실패 노드·단계, 남은 프로세스 없음, 무기한 대기 없음
    def assert_failed(self, result, node, stage, reason):
        report = result['report']

        self.assertEqual(result['code'], 1, self.note(result))
        self.assertEqual(report['status'], 'failed', self.note(result))
        self.assertIn(node, report['failure']['node'], self.note(result))
        self.assertEqual(report['failure']['stage'], stage, self.note(result))
        self.assertIn(reason, report['failure']['reason'], self.note(result))
        self.assertIn('시작 실패', result['log'], self.note(result))
        self.assertEqual(result['left'], [], self.note(result))

    # configure 거절
    def test_configure_rejected(self):
        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_2', 'reject_configure')])

        self.assert_failed(result, '/ax/robot_2/amcl', 'configure', 'configure 거절')

    # configure 무응답: 제한시간에 실패하고, 응답 없는 프로세스도 정리된다.
    def test_configure_hang(self):
        result = self.run_launch([('map_server', '', 'ok'), ('amcl', '/ax/robot_2', 'hang_configure')], timeout_s=5.0)

        self.assert_failed(result, '/ax/robot_2/amcl', 'timeout', '5.0 s 안에 active가 되지 않음')
        self.assertNotIn('/map_server', result['report']['failure']['node'], self.note(result))
        self.assertLess(result['elapsed_s'], 40.0, self.note(result))

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

        self.assertEqual(report['status'], 'active', self.note(result))
        self.assertIsNone(report['failure'], self.note(result))
        self.assertEqual(report['nodes']['/ax/robot_2/amcl']['exit_code'], 4, self.note(result))
        self.assertEqual(report['nodes']['/map_server']['state'], 'active', self.note(result))
        self.assertIsNone(report['nodes']['/map_server']['exit_code'], self.note(result))
        self.assertIn('시작 뒤 /ax/robot_2/amcl 프로세스 종료', result['log'], self.note(result))
        self.assertNotIn('시작 실패', result['log'], self.note(result))
        self.assertTrue(result['running_after_active'], self.note(result))
        self.assertEqual(result['code'], 0, self.note(result))
        self.assertEqual(result['left'], [], self.note(result))

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
        self.assertIsNone(other.poll(), self.note(result))
        self.assertEqual(marked_processes(other_marker), [other.pid], self.note(result))


if __name__ == '__main__':
    unittest.main()
