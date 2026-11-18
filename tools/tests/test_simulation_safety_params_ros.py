"""상위 simulation.launch.py 경로로 띄운 A4/B4 노드의 실제 매개변수를 GetParameters로 확인한다(실ROS, 가짜 환경).

- 설치된 ax_control의 simulation.launch.py 모듈을 읽어 그 launch_setup을 그대로 실행하고, 반환된 action 중
  로봇별 safety GroupAction만 띄운다. A1 robot_test(Gazebo·월드·모델), 제어기 spawner, 공구 노드는 띄우지 않는다.
  따라서 상위 인자 해석·검사·로봇별 include·scope와 하위 safety.launch.py·실제 노드 매개변수까지는 실행하지만
  Gazebo 물리·제어기·센서 입력은 확인하지 않는다.
- 전역 별칭 거부는 실제 `ros2 launch ax_control simulation.launch.py` CLI로 확인한다. 거부가 빠져도 Gazebo가
  뜨지 않도록 없는 world를 함께 준다(그 경우 오류 문구가 달라 실패로 드러난다).
- 로봇별 파일의 주행 자세·팔 한계는 시험용 구별값이다(A1 운영값 아님). 허가 플래그는 모두 기본 false다.
- 환경: 검증할 격리 install을 source한 셸에서 실행한다. domain은 ros_test_domain helper로 90~99에서 예약한다.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402

_domain_state = None

# 설치된 상위 launch를 그대로 읽어 safety GroupAction만 남기는 시험용 진입점
WRAPPER = '''
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction

PATH = Path(get_package_share_directory("ax_control")) / "launch" / "simulation.launch.py"
SPEC = importlib.util.spec_from_file_location("installed_simulation_launch", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


# 실제 launch_setup을 실행하고 로봇별 safety 그룹만 띄운다(A1·spawner·공구 노드 제외)
def setup(context):
    return [action for action in MODULE.launch_setup(context) if isinstance(action, GroupAction)]


def generate_launch_description():
    declared = [entity for entity in MODULE.generate_launch_description().entities
                if isinstance(entity, DeclareLaunchArgument)]
    return LaunchDescription(declared + [OpaqueFunction(function=setup)])
'''

ROBOT_A = ('/**/a4_safety_gate:\n  ros__parameters:\n'
           '    travel_joint_names: [test_joint_a, test_joint_b]\n    travel_joint_positions: [0.1, 0.2]\n'
           '/**/a4_protected_arm_server:\n  ros__parameters:\n'
           '    arm_joint_names: [test_joint_a]\n    arm_position_min_rad: [-1.0]\n    arm_position_max_rad: [1.0]\n'
           '    arm_max_velocity_rad_s: [0.5]\n    arm_max_acceleration_rad_s2: [0.5]\n')

AX1_JOINTS = ['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']
GATE_FLAGS = ['enable_simulation_drive', 'operational_permits_verified', 'travel_envelope_verified',
              'arm_operation_verified']


def setUpModule():
    global _domain_state
    domain, lock = ros_test_domain.reserve_domain(range(90, 100))
    _domain_state = {'domain': domain, 'lock': lock,
                     'previous': {key: os.environ.get(key) for key in
                                  ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'GZ_PARTITION')}}
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      GZ_PARTITION=f'ax_sim_safety_test_{os.getpid()}')


def tearDownModule():
    global _domain_state
    ros_test_domain.leave_module_domain(_domain_state)
    _domain_state = None


# 시험이 띄운 프로세스 그룹만 정리한다
def stop_group(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


class SimulationSafetyParamsRosTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.wrapper = Path(self.tmp.name) / 'simulation_safety_only.launch.py'
        self.wrapper.write_text(WRAPPER)
        # 경로 내부 공백 보존을 실제 launch에서도 확인한다
        folder = Path(self.tmp.name) / 'robot params'
        folder.mkdir()
        self.robot_a = folder / 'robot a.yaml'
        self.robot_a.write_text(ROBOT_A)
        self.share = Path(get_package_share_directory('ax_safety')) / 'config'
        self.process = None
        self.addCleanup(lambda: stop_group(self.process))
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = Node('simulation_safety_param_peer')
        self.addCleanup(self.node.destroy_node)

    # 시험 진입점을 띄운다. 같은 이름 노드가 이미 있으면 섞이므로 실패한다
    def launch(self, arguments):
        domain = int(os.environ['ROS_DOMAIN_ID'])
        names = ros_test_domain.conflicting_nodes(domain, r'^/ax/robot_[1-4]/(a4_|b4_)')
        self.assertFalse(names, ros_test_domain.conflict_message(domain, names))
        self.process = subprocess.Popen(['ros2', 'launch', str(self.wrapper)] + arguments,
                                        env=os.environ.copy(), start_new_session=True,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)

    def get(self, fqn, names):
        client = self.node.create_client(GetParameters, f'{fqn}/get_parameters')
        deadline = time.monotonic() + 30.0
        while not client.wait_for_service(timeout_sec=0.5):
            if self.process.poll() is not None:
                self.fail(f'launch 종료({self.process.returncode}): {self.process.stderr.read()[-2000:]}')
            self.assertLess(time.monotonic(), deadline, f'{fqn} 매개변수 서비스 없음')
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())
        values = []
        for value in future.result().values:
            values.append({1: value.bool_value, 2: value.integer_value, 3: value.double_value,
                           4: value.string_value, 8: list(value.double_array_value),
                           9: list(value.string_array_value)}.get(value.type))
        self.node.destroy_client(client)
        return dict(zip(names, values))

    # 두 로봇에 서로 다른 정지 역할, robot_1에만 로봇별 파일. 허가는 모두 false
    def test_two_robots_separated(self):
        self.launch(['safety:=true', 'robots:=2',
                     f'safety_stop_evidence_files:=robot_1={self.share / "stop_evidence_ax1.yaml"},'
                     f'robot_2={self.share / "stop_evidence_ax2.yaml"}',
                     f'safety_robot_params_files:=robot_1={self.robot_a}'])

        gate_names = ['robot_id', 'stop_arm_presence', 'stop_arm_joint_names', 'travel_joint_names',
                      'travel_joint_positions', 'max_linear_m_s', *GATE_FLAGS]
        gate_1 = self.get('/ax/robot_1/a4_safety_gate', gate_names)
        gate_2 = self.get('/ax/robot_2/a4_safety_gate', gate_names)

        self.assertEqual(gate_1['robot_id'], 'robot_1')
        self.assertEqual(gate_1['stop_arm_presence'], 'present')
        self.assertEqual(gate_1['stop_arm_joint_names'], AX1_JOINTS)
        self.assertEqual(gate_1['travel_joint_names'], ['test_joint_a', 'test_joint_b'])
        self.assertEqual(gate_1['travel_joint_positions'], [0.1, 0.2])

        self.assertEqual(gate_2['robot_id'], 'robot_2')
        self.assertEqual(gate_2['stop_arm_presence'], 'absent')
        self.assertEqual(gate_2['stop_arm_joint_names'], [])
        self.assertEqual(gate_2['travel_joint_names'], [])
        self.assertEqual(gate_2['travel_joint_positions'], [])

        # 공통 파일(패키지 기본)이 두 로봇에 모두 적용됐다
        for gate in (gate_1, gate_2):
            self.assertEqual(gate['max_linear_m_s'], 0.67)
            for flag in GATE_FLAGS:
                self.assertIs(gate[flag], False, flag)

        arm_names = ['robot_id', 'enable_simulation_arm', 'arm_joint_names', 'arm_max_velocity_rad_s']
        arm_1 = self.get('/ax/robot_1/a4_protected_arm_server', arm_names)
        arm_2 = self.get('/ax/robot_2/a4_protected_arm_server', arm_names)
        self.assertEqual(arm_1['arm_joint_names'], ['test_joint_a'])
        self.assertEqual(arm_1['arm_max_velocity_rad_s'], [0.5])
        self.assertEqual(arm_2['arm_joint_names'], [])
        self.assertEqual((arm_1['robot_id'], arm_2['robot_id']), ('robot_1', 'robot_2'))
        self.assertIs(arm_1['enable_simulation_arm'], False)
        self.assertIs(arm_2['enable_simulation_arm'], False)

        b4_1 = self.get('/ax/robot_1/b4_risk_node', ['robot_id', 'sensor_stale_s'])
        b4_2 = self.get('/ax/robot_2/b4_risk_node', ['robot_id', 'sensor_stale_s'])
        self.assertEqual((b4_1['robot_id'], b4_2['robot_id']), ('robot_1', 'robot_2'))
        self.assertEqual(b4_1['sensor_stale_s'], 0.25)

    # 매핑 없이 safety만 켜면 역할 미정·잠금, 허가 false
    def test_default_unconfirmed_and_locked(self):
        self.launch(['safety:=true', 'robot_ids:=robot_2'])
        gate = self.get('/ax/robot_2/a4_safety_gate',
                        ['stop_arm_presence', 'travel_joint_names', *GATE_FLAGS])
        self.assertEqual(gate['stop_arm_presence'], 'unconfirmed')
        self.assertEqual(gate['travel_joint_names'], [])
        for flag in GATE_FLAGS:
            self.assertIs(gate[flag], False, flag)
        arm = self.get('/ax/robot_2/a4_protected_arm_server', ['arm_joint_names', 'enable_simulation_arm'])
        self.assertEqual(arm['arm_joint_names'], [])
        self.assertIs(arm['enable_simulation_arm'], False)

    # 실제 CLI에서 미선언 하위 인자는 상위 context에 들어오며, 상위 진입점이 이를 거부한다
    def test_cli_global_alias_rejected(self):
        evidence = self.share / 'stop_evidence_ax2.yaml'
        for arguments in ([f'stop_evidence_file:={evidence}', 'safety:=true'],
                          [f'robot_params_file:={self.robot_a}', 'safety:=false'],
                          ['enable_simulation_drive:=true', 'safety:=true']):
            with self.subTest(arguments=arguments):
                result = subprocess.run(
                    ['ros2', 'launch', 'ax_control', 'simulation.launch.py', 'gui:=false', 'sensors:=false',
                     'world:=__ax_missing_world_for_test__.world'] + arguments,
                    env=os.environ.copy(), capture_output=True, text=True, timeout=60)
                output = result.stdout + result.stderr
                self.assertNotEqual(result.returncode, 0, output[-2000:])
                self.assertIn('하위 단일 로봇 인자를 받지 않는다', output, output[-2000:])
                self.assertIn(arguments[0].split(':=')[0], output)


if __name__ == '__main__':
    unittest.main()
