#!/usr/bin/env python3
# 제어 명령뿐 아니라 Gazebo 실제 자세·관절·LiDAR를 관측한다. 자체 partition과 프로세스만 정리한다.
"""Check actual Gazebo motion and all four wheel signs on each robot."""
import argparse
import json
import math
import os
from pathlib import Path
import secrets
import signal
import statistics
import subprocess
import time
import uuid
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from controller_manager_msgs.srv import ListControllers
from ament_index_python.packages import get_package_share_directory
from w1_processes import stop_tagged
from gz.transport13 import Node as GazeboNode
from gz.msgs10.pose_v_pb2 import Pose_V
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

WORKSPACE = Path(__file__).resolve().parents[1]
# 기본 속도와 정지 대기 시간을 실제 제어기 설정에서 읽는다.
_control = yaml.safe_load((WORKSPACE / 'src/ax_control/config/common_controllers.yaml').read_text(encoding='utf-8'))
_drive = _control['/**/diff_drive_controller']['ros__parameters']
_teleop = _control['/**/ax_keyboard_teleop']['ros__parameters']
LINEAR_SPEED = _teleop['default_linear_speed']
ANGULAR_SPEED = _teleop['default_angular_speed']
STOP_SETTLE_SECONDS = max(LINEAR_SPEED / abs(_drive['linear.x.max_deceleration']),
                          ANGULAR_SPEED / abs(_drive['angular.z.max_deceleration'])) + .3

WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint',
          'rear_left_wheel_joint', 'rear_right_wheel_joint']
ARM_JOINTS = ['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']
ARM_CHILDREN = ['column_link', 'upper_arm_link', 'forearm_link', 'wrist_link', 'gripper_base_link']


# Gazebo quaternion에서 수평 방향 yaw(rad)를 추출한다.
def yaw(rotation):
    return math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                      1 - 2 * (rotation.y ** 2 + rotation.z ** 2))


class DriveObserver(Node):
    # 명령과 독립된 Gazebo 자세·관절·LiDAR 관측을 준비한다.
    def __init__(self):
        super().__init__('independent_drive_direction_observer')
        self.sim_time = 0.0
        self.poses = {}
        self.joints = {}
        self.pose_names = set()
        self.pose_debug = {}
        self.link_poses = {}
        self.scans = {}
        self.gazebo_node = GazeboNode()
        self.gazebo_node.subscribe(Pose_V, '/world/ax_test/dynamic_pose/info', self.on_gazebo_pose)
        self.subscriptions_owned = [
            self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)]
        self.publishers_owned, self.controller_clients = {}, {}
        for robot_id in ('robot_1', 'robot_2'):
            self.publishers_owned[robot_id] = self.create_publisher(TwistStamped,
                f'/ax/{robot_id}/diff_drive_controller/cmd_vel', 10)
            self.subscriptions_owned.append(self.create_subscription(JointState,
                f'/ax/{robot_id}/joint_states', lambda message, robot=robot_id: self.on_joint(robot, message),
                qos_profile_sensor_data))
            self.controller_clients[robot_id] = self.create_client(ListControllers,
                f'/ax/{robot_id}/controller_manager/list_controllers')
            for sensor in ('front_lidar', 'rear_lidar'):
                scan_key = robot_id + '/' + sensor
                self.subscriptions_owned.append(self.create_subscription(LaserScan,
                    f'/ax/{robot_id}/sensors/{sensor}/scan',
                    lambda message, key=scan_key: self.scans.__setitem__(key, message), qos_profile_sensor_data))
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/ax/robot_1/arm_controller/follow_joint_trajectory')

    # 시뮬레이션 시각을 저장해 속도 메시지 stamp와 시험 진행에 사용한다.
    def on_clock(self, message):
        self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    # Gazebo의 실제 로봇·팔 링크 자세를 갱신한다. odom 적분값만으로 성공을 판단하지 않는다.
    def on_gazebo_pose(self, message):
        for pose in message.pose:
            name = pose.name
            self.pose_names.add(name)
            if name in ARM_CHILDREN:
                self.link_poses[name] = [pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            if name in ('robot_1', 'robot_2', 'base_link'):
                self.pose_debug[pose.id] = {'name': name, 'id': pose.id,
                    'xyz': [pose.position.x, pose.position.y, pose.position.z],
                    'yaw': yaw(pose.orientation),
                    'header': str(pose.header)}
            if name in ('robot_1', 'robot_2'):
                self.poses[name] = (pose.position.x, pose.position.y, pose.position.z, yaw(pose.orientation))

    # 로봇별 실제 관절 상태를 이름으로 구분해 저장한다.
    def on_joint(self, robot_id, message):
        self.joints[robot_id] = dict(zip(message.name, message.position))

    # 현재 시험 속도를 시뮬레이션 시각의 TwistStamped로 반복 전송한다.
    def command(self, robot_id=None, linear=0.0, angular=0.0):
        stamp = int(self.sim_time * 1e9)
        for robot, publisher in self.publishers_owned.items():
            message = TwistStamped()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1000000000)
            message.header.frame_id = robot + '/base_link'
            if robot == robot_id:
                message.twist.linear.x = linear
                message.twist.angular.z = angular
            publisher.publish(message)

    # ROS 관측과 명령 갱신을 유지하며 지정 시뮬레이션 시간만큼 진행한다.
    def advance(self, duration, robot_id=None, linear=0.0, angular=0.0):
        end = self.sim_time + duration
        deadline = time.monotonic() + max(45, duration * 30)
        last_command = 0.0
        while self.sim_time < end:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation did not advance during drive test')
            rclpy.spin_once(self, timeout_sec=.01)
            if time.monotonic() - last_command >= .02:
                self.command(robot_id, linear, angular)
                last_command = time.monotonic()

    # 시험에 필요한 로봇·시각·센서 관측이 준비될 때까지 기다린다.
    def wait_ready(self):
        deadline = time.monotonic() + 150
        progress_at = time.monotonic()
        active, pending = set(), {}
        while time.monotonic() < deadline:
            if self.launch_process.poll() is not None:
                raise RuntimeError('Robot launch exited; inspect the drive test log')
            rclpy.spin_once(self, timeout_sec=.05)
            if time.monotonic() - progress_at > 15:
                print(json.dumps({'sim_time': self.sim_time, 'active_robots': sorted(active),
                    'pose_names': sorted(self.pose_names)[:12], 'joints': sorted(self.joints)}), flush=True)
                progress_at = time.monotonic()
            if active:
                self.command()
            for robot, client in self.controller_clients.items():
                if robot not in active and robot not in pending and client.service_is_ready():
                    pending[robot] = client.call_async(ListControllers.Request())
            for robot, future in list(pending.items()):
                if future.done():
                    required = {'joint_state_broadcaster', 'diff_drive_controller'}
                    if robot == 'robot_1':
                        required.add('arm_controller')
                    states = {item.name: item.state for item in future.result().controller}
                    if all(states.get(name) == 'active' for name in required):
                        active.add(robot)
                    del pending[robot]
            if (len(active) == 2 and len(self.poses) == 2 and len(self.joints) == 2
                    and all(set(WHEELS) <= set(state) for state in self.joints.values())):
                self.advance(.5)
                return
        raise RuntimeError('Readiness timed out: poses=' + repr(self.poses) + ', pose names=' + repr(sorted(self.pose_names)[:30]))

    # 전후진 거리와 회전 방향을 Gazebo 실제 자세 변화로 판정한다.
    def trial(self, robot_id, name, linear, angular):
        self.advance(.3)
        start_pose = self.poses[robot_id]
        start_joints = dict(self.joints[robot_id])
        self.advance(1.5, robot_id, linear, angular)
        self.command()
        self.advance(STOP_SETTLE_SECONDS)
        end_pose = self.poses[robot_id]
        forward_distance = ((end_pose[0] - start_pose[0]) * math.cos(start_pose[3])
                            + (end_pose[1] - start_pose[1]) * math.sin(start_pose[3]))
        yaw_change = math.atan2(math.sin(end_pose[3] - start_pose[3]), math.cos(end_pose[3] - start_pose[3]))
        wheel_delta = {wheel: self.joints[robot_id][wheel] - start_joints[wheel] for wheel in WHEELS}
        left = statistics.mean(wheel_delta[wheel] for wheel in WHEELS if '_left_' in wheel)
        right = statistics.mean(wheel_delta[wheel] for wheel in WHEELS if '_right_' in wheel)
        if linear:
            passed = forward_distance * linear > .004 and left * linear > .005 and right * linear > .005
        else:
            passed = yaw_change * angular > .01 and left * angular < -.005 and right * angular > .005
        result = {'robot_id': robot_id, 'trial': name, 'command_linear_m_s': linear,
                  'command_angular_rad_s': angular, 'forward_distance_m': forward_distance,
                  'yaw_change_rad': yaw_change, 'wheel_delta_rad': wheel_delta,
                  'start_world_pose_xyzyaw': start_pose, 'end_world_pose_xyzyaw': end_pose,
                  'gazebo_pose_debug': list(self.pose_debug.values()),
                  'passed': passed}
        print(json.dumps(result), flush=True)
        return result

    # 4개 스캔의 근거리 반환·NaN·음의 무한대를 기록한다. 정상 장애물 반환 전체를 제거하지 않는다.
    def lidar_snapshot(self, label):
        self.advance(.4)
        readings = {}
        for key, scan in self.scans.items():
            finite = [float(value) for value in scan.ranges if math.isfinite(value)]
            close = [{'angle_deg': math.degrees(scan.angle_min + index * scan.angle_increment), 'range_m': float(value)}
                     for index, value in enumerate(scan.ranges) if math.isfinite(value) and value < .8]
            readings[key] = {'minimum_finite_range_m': min(finite) if finite else None,
                             'close_returns_under_0_8m': close, 'frame_id': scan.header.frame_id,
                             'negative_infinity_count': sum(value == -math.inf for value in scan.ranges),
                             'nan_count': sum(math.isnan(value) for value in scan.ranges),
                             'sample_count': len(scan.ranges)}
        result = {'label': label, 'readings': readings,
                  'passed': len(readings) == 4 and all(not value['close_returns_under_0_8m']
                      and not value['negative_infinity_count'] and not value['nan_count'] for value in readings.values())}
        print(json.dumps({'lidar_snapshot': label, 'passed': result['passed'],
                          'minima': {key: value['minimum_finite_range_m'] for key, value in readings.items()}}), flush=True)
        return result

    # 팔 Action 응답을 기다리는 동안 관측과 베이스 정지 명령을 유지한다.
    def await_future(self, future, timeout=90):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError('Arm action timed out')
            rclpy.spin_once(self, timeout_sec=.01)
            self.command()
        return future.result()

    # 팔 각 축을 0.15 rad 이동하고 실제 관절·자식 링크 회전을 확인한 뒤 중립으로 복귀한다.
    def check_arm(self):
        if not self.arm_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Arm action server is unavailable')
        targets = [0.0] * 5
        results, scans = [], [self.lidar_snapshot('before_arm_motion')]
        for index, joint in enumerate(ARM_JOINTS):
            child = ARM_CHILDREN[index]
            before = self.joints['robot_1'][joint]
            child_before = self.link_poses[child]
            targets[index] = .15
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM_JOINTS
            point = JointTrajectoryPoint()
            point.positions = targets[:]
            point.time_from_start.sec = 3
            goal.trajectory.points = [point]
            handle = self.await_future(self.arm_client.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Arm goal rejected: ' + joint)
            response = self.await_future(handle.get_result_async())
            self.advance(.3)
            actual = [self.joints['robot_1'][name] for name in ARM_JOINTS]
            quaternion_dot = abs(sum(a*b for a, b in zip(child_before[3:], self.link_poses[child][3:])))
            angle_changed = 2 * math.acos(min(1.0, quaternion_dot))
            passed = (response.result.error_code == 0 and max(abs(a-b) for a, b in zip(actual, targets)) < .03
                      and actual[index] - before > .1 and angle_changed > .1)
            entry = {'joint': joint, 'targets_rad': targets[:], 'actual_rad': actual,
                     'action_status': response.status, 'error_code': response.result.error_code,
                     'child_link': child, 'observed_child_rotation_rad': angle_changed, 'passed': passed}
            results.append(entry)
            print(json.dumps(entry), flush=True)
            scans.append(self.lidar_snapshot('after_' + joint))
        # Restore the neutral configuration before shutdown.
        goal.trajectory.points[0].positions = [0.0] * 5
        handle = self.await_future(self.arm_client.send_goal_async(goal))
        if not handle.accepted or self.await_future(handle.get_result_async()).result.error_code != 0:
            raise RuntimeError('Arm return to neutral failed')
        return results, scans

    # 센서 앞뒤에 구형 표적을 배치해 가까운 자기 반환 제거와 실제 표적 검출을 구분한다.
    def check_lidar_targets(self, directory, log):
        mounts = yaml.safe_load((Path(get_package_share_directory('ax_description')) / 'config/sensor_mounts.yaml').read_text())
        for robot_id, profile in [('robot_1', 'ax1'), ('robot_2', 'ax2')]:
            base_x, base_y, base_z, heading = self.poses[robot_id]
            for sensor, direction in [('front_lidar', 1), ('rear_lidar', -1)]:
                mount_x, mount_y, mount_z = map(float, mounts[profile][sensor]['xyz'].split())
                local_x = mount_x + direction * 2.0
                target_x = base_x + math.cos(heading) * local_x - math.sin(heading) * mount_y
                target_y = base_y + math.sin(heading) * local_x + math.cos(heading) * mount_y
                name = f'lidar_target_{robot_id}_{sensor}'
                file = directory / (name + '.sdf')
                file.write_text(f'''<sdf version="1.8"><model name="{name}"><static>true</static>
                  <link name="target"><collision name="collision"><geometry><sphere><radius>0.15</radius></sphere></geometry></collision>
                  <visual name="visual"><geometry><sphere><radius>0.15</radius></sphere></geometry></visual></link></model></sdf>''')
                subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'ax_test', '-name', name,
                    '-file', str(file), '-x', str(target_x), '-y', str(target_y), '-z', str(base_z + mount_z)],
                    stdout=log, stderr=subprocess.STDOUT, timeout=30, check=True)
        self.advance(1.0)
        checks = []
        for key, scan in sorted(self.scans.items()):
            center = sorted(range(len(scan.ranges)), key=lambda index: abs(scan.angle_min + index * scan.angle_increment))[:2]
            values = [float(scan.ranges[index]) for index in center]
            passed = all(math.isfinite(value) and abs(value - 1.85) < .03 for value in values)
            checks.append({'sensor': key, 'expected_surface_distance_m': 1.85,
                           'measured_center_ranges_m': values, 'passed': passed})
        print(json.dumps({'lidar_target_checks': checks}), flush=True)
        return checks


# 격리된 시험 세션을 실행하고 결과·잔류 프로세스 수를 기록한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=str(WORKSPACE / 'log/repair/drive_directions.json'))
    parser.add_argument('--robot-id', choices=['all', 'robot_1', 'robot_2'], default='all')
    parser.add_argument('--trial', choices=['all', 'forward', 'reverse', 'turn_left', 'turn_right'], default='all')
    parser.add_argument('--arm-and-lidar', action='store_true')
    parser.add_argument('--sensors-only', action='store_true')
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    partition = 'ax-w1-drive-' + uuid.uuid4().hex
    os.environ.update(GZ_PARTITION=partition, ROS_DOMAIN_ID=str(20 + secrets.randbelow(76)),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', LIBGL_ALWAYS_SOFTWARE='1',
                      ROS_LOG_DIR=str(output.parent / 'ros_logs'))
    log = output.with_suffix('.log').open('w')
    rclpy.init()
    observer = DriveObserver()
    process = subprocess.Popen(['ros2', 'launch', 'ax_control', 'simulation.launch.py',
        'robots:=2', 'gui:=false', 'world:=ax_test.world'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    observer.launch_process = process
    report = {'passed': False, 'installed_package': get_package_share_directory('ax_description'), 'trials': []}
    try:
        observer.wait_ready()
        report['lidar_before_drive'] = observer.lidar_snapshot('before_drive')
        for robot_id in ('robot_1', 'robot_2'):
            if args.sensors_only:
                break
            if args.robot_id != 'all' and args.robot_id != robot_id:
                continue
            for name, linear, angular in [('forward', LINEAR_SPEED, 0.0), ('reverse', -LINEAR_SPEED, 0.0),
                                          ('turn_left', 0.0, ANGULAR_SPEED), ('turn_right', 0.0, -ANGULAR_SPEED)]:
                if args.trial != 'all' and args.trial != name:
                    continue
                report['trials'].append(observer.trial(robot_id, name, linear, angular))
        report['passed'] = (all(trial['passed'] for trial in report['trials'])
                            and report['lidar_before_drive']['passed'])
        if args.arm_and_lidar:
            report['arm_trials'], report['lidar_snapshots'] = observer.check_arm()
            report['passed'] = (report['passed'] and all(trial['passed'] for trial in report['arm_trials'])
                                and all(snapshot['passed'] for snapshot in report['lidar_snapshots']))
        if args.sensors_only or args.arm_and_lidar:
            target_directory = output.parent / 'lidar_targets'
            target_directory.mkdir(exist_ok=True)
            report['lidar_obstacle_checks'] = observer.check_lidar_targets(target_directory, log)
            report['passed'] = report['passed'] and all(check['passed'] for check in report['lidar_obstacle_checks'])
    except Exception as error:
        report['error'] = repr(error)
        print(report['error'], flush=True)
    finally:
        observer.command()
        observer.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        report['cleanup'] = stop_tagged(partition)
        report['passed'] = report['passed'] and report['cleanup']['remaining_count'] == 0
        log.close()
        output.write_text(json.dumps(report, indent=2) + '\n')
    print('Drive direction checks passed:', report['passed'])
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
