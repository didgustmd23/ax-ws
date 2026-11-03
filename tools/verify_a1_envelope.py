#!/usr/bin/env python3
# EPick의 고정 하중 치구로 범위·유지·취소 정지를 검증한다. 실제 흡착 성공이나 실물 안전성을 인증하지 않는다.
"""A1 simulation range/load/hold/cancel tests; fixed payload is a test fixture."""
import argparse
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import time
import uuid

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gz.transport13 import Node as GazeboNode
from gz.msgs10.pose_v_pb2 import Pose_V
from ament_index_python.packages import get_package_share_directory
from arm_geometry import ArmGeometry, ARM_JOINTS
from w1_processes import stop_tagged

WORKSPACE = Path(__file__).resolve().parents[1]

class Observer(Node):
    # 시험 관측기: ROS 시각/관절과 Gazebo 실제 차체 자세를 함께 수집한다.
    def __init__(self, geometry, settings):
        super().__init__('a1_range_load_observer')
        self.geometry, self.settings = geometry, settings
        self.sim_time, self.positions, self.velocities = 0.0, {}, {}
        self.base_pose = None
        self.gazebo = GazeboNode()
        self.gazebo.subscribe(Pose_V, '/world/ax_test/dynamic_pose/info', self.pose)
        self.owned_subscriptions = [self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data),
            self.create_subscription(JointState, '/ax/robot_1/joint_states', self.state, qos_profile_sensor_data)]
        self.drive = self.create_publisher(TwistStamped, '/ax/robot_1/diff_drive_controller/cmd_vel', 10)
        self.arm = ActionClient(self, FollowJointTrajectory, '/ax/robot_1/arm_controller/follow_joint_trajectory')
        self.last_zero = 0.0

    # /clock을 초 단위로 변환한다. 시험 진행량은 시뮬레이션 시간으로 계산한다.
    def clock(self, message):
        self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    # 실제 joint_states를 이름별 위치(rad)·속도(rad/s)로 저장한다.
    def state(self, message):
        self.positions = dict(zip(message.name, message.position))
        self.velocities = dict(zip(message.name, message.velocity))

    # Gazebo 실제 robot_1 자세를 보관해 차체 기울기를 감시한다.
    def pose(self, message):
        for pose in message.pose:
            if pose.name == 'robot_1':
                self.base_pose = pose

    # 실행 생존을 확인하며 베이스 정지 명령을 유지한다. 기울기 0.25 rad 초과 시 시험을 중단한다.
    def spin(self):
        if self.launch_process.poll() is not None:
            raise RuntimeError('Gazebo launch exited')
        rclpy.spin_once(self, timeout_sec=.005)
        if time.monotonic() - self.last_zero > .03:
            message = TwistStamped()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(self.sim_time*1e9), 1000000000)
            self.drive.publish(message)
            self.last_zero = time.monotonic()
        if self.base_pose is not None:
            q = self.base_pose.orientation
            tilt = math.acos(max(-1, min(1, 1 - 2*(q.x*q.x+q.y*q.y))))
            if tilt > .25:
                raise RuntimeError('Base tilt exceeds 0.25 rad; test stopped')

    # 관측 조건을 기다리되 실제 시간 기준 제한을 두어 시뮬레이터 정지 시 무한 대기하지 않는다.
    def wait(self, predicate, timeout=180):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('A1 observation/action timed out')
            self.spin()

    # 시뮬레이션 시간이 지정 초만큼 진행하도록 기다린다.
    def advance(self, seconds):
        target = self.sim_time + seconds
        self.wait(lambda: self.sim_time >= target, max(45, seconds*10))

    # ROS 비동기 응답을 관측·정지 유지와 함께 기다린다.
    def future(self, future, timeout=180):
        self.wait(future.done, timeout)
        return future.result()

    # 공통 ARM_JOINTS 순서의 현재 위치(rad)를 돌려준다.
    def current(self):
        return [self.positions[joint] for joint in ARM_JOINTS]

    # 보수 경로 선별을 통과한 궤적만 전송하고 속도·가속도 기준으로 시험 시간을 정한다.
    def goal(self, target, seconds=None):
        start = self.current()
        screened = self.geometry.screen_path(start, target, step=.05)
        if not screened['allowed']:
            raise RuntimeError('Collision screening rejected path: ' + repr(screened))
        if seconds is None:
            seconds = 1.5
            for joint, first, last in zip(ARM_JOINTS, start, target):
                distance = abs(last-first)
                limits = self.settings['limits'][joint]
                seconds = max(seconds, 1.6*distance/limits['max_velocity'],
                              1.05*math.sqrt(6*distance/limits['max_acceleration']))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions, point.velocities = list(target), [0.0]*5
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(int(seconds*1e9), 1000000000)
        goal.trajectory.points = [point]
        handle = self.future(self.arm.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Trajectory goal rejected')
        return handle

    # 궤적 성공 후 유지 구간의 최대 관절 오차를 계산한다.
    def move_hold(self, label, target):
        handle = self.goal(target)
        response = self.future(handle.get_result_async())
        if response.status != 4 or response.result.error_code != 0:
            raise RuntimeError('Trajectory failed: ' + repr((response.status, response.result.error_code)))
        self.advance(.3)
        samples = []
        end = self.sim_time + self.settings['hold_seconds']
        while self.sim_time < end:
            self.spin()
            samples.append(self.current())
        maximum_error = float(np.max(np.abs(np.array(samples)-target)))
        result = {'case': label, 'target_rad': list(target), 'observed_rad': self.current(),
                  'hold_max_error_rad': maximum_error, 'samples': len(samples),
                  'passed': maximum_error <= self.settings['position_tolerance_rad']}
        print(json.dumps(result), flush=True)
        return result

    # 이동 도중 취소를 요청하고 실제 속도와 위치 드리프트를 별도로 관측한다.
    def stop_test(self):
        self.move_hold('before_stop_home', [0.0]*5)
        handle = self.goal([.4, -.3, .3, -.2, .2], seconds=6)
        self.advance(1.5)
        cancel = self.future(handle.cancel_goal_async())
        response = self.future(handle.get_result_async())
        self.advance(1.0)
        stopped = self.current()
        maximum_velocity, drift = 0.0, 0.0
        end = self.sim_time + 2.0
        while self.sim_time < end:
            self.spin()
            maximum_velocity = max(maximum_velocity, *(abs(self.velocities.get(joint, math.inf)) for joint in ARM_JOINTS))
            drift = max(drift, max(abs(a-b) for a,b in zip(self.current(), stopped)))
        result = {'cancel_accepted': bool(cancel.goals_canceling), 'action_status': response.status,
                  'max_stopped_velocity_rad_s': maximum_velocity, 'hold_drift_rad': drift,
                  'passed': bool(cancel.goals_canceling) and response.status == 5
                      and maximum_velocity < self.settings['stop_velocity_tolerance_rad_s']
                      and drift < self.settings['position_tolerance_rad']}
        print(json.dumps({'stop_test': result}), flush=True)
        return result

# 하중별 별도 partition/domain으로 시험을 실행하고 자기 프로세스만 정리한다.
def run_payload(payload, settings, output_directory):
    partition = 'ax-w1-a1-' + uuid.uuid4().hex
    os.environ.update(GZ_PARTITION=partition, ROS_DOMAIN_ID=str(20+secrets.randbelow(76)),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', LIBGL_ALWAYS_SOFTWARE='1',
                      ROS_LOG_DIR=str(output_directory / 'ros_logs'))
    geometry = ArmGeometry(payload)
    rclpy.init()
    observer = Observer(geometry, settings)
    log = (output_directory / f'payload_{payload}.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'ax_description', 'robot_test.launch.py',
        'robot_ids:=robot_1', 'gui:=false', 'sensors:=false', 'tool:=epick', f'payload_mass:={payload}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    observer.launch_process = process
    result = {'payload_kg': payload, 'fixture': 'fixed inertial test load, not a suction result', 'range_cases': [], 'load_cases': []}
    try:
        observer.wait(lambda: set(ARM_JOINTS) <= observer.positions.keys() and observer.base_pose is not None
                      and observer.arm.server_is_ready())
        observer.advance(1.0)
        if payload == 0:
            for index, joint in enumerate(ARM_JOINTS):
                for bound in geometry.limits[joint]:
                    target = [0.0]*5
                    # 0.01 rad inside the hard stop prevents numerical limit chatter.
                    target[index] = bound - math.copysign(.01, bound)
                    result['range_cases'].append(observer.move_hold(joint + '/' + str(bound), target))
                    observer.move_hold('range_return_home', [0.0]*5)
        for repetition in range(settings['repetitions']):
            for name, target in settings['poses'].items():
                result['load_cases'].append(observer.move_hold(f'{name}/repeat_{repetition+1}', target))
        result['stop'] = observer.stop_test()
        observer.move_hold('final_home', [0.0]*5)
        result['passed'] = (all(item['passed'] for item in result['range_cases']+result['load_cases']) and result['stop']['passed'])
    except Exception as error:
        result['passed'], result['error'] = False, repr(error)
        print('A1 test error:', repr(error), flush=True)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        result['cleanup'] = stop_tagged(partition)
        result['passed'] = result.get('passed', False) and result['cleanup']['remaining_count'] == 0
        log.close()
        (output_directory / f'payload_{payload}.json').write_text(json.dumps(result, indent=2)+'\n')
    return result

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--payload', type=float, choices=[0, .5, 1, 2])
    args = parser.parse_args()
    share = Path(get_package_share_directory('ax_description'))
    settings = yaml.safe_load((share/'config/a1_validation.yaml').read_text())
    settings['limits'] = yaml.safe_load((share/'config/arm_joint_limits.yaml').read_text())['joint_limits']
    output = WORKSPACE / 'log/a1'
    output.mkdir(parents=True, exist_ok=True)
    masses = settings['payloads_kg'] if args.payload is None else [args.payload]
    results = [run_payload(mass, settings, output) for mass in masses]
    passed = all(result['passed'] for result in results)
    (output/'envelope_summary.json').write_text(json.dumps({'passed': passed, 'results': results}, indent=2)+'\n')
    raise SystemExit(0 if passed else 1)
