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
    process = subprocess.Popen(['ros2', 'launch', 'ax_control', 'simulation.launch.py',
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

def run_mobility(settings, output, payloads=None):
    """A1 모델 확인: 주행 자세와 수동 적재판. A3 적재 판정/A4 게이트 시험이 아니다."""
    import xml.etree.ElementTree as ET
    import xacro
    from scipy.spatial.transform import Rotation
    from gz.msgs10.contacts_pb2 import Contacts
    from arm_geometry import origin

    partition = 'ax-a1-mobility-' + uuid.uuid4().hex
    os.environ.update(GZ_PARTITION=partition, ROS_DOMAIN_ID=str(20+secrets.randbelow(76)),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', LIBGL_ALWAYS_SOFTWARE='1',
        ROS_LOG_DIR=str(output/'ros_logs'))
    # 모델/설정 오류는 시뮬레이터를 띄우기 전에 검출한다.
    geometry = ArmGeometry()
    target = settings['mobility']['robot_1']['travel_pose_rad']
    share = Path(get_package_share_directory('ax_description'))
    robot2 = ET.fromstring(xacro.process_file(str(share/'urdf/ax_2.urdf.xacro')).toxml())
    frames, pending = {'base_link': np.eye(4)}, robot2.findall('joint')
    while pending:
        count = len(pending)
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent in frames:
                frames[joint.find('child').get('link')] = frames[parent] @ origin(joint.find('origin'))
                pending.remove(joint)
        if len(pending) == count:
            raise ValueError('Disconnected robot_2 model')
    cargo = frames['cargo_link']
    rclpy.init()
    observer = Observer(geometry, settings)
    poses, contacts = {}, []
    def on_pose(message):
        for pose in message.pose:
            if pose.name in ('robot_1', 'robot_2', 'a1_cargo_probe'):
                poses[pose.name] = pose
    def on_contact(message):
        if message.contact:
            contacts.append(observer.sim_time)
    observer.gazebo.subscribe(Pose_V, '/world/ax_test/dynamic_pose/info', on_pose)
    observer.gazebo.subscribe(Contacts, '/ax/robot_2/sensors/cargo_contact', on_contact)
    drive2 = observer.create_publisher(TwistStamped, '/ax/robot_2/diff_drive_controller/cmd_vel', 10)
    log = (output/'mobility.log').open('a')
    log.write('\n=== A1 mobility run: payloads '+repr(payloads)+' ===\n')
    log.flush()
    launch = None
    result = {'scope': 'flat-floor development model; no operational safety or hardware rating', 'cases': []}
    def matrix(pose):
        m = np.eye(4)
        q = pose.orientation
        m[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        m[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return m
    def cargo_local():
        return np.linalg.inv(matrix(poses['robot_2']) @ cargo) @ matrix(poses['a1_cargo_probe'])
    try:
        launch = subprocess.Popen(['ros2', 'launch', 'ax_control', 'simulation.launch.py',
            'robots:=2', 'gui:=false', 'sensors:=false', 'tool:=epick'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        observer.launch_process = launch
        observer.wait(lambda: all(name in poses for name in ('robot_1', 'robot_2'))
            and set(ARM_JOINTS) <= observer.positions.keys() and observer.arm.server_is_ready())
        observer.advance(2.)
        observer.move_hold('travel_forward', target)
        profiles = [('robot_1', [0.]), ('robot_2', [0., .5, 1., 2.])] if payloads is None else [('robot_2', payloads)]
        for robot_id, masses in profiles:
            for mass in masses:
                if mass:
                    p = (matrix(poses['robot_2']) @ cargo)[:3, 3] + np.array([0., 0., .12])
                    inertia = mass*.2*.2/6
                    sdf = f'''<sdf version="1.8"><model name="a1_cargo_probe"><pose>{p[0]} {p[1]} {p[2]} 0 0 0</pose>
                    <link name="box"><inertial><mass>{mass}</mass><inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz></inertia></inertial>
                    <collision name="box"><geometry><box><size>.2 .2 .2</size></box></geometry></collision>
                    <visual name="box"><geometry><box><size>.2 .2 .2</size></box></geometry></visual></link></model></sdf>'''
                    # 외부 C++ CLI로 요청하여 고빈도 Python pose 콜백과 동기
                    # request가 경합하지 않게 한다. 실제 생성은 pose 수신으로 확인한다.
                    subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'ax_test',
                        '-name', 'a1_cargo_probe', '-string', sdf,
                        '-x', str(p[0]), '-y', str(p[1]), '-z', str(p[2])], check=True,
                        stdout=log, stderr=subprocess.STDOUT, timeout=30)
                    observer.wait(lambda: 'a1_cargo_probe' in poses)
                    observer.advance(2.)
                    local = cargo_local()
                    if abs(local[2, 3]-.1) > .01:
                        raise RuntimeError('Cargo deck failed to support box: ' + repr(local[:3, 3].tolist()))
                for label, linear, angular, seconds in [
                    ('left', 0., .3, 5.), ('right', 0., -.3, 5.),
                    ('forward', .67, 0., 2.), ('reverse', -.67, 0., 2.),
                    ('curve', .3, .3, 3.), ('reverse_curve', -.3, -.3, 3.)]:
                    start = matrix(poses[robot_id])
                    start_time = observer.sim_time
                    samples, cargo_samples = [], []
                    wall_deadline = time.monotonic()+max(60., (seconds+3.5)*30)
                    # observer.spin은 robot_1 정지 heartbeat를 발행하므로 주행 시에는
                    # 같은 루프에서 목표 명령을 뒤이어 발행한다.
                    while observer.sim_time < start_time+seconds+3.5:
                        if time.monotonic() > wall_deadline:
                            raise TimeoutError('Simulation clock stopped during mobility check')
                        if robot_id == 'robot_1':
                            observer.last_zero = time.monotonic()
                        observer.spin()
                        elapsed = observer.sim_time-start_time
                        message = TwistStamped()
                        message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(observer.sim_time*1e9), 1000000000)
                        if elapsed < seconds:
                            message.twist.linear.x, message.twist.angular.z = linear, angular
                        (observer.drive if robot_id == 'robot_1' else drive2).publish(message)
                        samples.append(matrix(poses[robot_id]))
                        if mass:
                            cargo_samples.append(cargo_local())
                    end = samples[-1]
                    tilt = max(math.acos(np.clip(m[2, 2], -1., 1.)) for m in samples)
                    travel_error = max(abs(a-b) for a, b in zip(observer.current(), target))
                    displacement = float(np.linalg.norm((end-start)[:2, 3]))
                    drift = max((float(np.linalg.norm(m[:2, 3])) for m in cargo_samples), default=0.)
                    height_error = max((abs(float(m[2, 3])-.1) for m in cargo_samples), default=0.)
                    recent_contact = any(t >= start_time for t in contacts)
                    moved = displacement > (.5 if label in ('forward', 'reverse') else .3) if linear else True
                    passed = moved and tilt < .15 and travel_error < .03 and (label not in ('left', 'right') or displacement < .1) and (not mass or
                        drift < .02 and height_error < .01 and recent_contact)
                    case = dict(robot=robot_id, payload_kg=mass, case=label,
                        displacement_m=displacement, max_tilt_rad=tilt,
                        cargo_drift_m=drift, cargo_height_error_m=height_error,
                        contact_seen=recent_contact if mass else None, passed=passed)
                    result['cases'].append(case)
                    print(json.dumps(case), flush=True)
                    if not passed:
                        raise RuntimeError('Mobility support check failed')
                if mass:
                    entity_id = poses['a1_cargo_probe'].id
                    removed = subprocess.run(['gz', 'service', '-s', '/world/ax_test/remove',
                        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '10000', '--req', f'id: {entity_id} type: MODEL'],
                        check=True, capture_output=True, text=True, timeout=15)
                    if 'data: true' not in removed.stdout:
                        raise RuntimeError('Cargo test box removal failed: '+removed.stdout+removed.stderr)
                    poses.pop('a1_cargo_probe', None)
                    observer.advance(.3)
        result['passed'] = all(case['passed'] for case in result['cases'])
    except Exception as error:
        result.update(passed=False, error=repr(error))
        print('A1 mobility error:', repr(error), flush=True)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
        if launch is not None:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
        result['cleanup'] = stop_tagged(partition)
        result['passed'] = result.get('passed', False) and result['cleanup']['remaining_count'] == 0
        log.close()
        filename = 'mobility_summary.json' if payloads is None else 'mobility_payload_'+'_'.join(map(str, payloads))+'.json'
        previous = output/filename
        if previous.is_file():
            old = json.loads(previous.read_text())
            attempts = old.pop('previous_attempts', [])
            result['previous_attempts'] = attempts+[old]
        (output/filename).write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--payload', type=float, choices=[0, .5, 1, 2])
    parser.add_argument('--mobility', action='store_true', help='A1 travel pose and robot_2 passive cargo support checks')
    parser.add_argument('--mobility-payloads', nargs='+', type=float, choices=[0, .5, 1, 2],
                        help='With --mobility, recheck only these robot_2 loads')
    args = parser.parse_args()
    if args.mobility_payloads is not None and not args.mobility:
        parser.error('--mobility-payloads requires --mobility')
    if args.mobility and args.payload is not None:
        parser.error('--payload and --mobility are separate tests')
    share = Path(get_package_share_directory('ax_description'))
    settings = yaml.safe_load((share/'config/a1_validation.yaml').read_text())
    settings['limits'] = yaml.safe_load((share/'config/arm_joint_limits.yaml').read_text())['joint_limits']
    output = WORKSPACE / 'log/a1'
    output.mkdir(parents=True, exist_ok=True)
    if args.mobility:
        raise SystemExit(0 if run_mobility(settings, output, args.mobility_payloads)['passed'] else 1)
    masses = settings['payloads_kg'] if args.payload is None else [args.payload]
    results = [run_payload(mass, settings, output) for mass in masses]
    passed = all(result['passed'] for result in results)
    (output/'envelope_summary.json').write_text(json.dumps({'passed': passed, 'results': results}, indent=2)+'\n')
    raise SystemExit(0 if passed else 1)
