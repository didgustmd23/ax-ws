#!/usr/bin/env python3
# 이미 실행 중인 통합 월드를 읽기 전용으로 검사한다. 새 로봇 생성이나 주행 명령은 보내지 않는다.
"""Read-only smoke check of an already running two-robot test world."""
import argparse
import json
import math
import time
from pathlib import Path
import subprocess
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from controller_manager_msgs.srv import ListControllers

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path)
parser.add_argument('--world', choices=['ax_test.world', 'planned/ax_warehouse.world', 'aws_warehouse.world'], default='ax_test.world')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('ax_unified_runtime_check')
readings, clients = {}, {}
for robot in ('robot_1', 'robot_2'):
    topic = f'/ax/{robot}/joint_states'
    node.create_subscription(JointState, topic,
        lambda msg, topic=topic: readings.update({topic: {'joints': list(msg.name),
            'finite': all(math.isfinite(value) for value in msg.position)}}), 10)
    clients[robot] = node.create_client(ListControllers, f'/ax/{robot}/controller_manager/list_controllers')
    for side in ('front', 'rear'):
        topic = f'/ax/{robot}/sensors/{side}_lidar/scan'
        node.create_subscription(LaserScan, topic,
            lambda msg, topic=topic: readings.update({topic: {'samples': len(msg.ranges),
                'nan': sum(math.isnan(value) for value in msg.ranges)}}), qos_profile_sensor_data)
futures = {}
controllers = {}
deadline = time.monotonic() + 90
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
    for robot, client in clients.items():
        if robot not in futures and client.service_is_ready():
            futures[robot] = client.call_async(ListControllers.Request())
    if len(futures) == 2 and all(f.done() for f in futures.values()):
        controllers = {robot: {c.name: c.state for c in f.result().controller}
                       for robot, f in futures.items() if f.result()}
        # 시작 직후 빈 목록도 반환된다. 빈 목록을 준비 완료로 보지 않고 다시 조회한다.
        if len(readings) == 6 and all(all(controllers.get(robot, {}).get(name) == 'active'
                for name in (('joint_state_broadcaster', 'diff_drive_controller', 'arm_controller')
                             if robot == 'robot_1' else ('joint_state_broadcaster', 'diff_drive_controller')))
                for robot in clients):
            break
        futures.clear()
scene = subprocess.run(['gz', 'service', '-s', '/world/ax_test/scene/info',
    '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene', '--timeout', '5000', '--req', ''],
    capture_output=True, text=True, check=True).stdout
names = (('robot_1', 'robot_2', 'aws_robomaker_warehouse_ShelfF_01_001', 'aws_robomaker_warehouse_GroundB_01_001')
         if args.world == 'aws_warehouse.world' else
         ('robot_1', 'robot_2', 'station_src', 'station_dst', 'test_box', 'ground_plane'))
entities = {name: f'name: "{name}"' in scene for name in names}
passed = len(readings) == 6 and all(entities.values())
for robot in ('robot_1', 'robot_2'):
    expected = {'joint_state_broadcaster', 'diff_drive_controller'}
    if robot == 'robot_1': expected.add('arm_controller')
    passed = passed and all(controllers.get(robot, {}).get(c) == 'active' for c in expected)
    state = readings.get(f'/ax/{robot}/joint_states', {})
    expected_joints = {f'{end}_{side}_wheel_joint' for end in ('front', 'rear') for side in ('left', 'right')}
    if robot == 'robot_1':
        expected_joints.update(['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint'])
    passed = passed and state.get('finite', False) and expected_joints <= set(state.get('joints', []))
    for side in ('front', 'rear'):
        scan = readings.get(f'/ax/{robot}/sensors/{side}_lidar/scan', {})
        passed = passed and scan.get('samples', 0) == 360 and scan.get('nan', 1) == 0
result = {'passed': bool(passed), 'entities': entities, 'controllers': controllers, 'topics': readings}
if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if passed else 1)
