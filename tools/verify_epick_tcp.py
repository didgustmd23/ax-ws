#!/usr/bin/env python3
# 컵 끝면 앞에 기준 평면을 생성하여 ToF 거리와 TCP 기하 오프셋을 비교한다. 실물 컵 변형 보정은 아니다.
"""Calibrate the configured undeformed cup plane against Gazebo ToF returns."""
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from ament_index_python.packages import get_package_share_directory
from verify_a1_envelope import Observer
from arm_geometry import ArmGeometry, ARM_JOINTS
from w1_processes import stop_tagged

WORKSPACE = Path(__file__).resolve().parents[1]
output = WORKSPACE / 'log/a1'
output.mkdir(parents=True, exist_ok=True)
share = Path(get_package_share_directory('ax_description'))
epick = yaml.safe_load((share/'config/epick.yaml').read_text())
settings = yaml.safe_load((share/'config/a1_validation.yaml').read_text())
settings['limits'] = yaml.safe_load((share/'config/arm_joint_limits.yaml').read_text())['joint_limits']
partition = 'ax-w1-tcp-' + uuid.uuid4().hex
os.environ.update(GZ_PARTITION=partition, ROS_DOMAIN_ID=str(20+secrets.randbelow(76)),
                  ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', LIBGL_ALWAYS_SOFTWARE='1',
                  ROS_LOG_DIR=str(output/'ros_logs'))
rclpy.init()
geometry = ArmGeometry()
observer = Observer(geometry, settings)
readings = []
subscription = observer.create_subscription(Range, '/ax/robot_1/sensors/tool_tof/raw',
    lambda message: readings.append(message.range), qos_profile_sensor_data)
log = (output/'tcp_calibration.log').open('w')
process = subprocess.Popen(['ros2', 'launch', 'ax_control', 'simulation.launch.py',
    'robot_ids:=robot_1', 'gui:=false', 'tool:=epick'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
observer.launch_process = process
report = {'scope': 'geometric calibration of configured cup; not hardware compliance/pressure calibration', 'cases': []}
try:
    observer.wait(lambda: set(ARM_JOINTS) <= observer.positions.keys() and observer.base_pose is not None and bool(readings))
    observer.advance(1.0)
    for index, gap in enumerate((.10, .05, 0.0)):
        frames = geometry.frames(observer.current())
        pose = observer.base_pose
        world_base = np.eye(4)
        world_base[:3, :3] = Rotation.from_quat([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]).as_matrix()
        world_base[:3, 3] = [pose.position.x,pose.position.y,pose.position.z]
        tcp = world_base @ frames['epick_tcp']
        center = tcp[:3, 3] + tcp[:3, 2]*(gap+.005)
        angles = Rotation.from_matrix(tcp[:3,:3]).as_euler('xyz')
        name = 'tcp_reference_' + str(index)
        file = output / (name + '.sdf')
        file.write_text(f'''<sdf version="1.8"><model name="{name}"><static>true</static><link name="plate">
          <collision name="collision"><geometry><box><size>0.4 0.4 0.01</size></box></geometry></collision>
          <visual name="visual"><geometry><box><size>0.4 0.4 0.01</size></box></geometry></visual></link></model></sdf>''')
        subprocess.run(['ros2','run','ros_gz_sim','create','-world','ax_test','-name',name,'-file',str(file),
            '-x',str(center[0]),'-y',str(center[1]),'-z',str(center[2]),
            '-R',str(angles[0]),'-P',str(angles[1]),'-Y',str(angles[2])],
            check=True, stdout=log, stderr=subprocess.STDOUT, timeout=30)
        observer.advance(1.0)
        actual = float(np.median(readings[-10:]))
        expected = gap / epick.get('tof_normal_cosine', 1.0) + epick['tof_lip_offset_m']
        entry = {'gap_from_cup_lip_m': gap, 'expected_tof_m': expected, 'measured_tof_m': actual,
                 'bias_m': actual-expected, 'passed': math.isfinite(actual) and abs(actual-expected) < .002}
        report['cases'].append(entry)
        print(json.dumps(entry), flush=True)
    report['tcp_stroke_compensation_m'] = epick['tcp_stroke_compensation_m']
    report['passed'] = all(case['passed'] for case in report['cases'])
except Exception as error:
    report.update(passed=False, error=repr(error))
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
    report['cleanup'] = stop_tagged(partition)
    report['passed'] = report.get('passed', False) and report['cleanup']['remaining_count'] == 0
    log.close()
    (output/'tcp_calibration.json').write_text(json.dumps(report, indent=2)+'\n')
raise SystemExit(0 if report['passed'] else 1)
