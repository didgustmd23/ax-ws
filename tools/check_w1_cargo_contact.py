#!/usr/bin/env python3
"""A1/W1: 실행 중인 ax_test의 robot_2 적재판에 0.5 kg 치구를 놓고 원시 접촉을 확인한다."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from gz.transport13 import Node as GazeboNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts as GzContacts
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
import xacro
from arm_geometry import origin


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('log/w1/cargo_contact.json'))
    args = parser.parse_args()
    name = 'w1_cargo_probe_' + uuid.uuid4().hex[:8]
    model = ET.fromstring(xacro.process_file(str(Path(get_package_share_directory('ax_description')) /
                                                'urdf/ax_2.urdf.xacro')).toxml())
    frames, pending = {'base_link': np.eye(4)}, model.findall('joint')
    while pending:
        before = len(pending)
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent in frames:
                frames[joint.find('child').get('link')] = frames[parent] @ origin(joint.find('origin'))
                pending.remove(joint)
        if before == len(pending):
            raise ValueError('Disconnected robot_2 model')
    poses, samples, raw_contacts, ros_messages = {}, [], [], []
    transport = GazeboNode()

    def pose_received(message):
        for pose in message.pose:
            if pose.name in ('robot_2', name):
                poses[pose.name] = pose

    transport.subscribe(Pose_V, '/world/ax_test/dynamic_pose/info', pose_received)
    def raw_received(message):
        if message.contact and len(raw_contacts) < 3:
            raw_contacts.append(str(message))
    transport.subscribe(GzContacts, '/ax/robot_2/sensors/cargo_contact', raw_received)
    rclpy.init()
    node = rclpy.create_node('w1_cargo_contact_check')

    def contact_received(message):
        if len(ros_messages) < 3:
            ros_messages.append(str(message))
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if (any(name in value for value in names)
                    and any('robot_2::cargo_contact_link::' in value for value in names)):
                samples.append({'stamp_sec': message.header.stamp.sec,
                                'stamp_nanosec': message.header.stamp.nanosec,
                                'collisions': names})

    node.create_subscription(Contacts, '/ax/robot_2/sensors/cargo_contact',
                             contact_received, qos_profile_sensor_data)
    result = {'passed': False, 'fixture': name, 'mass_kg': .5,
              'scope': 'raw contact publication only; not A3 load state', 'samples': samples,
              'gazebo_contacts': raw_contacts, 'ros_messages': ros_messages}
    created = False
    try:
        deadline = time.monotonic() + 45
        while 'robot_2' not in poses and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if 'robot_2' not in poses:
            raise RuntimeError('robot_2 world pose not received')
        pose = poses['robot_2']
        q = pose.orientation
        world = np.eye(4)
        world[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        world[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        point = (world @ frames['cargo_link'] @ np.array([0., 0., .12, 1.]))[:3]
        result['spawn_position_m'] = point.tolist()
        sdf = f'''<sdf version="1.8"><model name="{name}"><pose>{point[0]} {point[1]} {point[2]} 0 0 0</pose>
        <link name="box"><inertial><mass>0.5</mass><inertia><ixx>0.003333333</ixx><iyy>0.003333333</iyy><izz>0.003333333</izz></inertia></inertial>
        <collision name="probe_collision"><geometry><box><size>.2 .2 .2</size></box></geometry></collision>
        <visual name="probe_visual"><geometry><box><size>.2 .2 .2</size></box></geometry></visual></link></model></sdf>'''
        created = True
        command = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'ax_test',
                                  '-name', name, '-string', sdf,
                                  '-x', str(point[0]), '-y', str(point[1]), '-z', str(point[2])],
                                 capture_output=True, text=True, timeout=20)
        result['create_output'] = command.stdout + command.stderr
        deadline = time.monotonic() + 45
        while len(samples) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        result['fixture_pose_seen'] = name in poses
        if name in poses:
            result['final_fixture_pose'] = str(poses[name])
        result['final_robot_pose'] = str(poses['robot_2'])
        stamps = {(s['stamp_sec'], s['stamp_nanosec']) for s in samples}
        result['passed'] = name in poses and len(stamps) >= 2
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if created:
            removed = subprocess.run(['gz', 'service', '-s', '/world/ax_test/remove', '--reqtype',
                                      'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                                      '--req', f'name: "{name}" type: MODEL'], capture_output=True, text=True, timeout=8)
            result['remove_output'] = removed.stdout + removed.stderr
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'passed': result['passed'], 'contact_samples': len(samples), 'error': result.get('error')}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
