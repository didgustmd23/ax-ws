#!/usr/bin/env python3
"""Survey rendered arm-camera visibility in frozen, test-only AX_1 poses.

Creates only labelled boards. Does not send arm, wheel, suction or navigation commands.
An occluded board is a measured limitation, not an extrinsic calibration failure.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from arm_geometry import ARM_JOINTS
from empty_test_fixture import write_empty_world


def freeze_pose(robot, positions):
    """Apply URDF joint motion after its origin, then freeze the copied model."""
    values = dict(zip(ARM_JOINTS, positions))
    for joint in robot.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        angle = values.get(joint.get('name'), 0.0)
        origin = joint.find('origin')
        if origin is None:
            origin = ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
        axis_node = joint.find('axis')
        axis = np.fromstring(axis_node.get('xyz', '1 0 0') if axis_node is not None else '1 0 0', sep=' ')
        axis /= np.linalg.norm(axis)
        if angle != 0:
            rotation = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
            origin.set('rpy', ' '.join(str(v) for v in (rotation * Rotation.from_rotvec(axis * angle)).as_euler('xyz')))
        joint.set('type', 'fixed')
        for child in list(joint):
            if child.tag != 'origin' and child.tag not in ('parent', 'child'):
                joint.remove(child)
    gazebo = ET.SubElement(robot, 'gazebo')
    ET.SubElement(gazebo, 'static').text = 'true'
    return robot


def prepare_fixture(pose, directory):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('ax_description'))
    config = yaml.safe_load((share / 'config/a1_validation.yaml').read_text())
    if pose not in config['poses']:
        raise ValueError('Unknown configured pose: ' + pose)
    positions = config['poses'][pose]
    robot = ET.fromstring(xacro.process_file(str(share / 'urdf/ax.urdf.xacro'), mappings={
        'robot_name': 'robot_1', 'sensor_robot_id': 'robot_1', 'frame_prefix': 'robot_1/',
        'sensors_enabled': 'true', 'gripper_model': 'epick', 'tool_attach': 'false',
        'ros2_control_params': ''}).toxml())
    xml = ET.tostring(freeze_pose(robot, positions), encoding='unicode')
    (directory / 'frozen_robot.urdf').write_text(xml)
    world = write_empty_world(directory)
    bridge = [{'ros_topic_name': '/clock', 'gz_topic_name': '/clock', 'ros_type_name': 'rosgraph_msgs/msg/Clock',
               'gz_type_name': 'gz.msgs.Clock', 'direction': 'GZ_TO_ROS'}]
    for name, suffix, cls, gz in [('arm_rgbd', 'depth_image', 'Image', 'Image'),
                                   ('arm_rgbd', 'camera_info', 'CameraInfo', 'CameraInfo'),
                                   ('arm_rgb', 'image_raw', 'Image', 'Image')]:
        topic = '/ax/robot_1/sensors/' + name + '/' + suffix
        bridge.append({'ros_topic_name': topic, 'gz_topic_name': topic, 'ros_type_name': 'sensor_msgs/msg/' + cls,
                       'gz_type_name': 'gz.msgs.' + gz, 'direction': 'GZ_TO_ROS', 'qos_profile': 'SENSOR_DATA'})
    (directory / 'bridge.yaml').write_text(yaml.safe_dump(bridge))
    (directory / 'pose.json').write_text(json.dumps({'pose': pose, 'joints': dict(zip(ARM_JOINTS, positions)),
                                                  'scope': 'Frozen geometry fixture; no actual arm movement.'}, indent=2))
    launch = f'''from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', {':'.join(str(Path(get_package_share_directory(name)).parent) for name in ('ax_description', 'epick_description', 'robotiq_description'))!r}),
        IncludeLaunchDescription(PythonLaunchDescriptionSource({str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')!r}), launch_arguments={{'gz_args': '-r -s --headless-rendering {world}', 'on_exit_shutdown': 'true'}}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='/ax/robot_1', parameters=[{{'robot_description': {xml!r}, 'frame_prefix': 'robot_1/', 'use_sim_time': True}}], remappings=[('tf','/tf'),('tf_static','/tf_static')]),
        Node(package='ros_gz_sim', executable='create', arguments=['-name','robot_1','-file',{str(directory / 'frozen_robot.urdf')!r},'-z','.42']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{{'config_file':{str(directory / 'bridge.yaml')!r},'use_sim_time':True}}], additional_env={{'FASTRTPS_DEFAULT_PROFILES_FILE':{str(share / 'config/fastdds_large_data.xml')!r}}}),
    ])
'''
    path = directory / 'visibility.launch.py'
    path.write_text(launch)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose', choices=['neutral', 'reach_left', 'reach_right'], required=True)
    parser.add_argument('--prepare-only', type=Path, help='Generate copied/frozen URDF and test launch, then exit')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.prepare_only:
        print(prepare_fixture(args.pose, args.prepare_only))
        return 0
    if args.output is None:
        parser.error('--output required for measurement')
    rclpy.init()
    node = rclpy.create_node('w2_arm_visibility', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    latest = {}
    for key, cls, suffix in [('depth', Image, 'arm_rgbd/depth_image'), ('info', CameraInfo, 'arm_rgbd/camera_info'),
                              ('rgb', Image, 'arm_rgb/image_raw')]:
        node.create_subscription(cls, '/ax/robot_1/sensors/' + suffix,
                                 lambda m, key=key: latest.update({key: m}), SENSOR)
    result = {'passed': False, 'pose': args.pose, 'scope': 'Rendered visibility at frozen configured geometry; no arm motion.',
              'distance_tolerance_m': .02, 'scene_settle_sim_s': 1.0, 'cases': []}

    def wait(predicate, timeout=45):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
            if predicate():
                return
        raise RuntimeError('Timed out waiting for visibility samples')

    def command(args_list):
        response = subprocess.run(args_list, capture_output=True, text=True, timeout=15)
        if response.returncode:
            raise RuntimeError(response.stderr or response.stdout)

    def remove(name):
        command(['gz', 'service', '-s', '/world/ax_test/remove', '--reqtype', 'gz.msgs.Entity',
                 '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', f'name: "{name}" type: MODEL'])

    active = None
    try:
        wait(lambda: len(latest) == 3 and buffer.can_transform('robot_1/base_link',
                                                              latest['depth'].header.frame_id, Time()))
        info = latest['info']
        tf = buffer.lookup_transform('robot_1/base_link', latest['depth'].header.frame_id, Time()).transform
        r = Rotation.from_quat([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]).as_matrix()
        origin = np.array([tf.translation.x, tf.translation.y, tf.translation.z + .42])
        for region, fraction in [('center', .5), ('left', .3), ('right', .7)]:
            u, v = int(info.width * fraction), int(info.height * .5)
            ray_optical = np.array([(u - info.k[2]) / info.k[0], (v - info.k[5]) / info.k[4], 1.])
            for depth in [.10, .15, .25]:
                surface = origin + r @ (ray_optical * depth)
                normal = r[:, 2]
                center = surface + normal * .005
                angles = Rotation.from_matrix(r @ Rotation.from_euler('y', -np.pi / 2).as_matrix()).as_euler('xyz')
                active = 'w2_visibility_' + uuid.uuid4().hex[:8]
                sdf = f'<sdf version="1.8"><model name="{active}"><static>true</static><link name="board"><collision name="collision"><geometry><box><size>.01 .06 .06</size></box></geometry></collision><visual name="visual"><geometry><box><size>.01 .06 .06</size></box></geometry><material><ambient>1 0 1 1</ambient><diffuse>1 0 1 1</diffuse></material></visual></link></model></sdf>'
                command(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'ax_test', '-name', active,
                         '-string', sdf, '-x', str(center[0]), '-y', str(center[1]), '-z', str(center[2]),
                         '-R', str(angles[0]), '-P', str(angles[1]), '-Y', str(angles[2])])
                after = stamp_seconds(latest['depth']) + result['scene_settle_sim_s']
                samples, seen = [], set()

                def collect():
                    msg = latest['depth']
                    stamp = stamp_seconds(msg)
                    if stamp <= after or stamp in seen:
                        return False
                    measured = float(np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data,
                                                strides=(msg.step, 4))[v, u])
                    samples.append({'stamp_s': stamp, 'measured_depth_m': measured if np.isfinite(measured) else None,
                                    'target_visible': bool(np.isfinite(measured) and abs(measured - depth) <= .02)})
                    seen.add(stamp)
                    return len(samples) >= 3

                wait(collect)
                visible = all(s['target_visible'] for s in samples)
                result['cases'].append({'region': region, 'pixel_uv': [u, v], 'target_depth_m': depth,
                                        'target_visible': visible, 'samples': samples})
                if region == 'center' and depth == .25:
                    rgb = latest['rgb']
                    image = np.ndarray((rgb.height, rgb.width, 3), dtype='u1', buffer=rgb.data,
                                       strides=(rgb.step, 3, 1))
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    target = args.output.with_name(args.output.stem + '_arm_rgb.png')
                    if not cv2.imwrite(str(target), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
                        raise RuntimeError('Failed to save arm RGB reference image')
                    result['rgb_reference_image'] = target.name
                remove(active)
                active = None
        # Completion is separate from visibility; an occluded board must never be called visible.
        result['fixture_control_passed'] = any(c['region'] == 'center' and c['target_depth_m'] == .10 and
                                              c['target_visible'] for c in result['cases'])
        result['survey_completed'] = len(result['cases']) == 9 and all(len(c['samples']) == 3 for c in result['cases'])
        result['passed'] = result['survey_completed'] and result['fixture_control_passed']
        result['visible_conditions'] = sum(c['target_visible'] for c in result['cases'])
        result['occluded_or_unobserved_conditions'] = 9 - result['visible_conditions']
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if active:
            remove(active)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


if __name__ == '__main__':
    raise SystemExit(main())
