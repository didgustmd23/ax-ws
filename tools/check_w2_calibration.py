#!/usr/bin/env python3
"""B1 development calibration: two known planes, actual ROS depth/scan/cloud and TF.

Run against an isolated ax_test world and B1. Creates/removes only its own static
boards; sends no drive/arm commands. This is ideal simulator geometry, not hardware calibration.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
import numpy as np
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data as SENSOR
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from gz.transport13 import Node as GazeboNode
from gz.msgs10.pose_v_pb2 import Pose_V
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory


def finite_result(value):
    """Keep failed non-finite observations as null instead of losing the whole report."""
    if isinstance(value, dict):
        return {key: finite_result(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_result(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', choices=['robot_1', 'robot_2'], default='robot_2')
    parser.add_argument('--output', type=Path, default=Path('log/w2/calibration.json'))
    args = parser.parse_args()
    robot = args.robot_id
    mounts = yaml.safe_load((Path(get_package_share_directory('ax_description')) /
                            'config/sensor_mounts.yaml').read_text())['ax1' if robot == 'robot_1' else 'ax2']
    rclpy.init()
    node = rclpy.create_node('w2_calibration_check', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    tf = Buffer(node=node)
    listener = TransformListener(tf, node)
    poses, latest, records = {}, {}, []
    transport = GazeboNode()
    def pose_received(message):
        for pose in message.pose:
            if pose.name == robot:
                poses[robot] = pose
    transport.subscribe(Pose_V, '/world/ax_test/dynamic_pose/info', pose_received)
    root = '/ax/' + robot + '/sensors/'
    for key, cls, suffix in [('depth', Image, 'front_rgbd/processed/depth_image'),
                              ('rgb', Image, 'front_rgbd/processed/image'),
                              ('info', CameraInfo, 'front_rgbd/processed/camera_info'),
                              ('cloud', PointCloud2, 'front_rgbd/processed/points'),
                              ('scan', LaserScan, 'front_lidar/scan_filtered')]:
        node.create_subscription(cls, root + suffix,
            lambda message, key=key: latest.update({key: message}), SENSOR)
    def wait_for(predicate, timeout=60):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise RuntimeError('Timed out waiting for calibration observations')
    def remove(name):
        subprocess.run(['gz', 'service', '-s', '/world/ax_test/remove', '--reqtype',
            'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
            '--req', f'name: "{name}" type: MODEL'], capture_output=True, timeout=8, check=True)
    active = None
    result = {'passed': False, 'robot_id': robot, 'scope': 'ideal front sensor geometry only',
              'distance_tolerance_m': .015, 'scene_settle_sim_s': 1.0, 'cases': records}
    try:
        wait_for(lambda: robot in poses and len(latest) == 5)
        for distance in (1., 2.):
            pose = poses[robot]
            q = pose.orientation
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            translation = np.array([pose.position.x, pose.position.y, pose.position.z])
            camera = np.fromstring(mounts['front_rgbd']['xyz'], sep=' ')
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            normal = np.array([math.cos(yaw), math.sin(yaw), 0.])
            surface = translation + rotation @ (camera + [distance, 0, 0])
            center = surface + normal * .025
            active = 'w2_board_' + uuid.uuid4().hex[:8]
            sdf = f'''<sdf version="1.8"><model name="{active}"><static>true</static><link name="board">
              <collision name="collision"><geometry><box><size>.05 2 2</size></box></geometry></collision>
              <visual name="visual"><geometry><box><size>.05 2 2</size></box></geometry></visual>
              </link></model></sdf>'''
            subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'ax_test',
                '-name', active, '-string', sdf, '-x', str(center[0]), '-y', str(center[1]),
                '-z', str(center[2]), '-Y', str(yaw)], capture_output=True, timeout=20, check=True)
            # Ignore pre-fixture frames and let rendering catch up in sim time.
            start_stamp = latest['depth'].header.stamp
            after = start_stamp.sec + start_stamp.nanosec * 1e-9 + result['scene_settle_sim_s']
            samples, seen = [], set()
            def collect():
                depth, rgb, info, cloud, scan = [latest[k] for k in ('depth', 'rgb', 'info', 'cloud', 'scan')]
                stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in (depth, rgb, info, cloud)]
                if min(stamps) <= after or max(stamps)-min(stamps) > .03 or stamps[0] in seen:
                    return False
                if cloud.header.stamp != depth.header.stamp:
                    return False
                scan_time = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9
                if scan_time <= after or abs(scan_time-stamps[0]) > .15:
                    return False
                # Off-centre pixel also tests optical X-right/Y-down, not just Z.
                u, v = int(round(info.k[2]))+50, int(round(info.k[5]))+30
                z = float(np.ndarray((depth.height, depth.width), dtype=('>' if depth.is_bigendian else '<')+'f4',
                    buffer=depth.data, strides=(depth.step, 4))[v, u])
                xyz = np.frombuffer(bytes(cloud.data), dtype='<f4', count=3,
                    offset=v*cloud.row_step+u*cloud.point_step).astype(float)
                index = round(-scan.angle_min / scan.angle_increment)
                ray_angle = scan.angle_min + index * scan.angle_increment
                current = poses[robot]
                cq = current.orientation
                current_rotation = Rotation.from_quat([cq.x,cq.y,cq.z,cq.w]).as_matrix()
                current_translation = np.array([current.position.x,current.position.y,current.position.z])
                scan_origin = current_translation + current_rotation @ np.fromstring(mounts['front_lidar']['xyz'], sep=' ')
                scan_direction = current_rotation @ np.array([math.cos(ray_angle),math.sin(ray_angle),0.])
                expected_scan = float(np.dot(surface-scan_origin, normal) / np.dot(scan_direction,normal))
                dx,dy = (u-info.k[2])/info.k[0], (v-info.k[5])/info.k[4]
                camera_origin = current_translation + current_rotation @ camera
                depth_direction = current_rotation @ np.array([1.,-dx,-dy])
                expected_depth = float(np.dot(surface-camera_origin,normal)/np.dot(depth_direction,normal))
                measured_scan = float(scan.ranges[index])
                focal = info.width/(2*math.tan(math.pi/6))
                projected = np.array([dx*expected_depth,dy*expected_depth,expected_depth])
                passed = (abs(z-expected_depth) <= .015 and abs(measured_scan-expected_scan) <= .015
                    and np.max(np.abs(xyz-projected)) <= .015 and abs(info.k[0]-focal) < .1
                    and all(m.header.frame_id == robot+'/front_rgbd_optical_frame' for m in (depth,rgb,info,cloud)))
                samples.append({'stamp_s': stamps[0], 'skew_s': max(stamps)-min(stamps),
                    'depth_m': z, 'expected_depth_m': expected_depth,
                    'scan_m': measured_scan, 'expected_scan_m': expected_scan,
                    'cloud_xyz_m': xyz.tolist(), 'focal_px': info.k[0], 'passed': bool(passed)})
                seen.add(stamps[0])
                return len(samples) >= 3
            wait_for(collect)
            mount_checks = []
            for name in ('front_lidar', 'rear_lidar', 'front_rgbd', 'front_rgb'):
                transform = tf.lookup_transform(robot+'/base_link', robot+'/'+name+'_link', Time()).transform
                actual = np.array([transform.translation.x,transform.translation.y,transform.translation.z])
                expected = np.fromstring(mounts[name]['xyz'], sep=' ')
                quat = transform.rotation
                angle_error = (Rotation.from_euler('xyz', np.fromstring(mounts[name]['rpy'], sep=' ')).inv() *
                               Rotation.from_quat([quat.x,quat.y,quat.z,quat.w])).magnitude()
                mount_checks.append({'frame': name, 'translation_error_m': float(np.linalg.norm(actual-expected)),
                    'rotation_error_rad': float(angle_error)})
            records.append({'target_depth_m': distance, 'samples': samples, 'mount_checks': mount_checks,
                'passed': all(s['passed'] for s in samples) and all(m['translation_error_m'] < 1e-6
                    and m['rotation_error_rad'] < 1e-6 for m in mount_checks)})
            remove(active)
            active = None
        result['passed'] = all(case['passed'] for case in records)
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if active:
            remove(active)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        result = finite_result(result)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
