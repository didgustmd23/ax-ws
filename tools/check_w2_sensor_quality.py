#!/usr/bin/env python3
"""Measure B1 stream rates, matched stamps and synchronization; never command motion.

Rate/missing estimates are observations, not an approved performance judgment.
Use a separate Gazebo/ROS test session with B1 already running.
"""
import argparse
from bisect import bisect_left
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2, Range
import yaml
from ament_index_python.packages import get_package_share_directory


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def stream_metrics(stamps, expected_hz):
    unique = sorted(set(stamps))
    gaps = np.diff(unique) / 1e9
    span = (unique[-1] - unique[0]) / 1e9 if len(unique) > 1 else 0.0
    rate = (len(unique) - 1) / span if span > 0 else None
    return {'received': len(stamps), 'unique_stamps': len(unique),
            'duplicates': len(stamps) - len(unique),
            'reversed': sum(b < a for a, b in zip(stamps, stamps[1:])),
            'expected_hz': expected_hz, 'sim_rate_hz': rate,
            'rate_deficit_fraction': max(0.0, 1.0 - rate / expected_hz) if rate else None,
            'max_gap_sim_s': float(max(gaps)) if len(gaps) else None,
            'p95_gap_sim_s': float(np.percentile(gaps, 95)) if len(gaps) else None}


def nearest_skew_s(reference, candidates):
    candidates = sorted(set(candidates))
    errors = []
    for value in sorted(set(reference)):
        index = bisect_left(candidates, value)
        nearby = candidates[max(0, index - 1):index + 1]
        if nearby:
            errors.append(min(abs(value - other) for other in nearby) / 1e9)
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', choices=['robot_1', 'robot_2'], required=True)
    parser.add_argument('--duration-s', type=float, default=15.0, help='ROS simulation seconds')
    parser.add_argument('--wall-timeout-s', type=float, default=120.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.duration_s <= 0 or args.wall_timeout_s <= 0:
        parser.error('Positive measurement durations required')
    contract = yaml.safe_load((Path(get_package_share_directory('ax_description')) /
                               'config/sensor_contract.yaml').read_text())
    rclpy.init()
    node = rclpy.create_node('w2_sensor_quality')
    clock = [None]
    node.create_subscription(Clock, '/clock', lambda m: clock.__setitem__(
        0, m.clock.sec + m.clock.nanosec / 1e9), 10)
    samples, frames, specs = {}, {}, {}
    imu_values = []
    invalid_imu = [0]
    root = '/ax/' + args.robot_id + '/sensors/'
    manipulator = args.robot_id == 'robot_1'

    def add(suffix, cls, hz):
        specs[suffix] = hz
        samples[suffix], frames[suffix] = [], set()

        def receive(message):
            samples[suffix].append(stamp_ns(message))
            frames[suffix].add(message.header.frame_id)
            if isinstance(message, Imu):
                row = [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                       message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z]
                if np.isfinite(row).all():
                    imu_values.append(row)
                else:
                    invalid_imu[0] += 1

        node.create_subscription(cls, root + suffix, receive, SENSOR)

    rgb_hz = contract['cameras']['rate_hz']
    for name in ['front_rgb'] + (['arm_rgb'] if manipulator else []):
        add(name + '/image_raw', Image, rgb_hz)
        add(name + '/processed/image_raw', Image, rgb_hz)
    cameras = ['front_rgbd'] + (['arm_rgbd'] if manipulator else [])
    for name in cameras:
        for suffix, cls in [('image', Image), ('depth_image', Image), ('camera_info', CameraInfo)]:
            add(name + '/' + suffix, cls, rgb_hz)
        add(name + '/processed/points', PointCloud2, rgb_hz)
    for name in ['front_lidar', 'rear_lidar']:
        add(name + '/scan', LaserScan, contract['lidar']['rate_hz'])
        add(name + '/scan_filtered', LaserScan, contract['lidar']['rate_hz'])
    add('imu', Imu, contract['imu']['rate_hz'])
    if manipulator:
        add('tool_tof/raw', Range, contract['tool_tof']['rate_hz'])
        add('tool_tof', Range, contract['tool_tof']['rate_hz'])
    result = {'passed': False, 'scope': 'B1 input/output integrity and measured stream quality',
              'robot_id': args.robot_id, 'performance_judgment': 'not_judged',
              'performance_note': 'No approved rate/loss threshold; observer stamp gaps are not proof of transport loss.'}
    try:
        deadline = time.monotonic() + args.wall_timeout_s
        while time.monotonic() < deadline and (clock[0] is None or not all(samples.values())):
            rclpy.spin_once(node, timeout_sec=.03)
        if clock[0] is None or not all(samples.values()):
            raise RuntimeError('Missing streams: ' + ', '.join(k for k, v in samples.items() if not v))
        start_sim, start_wall = clock[0], time.monotonic()
        for values in samples.values():
            values.clear()
        for values in frames.values():
            values.clear()
        imu_values.clear()
        invalid_imu[0] = 0
        while time.monotonic() < deadline and clock[0] - start_sim < args.duration_s:
            rclpy.spin_once(node, timeout_sec=.03)
        end_sim = clock[0]
        # Drain late in-flight samples, but evaluate only the common recorded stamp window.
        drain = time.monotonic() + .5
        while time.monotonic() < drain:
            rclpy.spin_once(node, timeout_sec=.03)
        for key in samples:
            samples[key] = [v for v in samples[key] if start_sim <= v / 1e9 <= end_sim]
        rows = {k: {**stream_metrics(v, specs[k]), 'frames': sorted(frames[k])}
                for k, v in samples.items()}
        pairs = {}
        sources = [('front_rgb/image_raw', 'front_rgb/processed/image_raw')]
        if manipulator:
            sources += [('arm_rgb/image_raw', 'arm_rgb/processed/image_raw'), ('tool_tof/raw', 'tool_tof')]
        sources += [(name + '/scan', name + '/scan_filtered') for name in ['front_lidar', 'rear_lidar']]
        sources += [(name + '/depth_image', name + '/processed/points') for name in cameras]
        for raw, processed in sources:
            a, b = set(samples[raw]), set(samples[processed])
            pairs[processed] = {'input_observed': len(a), 'output_observed': len(b),
                                'input_without_observed_output': len(a - b),
                                'output_without_observed_input': len(b - a),
                                'note': 'Independent subscriptions; unmatched observations can include observer loss.'}
        sync = {}
        for name in cameras:
            errors = nearest_skew_s(samples[name + '/depth_image'], samples[name + '/image'])
            errors += nearest_skew_s(samples[name + '/depth_image'], samples[name + '/camera_info'])
            sync[name] = {'matched_comparisons': len(errors), 'max_nearest_skew_s': max(errors) if errors else None,
                          'p95_nearest_skew_s': float(np.percentile(errors, 95)) if errors else None,
                          'over_30ms': sum(v > .03 for v in errors)}
        result.update(sim_duration_s=end_sim - start_sim, wall_duration_s=time.monotonic() - start_wall,
                      streams=rows, input_output_pairs=pairs, synchronization=sync)
        if imu_values:
            values = np.asarray(imu_values)
            result['imu_static_observation'] = {
                'finite_samples': len(imu_values), 'nonfinite_samples': invalid_imu[0],
                'mean_angular_velocity_rad_s': values[:, :3].mean(axis=0).tolist(),
                'mean_linear_acceleration_m_s2': values[:, 3:].mean(axis=0).tolist(),
                'std_linear_acceleration_m_s2': values[:, 3:].std(axis=0).tolist(),
                'scope': 'Stationary simulation observation; not hardware noise/bias calibration.'}
        result['passed'] = (end_sim - start_sim >= args.duration_s and
                            all(v['unique_stamps'] >= 3 and v['duplicates'] == 0 and v['reversed'] == 0 and
                                v['frames'] and all(f.startswith(args.robot_id + '/') for f in v['frames'])
                                for v in rows.values()) and
                            all(v['matched_comparisons'] > 0 and v['over_30ms'] == 0 for v in sync.values()) and
                            bool(imu_values) and invalid_imu[0] == 0)
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
