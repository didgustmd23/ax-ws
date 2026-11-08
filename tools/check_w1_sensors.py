#!/usr/bin/env python3
"""W1 모델 센서 발행 확인. 실행 중인 월드를 구독하며 구동 명령은 보내지 않는다."""
import argparse
import importlib.util
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ids', default='robot_1,robot_2')
    parser.add_argument('--tool', default='epick', choices=['epick', 'suction', 'robotiq', 'primitive', 'parallel_wide'])
    parser.add_argument('--timeout', type=float, default=60.)
    parser.add_argument('--output', type=Path, default=Path('log/w1/sensors.json'))
    args = parser.parse_args()
    robots = args.robot_ids.split(',')
    if (not robots or len(set(robots)) != len(robots)
            or any(r not in ('robot_1', 'robot_2', 'robot_3', 'robot_4') for r in robots)
            or not 0 < args.timeout < float('inf')):
        parser.error('unique robot_1..4 and a finite positive timeout required')
    path = Path(__file__).resolve().parents[1] / 'src/ax_description/launch/robot_test.launch.py'
    spec = importlib.util.spec_from_file_location('w1_model_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    expected = {}
    for robot in robots:
        manipulator = robot in ('robot_1', 'robot_3')
        for bridge in module.bridges(robot, manipulator, args.tool):
            expected[bridge['ros_topic_name']] = bridge['ros_type_name']
        if manipulator and args.tool in ('epick', 'suction'):
            expected[f'/ax/{robot}/sensors/tool_tof/raw'] = 'sensor_msgs/msg/Range'
    observed = {topic: {'type': kind, 'count': 0, 'first_stamp_ns': None,
                        'last_stamp_ns': None, 'frame_id': '', 'error': ''}
                for topic, kind in expected.items()}
    rclpy.init()
    node = rclpy.create_node('w1_sensor_configuration_check')

    def receive(message, topic):
        row = observed[topic]
        row['count'] += 1
        header = message.header
        stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        row['frame_id'] = header.frame_id
        if row['first_stamp_ns'] is None:
            row['first_stamp_ns'] = stamp
        if row['last_stamp_ns'] is not None and stamp < row['last_stamp_ns']:
            row['error'] = 'STAMP_REVERSED'
        row['last_stamp_ns'] = stamp
        # Contacts는 빈 frame을 허용한다. 다른 센서는 로봇별 frame 분리를 확인한다.
        robot = topic.split('/')[2]
        if not topic.endswith('/cargo_contact') and not header.frame_id.startswith(robot + '/'):
            row['error'] = 'WRONG_ROBOT_FRAME'

    def ready(row):
        return (row['count'] >= 2 and not row['error']
                and row['last_stamp_ns'] > row['first_stamp_ns'] >= 0)

    try:
        for topic, kind in expected.items():
            node.create_subscription(get_message(kind), topic,
                                     lambda msg, topic=topic: receive(msg, topic), qos_profile_sensor_data)
        started = time.monotonic()
        while time.monotonic() - started < args.timeout and not all(map(ready, observed.values())):
            rclpy.spin_once(node, timeout_sec=.1)
        result = {'passed': all(map(ready, observed.values())), 'robots': robots,
                  'tool': args.tool, 'topics': observed,
                  'scope': 'sensor publication/frame/stamp smoke check; not rate, calibration, safety or G1'}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({'passed': result['passed'], 'topics': len(expected),
                          'incomplete': [t for t, r in observed.items() if not ready(r)],
                          'output': str(args.output)}, ensure_ascii=False))
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
