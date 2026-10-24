#!/usr/bin/env python3
"""Basic communication probe: checks that messages are actually RECEIVED (not that topics exist).

  comm_probe.py --robot robot_1= --robot robot_2=/ax/robot_2 [--timeout 120]
      /clock: two messages with increasing time; per robot: <ns>/joint_states with a non-empty
      name list and <ns>/odom, one message each. Prints one JSON object; exit 0 = every check passed.
  comm_probe.py --graph-only [--wait 3]
      Prints the other ROS nodes visible on this ROS_DOMAIN_ID (best effort, reference only).

Run it with the run's ROS_DOMAIN_ID and ROS environment. This does NOT check controller activation,
command tracking or stopping. Topic and node counts are reported as reference information only.
"""
import argparse
import json
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


def stamp(msg):
    return msg.clock.sec + msg.clock.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', action='append', default=[], metavar='NAME=NAMESPACE')
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--graph-only', action='store_true')
    parser.add_argument('--wait', type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('ax_comm_probe')
    if args.graph_only:
        end = time.monotonic() + args.wait
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        others = sorted(f'{ns.rstrip("/")}/{name}' for name, ns in node.get_node_names_and_namespaces()
                        if name != 'ax_comm_probe')
        print(json.dumps({'other_nodes': others}))
        node.destroy_node(); rclpy.shutdown()
        return 0

    checks = {}

    def check(name, topic):
        checks[name] = {'name': name, 'topic': topic, 'status': 'FAIL', 'detail': 'no message received'}
        return checks[name]

    clock = check('clock', '/clock')
    first = []

    def on_clock(msg):
        if not first:
            first.append(stamp(msg)); clock['detail'] = f'one message t={first[0]:.3f}'
        elif stamp(msg) > first[0] and clock['status'] != 'PASS':
            clock.update(status='PASS', detail=f't {first[0]:.3f} -> {stamp(msg):.3f} s')

    subs = [node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)]
    for spec in args.robot:
        robot, _, namespace = spec.partition('=')
        namespace = namespace.rstrip('/')
        joints = check(f'{robot}/joint_states', f'{namespace}/joint_states')
        odom = check(f'{robot}/odom', f'{namespace}/odom')

        def on_joints(msg, entry=joints):
            if entry['status'] == 'PASS':
                return
            if msg.name:
                entry.update(status='PASS', detail=f'{len(msg.name)} joints, '
                             f'{len(msg.position)} positions')
            else:
                entry['detail'] = 'message received with an empty name list'

        def on_odom(msg, entry=odom):
            if entry['status'] != 'PASS':
                entry.update(status='PASS', detail=f'frame {msg.header.frame_id} -> {msg.child_frame_id}')

        subs.append(node.create_subscription(JointState, joints['topic'], on_joints,
                                             qos_profile_sensor_data))
        subs.append(node.create_subscription(Odometry, odom['topic'], on_odom,
                                             qos_profile_sensor_data))

    start = time.monotonic()
    while time.monotonic() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(c['status'] == 'PASS' for c in checks.values()):
            break
    info = {'seconds': round(time.monotonic() - start, 1),
            'topics_seen': len(node.get_topic_names_and_types()),
            'nodes_seen': len(node.get_node_names())}
    print(json.dumps({'checks': list(checks.values()), 'info': info}))
    node.destroy_node(); rclpy.shutdown()
    return 0 if all(c['status'] == 'PASS' for c in checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
