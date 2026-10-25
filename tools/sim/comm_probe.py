#!/usr/bin/env python3
"""기본 통신 확인: 토픽이 있는지가 아니라 메시지가 실제로 수신되는지 확인한다.

  comm_probe.py --robot robot_1= --robot robot_2=/ax/robot_2 [--timeout 120]
      /clock: 시각이 증가하는 메시지 2개. 로봇별: 이름 목록이 비어 있지 않은 <ns>/joint_states와
      <ns>/odom을 각각 1개. JSON 객체 하나를 출력하며 종료 코드 0은 모든 확인 통과다.
  comm_probe.py --graph-only [--wait 3]
      이 ROS_DOMAIN_ID에서 보이는 다른 ROS 노드를 출력한다(가능한 범위, 참고용).

실행의 ROS_DOMAIN_ID와 ROS 환경으로 실행한다. 제어기 활성, 명령 추종, 정지는 확인하지 않는다.
토픽·노드 수는 참고 정보로만 보고한다.
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


# Clock 메시지의 시각을 초 단위 실수로 바꾼다.
def stamp(msg):
    return msg.clock.sec + msg.clock.nanosec * 1e-9


# /clock과 로봇별 joint_states·odom이 실제로 수신되는지 확인해 JSON 한 줄로 출력한다.
# --graph-only면 같은 domain의 다른 노드만 출력한다.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', action='append', default=[], metavar='NAME=NAMESPACE')
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--graph-only', action='store_true')
    parser.add_argument('--wait', type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('ax_comm_probe')

    # --graph-only: 잠시 그래프를 받은 뒤 자기 노드를 뺀 노드 목록만 출력한다.
    if args.graph_only:
        end = time.monotonic() + args.wait
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        others = sorted(f'{ns.rstrip("/")}/{name}'
                        for name, ns in node.get_node_names_and_namespaces()
                        if name != 'ax_comm_probe')
        print(json.dumps({'other_nodes': others}))
        node.destroy_node()
        rclpy.shutdown()
        return 0

    checks = {}

    # 확인 항목 하나를 FAIL(수신 없음) 상태로 등록하고 돌려준다.
    def check(name, topic):
        checks[name] = {'name': name, 'topic': topic, 'status': 'FAIL',
                        'detail': 'no message received'}
        return checks[name]

    clock = check('clock', '/clock')
    first = []

    # /clock 첫 시각을 기록하고, 더 늦은 시각을 받으면 통과로 바꾼다.
    def on_clock(msg):
        if not first:
            first.append(stamp(msg))
            clock['detail'] = f'one message t={first[0]:.3f}'
        elif stamp(msg) > first[0] and clock['status'] != 'PASS':
            clock.update(status='PASS', detail=f't {first[0]:.3f} -> {stamp(msg):.3f} s')

    # /clock과 로봇별 joint_states·odom을 구독한다.
    subs = [node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)]
    for spec in args.robot:
        robot, _, namespace = spec.partition('=')
        namespace = namespace.rstrip('/')
        joints = check(f'{robot}/joint_states', f'{namespace}/joint_states')
        odom = check(f'{robot}/odom', f'{namespace}/odom')

        # 관절 이름 목록이 비어 있지 않은 joint_states를 받으면 통과로 바꾼다.
        def on_joints(msg, entry=joints):
            if entry['status'] == 'PASS':
                return
            if msg.name:
                entry.update(status='PASS', detail=f'{len(msg.name)} joints, '
                             f'{len(msg.position)} positions')
            else:
                entry['detail'] = 'message received with an empty name list'

        # odom을 한 번 받으면 프레임 이름과 함께 통과로 바꾼다.
        def on_odom(msg, entry=odom):
            if entry['status'] != 'PASS':
                entry.update(status='PASS',
                             detail=f'frame {msg.header.frame_id} -> {msg.child_frame_id}')

        subs.append(node.create_subscription(JointState, joints['topic'], on_joints,
                                             qos_profile_sensor_data))
        subs.append(node.create_subscription(Odometry, odom['topic'], on_odom,
                                             qos_profile_sensor_data))

    # 모든 확인이 통과하거나 시간 초과까지 수신하고 결과를 JSON 한 줄로 출력한다.
    start = time.monotonic()
    while time.monotonic() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(c['status'] == 'PASS' for c in checks.values()):
            break

    info = {'seconds': round(time.monotonic() - start, 1),
            'topics_seen': len(node.get_topic_names_and_types()),
            'nodes_seen': len(node.get_node_names())}
    print(json.dumps({'checks': list(checks.values()), 'info': info}))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(c['status'] == 'PASS' for c in checks.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
