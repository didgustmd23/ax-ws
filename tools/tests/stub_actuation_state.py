#!/usr/bin/env python3
# A4 미구현 동안 B2 재위치 시험용 가짜 ActuationState 발행기. A4 구현이 아니며 운영에 쓰지 않는다.
# odom 속도가 D-06 값(0.01 m/s·0.02 rad/s) 안에 0.5 s 머물면 STOPPED, 아니면 RUNNING을 20 Hz로 낸다.
#   stub_actuation_state.py --ros-args -p robot_ids:="robot_1,robot_2" -p use_sim_time:=true
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ax_interfaces.msg import ActuationState
from nav_msgs.msg import Odometry

STATE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
STOP_LINEAR, STOP_ANGULAR, STOP_HOLD_S = 0.01, 0.02, 0.5


# 로봇별 odom으로 정지 여부를 정하는 가짜 A4
class StubActuation(Node):
    # 로봇별 구독·발행과 20 Hz 타이머를 만든다.
    def __init__(self):
        super().__init__('stub_actuation_state')
        self.robot_ids = [value.strip() for value in self.declare_parameter('robot_ids', 'robot_2').value.split(',')]
        self.session = uuid.uuid4().hex
        self.revision = {robot_id: 0 for robot_id in self.robot_ids}
        self.last_state = {}
        self.still_since = {}
        self.speed = {}
        self.pubs = {}

        for robot_id in self.robot_ids:
            ns = f'/ax/{robot_id}'
            self.pubs[robot_id] = self.create_publisher(ActuationState, f'{ns}/safety/actuation_state', STATE_QOS)
            self.create_subscription(Odometry, f'{ns}/diff_drive_controller/odom',
                                     lambda message, r=robot_id: self.on_odom(r, message), 50)

        self.create_timer(0.05, self.publish)

    # odom 속도를 기록하고 정지 시작 시각을 갱신한다.
    def on_odom(self, robot_id, message):
        twist = message.twist.twist
        now = self.get_clock().now().nanoseconds * 1e-9
        linear, angular = abs(twist.linear.x), abs(twist.angular.z)
        self.speed[robot_id] = (linear, angular)

        if linear < STOP_LINEAR and angular < STOP_ANGULAR:
            self.still_since.setdefault(robot_id, now)
        else:
            self.still_since.pop(robot_id, None)

    # 로봇별 상태를 발행한다. odom을 받은 적 없으면 UNKNOWN(무효)이다.
    def publish(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds * 1e-9

        for robot_id in self.robot_ids:
            message = ActuationState()

            if robot_id not in self.speed:
                state, valid = ActuationState.UNKNOWN, False
            else:
                since = self.still_since.get(robot_id)
                state = ActuationState.STOPPED if since is not None and seconds - since >= STOP_HOLD_S \
                    else ActuationState.RUNNING
                valid = True

            if self.last_state.get(robot_id) != state:
                self.revision[robot_id] += 1
                self.last_state[robot_id] = state

            message.meta.header.stamp = now.to_msg()
            message.meta.schema_version = 1
            message.meta.source_id = 'stub_actuation_state'
            message.meta.source_session_id = self.session
            message.meta.revision = self.revision[robot_id]
            message.meta.valid = valid
            message.meta.validity_reason = '' if valid else 'NO_ODOM'
            message.robot_id = robot_id
            message.state = state
            linear, angular = self.speed.get(robot_id, (0.0, 0.0))
            message.linear_speed, message.angular_speed = float(linear), float(angular)
            self.pubs[robot_id].publish(message)


# 노드를 실행한다.
def main():
    rclpy.init()
    node = StubActuation()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
