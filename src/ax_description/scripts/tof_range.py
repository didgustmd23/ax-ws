#!/usr/bin/env python3
# 단일 광선 ToF의 최소/최대 범위와 NaN/±inf 의미를 유지한 채 LaserScan을 Range로 변환한다.
"""Preserve raw one-ray ToF semantics; publish the K0 Range contract."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


# 유효 구간 밖은 -inf/+inf, 형식 오류는 NaN으로 구분한다. 짧은 거리 값을 접촉 성공으로 바꾸지 않는다.
def checked_range(values, minimum, maximum):
    if len(values) != 1 or not (0 < minimum < maximum):
        return math.nan
    value = float(values[0])
    if math.isnan(value) or math.isinf(value):
        return value
    if value < minimum:
        return -math.inf
    if value > maximum:
        return math.inf
    return value


class ToFRange(Node):
    # 상대 토픽으로 연결하므로 launch namespace 아래에서 로봇별 센서가 분리된다.
    def __init__(self):
        super().__init__('tof_range')
        self.declare_parameter('frame_id', 'robot_1/tool_tof_link')
        self.declare_parameter('input_topic', 'sensors/epick_tof/scan')
        self.raw = self.create_publisher(Range, 'sensors/tool_tof/raw', qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value, self.convert, qos_profile_sensor_data)

    # 원래 측정 시각과 min/max를 유지하고 계약된 tool_tof_link 프레임으로 발행한다.
    def convert(self, scan):
        message = Range()
        message.header = scan.header
        message.header.frame_id = self.get_parameter('frame_id').value
        message.radiation_type = Range.INFRARED
        # A single ideal simulation ray, not a measured hardware field of view.
        message.field_of_view = .001
        message.min_range, message.max_range = scan.range_min, scan.range_max
        message.range = checked_range(scan.ranges, scan.range_min, scan.range_max)
        self.raw.publish(message)


# ROS 수신 루프를 실행하고 종료 시 노드를 정리한다.
def main():
    rclpy.init()
    node = ToFRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
