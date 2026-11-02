#!/usr/bin/env python3
"""Preserve raw one-ray ToF semantics; publish the K0 Range contract."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


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
    def __init__(self):
        super().__init__('tof_range')
        self.declare_parameter('frame_id', 'robot_1/tool_tof_link')
        self.declare_parameter('input_topic', 'sensors/epick_tof/scan')
        self.raw = self.create_publisher(Range, 'sensors/tool_tof/raw', qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value, self.convert, qos_profile_sensor_data)

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
