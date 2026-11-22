#!/usr/bin/env python3
"""Simulated battery state from Gazebo model motion and charging pads."""

from math import cos, hypot
from threading import Lock


RESERVE = 0.22
IDLE_POWER_RATIO = 0.15
ACTIVE_DRAIN_PER_SECOND = 1.0 / (18 * 3600)
CHARGE_PER_SECOND = 1.0 / (4 * 3600)
DOCK_RADIUS = 0.45


class Battery:
    def __init__(self, stations):
        self.stations = stations
        self.remaining = 1.0
        self.previous = None
        self.docked_since = None
        self.charging = False

    def update(self, stamp, x, y, orientation):
        previous = self.previous
        self.previous = (stamp, x, y, orientation)
        if previous is None or not 0 < stamp - previous[0] <= 0.5:
            self.docked_since = None
            self.charging = False
            return

        dt = stamp - previous[0]
        rotation_dot = abs(sum(a * b for a, b in zip(orientation, previous[3])))
        moving = (hypot(x - previous[1], y - previous[2]) > 0.02 * dt
                  or rotation_dot < cos(0.05 * dt / 2))
        on_pad = any(hypot(x - sx, y - sy) <= DOCK_RADIUS for sx, sy in self.stations)
        if moving or not on_pad:
            self.docked_since = None
        elif self.docked_since is None:
            self.docked_since = stamp
        self.charging = self.docked_since is not None and stamp - self.docked_since >= 1.0

        if self.charging:
            self.remaining = min(1.0, self.remaining + CHARGE_PER_SECOND * dt)
        else:
            drain = ACTIVE_DRAIN_PER_SECOND * (1.0 if moving else IDLE_POWER_RATIO)
            self.remaining = max(0.0, self.remaining - drain * dt)


def main():
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GazeboNode
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool

    class BatteryNode(Node):
        def __init__(self):
            super().__init__('sim_battery')
            robot = self.declare_parameter('robot_id', '').value
            world = self.declare_parameter('world_name', '').value
            locations = self.declare_parameter('station_xy', [0.0, 0.0]).value
            self.battery = Battery(list(zip(locations[::2], locations[1::2])))
            self.lock = Lock()
            self.publisher = self.create_publisher(BatteryState, 'battery_state', 10)
            self.low = self.create_publisher(Bool, 'battery_low', 10)
            self.topic = f'/world/{world}/dynamic_pose/info'
            self.transport = GazeboNode()

            def receive(message):
                for pose in message.pose:
                    if pose.name == robot:
                        stamp = message.header.stamp
                        q = pose.orientation
                        with self.lock:
                            self.battery.update(stamp.sec + stamp.nsec * 1e-9,
                                                pose.position.x, pose.position.y,
                                                (q.x, q.y, q.z, q.w))
                        break

            if not self.transport.subscribe(Pose_V, self.topic, receive):
                raise RuntimeError(f'Cannot subscribe to {self.topic}')
            self.create_timer(0.2, self.publish_state)

        def publish_state(self):
            now = self.get_clock().now()
            with self.lock:
                sample = self.battery.previous
                if sample is None or not 0 <= now.nanoseconds / 1e9 - sample[0] <= 0.5:
                    return
                remaining = self.battery.remaining
                charging = self.battery.charging

            msg = BatteryState()
            msg.header.stamp = now.to_msg()
            msg.present = True
            msg.percentage = remaining
            for field in ('voltage', 'temperature', 'current', 'charge', 'capacity',
                          'design_capacity'):
                setattr(msg, field, float('nan'))
            msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING
            if charging:
                msg.power_supply_status = (msg.POWER_SUPPLY_STATUS_FULL if remaining >= 1.0
                                           else msg.POWER_SUPPLY_STATUS_CHARGING)
            self.publisher.publish(msg)
            self.low.publish(Bool(data=remaining <= RESERVE))

    rclpy.init()
    node = None
    try:
        node = BatteryNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.transport.unsubscribe(node.topic)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
