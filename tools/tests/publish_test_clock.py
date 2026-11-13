"""D4 W0 통합 시험을 위해 진행하는 ROS2 /clock을 발행하는 도구."""

import argparse
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMessage

    rclpy.init()

    # D4의 use_sim_time 입력이 실제로 진행되도록 시험용 시계를 발행한다.
    class TestClock(Node):
        def __init__(self):
            super().__init__("d4_w0_test_clock")
            self.publisher = self.create_publisher(
                ClockMessage,
                "/clock",
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.started_at = time.monotonic()
            self.timer = self.create_timer(
                0.05,
                self.publish_clock,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )

        def publish_clock(self):
            # 실제 경과 시간을 ROS2 Clock 메시지의 초·나노초로 변환한다.
            elapsed = time.monotonic() - self.started_at
            seconds = int(elapsed)
            nanoseconds = int((elapsed - seconds) * 1_000_000_000)
            message = ClockMessage()
            message.clock.sec = seconds
            message.clock.nanosec = nanoseconds
            self.publisher.publish(message)
            if elapsed >= args.duration:
                rclpy.shutdown()

    node = TestClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
