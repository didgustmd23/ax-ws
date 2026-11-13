"""공통 계약에 맞는 D4 W0 RobotTaskStatus를 ROS2로 발행하는 시험 도구.

기존 ``test_d4_w0.make_status`` fixture를 사용하는 실제 ROS2 연결 시험용이다.
Action이나 로봇 제어를 실행하지 않으며 A3의 ``mock_executor``도 수정하지 않는다.
"""

from argparse import ArgumentParser
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "tests"))
sys.path.insert(0, str(ROOT / "src" / "ax_d4" / "ax_d4"))

from test_d4_w0 import make_status  # noqa: E402


def _stamp_from_node(node):
    """D4와 같은 simulated clock을 사용하며 /clock이 없으면 0을 사용한다."""
    now = node.get_clock().now().nanoseconds
    return {"sec": now // 1_000_000_000, "nanosec": now % 1_000_000_000}


def _message_from_fixture(node, robot_id, *, valid, revision):
    from ax_interfaces.msg import RobotTaskStatus
    from rosidl_runtime_py.set_message import set_message_fields

    payload = make_status(revision=revision, session="d4-w0-ros-test")
    payload["robot_id"] = robot_id
    payload["meta"]["header"]["stamp"] = _stamp_from_node(node)
    payload["meta"]["valid"] = valid
    payload["meta"]["validity_reason"] = "" if valid else "MOCK_ONLY"

    message = RobotTaskStatus()
    set_message_fields(message, payload)
    return message


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_1")
    parser.add_argument("--valid-seconds", type=float, default=3.0)
    parser.add_argument("--mock-seconds", type=float, default=3.0)
    parser.add_argument(
        "--expiry-seconds",
        type=float,
        default=0.0,
        help="Republish the first message after this delay to exercise expiry.",
    )
    args = parser.parse_args()

    import rclpy
    from ax_interfaces.msg import RobotTaskStatus
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()

    # D4 W0 계약 검증에 필요한 정상 상태와 MOCK_ONLY 상태를 순서대로 발행한다.
    class D4W0RosPublisher(Node):
        def __init__(self):
            super().__init__(
                "d4_w0_contract_publisher",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            # 늦게 시작한 D4도 마지막 시험 메시지를 받을 수 있도록 transient-local을 사용한다.
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.publisher = self.create_publisher(
                RobotTaskStatus,
                f"/ax/{args.robot_id}/executor/status",
                qos,
            )
            self.started_at = time.monotonic()
            self.sent_valid = False
            self.sent_mock = False
            self.sent_stale = False
            self.valid_message = None
            self.timer = self.create_timer(
                0.5,
                self.publish_once,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )

        def publish_once(self):
            # 정상 메시지, MOCK_ONLY 메시지, 선택적 만료 메시지를 단계별로 발행한다.
            elapsed = time.monotonic() - self.started_at
            if not self.sent_valid:
                self.valid_message = _message_from_fixture(
                    self, args.robot_id, valid=True, revision=1
                )
                self.publisher.publish(self.valid_message)
                self.sent_valid = True
                self.get_logger().info("published contract-valid revision=1")
                return

            if elapsed < args.valid_seconds:
                return

            if not self.sent_mock:
                self.publisher.publish(
                    _message_from_fixture(
                        self, args.robot_id, valid=False, revision=2
                    )
                )
                self.sent_mock = True
                self.get_logger().info(
                    "published contract-valid MOCK_ONLY revision=2"
                )
                return

            if (
                args.expiry_seconds > 0
                and not self.sent_stale
                and elapsed >= args.valid_seconds + args.mock_seconds + args.expiry_seconds
            ):
                self.publisher.publish(self.valid_message)
                self.sent_stale = True
                self.get_logger().info("republished stale revision=1")
                return

            end_time = args.valid_seconds + args.mock_seconds
            if args.expiry_seconds > 0:
                end_time += args.expiry_seconds + 1.0
            if elapsed >= end_time:
                rclpy.shutdown()

    node = D4W0RosPublisher()
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
