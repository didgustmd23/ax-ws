import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        self.get_logger().info('AX Gripper Controller started')

        self.gripper_sub = self.create_subscription(
            Float64,                # 메시지 타입 : Float64
            '/gripper/command',     # 구독할 토픽 : gripper/command
            self.gripper_callback,  # 실행 함수 : gripper_callback
            10                      # 메시지 대기 큐의 깊이
        )

        self.left_pub = self.create_publisher(
            Float64,
            '/left_finger/command',
            10
        )

        self.right_pub = self.create_publisher(
            Float64,
            '/right_finger/command',
            10
        )

    def gripper_callback(self, msg):
        self.get_logger().info(f'Gripper target: {msg.data:.3f} m')

        self.left_pub.publish(msg)
        self.right_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = GripperController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
