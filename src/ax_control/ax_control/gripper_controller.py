import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


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

        # finger_controller(JointGroupPositionController) 입력: [left, right] 목표 변위(m).
        # 이 노드가 손가락 명령의 유일한 발행 주체다. 범위 검사·집기 완료 판정은 하지 않는다.
        self.finger_pub = self.create_publisher(
            Float64MultiArray,
            '/finger_controller/commands',
            10
        )

    def gripper_callback(self, msg):
        self.get_logger().info(f'Gripper target: {msg.data:.3f} m')

        # 두 손가락의 양의 축 방향이 반대이므로 같은 값이면 함께 벌어진다.
        self.finger_pub.publish(Float64MultiArray(data=[msg.data, msg.data]))


def main(args=None):
    rclpy.init(args=args)

    node = GripperController()

    # launch 종료(SIGINT) 시 Jazzy rclpy가 context를 먼저 내려도 예외 없이 끝낸다.
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
