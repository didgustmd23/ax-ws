"""Twist /cmd_vel 계약을 diff_drive_controller의 TwistStamped 입력으로 중계한다."""
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# 각 Twist에 현재(시뮬레이션) 시각과 frame_id를 붙여 값은 그대로 TwistStamped로 전달하는 노드.
class CmdVelRelay(Node):
    # frame_id 매개변수를 읽고 cmd_vel 구독과 diff_drive_controller 입력 발행을 만든다.
    def __init__(self):
        super().__init__('cmd_vel_relay')
        # Jazzy diff_drive_controller는 TwistStamped만 받는다. 기존 Twist 입력을 유지하려고
        # 수신 시각을 붙여 전달하며, 속도 제한·정지 판단은 하지 않는다.
        # 명령이 끊기면 diff_drive_controller의 cmd_vel_timeout이 바퀴 목표를 0으로 만든다.
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.pub = self.create_publisher(TwistStamped, 'diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel', self.forward, 10)

    # 받은 Twist를 시각·frame_id가 붙은 TwistStamped로 바꿔 발행한다.
    def forward(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


# 노드를 실행하고 종료(SIGINT·외부 종료) 시 정리한다.
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
