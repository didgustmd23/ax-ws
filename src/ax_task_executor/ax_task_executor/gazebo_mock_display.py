"""
Show A3 mock Action results as an isolated, static Gazebo indicator.

This is not a load observer or a physical loading demonstration.
Only entities named a3_mock_indicator_* are created or removed.
"""

import subprocess
import time

from ax_task_executor.mock_executor import result_display_topic, ROBOT_IDS
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


# 로봇별 표시등 x 위치(m). robot_1·robot_2는 이전 위치를 유지하고 0.6 m 상자가 겹치지 않게 1 m 간격으로 둔다.
INDICATOR_X = {'robot_1': -1, 'robot_2': 0, 'robot_3': -2, 'robot_4': 1}

COLORS = {
    'EMPTY': '1 0.05 0.05 1',
    'STOWED': '0.05 1 0.05 1',
    'UNKNOWN': '1 0.8 0.05 1',
}


# 로봇별 표시등 이름 접두어. robot_2는 이전 이름을 유지한다
def indicator_prefix(robot_id):
    if robot_id == 'robot_2':
        return 'a3_mock_indicator_'

    return f'a3_mock_indicator_{robot_id}_'


def indicator_sdf(state, robot_id='robot_2'):
    """Make an indicator with no collision or effect on robot physics."""
    color = COLORS[state]
    prefix = indicator_prefix(robot_id)
    x = INDICATOR_X[robot_id]
    return (
        '<sdf version="1.6"><model name="' + prefix + state + '">'
        f'<static>true</static><pose>{x} -3 2 0 0 0</pose><link name="display">'
        '<visual name="mock_result"><geometry><box><size>0.6 0.6 0.6</size>'
        '</box></geometry><material><ambient>' + color + '</ambient>'
        '<diffuse>' + color + '</diffuse><emissive>' + color + '</emissive>'
        '</material></visual></link></model></sdf>'
    )


def protobuf_string(value):
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"') + '"'


class MockDisplay(Node):
    def __init__(self):
        super().__init__('a3_gazebo_mock_display')
        self.declare_parameter('robot_id', 'robot_2')
        self.robot_id = self.get_parameter('robot_id').value
        if self.robot_id not in ROBOT_IDS:
            raise ValueError('robot_id must be one of robot_1~robot_4')
        self.indicator_prefix = indicator_prefix(self.robot_id)
        self.declare_parameter('world', 'ax_test')
        self.world = self.get_parameter('world').value
        self.pending = None
        self.shown = None
        self.retry_at = 0.0
        self.create_subscription(
            String, result_display_topic(self.robot_id), self.receive,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0, self.update)
        self.get_logger().info(
            'Waiting for A3 Action result. MOCK indicator only: '
            'EMPTY=red, STOWED=green, UNKNOWN=yellow.')

    def receive(self, message):
        if message.data in COLORS:
            self.pending = message.data

    def service(self, operation, request_type, request):
        command = [
            'gz', 'service', '-s', f'/world/{self.world}/{operation}',
            '--reqtype', request_type, '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', request,
        ]
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=4, check=False)
        return result.returncode == 0 and 'data: true' in result.stdout

    def update(self):
        if self.pending is None or self.pending == self.shown:
            return
        if time.monotonic() < self.retry_at:
            return
        try:
            # Remove only our own indicators, never a robot or task object.
            for state in COLORS:
                self.service('remove', 'gz.msgs.Entity',
                             f'name: "{self.indicator_prefix}{state}" type: MODEL')
            state = self.pending
            success = self.service(
                'create', 'gz.msgs.EntityFactory',
                'sdf: ' + protobuf_string(indicator_sdf(state, self.robot_id)))
            if success:
                self.shown = state
                self.get_logger().info(
                    f'MOCK result displayed: {state}; '
                    f'Entity Tree: {self.indicator_prefix}{state}')
            else:
                raise RuntimeError('Gazebo create service did not confirm success')
        except (OSError, subprocess.TimeoutExpired, RuntimeError) as error:
            self.retry_at = time.monotonic() + 5.0
            self.get_logger().warning(
                f'{error}. Check Gazebo launch and identical GZ_PARTITION.')


def main(args=None):
    rclpy.init(args=args)
    node = MockDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
