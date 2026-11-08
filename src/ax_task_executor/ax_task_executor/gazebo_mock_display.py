"""
Show A3 mock Action results as an isolated, static Gazebo indicator.

This is not a load observer or a physical loading demonstration.
Only entities named a3_mock_indicator_* are created or removed.
"""

import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


COLORS = {
    'EMPTY': '1 0.05 0.05 1',
    'STOWED': '0.05 1 0.05 1',
    'UNKNOWN': '1 0.8 0.05 1',
}


def indicator_sdf(state, robot_id='robot_2'):
    """Make an indicator with no collision or effect on robot physics."""
    color = COLORS[state]
    prefix = 'a3_mock_indicator_' if robot_id == 'robot_2' else 'a3_mock_indicator_robot_1_'
    x = 0 if robot_id == 'robot_2' else -1
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
        if self.robot_id not in ('robot_1', 'robot_2'):
            raise ValueError('robot_id must be robot_1 or robot_2')
        self.indicator_prefix = (
            'a3_mock_indicator_' if self.robot_id == 'robot_2'
            else 'a3_mock_indicator_robot_1_')
        self.declare_parameter('world', 'ax_test')
        self.world = self.get_parameter('world').value
        self.pending = None
        self.shown = None
        self.retry_at = 0.0
        self.create_subscription(
            String, ('/a3_mock/result_display' if self.robot_id == 'robot_2'
                     else '/a3_mock/robot_1/result_display'), self.receive,
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
