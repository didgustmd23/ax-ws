"""모의 작업 결과를 Gazebo 표시로 보여 주며 실제 센서와 구분합니다."""

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
    # 역할: 해당 로봇이 소유한 모의 표시 이름 접두어를 만듭니다.
    if robot_id == 'robot_2':
        return 'a3_mock_indicator_'

    return f'a3_mock_indicator_{robot_id}_'


def indicator_sdf(state, robot_id='robot_2'):
    """모의 작업 상태를 표시할 Gazebo 시각 모델을 만듭니다."""
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
    # 역할: 시뮬레이터 요청에 넣을 문자열을 이스케이프합니다.
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"') + '"'


class MockDisplay(Node):
    def __init__(self):
        # 역할: 설정과 내부 상태를 준비합니다.
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
        # 역할: 해당 로봇의 모의 결과 표시 메시지를 받습니다.
        if message.data in COLORS:
            self.pending = message.data

    def service(self, operation, request_type, request):
        # 역할: 시뮬레이터 서비스에 표시 생성 또는 삭제 요청을 보냅니다.
        command = [
            'gz', 'service', '-s', f'/world/{self.world}/{operation}',
            '--reqtype', request_type, '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', request,
        ]
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=4, check=False)
        return result.returncode == 0 and 'data: true' in result.stdout

    def update(self):
        # 역할: 현재 모의 결과에 맞춰 소유한 표시만 갱신합니다.
        if self.pending is None or self.pending == self.shown:
            return
        if time.monotonic() < self.retry_at:
            return
        try:
            # 이 노드 소유의 표시만 지우며 로봇이나 작업 물체는 삭제하지 않습니다.
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
    # 역할: 실행 인자를 받아 노드를 시작하고 종료 시 자원을 정리합니다.
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
