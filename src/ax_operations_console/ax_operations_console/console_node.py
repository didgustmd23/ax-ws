"""관제 화면 표시 중계 노드(D2). Topic 구독 전용 + HTTP GET·HEAD 표시 서버.

- publisher·service client·action client를 만들지 않는다.
  파라미터 Service도 끈다(start_parameter_services=False).
  rclpy가 기본으로 만드는 /rosout·/parameter_events 발행만 남는다.
- 판정 시각: sim time은 노드 clock(use_sim_time 매개변수), 무수신 시간은 CLOCK_BOOTTIME.
- 응답은 화면 표시 전용이며 다른 노드·Agent의 상태 원천이 아니다(계약 8절).
- 매개변수: robots_file, topics_file, topics_profile(operational|dev), map_yaml, web_dir,
  bind_address(기본 127.0.0.1), port(기본 18780), future_tolerance_s(0.1), display_grace_s(0.5)
"""
from pathlib import Path
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from ax_interfaces.msg import ActuationState, GroupTaskList, LocalizationHealth
from ax_interfaces.msg import RobotTaskStatus, SafetyState
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState

from . import console_config as cfg
from . import console_messages as cm
from .console_http import allowed_hosts, make_handler, make_server
from .console_state import Clocks, ConsoleState

DEFAULT_PORT = 18780

# 계약 STATE 프로필(reliable·transient_local·keep_last 1). amcl_pose도 같은 QoS로 발행된다(B2 감독기 구독과 동일)
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
MESSAGE_TYPES = {'actuation': ActuationState, 'safety': SafetyState, 'health': LocalizationHealth,
                 'executor': RobotTaskStatus, 'pose': PoseWithCovarianceStamped,
                 'battery': BatteryState}
# 배터리는 B4와 같이 sensor QoS(best effort)로 받아 reliable·best effort 발행자 모두와 맞춘다
TOPIC_QOS = {'battery': qos_profile_sensor_data}


# CLOCK_BOOTTIME(초)
def boot_now():
    return time.clock_gettime(time.CLOCK_BOOTTIME)


class OperationsConsole(Node):
    """구독 → ConsoleState 갱신, HTTP 요청 시 상태 조립."""

    # 매개변수 검사 → 설정·지도 읽기 → 구독 생성 → HTTP 서버 시작
    def __init__(self):
        super().__init__('ax_operations_console', start_parameter_services=False)
        share = Path(get_package_share_directory('ax_operations_console'))

        config = share / 'config'
        robots_file = self.declare_parameter(
            'robots_file', str(config / 'console_robots_dev.yaml')).value
        topics_file = self.declare_parameter(
            'topics_file', str(config / 'console_topics.yaml')).value
        profile = self.declare_parameter('topics_profile', 'operational').value
        map_yaml = self.declare_parameter('map_yaml', default_map_yaml()).value
        self.web_dir = self.declare_parameter('web_dir', str(share / 'web')).value
        bind = self.declare_parameter('bind_address', '127.0.0.1').value
        port = self.declare_parameter('port', DEFAULT_PORT).value
        future = float(self.declare_parameter('future_tolerance_s', 0.1).value)
        grace = float(self.declare_parameter('display_grace_s', 0.5).value)

        external = cfg.check_bind(bind, port)
        cfg.check_timing(future, grace)
        source, robots = cfg.load_robots(robots_file)
        topics = cfg.load_topics(topics_file, profile)
        map_info, map_bytes = cfg.load_map(map_yaml)

        self.lock = threading.Lock()
        self.state = ConsoleState(robots, source, future_tolerance_s=future, display_grace_s=grace)
        self.topics = {'profile': profile, 'tasks': topics['tasks']}

        # 로봇별 6종 + 작업 목록 구독
        for robot in robots:
            for kind, relative in topics['robot'].items():
                self.create_subscription(MESSAGE_TYPES[kind],
                                         cfg.robot_topic(robot['robot_id'], relative),
                                         self.robot_callback(robot['robot_id'], kind),
                                         TOPIC_QOS.get(kind, STATE_QOS))

        self.create_subscription(GroupTaskList, topics['tasks'], self.on_tasks, STATE_QOS)

        # HTTP 서버(데몬 스레드)
        handler = make_handler(self.snapshot, self.web_dir, map_info, map_bytes,
                               allowed_hosts(bind, port),
                               lambda text: self.get_logger().error(text))
        self.server = make_server(bind, port, handler)
        threading.Thread(target=self.server.serve_forever, name='console_http',
                         daemon=True).start()

        if external:
            self.get_logger().warn(f'외부 바인딩 {bind}:{port}: 같은 망에서 상태를 볼 수 있다(표시 전용)')

        host = bind if external else 'localhost'
        self.get_logger().info(f'관제 화면 http://{host}:{port}/ profile={profile} '
                               f'작업={topics["tasks"]} 로봇={len(robots)}대 출처={source}')

    # 현재 시각 묶음
    def clocks(self):
        return Clocks(sim=self.get_clock().now().nanoseconds * 1e-9, boot=boot_now())

    # 로봇 입력 콜백 생성기
    def robot_callback(self, robot_id, kind):
        convert = cm.CONVERTERS[kind]

        def on_message(msg):
            sample = convert(msg)

            with self.lock:
                self.state.update_robot(robot_id, kind, sample, self.clocks())

        return on_message

    # 작업 목록 콜백
    def on_tasks(self, msg):
        sample = cm.tasks(msg)

        with self.lock:
            self.state.update_tasks(sample, self.clocks())

    # HTTP 스레드에서 부르는 상태 조립
    def snapshot(self):
        with self.lock:
            state = self.state.build_state(self.clocks())

        state['topics'] = self.topics
        return state

    # 서버 정지
    def close(self):
        self.server.shutdown()
        self.server.server_close()


# 기본 지도: ax_localization 설치본의 maps/ax_warehouse/map.yaml
def default_map_yaml():
    try:
        share = Path(get_package_share_directory('ax_localization'))
        return str(share / 'maps' / 'ax_warehouse' / 'map.yaml')
    except Exception:
        return ''


# 진입점: 설정 오류는 한 줄로 알리고 종료 코드 2
def main(args=None):
    rclpy.init(args=args)

    try:
        node = OperationsConsole()
    except (cfg.ConfigError, OSError) as error:
        print(f'[ax_operations_console] 시작 거부: {error}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
