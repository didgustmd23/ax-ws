# A3에서 로봇 1·2의 작업 결과를 
# D4에서 검사하고 기록할 수 있도록 전달 및 연결 하는 파일





import time


def to_d4_status(message):
    # 받은 데이터를 D4에서 읽을 수 있는 형태로 바꾼다.
    from rosidl_runtime_py.convert import message_to_ordereddict

    return message_to_ordereddict(message)


def _received_boottime_s():
    # 데이터를 받은 시각을 초 단위로 확인한다.
    return time.clock_gettime(time.CLOCK_BOOTTIME)


def run(on_status):
    # 로봇 1·2의 작업 결과를 받기 위해 필요한 기능을 불러옴.
    import rclpy
    from ax_interfaces.msg import RobotTaskStatus
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    class D4Input(Node):
        def __init__(self):
            super().__init__("d4_a3_status_input")
            # 데이터를 받을 때 사용할 설정
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._subscriptions = []
            # 두 로봇의 결과를 각각 받음.
            for robot_id in ("robot_1", "robot_2"):
                topic = f"/ax/{robot_id}/executor/status"
                subscription = self.create_subscription(
                    RobotTaskStatus,
                    topic,
                    lambda message, expected=robot_id: self.receive(message, expected),
                    qos,
                )
                self._subscriptions.append(subscription)
                self.get_logger().info(f"A3 상태 수신 대기: {topic}")

        def receive(self, message, expected_robot_id):
            # 데이터를 받은 시각과 내용을 확인함.
            received_boottime_s = _received_boottime_s()
            status = to_d4_status(message)
            # 로봇 번호가 다르면 잘못 받은 데이터이므로 넘기지 않는다.
            if status.get("robot_id") != expected_robot_id:
                self.get_logger().error(
                    f"토픽과 robot_id 불일치: {expected_robot_id} / "
                    f"{status.get('robot_id')}"
                )
                return

            # 현재 시각을 초 단위로 가져온다.
            now_sim_s = self.get_clock().now().nanoseconds / 1_000_000_000
            try:
                # 받은 결과를 D4의 검사·기록 기능으로 보냄.
                on_status(
                    status,
                    now_sim_s=now_sim_s,
                    received_boottime_s=received_boottime_s,
                )
            except (ValueError, KeyError, TypeError) as exc:
                # 검사하거나 기록하다 문제가 생기면 알려줌.
                self.get_logger().error(f"D4 입력 검증 또는 기록 실패: {exc}")

    # 아직 ROS2가 시작되지 않았다면 여기서 시작.
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    node = D4Input()
    try:
        # 로봇의 결과가 들어올 때까지 계속 기다림.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 프로그램을 끝낼 때 사용하던 기능을 정리함.
        node.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()
