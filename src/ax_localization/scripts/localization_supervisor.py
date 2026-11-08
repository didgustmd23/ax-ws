#!/usr/bin/env python3
"""B2 로봇별 위치 신뢰성 감독 노드. /ax/<robot_id>에서 실행한다(localization.launch.py가 띄움).

- 발행: localization/health(LocalizationHealth, STATE QoS, 5 Hz 재발행·변경 시 revision 증가)
- 서비스: localization/reinitialize(std_srvs/Trigger). 접수 여부만 응답하고 진행·완료·실패는 health로 알린다.
- 입력: amcl_pose, sensors/front_lidar/scan_filtered, diff_drive_controller/odom, /tf(map→odom), /map,
  safety/actuation_state(A4, 재위치 전 실제 정지 확인)
- 고정 출발 위치: start_pose_* 매개변수가 있으면 이 세션에서 AMCL이 처음 active가 됐을 때 한 번만 initialpose로 보낸다.
  위치를 잃었다는 이유로 다시 보내지 않는다.
- 로봇을 움직이지 않는다. 판단 로직은 localization_health.py에 있다.
"""

import hashlib
from pathlib import Path
import sys
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import localization_health as lh  # noqa: E402
from map_tools import yaw_from_quaternion  # noqa: E402

from ax_interfaces.msg import ActuationState, LocalizationHealth  # noqa: E402

SCHEMA_VERSION = 1
SOURCE_ID = "b2_localization_supervisor"

# 계약 2.4절 STATE 프로필
STATE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


# geometry_msgs Pose를 (x, y, yaw)로 바꾼다.
def pose2d(pose):
    q = pose.orientation

    return (pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


# 메시지 stamp(s)
def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# 로봇 하나의 위치 신뢰성 감독
class LocalizationSupervisor(Node):
    # 매개변수·구독·발행·서비스·타이머를 만든다.
    def __init__(self):
        super().__init__("localization_supervisor", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot_id = self.declare_parameter("robot_id", "robot_2").value
        config_file = self.declare_parameter("config_file", "").value
        user = yaml.safe_load(Path(config_file).read_text(encoding="utf-8")) if config_file else None
        self.config = lh.merge_config(user)
        self.start_pose = None

        if self.declare_parameter("start_pose_enabled", False).value:
            self.start_pose = {key: float(self.declare_parameter(f"start_pose_{key}", 0.0).value)
                               for key in ("x", "y", "yaw", "position_std_m", "yaw_std_rad")}

        self.odom_frame = f"{self.robot_id}/odom"
        self.base_frame = f"{self.robot_id}/base_link"
        self.session = uuid.uuid4().hex
        self.revision = 0
        self.last_key = None
        self.inputs = lh.Inputs()
        self.clock_monitor = lh.ClockMonitor(self.config["health"]["clock_stall_wall_s"])
        self.clock_valid = False
        self.continuity = lh.OdomContinuity(self.config["odom_continuity"])
        self.tracker = lh.TrustedPoseTracker(self.config)
        self.reinit = lh.Reinitializer(self.config)
        self.map_id = None
        self.start_pose_sent = False
        self.health = lh.Health(lh.UNKNOWN, False, "NOT_INITIALIZED")

        # 입력
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, STATE_QOS)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.on_amcl, STATE_QOS)
        self.create_subscription(LaserScan, "sensors/front_lidar/scan_filtered", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "diff_drive_controller/odom", self.on_odom, 50)
        self.create_subscription(ActuationState, "safety/actuation_state", self.on_actuation, STATE_QOS)

        # 출력
        self.publisher = self.create_publisher(LocalizationHealth, "localization/health", STATE_QOS)
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.create_service(Trigger, "localization/reinitialize", self.on_reinitialize)
        self.nomotion = self.create_client(Empty, "request_nomotion_update")
        self.amcl_state = self.create_client(GetState, "amcl/get_state")
        self.amcl_state_pending = False

        # sim time이 멈춰도 상태를 낼 수 있도록 벽시계 타이머로 판정·발행한다.
        period = 1.0 / self.config["health"]["publish_rate_hz"]
        self.create_timer(period, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"{self.robot_id} 위치 신뢰성 감독 시작(session {self.session[:8]}, "
                               f"출발 위치 {'있음' if self.start_pose else '없음'})")

    # 현재 sim time(s)
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # map→<robot>/odom 변환만 기록한다. 유효 시각 = stamp − tf_future_s(AMCL이 앞당긴 시각).
    def on_tf(self, message):
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == self.odom_frame:
                t, q = transform.transform.translation, transform.transform.rotation
                self.inputs.tf_valid_time = seconds(transform.header.stamp) - self.config["health"]["tf_future_s"]
                self.inputs.map_odom = (t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    # 지도 식별값(격자 정보·내용 해시). 바뀌면 마지막 신뢰 위치를 무효로 만든다.
    def on_map(self, message):
        info = message.info
        digest = hashlib.sha256()
        digest.update(f"{info.resolution}:{info.width}:{info.height}:{info.origin.position.x}:"
                      f"{info.origin.position.y}".encode())
        digest.update(bytes(bytearray(int(value) & 0xFF for value in message.data)))
        new_id = digest.hexdigest()[:16]

        if self.map_id is not None and new_id != self.map_id:
            self.get_logger().warn(f"지도 변경 감지({self.map_id} → {new_id}): 마지막 신뢰 위치 무효")

        self.map_id = new_id
        self.tracker.check_map(new_id)

    # AMCL 추정. 수신 시점의 odom 자세도 함께 남긴다(정지 중 무발행 판단).
    def on_amcl(self, message):
        pose = pose2d(message.pose.pose)
        stamp = seconds(message.header.stamp)
        self.inputs.amcl_stamp = stamp
        self.inputs.amcl_pose = pose
        self.inputs.amcl_covariance = tuple(message.pose.covariance)
        self.inputs.amcl_odom_pose = lh.estimate_baseline(None, self.inputs.odom_pose, self.inputs.amcl_updates + 1,
                                                          on_amcl=True)
        self.inputs.amcl_updates += 1
        self.continuity.note_amcl(stamp)
        self.inputs.amcl_after_break = self.continuity.amcl_after_break
        self.reinit.on_amcl(stamp, pose, message.pose.covariance)

    # 스캔 수신 시각
    def on_scan(self, message):
        self.inputs.scan_stamp = seconds(message.header.stamp)

    # odom 자세·연속성·누적 이동량. 끊기면 마지막 신뢰 위치를 무효로 하고 판정에도 끊김을 넘긴다.
    def on_odom(self, message):
        stamp = seconds(message.header.stamp)
        pose = pose2d(message.pose.pose)
        twist = message.twist.twist
        problem = self.continuity.update(stamp, pose, twist.linear.x, twist.angular.z)

        if problem is not None:
            self.get_logger().warn(f"odom 연속성 상실({problem}): 마지막 신뢰 위치 무효")
            self.tracker.invalidate(problem)

        self.tracker.on_odom(pose)
        self.inputs.odom_break_at = self.continuity.broken_at
        self.inputs.amcl_after_break = self.continuity.amcl_after_break

        self.inputs.odom_stamp = stamp
        self.inputs.odom_pose = pose
        # AMCL 추정이 odom보다 먼저 왔으면 최초의 유효 odom을 갱신 정체 검사 기준으로 삼는다(이미 있으면 유지).
        self.inputs.amcl_odom_pose = lh.estimate_baseline(self.inputs.amcl_odom_pose, pose, self.inputs.amcl_updates,
                                                          problem)

    # A4 구동 상태(이 로봇 것만)
    def on_actuation(self, message):
        if message.robot_id != self.robot_id:
            return

        self.inputs.actuation_state = message.state
        self.inputs.actuation_valid = bool(message.meta.valid) and message.meta.revision > 0
        self.inputs.actuation_stamp = seconds(message.meta.header.stamp)

    # map → base_link TF 조회
    def tf_chain_ok(self):
        try:
            self.tf_buffer.lookup_transform("map", self.base_frame, Time())
            return True
        except Exception:
            return False

    # 판정 한 번: 시계·입력 → 상태, 재위치 진행, 신뢰 기록 갱신, 발행
    def tick(self):
        now = self.now()
        self.clock_valid, jumped_back = self.clock_monitor.update(now, time.monotonic())

        if jumped_back:
            self.get_logger().warn("sim time 역행: 마지막 신뢰 위치 무효")
            self.tracker.invalidate("CLOCK_REVERSED")

        self.inputs.tf_chain_ok = self.tf_chain_ok()
        health = lh.evaluate(self.inputs, now, self.config, self.clock_valid)

        # 재위치 진행: 무이동 갱신 요청과 회복·실패 판정
        if self.reinit.active():
            if self.reinit.nomotion_due(now) and self.nomotion.service_is_ready():
                self.nomotion.call_async(Empty.Request())

            result = self.reinit.step(self.inputs, now, self.clock_valid, time.monotonic())

            if result is not None:
                self.get_logger().info(f"재위치 종료: {result}")

        health = self.reinit.apply(health)
        self.tracker.check_map(self.map_id)
        self.tracker.consider(health, self.inputs, now, self.map_id, self.reinit.active())
        self.maybe_send_start_pose()
        self.publish(health, now)

    # 판정 결과를 발행한다. 상태·유효성·사유가 바뀌면 revision을 올린다.
    def publish(self, health, now):
        key = (health.state, health.valid, health.reason)

        if key != self.last_key:
            self.revision += 1
            self.last_key = key
            self.get_logger().info(f"상태 {lh.STATE_NAMES[health.state]} ({health.reason or '-'}) rev {self.revision}")

        self.health = health
        message = LocalizationHealth()
        message.meta.header.stamp = Time(nanoseconds=int(now * 1e9)).to_msg()
        message.meta.header.frame_id = "map"
        message.meta.schema_version = SCHEMA_VERSION
        message.meta.source_id = SOURCE_ID
        message.meta.source_session_id = self.session
        message.meta.revision = self.revision
        message.meta.valid = health.valid
        message.meta.validity_reason = health.reason
        message.robot_id = self.robot_id
        message.state = health.state
        message.position_std_m = float(health.position_std_m)
        message.yaw_std_rad = float(health.yaw_std_rad)
        message.anomaly_score = 0.0
        message.anomaly_model_available = False
        self.publisher.publish(message)

    # D의 재위치 요청. 접수하면 진행 상태를 먼저 발행하고 응답한다.
    def on_reinitialize(self, request, response):
        now = self.now()
        self.inputs.tf_chain_ok = self.tf_chain_ok()
        health = self.reinit.apply(lh.evaluate(self.inputs, now, self.config, self.clock_valid))
        accepted, code = self.reinit.request(health, self.inputs, self.tracker, now, self.map_id, self.clock_valid,
                                             time.monotonic())

        if accepted and code == "REINIT_ACCEPTED":
            candidate = self.reinit.attempt.candidate
            self.get_logger().info(f"재위치 접수: 후보 {tuple(round(v, 3) for v in candidate['pose'])}, "
                                   f"누적 이동 {candidate['travel_m']:.3f} m·{candidate['rotation_rad']:.3f} rad "
                                   f"(순변위 {candidate['net_travel_m']:.3f} m)")
            self.send_initialpose(candidate["pose"], candidate["position_std_m"], candidate["yaw_std_rad"], now)
            self.reinit.mark_sent(now)
            self.publish(self.reinit.apply(health), now)
        else:
            self.get_logger().info(f"재위치 요청 응답: {code}")

        response.success = accepted
        response.message = code

        return response

    # AMCL에 초기 위치를 보낸다. stamp는 0으로 둔다. Nav2 AMCL은 0을 자기 현재 시각으로 바꿔 그 시각의 odom 변환으로
    # 이동 보정을 한다. AMCL의 TF 버퍼가 현재 시각보다 수십 ms 늦으면 "extrapolation into the future" 경고 뒤
    # 보정 없이 그대로 쓴다(Gazebo 시험에서 확인, 감독 노드가 다른 시각을 넣어도 같은 경고가 났다).
    # 재위치·최초 초기화는 정지 상태에서만 하므로 보정을 건너뛰어도 자세 차이가 없다.
    def send_initialpose(self, pose, position_std, yaw_std, now):
        import math

        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = pose[0]
        message.pose.pose.position.y = pose[1]
        message.pose.pose.orientation.z = math.sin(pose[2] / 2)
        message.pose.pose.orientation.w = math.cos(pose[2] / 2)
        message.pose.covariance = lh.pose_covariance(position_std, yaw_std)
        self.initialpose.publish(message)

    # 고정 출발 위치를 이 세션에서 한 번만 보낸다. AMCL이 active이고 initialpose 구독자가 있을 때만.
    def maybe_send_start_pose(self):
        if self.start_pose is None or self.start_pose_sent or self.amcl_state_pending:
            return

        if self.inputs.amcl_updates > 0:
            # 다른 경로(launch initial_poses 등)로 이미 초기화됐으면 보내지 않는다.
            self.start_pose_sent = True
            self.get_logger().info("AMCL이 이미 초기화돼 고정 출발 위치를 보내지 않음")
            return

        if not self.amcl_state.service_is_ready() or self.initialpose.get_subscription_count() == 0:
            return

        self.amcl_state_pending = True
        future = self.amcl_state.call_async(GetState.Request())
        future.add_done_callback(self.on_amcl_state)

    # AMCL 상태 응답. active면 출발 위치를 보낸다.
    def on_amcl_state(self, future):
        self.amcl_state_pending = False

        try:
            active = future.result().current_state.label == "active"
        except Exception:
            return

        if not active or self.start_pose_sent:
            return

        pose = self.start_pose
        self.send_initialpose((pose["x"], pose["y"], pose["yaw"]), pose["position_std_m"], pose["yaw_std_rad"],
                              self.now())
        self.start_pose_sent = True
        self.get_logger().info(f"고정 출발 위치로 최초 초기화: map ({pose['x']}, {pose['y']}, yaw {pose['yaw']})")


# 노드를 실행한다.
def main():
    rclpy.init()
    node = LocalizationSupervisor()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
