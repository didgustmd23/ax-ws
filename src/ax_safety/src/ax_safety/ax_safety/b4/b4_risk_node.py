#!/usr/bin/env python3
"""실제 ROS2 입력 -> B4 위험 판단 -> 계약 SafetyState 10Hz.

센서 Topic은 임의로 가정하지 않는다: scan_topic / front_range_topic /
rear_range_topic은 A1/B1 토픽 확인 후 파라미터로 입력한다.
미연결·오래됨·손상됨은 UNKNOWN + 완전 차단한다.
"""
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Range
from ax_interfaces.msg import SafetyState, LocalizationHealth, CorridorObservationList
from ax_safety.b4.risk_rules import RiskInputs, assess_risk, UNKNOWN




# ROS Time 메시지를 초 단위 실수로 바꿔 센서 timestamp 신선도를 계산한다.
def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9




def minimum_sector(scan, target, half_width):
    """레이저 부채꼴에서 가장 가까운 유효 거리. inf는 측정 최대거리로 취급."""
    distances = []
    for idx, raw in enumerate(scan.ranges):
        angle = scan.angle_min + idx * scan.angle_increment
        difference = math.atan2(math.sin(angle - target), math.cos(angle - target))
        if abs(difference) > half_width:
            continue
        if math.isnan(raw) or raw < 0:
            continue
        if math.isinf(raw):
            if raw > 0 and scan.range_max > 0:
                distances.append(float(scan.range_max))
        elif scan.range_min <= raw <= scan.range_max:
            distances.append(float(raw))
    return min(distances) if distances else None




# B4 센서·위치·통로 관측을 받아 10 Hz SafetyState를 발행하는 ROS 노드이다.
# 실제 센서 토픽 이름은 launch/config에서 주입하고, A4는 이 노드의 결과를 최종 게이트 입력으로 사용한다.
class B4RiskNode(Node):
    def __init__(self):
        # 파라미터는 로봇별 namespace와 A1/B1에서 전달하는 토픽 경로를 결정한다.
        super().__init__('b4_risk_node')
        p = self.declare_parameter
        self.robot_id = str(p('robot_id', 'robot_1').value)
        self.scan_topic = str(p('scan_topic', '').value)
        self.front_scan_topic = str(p('front_scan_topic', '').value)
        self.rear_scan_topic = str(p('rear_scan_topic', '').value)
        self.front_topic = str(p('front_range_topic', '').value)
        self.rear_topic = str(p('rear_range_topic', '').value)
        self.max_linear = float(p('max_linear_m_s', 0.67).value)
        self.max_angular = float(p('max_angular_rad_s', 0.50).value)
        self.stop_distance = float(p('stop_distance_m', 0.50).value)
        self.slowdown_distance = float(p('slowdown_distance_m', 1.00).value)
        self.sensor_ttl = float(p('sensor_stale_s', 0.25).value)
        self.loc_ttl = float(p('localization_stale_s', 1.0).value)
        self.corridor_ttl = float(p('corridor_stale_s', 1.0).value)
        self.sector_width = math.radians(float(p('sector_half_width_deg', 30.0).value))
        self.front = self.rear = self.localization = self.corridor = None
        # 값 자체와 수신 시각을 함께 저장해 메시지 내용과 wall-clock 신선도를 모두 확인한다.
        self.front_time = self.rear_time = self.loc_time = self.corridor_time = None
        self.session = str(uuid.uuid4())
        self.revision = 0
        base = f'/ax/{self.robot_id}'
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(SafetyState, base + '/safety/state', qos)
        self.create_subscription(LocalizationHealth, base + '/localization/health', self.on_loc, qos)
        self.create_subscription(CorridorObservationList, base + '/perception/corridor_observation', self.on_corridor, qos)
        if self.scan_topic:
            self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        if self.front_scan_topic:
            self.create_subscription(LaserScan, self.front_scan_topic, self.on_front_scan,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        if self.rear_scan_topic:
            self.create_subscription(LaserScan, self.rear_scan_topic, self.on_rear_scan,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        if self.front_topic:
            self.create_subscription(Range, self.front_topic, self.on_front,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        if self.rear_topic:
            self.create_subscription(Range, self.rear_topic, self.on_rear,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.1, self.publish_state)
        self.get_logger().info('B4 initialized: sensor subscriptions only on explicitly configured topics')



    def fresh(self, received, ttl):
        # monotonic 시계는 시스템 시간이 바뀌어도 timeout 판단이 흔들리지 않게 한다.
        return received is not None and 0.0 <= time.monotonic() - received <= ttl



    def recent_stamp(self, stamp, limit):
        # ROS timestamp도 별도로 확인해 지연된 메시지가 새로 도착한 것처럼 보이지 않게 한다.
        stamp_value = stamp_seconds(stamp)
        current = self.get_clock().now().nanoseconds / 1e9
        return stamp_value > 0 and math.isfinite(current) and 0 <= current - stamp_value <= limit



    def on_scan(self, msg):
        # 하나의 LaserScan에서 전방(0 rad)과 후방(pi rad)의 최소 유효거리를 함께 계산한다.
        if not self.recent_stamp(msg.header.stamp, self.sensor_ttl):
            return
        self.front = minimum_sector(msg, 0.0, self.sector_width)
        self.rear = minimum_sector(msg, math.pi, self.sector_width)
        self.front_time = self.rear_time = time.monotonic()



    def on_front_scan(self, msg):
        # 분리된 전방 LaserScan은 전방 거리만 갱신한다.
        if not self.recent_stamp(msg.header.stamp, self.sensor_ttl):
            return
        self.front = minimum_sector(msg, 0.0, self.sector_width)
        self.front_time = time.monotonic()



    def on_rear_scan(self, msg):
        # 분리된 후방 LaserScan은 후방 거리만 갱신한다.
        if not self.recent_stamp(msg.header.stamp, self.sensor_ttl):
            return
        self.rear = minimum_sector(msg, 0.0, self.sector_width)
        self.rear_time = time.monotonic()



    def on_front(self, msg):
        # Range 센서는 유한하고 min/max 범위 안에 있는 값만 안전 입력으로 저장한다.
        if not self.recent_stamp(msg.header.stamp, self.sensor_ttl):
            return
        self.front = float(msg.range) if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range else None
        self.front_time = time.monotonic()



    def on_rear(self, msg):
        # 후방 Range도 같은 유효성 규칙을 적용해 잘못된 값으로 후진을 허용하지 않는다.
        if not self.recent_stamp(msg.header.stamp, self.sensor_ttl):
            return
        self.rear = float(msg.range) if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range else None
        self.rear_time = time.monotonic()



    def on_loc(self, msg):
        # 위치 상태와 수신 시각을 저장하며, 실제 유효성 판정은 publish_state에서 수행한다.
        self.localization, self.loc_time = msg, time.monotonic()



    def on_corridor(self, msg):
        # B3 통로 관측을 저장한다. 관측의 robot_id·revision·timestamp는 나중에 다시 확인한다.
        self.corridor, self.corridor_time = msg, time.monotonic()



    def meta_ok(self, msg, ttl):
        # 공통 메타데이터가 유효하고 revision/session이 있으며 최근 timestamp인지 확인한다.
        if msg is None or not msg.meta.valid or not msg.meta.revision or not msg.meta.source_session_id:
            return False
        return self.recent_stamp(msg.meta.header.stamp, ttl)



    def publish_state(self):
        # 1) 위치·통로·센서 입력을 검증하고 2) 순수 규칙 엔진에 전달한 뒤
        # 3) A4가 사용할 SafetyState와 차단 사유를 발행한다.
        loc_valid = (self.localization is not None and self.localization.robot_id == self.robot_id
                     and self.meta_ok(self.localization, self.loc_ttl)
                     and self.localization.state == LocalizationHealth.OK)
        corridor_valid = (self.corridor is not None and self.corridor.robot_id == self.robot_id
                          and self.meta_ok(self.corridor, self.corridor_ttl)
                          and all(self.meta_ok(obs, self.corridor_ttl)
                                  and obs.robot_id == self.robot_id and obs.position_valid
                                  and obs.observation != obs.UNKNOWN for obs in self.corridor.observations))
        front_fresh = self.fresh(self.front_time, self.sensor_ttl)
        rear_fresh = self.fresh(self.rear_time, self.sensor_ttl)
        inputs = RiskInputs(front_sensor_valid=front_fresh and self.front is not None,
                            rear_sensor_valid=rear_fresh and self.rear is not None,
                            sensor_fresh=front_fresh,
                            localization_valid=loc_valid,
                            localization_fresh=self.fresh(self.loc_time, self.loc_ttl),
                            corridor_valid=corridor_valid and self.fresh(self.corridor_time, self.corridor_ttl),
                            obstacle_distance_m=self.front, rear_distance_m=self.rear,
                            max_linear_m_s=self.max_linear, max_angular_rad_s=self.max_angular,
                            stop_distance_m=self.stop_distance,
                            slowdown_distance_m=self.slowdown_distance)
        result = assess_risk(inputs)
        # 매 발행마다 session/revision을 갱신해 A4가 오래된 상태와 새 상태를 구분한다.
        msg = SafetyState()
        msg.meta.header.stamp = self.get_clock().now().to_msg()
        msg.meta.schema_version = 1
        msg.meta.source_id = 'b4_risk_node'
        msg.meta.source_session_id = self.session
        self.revision += 1
        msg.meta.revision = self.revision
        # 위험 신호라도 판단 결과는 발행. 센서 결손이면 meta.valid=false로 추가 차단.
        msg.meta.valid = result.level != UNKNOWN
        msg.meta.validity_reason = '' if msg.meta.valid else result.reason_code
        msg.robot_id = self.robot_id
        for field in ('level', 'reason_code', 'sensor_valid', 'front_sensor_valid',
                      'rear_sensor_valid', 'localization_valid', 'allow_forward',
                      'allow_backward', 'allow_rotate', 'max_linear_speed', 'max_angular_speed'):
            setattr(msg, field, getattr(result, field))
        self.publisher.publish(msg)




def main(args=None):
    # ROS executor가 센서 callback과 10 Hz publish timer를 처리하도록 노드를 실행한다.
    rclpy.init(args=args)
    node = B4RiskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
