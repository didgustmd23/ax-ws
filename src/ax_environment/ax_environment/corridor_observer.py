"""B3 통로 관측 발행 노드: 로봇 1대의 위치·외곽으로 모든 통로의 OUTSIDE·APPROACHING·INSIDE·UNKNOWN을 낸다.

- 입력: TF map→<robot_id>/base_link(B2 map→odom + A2 odom→base_link), localization/health(LocalizationHealth, STATE QoS).
- 출력: perception/corridor_observation(CorridorObservationList, STATE QoS, 5 Hz 재발행, 내용 변경 시 revision+1).
- 위치 유효 후보는 health가 valid·revision>0이고 나이가 health_timeout_s 안(sim time·벽시계 둘 다)인 OK 또는 DEGRADED다.
  LOST·UNKNOWN·만료·미수신은 모든 통로 UNKNOWN(position_valid=false)이다.
- DEGRADED: 불확실성 = max(min_uncertainty_m, k·position_std_m) + degraded_margin_m(yaw는 k·yaw_std_rad)으로 넓힌 외곽이
  그 통로·그 통로의 접근 구역과 겹치지 않으면 OUTSIDE(position_valid=true), 겹치면 UNKNOWN. INSIDE·APPROACHING은 확정하지 않는다.
  std가 비유한·0 이하·상한 초과면 LOST와 같게 모든 통로 UNKNOWN(HEALTH_STD_INVALID). 목록 meta.valid=true·사유 없음, 로그에 기록.
- 영역 파일이 미승인(approved=false)이면 meta.valid=false, validity_reason REGIONS_NOT_APPROVED, 모든 통로 UNKNOWN이다.
- 통로 허가·해제·정지 명령은 하지 않는다(D3·A4 소유).
"""
import math
from pathlib import Path
import time
import uuid

from ax_environment.outline import model_outline
from ax_environment.regions import HEADING_TOLERANCE_RAD, load_regions, observe_robot
from ax_interfaces.msg import CorridorObservation, CorridorObservationList, LocalizationHealth, StateMeta
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros

# 계약 2.4절 STATE 프로필(B2 localization_supervisor.py STATE_QOS와 같음)
STATE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
SCHEMA_VERSION = 1
SOURCE_ID = 'b3_corridor_observer'
ROBOT_IDS = ('robot_1', 'robot_2', 'robot_3', 'robot_4')
STATES = {'OUTSIDE': CorridorObservation.OUTSIDE, 'APPROACHING': CorridorObservation.APPROACHING,
          'INSIDE': CorridorObservation.INSIDE, 'UNKNOWN': CorridorObservation.UNKNOWN}
HEALTH_NAMES = {LocalizationHealth.UNKNOWN: 'UNKNOWN', LocalizationHealth.OK: 'OK',
                LocalizationHealth.DEGRADED: 'DEGRADED', LocalizationHealth.LOST: 'LOST'}


# ROS Time 메시지 → 초
def seconds(stamp):
    # 역할: 메시지 시각을 초 단위 실수로 변환합니다.
    return stamp.sec + stamp.nanosec * 1e-9


# 쿼터니언 → yaw
def yaw_of(q):
    # 역할: 회전 사원수에서 평면 방향각을 구합니다.
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# 패키지 share 폴더(설치 환경). 없으면 None
def share_dir(package):
    # 역할: 설치된 패키지의 공유 경로를 찾습니다.
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package))
    except Exception:  # noqa: BLE001 - 패키지 없음은 설정 오류로 따로 보고한다
        return None


class CorridorObserver(Node):
    """로봇 1대의 통로 관측을 주기적으로 판정·발행한다."""

    # 파라미터를 읽고 영역·외곽을 준비한 뒤 구독·발행·타이머를 만든다.
    def __init__(self):
        # 역할: 설정과 내부 상태를 준비합니다.
        super().__init__('corridor_observer')

        # 1) 파라미터
        self.robot_id = self.declare_parameter('robot_id', '').value
        self.robot_model = self.declare_parameter('robot_model', '').value
        regions_file = self.declare_parameter('regions_file', '').value
        description_config = self.declare_parameter('description_config_dir', '').value
        margin = float(self.declare_parameter('outline_margin_m', 0.0).value)
        self.map_revision = int(self.declare_parameter('map_revision', 1).value)
        rate = float(self.declare_parameter('publish_rate_hz', 5.0).value)
        self.max_pose_age_s = float(self.declare_parameter('max_pose_age_s', 1.0).value)
        self.future_tolerance_s = float(self.declare_parameter('future_tolerance_s', 1.0).value)
        self.health_timeout_s = float(self.declare_parameter('health_timeout_s', 1.0).value)
        self.sigma_k = float(self.declare_parameter('position_sigma_k', 2.0).value)
        self.min_uncertainty_m = float(self.declare_parameter('min_uncertainty_m', 0.0).value)
        self.heading_tolerance = float(self.declare_parameter('heading_tolerance_rad', HEADING_TOLERANCE_RAD).value)
        # DEGRADED 완화(개발값): 0.5 m는 B2 estimate_travel_m(AMCL 갱신 없이 odom 0.5 m 이동 시 DEGRADED)와 같게 둔 추가 여유.
        # std 상한 1.0 m·0.5 rad는 B2 position_std_lost_m·yaw_std_lost_rad(넘으면 LOST)와 같다(localization_health.yaml).
        self.degraded_margin_m = float(self.declare_parameter('degraded_margin_m', 0.5).value)
        self.max_position_std_m = float(self.declare_parameter('max_position_std_m', 1.0).value)
        self.max_yaw_std_rad = float(self.declare_parameter('max_yaw_std_rad', 0.5).value)
        self.degraded_active = False

        if self.robot_id not in ROBOT_IDS:
            raise ValueError(f'robot_id는 {ROBOT_IDS} 중 하나여야 합니다: {self.robot_id!r}')

        # 2) 영역 파일(실패해도 노드는 UNKNOWN을 낸다)
        self.catalog, self.catalog_error = None, None
        default_share = share_dir('ax_environment') or Path('.')
        path = Path(regions_file) if regions_file else default_share / 'config/ax_warehouse_regions.json'
        try:
            self.catalog = load_regions(path)
        except (OSError, ValueError) as error:
            self.catalog_error = f'{type(error).__name__}: {error}'
        self.corridor_ids = [r['id'] for r in (self.catalog or {}).get('regions', []) if r['kind'] == 'corridor']

        # 3) 로봇 외곽(역할을 robot_id에서 추론하지 않고 robot_model 파라미터로 받는다)
        self.outline, self.outline_error = None, None
        description_share = share_dir('ax_description') or Path('.')
        config_dir = Path(description_config) if description_config else description_share / 'config'
        try:
            self.outline, self.outline_basis = model_outline(self.robot_model, config_dir, margin)
        except (OSError, ValueError, KeyError) as error:
            self.outline_error = f'{type(error).__name__}: {error}'

        # 4) 상태·통신
        self.session = uuid.uuid4().hex
        self.revision = 0
        self.last_key = None
        self.health = None
        self.health_received = None
        self.health_session = None
        self.health_revision = 0
        self.retired_sessions = set()
        self.base_frame = f'{self.robot_id}/base_link'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(LocalizationHealth, 'localization/health', self.on_health, STATE_QOS)
        self.publisher = self.create_publisher(CorridorObservationList, 'perception/corridor_observation', STATE_QOS)
        self.create_timer(1.0 / rate, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        approval = '승인' if (self.catalog or {}).get('approved') else '미승인'
        error = f', 오류 {self.catalog_error}' if self.catalog_error else ''
        self.get_logger().info(
            f'{self.robot_id} 통로 관측 시작(session {self.session[:8]}, 모델 {self.robot_model or "-"}, '
            f'외곽 {self.outline or self.outline_error}, 영역 {path} {approval}{error})')

    # health 수신: 같은 세션의 revision 감소와 이미 대체된 세션은 버린다(계약 2.3절).
    def on_health(self, message):
        # B2의 상태를 받아도 곧바로 믿지 않습니다. 대상 로봇·세션·수정 번호·시각을 확인합니다.
        # 역할: 위치 신뢰성의 로봇과 세션 및 수정 번호를 검사해 수신합니다.
        if message.robot_id != self.robot_id:
            return

        session, revision = message.meta.source_session_id, message.meta.revision
        if session in self.retired_sessions:
            return
        # 형식 오류나 미초기화 입력은 현재 B2 세션을 폐기하지 않습니다.
        # 다만 무효 입력 자체는 반영해 이전의 안전한 관측을 계속 보고하지 않습니다.
        # 새 유효 세션이 확인될 때까지 통로를 안전하다고 추정하지 않습니다.
        now = self.get_clock().now().nanoseconds * 1e-9
        sample = seconds(message.meta.header.stamp)
        sample_fresh = (math.isfinite(now) and now > 0.0 and sample > 0.0
                        and -self.future_tolerance_s <= now - sample <= self.health_timeout_s)
        # 오래된 새 세션은 기존 정상 세션을 폐기하지 않습니다. 현재 관측은 무효 처리해 복구를 기다립니다.
        if (not session or revision == 0 or
                (session != self.health_session and (not message.meta.valid or not sample_fresh))):
            self.health = message
            self.health_received = time.monotonic()
            return
        if session == self.health_session and revision < self.health_revision:
            return

        # 새 세션의 첫 메시지를 받으면 이전 세션은 버린다(이후 이전 세션 메시지가 와도 무시).
        if self.health_session is not None and session != self.health_session:
            self.retired_sessions.add(self.health_session)

        self.health_session, self.health_revision = session, revision
        self.health = message
        self.health_received = time.monotonic()

    # 현재 health가 위치 판정에 쓸 수 있는지를 (사유, degraded)로 돌려준다. 사유가 있으면 모든 통로 UNKNOWN이다.
    # - OK: 이전과 같다(std 비유한·음수만 거절).
    # - DEGRADED(valid·나이 OK): std가 유한·0 초과·상한 이하일 때만 완화 판정, 아니면 LOST와 같게 HEALTH_STD_INVALID.
    # - LOST·UNKNOWN·무효·만료·미수신: 모든 통로 UNKNOWN.
    def health_problem(self, now):
        # 역할: 위치 신뢰성의 시각과 오차를 검사해 판정 불가 사유를 반환합니다.
        message = self.health
        if message is None:
            return 'HEALTH_UNAVAILABLE', False
        if (not message.meta.valid or message.meta.revision == 0
                or not message.meta.source_session_id):
            return 'HEALTH_INVALID', False
        if message.state not in (LocalizationHealth.OK, LocalizationHealth.DEGRADED):
            return f'HEALTH_{HEALTH_NAMES.get(message.state, "UNKNOWN")}', False

        sample = seconds(message.meta.header.stamp)
        sample_age = now - sample
        silence = time.monotonic() - self.health_received
        fresh = (math.isfinite(now) and now > 0.0 and sample > 0.0
                 and -self.future_tolerance_s <= sample_age <= self.health_timeout_s)
        if not fresh or silence > self.health_timeout_s:
            return 'HEALTH_STALE', False

        position_std, yaw_std = message.position_std_m, message.yaw_std_rad
        finite = math.isfinite(position_std) and math.isfinite(yaw_std)

        if message.state == LocalizationHealth.DEGRADED:
            usable = (finite and 0 < position_std <= self.max_position_std_m and 0 < yaw_std <= self.max_yaw_std_rad)
            return (None, True) if usable else ('HEALTH_STD_INVALID', False)

        if not (finite and position_std >= 0 and yaw_std >= 0):
            return 'HEALTH_STD_INVALID', False
        return None, False

    # 한 번 판정해 {통로: (상태 이름, position_valid)}와 목록 사유를 돌려준다.
    def evaluate(self, now):
        # 승인·시각·외곽·위치 신뢰성·TF를 확인한 뒤 기하 판정을 합니다. 통행 허가는 하지 않습니다.
        # B4는 목록 전체를 검사하므로 빈 목록의 전체 참 판정을 주의합니다.
        # 비어 있는 통로 목록을 유효한 비점유 관측으로 게시하지 않습니다.
        # 역할: 현재 위치와 로봇 외곽에서 통로별 관측을 계산합니다.
        if self.catalog is None or not self.corridor_ids:
            return {}, 'REGIONS_INVALID'
        if not self.catalog.get('approved'):
            return {c: ('UNKNOWN', False) for c in self.corridor_ids}, 'REGIONS_NOT_APPROVED'
        if not math.isfinite(now) or now <= 0.0:
            return {c: ('UNKNOWN', False) for c in self.corridor_ids}, 'CLOCK_INVALID'
        if self.outline is None:
            return {c: ('UNKNOWN', False) for c in self.corridor_ids}, 'OUTLINE_UNDEFINED'

        problem, degraded = self.health_problem(now)
        self.note_degraded(degraded and not problem)
        if problem:
            return {c: ('UNKNOWN', False) for c in self.corridor_ids}, problem

        try:
            transform = self.tf_buffer.lookup_transform('map', self.base_frame, Time())
        except tf2_ros.TransformException:
            return {c: ('UNKNOWN', False) for c in self.corridor_ids}, 'POSE_UNAVAILABLE'

        t = transform.transform
        pose = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        age = now - seconds(transform.header.stamp)
        uncertainty = max(self.min_uncertainty_m, self.sigma_k * self.health.position_std_m)
        if degraded:
            uncertainty += self.degraded_margin_m
        observations = observe_robot(
            self.catalog, self.outline, pose, frame_id='map', map_revision=self.map_revision,
            position_valid=True, outline_complete=True, age_s=age, max_age_s=self.max_pose_age_s,
            uncertainty_m=uncertainty, yaw_uncertainty_rad=self.sigma_k * self.health.yaw_std_rad,
            heading_tolerance_rad=self.heading_tolerance, future_tolerance_s=self.future_tolerance_s,
            degraded=degraded)

        # DEGRADED_NEAR(통로 근처 UNKNOWN)는 그 통로만의 결과이고 목록 전체 판정 불가가 아니다.
        reasons = {o.reason for o in observations.values() if o.state == 'UNKNOWN' and o.reason != 'DEGRADED_NEAR'}
        if reasons:
            reason = 'POSE_STALE' if 'POSITION_OR_OUTLINE_INVALID' in reasons else sorted(reasons)[0]
            return {c: ('UNKNOWN', False) for c in self.corridor_ids}, reason
        return {c: (o.state, o.position_valid) for c, o in observations.items()}, ''

    # DEGRADED 완화 판정 시작·종료를 로그로 남긴다(validity_reason은 valid=false용이라 비워 두므로).
    def note_degraded(self, active):
        # 역할: 위치 신뢰성 저하에 따른 완화 판정 변경을 로그로 남깁니다.
        if active != self.degraded_active:
            self.degraded_active = active
            self.get_logger().info('위치 DEGRADED: 넓힌 외곽(+%.2f m)이 통로·접근 구역 밖인 통로만 OUTSIDE, 나머지 UNKNOWN'
                                   % self.degraded_margin_m if active else '위치 DEGRADED 완화 판정 종료')

    # 주기 판정·발행(5 Hz). 내용·유효성·사유가 바뀔 때만 revision을 올린다.
    def tick(self):
        # 승연님 B4가 목록과 개별 관측을 모두 검사하므로 각각의 유효성·시각·로봇 번호를 채웁니다.
        # 역할: 현재 위치로 통로 관측을 계산하고 내용 변경 시 수정 번호를 올려 게시합니다.
        now = self.get_clock().now().nanoseconds * 1e-9
        result, reason = self.evaluate(now)

        key = (tuple(sorted(result.items())), reason)
        if key != self.last_key:
            self.last_key = key
            self.revision += 1

        message = CorridorObservationList()
        message.robot_id = self.robot_id
        message.meta = self.meta(now, reason)

        for corridor_id in self.corridor_ids:
            state, position_valid = result.get(corridor_id, ('UNKNOWN', False))
            item = CorridorObservation()
            item.meta = self.meta(now, reason)
            item.corridor_id = corridor_id
            item.robot_id = self.robot_id
            item.observation = STATES[state]
            item.position_valid = bool(position_valid and not reason)
            message.observations.append(item)

        self.publisher.publish(message)

    # 공통 StateMeta를 만든다. 사유가 있으면 valid=false다.
    def meta(self, now, reason):
        # 역할: 공통 상태 메타데이터와 유효성 사유를 만듭니다.
        meta = StateMeta()
        stamp_s = max(now, 0.0) if math.isfinite(now) else 0.0
        meta.header.stamp = Time(nanoseconds=int(stamp_s * 1e9)).to_msg()
        meta.header.frame_id = 'map'
        meta.schema_version = SCHEMA_VERSION
        meta.source_id = SOURCE_ID
        meta.source_session_id = self.session
        meta.revision = self.revision
        meta.valid = not reason
        meta.validity_reason = reason
        return meta


# 노드 실행 진입점
def main(args=None):
    # 역할: 실행 인자를 받아 노드를 시작하고 종료 시 자원을 정리합니다.
    rclpy.init(args=args)
    node = CorridorObserver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
