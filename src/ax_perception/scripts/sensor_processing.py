#!/usr/bin/env python3
"""B1: 원본 시각/좌표 보존, 거리 무효값 처리, RGB-D 묶음, 수신 건강 상태.

시뮬레이션의 정렬된 무왜곡 RGB-D만 지원한다. 광학 좌표의 점군을 생성하며
실물 보정 추정이나 위험/적재 판정을 수행하지 않는다.
"""
from collections import deque
from copy import deepcopy
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo, Image, LaserScan, Range, PointCloud2, PointField


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def checked_distance(value, minimum, maximum):
    # 너무 가까운 -inf와 너무 먼 +inf를 구분하고 NaN을 접촉으로 바꾸지 않는다.
    if not (math.isfinite(minimum) and math.isfinite(maximum) and 0 < minimum < maximum):
        raise ValueError('Invalid distance limits')
    if math.isnan(value) or math.isinf(value):
        return value
    return -math.inf if value < minimum else math.inf if value > maximum else value


def rotation_matrix(x, y, z, w):
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError('Invalid rotation')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def body_triangles(description):
    """차체 visual OBJ를 사용한다. 큰 collision 상자로 실제 빈 공간까지 지우지 않는다."""
    robot = ET.fromstring(description)
    base = robot.find("link[@name='base_link']")
    if base is None:
        raise ValueError('Missing base_link')
    chunks = []
    for visual in base.findall('visual'):
        mesh = visual.find('geometry/mesh')
        if mesh is None:
            raise ValueError('Body self-filter requires OBJ visual geometry')
        uri = mesh.get('filename', '')
        if not uri.startswith('package://'):
            raise ValueError('Body mesh must use package URI')
        package, relative = uri[10:].split('/', 1)
        if Path(relative).is_absolute() or '..' in Path(relative).parts:
            raise ValueError('Unsupported body mesh path')
        folder = Path(get_package_share_directory(package))
        path = (folder / relative).resolve()
        # colcon --symlink-install의 메시 링크는 소스 폴더를 가리킬 수 있다.
        if path.suffix.lower() != '.obj':
            raise ValueError('Unsupported body mesh path')
        vertices, faces = [], []
        for line in path.read_text().splitlines():
            row = line.split()
            if not row:
                continue
            if row[0] == 'v':
                vertices.append(list(map(float, row[1:4])))
            elif row[0] == 'f':
                indices = [int(token.split('/')[0]) for token in row[1:]]
                indices = [i-1 if i > 0 else len(vertices)+i for i in indices]
                faces.extend([[indices[0], indices[i], indices[i+1]] for i in range(1, len(indices)-1)])
        triangles = np.asarray(vertices)[np.asarray(faces)] * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        element = visual.find('origin')
        xyz = np.zeros(3) if element is None else np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        r, p, y = np.zeros(3) if element is None else np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        # URDF 고정축 회전 Rz(yaw) Ry(pitch) Rx(roll).
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
        chunks.append(triangles @ rotation.T + xyz)
    if not chunks:
        raise ValueError('Missing body mesh')
    result = np.concatenate(chunks)
    if not np.isfinite(result).all():
        raise ValueError('Invalid body vertices')
    return result


def ray_body_distance(start, direction, triangles):
    """센서 +X 광선과 차체 메시의 첫 교차 거리. 측정 최소거리 안쪽도 검사한다."""
    if not np.isfinite(start).all() or not np.isfinite(direction).all():
        raise ValueError('Invalid sensor transform')
    a = triangles[:, 0]
    e1, e2 = triangles[:, 1]-a, triangles[:, 2]-a
    h = np.cross(np.broadcast_to(direction, e2.shape), e2)
    det = np.einsum('ij,ij->i', e1, h)
    valid = np.abs(det) > 1e-10
    inv = np.zeros(len(det))
    inv[valid] = 1 / det[valid]
    s = start-a
    u = inv*np.einsum('ij,ij->i', s, h)
    q = np.cross(s, e1)
    v = inv*(q @ direction)
    distance = inv*np.einsum('ij,ij->i', e2, q)
    valid &= (u >= -1e-8) & (v >= -1e-8) & (u+v <= 1+1e-8) & (distance > 1e-5)
    return float(distance[valid].min()) if valid.any() else math.inf


def body_blocks_measurement(measured, expected, maximum, tolerance):
    # 차체보다 분명히 가까운 물체는 유지한다. 경계에서 구분 불가능한 값은 무효다.
    return (math.isfinite(expected) and expected <= maximum and
            not (measured == -math.inf or math.isfinite(measured) and measured < expected-tolerance))


def checked_image(message, depth=False):
    formats = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    pixel = 4 if depth and message.encoding == '32FC1' else formats.get(message.encoding)
    if depth and message.encoding != '32FC1':
        raise ValueError('Depth must be 32FC1 meters')
    if not pixel or message.width <= 0 or message.height <= 0:
        raise ValueError('Unsupported image format or size')
    if message.step < message.width * pixel or len(message.data) != message.step * message.height:
        raise ValueError('Invalid image buffer layout')


def cleaned_depth(message, minimum, maximum):
    checked_image(message, depth=True)
    result = deepcopy(message)
    endian = '>' if message.is_bigendian else '<'
    values = np.ndarray((message.height, message.width), dtype=endian + 'f4',
                        buffer=message.data, strides=(message.step, 4))
    data = bytearray(message.data)
    output = np.ndarray(values.shape, dtype=endian + 'f4', buffer=data, strides=(message.step, 4))
    output[~np.isfinite(values) | (values < minimum) | (values > maximum)] = np.nan
    result.data = bytes(data)
    return result


def checked_camera_info(message):
    # 이 노드는 동일 광학 중심의 정렬 RGB-D만 처리한다. 실제 스테레오나
    # 왜곡 영상에 단안 공식을 적용해 그럴듯한 3D 좌표를 만들지 않는다.
    k, p = np.asarray(message.k), np.asarray(message.p)
    if message.width <= 0 or message.height <= 0 or not np.isfinite(np.r_[k, p, message.d]).all():
        raise ValueError('Invalid CameraInfo')
    if k[0] <= 0 or k[4] <= 0 or not np.allclose(k[[1, 3, 6, 7, 8]], [0, 0, 0, 0, 1]):
        raise ValueError('Unsupported camera intrinsics')
    expected = [k[0], 0, k[2], 0, 0, k[4], k[5], 0, 0, 0, 1, 0]
    if not np.allclose(p, expected, rtol=1e-5, atol=1e-6) or not np.allclose(message.d, 0):
        raise ValueError('RGB-D requires undistorted single-camera projection')


def optical_cloud(depth, info, rgb):
    """Z=깊이, X=오른쪽, Y=아래. 원시 Gazebo 점군의 축/초점거리 가정을 쓰지 않는다."""
    checked_camera_info(info)
    checked_image(depth, depth=True)
    checked_image(rgb)
    if len({(m.width, m.height) for m in (depth, info, rgb)}) != 1:
        raise ValueError('RGB-D point cloud size mismatch')
    h, w = depth.height, depth.width
    z = np.ndarray((h, w), dtype=('>' if depth.is_bigendian else '<') + 'f4',
                   buffer=depth.data, strides=(depth.step, 4))
    points = np.empty((h, w), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    points['z'] = z
    points['x'] = (np.arange(w) - info.k[2])[None, :] * z / info.k[0]
    points['y'] = (np.arange(h) - info.k[5])[:, None] * z / info.k[4]
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}[rgb.encoding]
    colors = np.ndarray((h, w, channels), dtype='u1', buffer=rgb.data,
                        strides=(rgb.step, channels, 1)).astype(np.uint32)
    if rgb.encoding == 'mono8':
        red = green = blue = colors[:, :, 0]
    elif rgb.encoding.startswith('bgr'):
        blue, green, red = colors[:, :, 0], colors[:, :, 1], colors[:, :, 2]
    else:
        red, green, blue = colors[:, :, 0], colors[:, :, 1], colors[:, :, 2]
    points['rgb'] = (red << 16) | (green << 8) | blue
    return PointCloud2(header=deepcopy(depth.header), height=h, width=w,
        fields=[PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                for index, name in enumerate(('x', 'y', 'z', 'rgb'))],
        is_bigendian=False, point_step=16, row_step=w * 16,
        data=points.tobytes(), is_dense=bool(np.isfinite(z).all()))


class SensorProcessing(Node):
    def __init__(self):
        super().__init__('sensor_processing')
        defaults = {'robot_id': 'robot_1', 'manipulator': True, 'tool_tof': True,
                    'max_age_s': 0.5, 'future_tolerance_s': 0.05,
                    'sync_slop_s': 0.03, 'missing_timeout_s': 2.0,
                    'depth_min_m': 0.05, 'depth_max_m': 5.0,
                    'tof_body_filter': True, 'tof_body_tolerance_m': 0.005,
                    'tof_tf_wait_s': 0.15}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for name in ('max_age_s', 'future_tolerance_s', 'sync_slop_s', 'missing_timeout_s', 'tof_tf_wait_s'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(name + ' must be positive')
        checked_distance(1.0, self.cfg['depth_min_m'], self.cfg['depth_max_m'])
        self.robot = self.cfg['robot_id']
        if not math.isfinite(self.cfg['tof_body_tolerance_m']) or self.cfg['tof_body_tolerance_m'] < 0:
            raise ValueError('tof_body_tolerance_m must be nonnegative')
        self.body_mesh = None
        self.body_error = 'Waiting for robot_description'
        if self.cfg['tool_tof'] and self.cfg['tof_body_filter']:
            # 시뮬레이션 시계 초기화 시 TF 캐시도 같은 시계로 초기화한다.
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.description_sub = self.create_subscription(String, 'robot_description', self.description,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.health, self.last_stamp, self.buffers, self.outputs = {}, {}, {}, {}
        self.clock_previous = 0.0
        self.owned_subs = []
        self.diagnostics = self.create_publisher(DiagnosticArray, 'sensors/health', 10)
        for name in ('front_lidar', 'rear_lidar'):
            pub = self.create_publisher(LaserScan, f'sensors/{name}/scan_filtered', SENSOR)
            self.subscribe(LaserScan, f'{name}/scan', name + '_link',
                           lambda msg, pub=pub: self.scan(msg, pub))
        if self.cfg['tool_tof']:
            self.tof_pub = self.create_publisher(Range, 'sensors/tool_tof', SENSOR)
            self.tof_pending = deque()
            self.subscribe(Range, 'tool_tof/raw', 'tool_tof_link', self.queue_tof)
            self.create_timer(.01, self.drain_tof, clock=Clock(clock_type=ClockType.STEADY_TIME))
        for name in (('front_rgb', 'arm_rgb') if self.cfg['manipulator'] else ('front_rgb',)):
            pub = self.create_publisher(Image, f'sensors/{name}/processed/image_raw', SENSOR)
            self.subscribe(Image, name + '/image_raw', name + '_optical_frame',
                           lambda msg, pub=pub: self.rgb(msg, pub))
        for name in (('front_rgbd', 'arm_rgbd') if self.cfg['manipulator'] else ('front_rgbd',)):
            self.buffers[name] = {part: deque(maxlen=10) for part in ('image', 'depth_image', 'camera_info')}
            self.health[name + '/synchronized'] = {'received': None, 'ok': False, 'reason': 'Waiting for synchronized RGB-D'}
            self.health[name + '/depth_valid'] = {'received': None, 'ok': False, 'reason': 'Waiting for usable depth'}
            self.outputs[name, 'points'] = self.create_publisher(PointCloud2, f'sensors/{name}/processed/points', SENSOR)
            for part, cls in (('image', Image), ('depth_image', Image), ('camera_info', CameraInfo)):
                self.outputs[name, part] = self.create_publisher(cls, f'sensors/{name}/processed/{part}', SENSOR)
                self.subscribe(cls, f'{name}/{part}', name + '_optical_frame',
                               lambda msg, name=name, part=part: self.rgbd(name, part, msg))
        # 시뮬레이션 일시정지/센서 단절 중에도 진단은 벽시계로 계속 발행한다.
        self.create_timer(0.5, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe_clock(self, now):
        if now < self.clock_previous:
            self.last_stamp.clear()
            if hasattr(self, 'tof_pending'):
                self.tof_pending.clear()
            for queues in self.buffers.values():
                for queue in queues.values():
                    queue.clear()
            # 이전 실행에서 정상이라는 진단을 새 실행으로 넘기지 않는다.
            for state in self.health.values():
                state.update(received=None, ok=False, reason='Clock reset; awaiting new data')
        self.clock_previous = now

    def subscribe(self, cls, suffix, frame, handler):
        self.health[suffix] = {'received': None, 'ok': False, 'reason': 'No data'}
        def receive(message):
            state = self.health[suffix]
            try:
                now = self.get_clock().now().nanoseconds * 1e-9
                self.observe_clock(now)
                state['received'] = time.monotonic()
                measured = stamp(message)
                if now <= 0 or measured <= 0 or now - measured > self.cfg['max_age_s']:
                    raise ValueError('Missing clock or stale measurement')
                if measured - now > self.cfg['future_tolerance_s']:
                    raise ValueError('Future measurement')
                if measured <= self.last_stamp.get(suffix, -1):
                    raise ValueError('Duplicate or reversed timestamp')
                if message.header.frame_id != self.robot + '/' + frame:
                    raise ValueError('Unexpected frame_id')
                completed = handler(message)
                self.last_stamp[suffix] = measured
                if completed is not False:
                    state.update(ok=True, reason='Valid input')
            except (ValueError, TypeError, OverflowError) as error:
                state.update(ok=False, reason=str(error))
        self.owned_subs.append(self.create_subscription(cls, 'sensors/' + suffix, receive, SENSOR))

    def queue_tof(self, message):
        if len(self.tof_pending) >= 10:
            self.tof_pending.popleft()
            self.health['tool_tof/raw'].update(ok=False, reason='ToF TF queue overflow; oldest input dropped')
        self.tof_pending.append((message, time.monotonic()))
        return False

    def drain_tof(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.observe_clock(now)
        while self.tof_pending:
            message, received = self.tof_pending[0]
            state = self.health['tool_tof/raw']
            if now-stamp(message) > self.cfg['max_age_s']:
                self.tof_pending.popleft()
                state.update(ok=False, reason='ToF measurement became stale while waiting for TF')
                continue
            # 센서와 TF의 전송 순서는 보장되지 않는다. 최신 TF로 대체하지 않고
            # 한정된 시간만 기다린다. 단일 실행기 콜백을 막는 sleep/wait는 쓰지 않는다.
            if (self.cfg['tof_body_filter'] and self.body_mesh is not None and
                    time.monotonic()-received < self.cfg['tof_tf_wait_s'] and
                    not self.tf_buffer.can_transform(self.robot + '/base_link',
                        message.header.frame_id, Time.from_msg(message.header.stamp))):
                break
            self.tof_pending.popleft()
            try:
                self.tof(message)
                state.update(ok=True, reason='Valid input')
            except (ValueError, TypeError, OverflowError) as error:
                state.update(ok=False, reason=str(error))

    def description(self, message):
        try:
            self.body_mesh = body_triangles(message.data)
            self.body_error = ''
        except (ValueError, OSError, ET.ParseError, LookupError, IndexError, TypeError) as error:
            self.body_mesh = None
            self.body_error = 'Body model unavailable: ' + str(error)

    def tof(self, message):
        result = deepcopy(message)
        result.range = checked_distance(message.range, message.min_range, message.max_range)
        reason = ''
        if self.cfg['tof_body_filter']:
            try:
                if self.body_mesh is None:
                    raise ValueError(self.body_error)
                # 최신 TF가 아니라 측정 시각의 TF로 팔 자세를 맞춘다.
                transform = self.tf_buffer.lookup_transform(self.robot + '/base_link',
                    message.header.frame_id, Time.from_msg(message.header.stamp)).transform
                q, t = transform.rotation, transform.translation
                direction = rotation_matrix(q.x, q.y, q.z, q.w)[:, 0]
                expected = ray_body_distance(np.array([t.x, t.y, t.z]), direction, self.body_mesh)
                if body_blocks_measurement(result.range, expected, message.max_range, self.cfg['tof_body_tolerance_m']):
                    reason = f'ToF ray blocked by robot body at {expected:.4f} m'
            except (TransformException, ValueError) as error:
                reason = 'ToF self-filter unavailable: ' + str(error)
            if reason:
                result.range = math.nan
        self.tof_pub.publish(result)
        if reason:
            raise ValueError(reason)
        if math.isnan(result.range):
            raise ValueError('Invalid ToF measurement (NaN)')

    def scan(self, message, publisher):
        if not message.ranges or not math.isfinite(message.angle_increment) or message.angle_increment <= 0:
            raise ValueError('Invalid scan layout')
        result = deepcopy(message)
        result.ranges = [checked_distance(v, message.range_min, message.range_max) for v in message.ranges]
        publisher.publish(result)
        if all(math.isnan(v) for v in result.ranges):
            raise ValueError('All scan measurements invalid')

    def rgb(self, message, publisher):
        checked_image(message)
        publisher.publish(message)

    def rgbd(self, name, part, message):
        if part == 'camera_info':
            checked_camera_info(message)
        else:
            checked_image(message, depth=part == 'depth_image')
        queues = self.buffers[name]
        queues[part].append(message)
        if any(not q for q in queues.values()):
            return
        # 유한 큐에서 새 입력과 가장 가까운 측정 시각을 고른다. 임의 시각 재기록 금지.
        chosen = {key: min(q, key=lambda m: abs(stamp(m) - stamp(message))) for key, q in queues.items()}
        times = [stamp(m) for m in chosen.values()]
        if max(times) - min(times) > self.cfg['sync_slop_s']:
            return
        if self.get_clock().now().nanoseconds * 1e-9 - min(times) > self.cfg['max_age_s']:
            return
        if len({(m.width, m.height) for m in chosen.values()}) != 1:
            raise ValueError('RGB/depth/CameraInfo size mismatch')
        chosen['depth_image'] = cleaned_depth(chosen['depth_image'], self.cfg['depth_min_m'], self.cfg['depth_max_m'])
        depth = chosen['depth_image']
        values = np.ndarray((depth.height, depth.width), dtype=('>' if depth.is_bigendian else '<') + 'f4',
                            buffer=depth.data, strides=(depth.step, 4))
        valid_ratio = float(np.count_nonzero(np.isfinite(values))) / values.size
        cloud = optical_cloud(depth, chosen['camera_info'], chosen['image'])
        for key, msg in chosen.items():
            self.outputs[name, key].publish(msg)
            # 일치한 프레임과 그 이전은 버려 중복 쌍 발행·무한 적체를 막는다.
            while queues[key] and stamp(queues[key][0]) <= stamp(msg):
                queues[key].popleft()
        # 동기화 성공과 거리 정보의 유효성은 별개다. 전부 무효인 점군도
        # 그대로 발행해 이전 관측으로 대체하지 않되, 소비자에게 오류를 알린다.
        reason = f'Valid depth fraction {valid_ratio:.4f}'
        if valid_ratio == 0:
            reason = 'No valid depth measurements; ' + reason
        received = time.monotonic()
        self.health[name + '/synchronized'].update(received=received, ok=True,
            reason=f'RGB-D synchronized; skew {max(times)-min(times):.6f} s')
        self.health[name + '/depth_valid'] = dict(received=received, ok=valid_ratio > 0,
            reason=reason, valid_depth_fraction=valid_ratio)
        self.outputs[name, 'points'].publish(cloud)

    def report(self):
        self.observe_clock(self.get_clock().now().nanoseconds * 1e-9)
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for name, state in self.health.items():
            age = time.monotonic() - state['received'] if state['received'] is not None else math.inf
            row = DiagnosticStatus(name=self.robot + '/' + name, hardware_id=self.robot)
            row.level = DiagnosticStatus.STALE if age > self.cfg['missing_timeout_s'] else DiagnosticStatus.OK if state['ok'] else DiagnosticStatus.ERROR
            row.message = 'No recent input' if row.level == DiagnosticStatus.STALE else state['reason']
            row.values = [KeyValue(key='wall_receive_age_s', value=str(age))]
            if 'valid_depth_fraction' in state:
                row.values.append(KeyValue(key='valid_depth_fraction', value=str(state['valid_depth_fraction'])))
            message.status.append(row)
        self.diagnostics.publish(message)


def main():
    rclpy.init()
    node = SensorProcessing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
