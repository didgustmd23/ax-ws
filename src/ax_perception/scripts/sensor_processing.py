#!/usr/bin/env python3
"""B1: 원본 시각/좌표 보존, 거리 무효값 처리, RGB-D 묶음, 수신 건강 상태.

시뮬레이션의 정렬된 무왜곡 RGB-D만 지원한다. 광학 좌표의 점군을 생성하며
실물 보정 추정이나 위험/적재 판정을 수행하지 않는다.
"""
from collections import deque
from copy import deepcopy
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
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
                    'depth_min_m': 0.05, 'depth_max_m': 5.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for name in ('max_age_s', 'future_tolerance_s', 'sync_slop_s', 'missing_timeout_s'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(name + ' must be positive')
        checked_distance(1.0, self.cfg['depth_min_m'], self.cfg['depth_max_m'])
        self.robot = self.cfg['robot_id']
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
            self.subscribe(Range, 'tool_tof/raw', 'tool_tof_link', self.tof)
        for name in (('front_rgb', 'arm_rgb') if self.cfg['manipulator'] else ('front_rgb',)):
            pub = self.create_publisher(Image, f'sensors/{name}/processed/image_raw', SENSOR)
            self.subscribe(Image, name + '/image_raw', name + '_optical_frame',
                           lambda msg, pub=pub: self.rgb(msg, pub))
        for name in (('front_rgbd', 'arm_rgbd') if self.cfg['manipulator'] else ('front_rgbd',)):
            self.buffers[name] = {part: deque(maxlen=10) for part in ('image', 'depth_image', 'camera_info')}
            self.health[name + '/synchronized'] = {'received': None, 'ok': False, 'reason': 'Waiting for synchronized RGB-D'}
            self.outputs[name, 'points'] = self.create_publisher(PointCloud2, f'sensors/{name}/processed/points', SENSOR)
            for part, cls in (('image', Image), ('depth_image', Image), ('camera_info', CameraInfo)):
                self.outputs[name, part] = self.create_publisher(cls, f'sensors/{name}/processed/{part}', SENSOR)
                self.subscribe(cls, f'{name}/{part}', name + '_optical_frame',
                               lambda msg, name=name, part=part: self.rgbd(name, part, msg))
        # 시뮬레이션 일시정지/센서 단절 중에도 진단은 벽시계로 계속 발행한다.
        self.create_timer(0.5, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def subscribe(self, cls, suffix, frame, handler):
        self.health[suffix] = {'received': None, 'ok': False, 'reason': 'No data'}
        def receive(message):
            state = self.health[suffix]
            state['received'] = time.monotonic()
            try:
                now = self.get_clock().now().nanoseconds * 1e-9
                if now < self.clock_previous:
                    self.last_stamp.clear()
                    for queues in self.buffers.values():
                        for queue in queues.values():
                            queue.clear()
                self.clock_previous = now
                measured = stamp(message)
                if now <= 0 or measured <= 0 or now - measured > self.cfg['max_age_s']:
                    raise ValueError('Missing clock or stale measurement')
                if measured - now > self.cfg['future_tolerance_s']:
                    raise ValueError('Future measurement')
                if measured <= self.last_stamp.get(suffix, -1):
                    raise ValueError('Duplicate or reversed timestamp')
                if message.header.frame_id != self.robot + '/' + frame:
                    raise ValueError('Unexpected frame_id')
                handler(message)
                self.last_stamp[suffix] = measured
                state.update(ok=True, reason='Valid input')
            except (ValueError, TypeError, OverflowError) as error:
                state.update(ok=False, reason=str(error))
        self.owned_subs.append(self.create_subscription(cls, 'sensors/' + suffix, receive, SENSOR))

    def tof(self, message):
        result = deepcopy(message)
        result.range = checked_distance(message.range, message.min_range, message.max_range)
        self.tof_pub.publish(result)
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
        reason = f'RGB-D skew {max(times)-min(times):.6f} s; valid depth fraction {valid_ratio:.4f}'
        if valid_ratio == 0:
            reason = 'No valid depth measurements; ' + reason
        self.health[name + '/synchronized'].update(received=time.monotonic(),
                                                  ok=valid_ratio > 0, reason=reason)
        self.outputs[name, 'points'].publish(cloud)

    def report(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for name, state in self.health.items():
            age = time.monotonic() - state['received'] if state['received'] is not None else math.inf
            row = DiagnosticStatus(name=self.robot + '/' + name, hardware_id=self.robot)
            row.level = DiagnosticStatus.STALE if age > self.cfg['missing_timeout_s'] else DiagnosticStatus.OK if state['ok'] else DiagnosticStatus.ERROR
            row.message = 'No recent input' if row.level == DiagnosticStatus.STALE else state['reason']
            row.values = [KeyValue(key='wall_receive_age_s', value=str(age))]
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
