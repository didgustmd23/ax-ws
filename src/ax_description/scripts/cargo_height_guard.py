#!/usr/bin/env python3
"""Downward camera height monitor and loading permission gate (metres).

The loader must check loading/check before placing and obey loading/stop
continuously. This node does not operate a manipulator or release held objects.
"""
import json
import math
import time

import numpy as np

# 적재판을 수직으로 내려다보는 카메라 전용이다. 모든 높이·거리는 m 단위.
# 물리적 적재 실행기는 loading/check 허가와 loading/stop 신호를 직접 따라야 한다.
# 이 파일은 팔 정지·박스 방출 명령을 보내지 않고 높이 판단과 허가 상태만 제공한다.

def decode_depth(msg):
    """Respect ROS image row padding, encoding and byte order."""
    # ROS 깊이 영상: 32FC1은 실수 m, 16UC1은 정수 mm로 해석한다.
    dtype = {'32FC1': 'f4', '16UC1': 'u2'}.get(msg.encoding)
    if dtype is None:
        raise ValueError('unsupported depth encoding')
    # 송신 측 바이트 순서와 행 끝 패딩(step)을 보존하여 다른 장치의 영상도 읽는다.
    dt = np.dtype(('>' if msg.is_bigendian else '<') + dtype)
    if msg.step < msg.width * dt.itemsize or len(msg.data) < msg.height * msg.step:
        raise ValueError('truncated depth image')
    depth = np.ndarray((msg.height, msg.width), dtype=dt,
                       buffer=bytes(msg.data), strides=(msg.step, dt.itemsize))
    # 이후 계산의 단위를 m로 통일한다.
    return depth.astype(float) * (0.001 if msg.encoding == '16UC1' else 1.0)


def measure_height(depth, k, camera_height, limit, width, length, valid_fraction, pixel_center_offset=0.5):
    """Optical Z is vertical down. Inspect points inside the deck footprint.

    Require the camera to cover the entire deck at the height limit. Invalid
    depth in that field blocks permission instead of implying an empty deck.
    """
    # 카메라 내부 보정값: fx/fy=초점거리(px), cx/cy=광학 중심(px).
    fx, fy, cx, cy = k
    if not all(math.isfinite(v) for v in k) or min(fx, fy) <= 0:
        raise ValueError('invalid camera calibration')
    # 픽셀 좌표를 광학 Z축 기준 광선 기울기로 변환한다.
    # 기울기에 깊이를 곱하면 광학 X/Y 평면의 실제 거리(m)가 된다.
    rows, cols = np.indices(depth.shape)
    # Gazebo의 렌더링 픽셀은 셀 중심(+0.5)에서 표본화된다. 이를 빼먹으면 경계 밖
    # 골격이 적재 영역 안으로 투영된다. 실물 카메라는 보정 규약에 맞게 offset을 지정한다.
    rx, ry = (cols+pixel_center_offset-cx)/fx, (rows+pixel_center_offset-cy)/fy
    # 한계 높이 평면까지의 거리. 가장 가까운 허용 평면에서도 적재판 전체가 보여야 한다.
    gap = camera_height-limit
    if gap <= 0 or min(cx, depth.shape[1]-1-cx)/fx*gap < width/2 or min(cy, depth.shape[0]-1-cy)/fy*gap < length/2:
        raise ValueError('camera does not cover deck at limit')
    # 한계 평면에서 적재 영역에 해당하는 시야를 검사 영역으로 선택한다.
    field = (np.abs(rx*gap) <= width/2) & (np.abs(ry*gap) <= length/2)
    # 렌더링 깊이 범위 밖, 무한대, NaN은 빈 공간으로 간주하지 않는다.
    valid = np.isfinite(depth) & (depth >= 0.05) & (depth <= 5.0)
    if not field.any() or valid[field].mean() < valid_fraction:
        raise ValueError('insufficient valid depth')
    # 실제 측정점이 적재판 범위 안에 있는 픽셀만 높이 계산에 사용한다.
    inside = field & valid & (np.abs(rx*depth) <= width/2) & (np.abs(ry*depth) <= length/2)
    if not inside.any():
        raise ValueError('no deck measurements')
    # 카메라에 가장 가까운 점이 가장 높은 적재점이다. 평균을 쓰면 작은 돌출물을 놓친다.
    return max(0.0, float(camera_height-depth[inside].min()))


def loading_blocked(height, limit, reserve=0.0):
    # 판단 불가, 현재 한계 도달, 다음 상자를 더했을 때 한계 초과 중 하나면 금지.
    return height is None or height >= limit or height+reserve > limit


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Bool, Float32, String
    from std_srvs.srv import Trigger

    # ROS 토픽·서비스 연결부. 위의 수치 계산 함수는 ROS 없이도 별도로 검증할 수 있다.
    class Guard(Node):
        def __init__(self):
            super().__init__('cargo_height_guard')
            # YAML 미지정 시 기본값. 파일 설정은 시작 시에 읽으며 변경 후 재시작한다.
            defaults = dict(max_height_m=.5, camera_height_m=1.026,
                            deck_width_m=.72, deck_length_m=.72, timeout_s=1.,
                            min_valid_fraction=.95, next_box_height_m=0., pixel_center_offset=.5)
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.cfg = {name: self.get_parameter(name).value for name in defaults}
            # 잘못된 설정으로 적재를 허용하지 않도록 시작 단계에서 범위를 확인한다.
            c = self.cfg
            if not all(math.isfinite(v) for v in c.values()) or not (0 < c['max_height_m'] < c['camera_height_m'] and c['timeout_s'] > 0 and c['deck_width_m'] > 0 and c['deck_length_m'] > 0 and 0 < c['min_valid_fraction'] <= 1 and c['next_box_height_m'] >= 0):
                raise ValueError('invalid cargo height configuration')
            # 최초 영상·보정값을 받기 전에는 허가하지 않는다.
            self.k = None
            self.height = None
            self.received = None
            self.reason = 'NO_DEPTH'
            # 마지막 금지 상태를 보관해 늦게 접속한 적재 제어기도 바로 상태를 받게 한다.
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            # stop=true는 추가 적재 금지, height_m은 판 상면 기준 최고 높이, status는 사유 JSON.
            self.stop_pub = self.create_publisher(Bool, 'loading/stop', qos)
            self.height_pub = self.create_publisher(Float32, 'loading/height_m', 10)
            self.status_pub = self.create_publisher(String, 'loading/status', qos)
            # 센서 데이터에는 best-effort QoS를 사용해 Gazebo 브리지 발행 설정과 맞춘다.
            self.create_subscription(CameraInfo, 'sensors/cargo_rgbd/camera_info', self.info, qos_profile_sensor_data)
            self.create_subscription(Image, 'sensors/cargo_rgbd/depth_image', self.depth, qos_profile_sensor_data)
            # 적재 시작 직전 Trigger 서비스를 호출하면 현재 허가 여부를 응답한다.
            self.create_service(Trigger, 'loading/check', self.check)
            # 10Hz 실시간 감시. 시뮬레이션이 멈춰도 데이터 수신 만료 판정은 계속한다.
            self.create_timer(.1, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.publish()

        def info(self, msg):
            # 깊이 영상과 같은 해상도의 보정 행렬인지 depth 콜백에서 검사한다.
            self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.info_size = (msg.height, msg.width)

        def depth(self, msg):
            # 수신 시각은 실제 경과 시간으로 기록해 시뮬레이션 일시정지의 영향을 피한다.
            c = self.cfg
            self.received = time.monotonic()
            try:
                if self.k is None or self.info_size != (msg.height, msg.width):
                    raise ValueError('missing or mismatched calibration')
                # 메시지 자체의 시각은 ROS 시계와 비교한다. 지연되거나 미래 시각이면 거절한다.
                age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
                if not 0 <= age <= c['timeout_s']:
                    raise ValueError('stale depth timestamp')
                self.height = measure_height(decode_depth(msg), self.k,
                    c['camera_height_m'], c['max_height_m'], c['deck_width_m'],
                    c['deck_length_m'], c['min_valid_fraction'], c['pixel_center_offset'])
                self.reason = 'HEIGHT_LIMIT' if loading_blocked(self.height, c['max_height_m'], c['next_box_height_m']) else 'READY'
            except (ValueError, TypeError) as exc:
                # 보정값 누락·손상 영상도 적재 금지로 처리하고 원인을 발행한다.
                self.height = None
                self.reason = str(exc)
            self.publish()

        def state(self):
            # 콜백이 오지 않는 장애도 검출한다. 미수신과 시간 초과는 모두 금지다.
            if self.received is None or time.monotonic()-self.received > self.cfg['timeout_s']:
                return True, 'NO_FRESH_DEPTH'
            return loading_blocked(self.height, self.cfg['max_height_m'], self.cfg['next_box_height_m']), self.reason

        def publish(self):
            # 측정 불가 높이는 0 대신 NaN으로 발행하여 빈 적재함과 구분한다.
            stop, reason = self.state()
            self.stop_pub.publish(Bool(data=stop))
            self.height_pub.publish(Float32(data=float('nan') if stop and reason == 'NO_FRESH_DEPTH' or self.height is None else self.height))
            self.status_pub.publish(String(data=json.dumps(dict(stop=stop, reason=reason,
                max_height_m=self.cfg['max_height_m']))))

        def check(self, request, response):
            # Trigger.success=true만 적재 허가다. 서비스를 호출한 후에도 stop을 계속 감시해야 한다.
            stop, reason = self.state()
            response.success = not stop
            response.message = reason
            return response

    # ROS 이벤트 루프를 시작하고 종료 시 노드와 통신 자원을 정리한다.
    rclpy.init(args=args)
    node = Guard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
