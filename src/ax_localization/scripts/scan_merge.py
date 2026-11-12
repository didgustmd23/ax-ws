#!/usr/bin/env python3
"""B2 비교 시험용 앞·뒤 LiDAR 병합 노드(시험용 최소 구현, 운영 입력 아님).

  ros2 run ax_localization scan_merge.py --ros-args -r __ns:=/ax/robot_2 -p robot_id:=robot_2 -p use_sim_time:=true

- 입력: B1 처리본 `sensors/front_lidar/scan_filtered`·`sensors/rear_lidar/scan_filtered`. 차체 자기 가림은 B1 처리본 기준이다.
- 장착 자세: tf_static의 <robot>/base_link → <robot>/front_lidar_link·rear_lidar_link에서 한 번 읽는다(평면 x, y, yaw만).
- 출력: `sensors/merged/scan_filtered`(LaserScan, frame <robot>/base_link, −π부터 0.5° 간격 720빔).
  두 스캔의 유효 반사점을 base_link 평면 좌표로 옮긴 뒤, base_link 원점 기준 각도 칸마다 가장 가까운 거리를 남긴다.
  반사가 없는 칸은 +inf다.
- 짝짓기: 앞·뒤 스캔을 모두 대기열에 두고, 뒤 스캔 흐름이 앞 스캔 시각까지 도착한 뒤(가장 늦은 뒤 stamp ≥ 앞 stamp)
  stamp 차가 pair_slop_s(기본 0.06 s) 안인 가장 가까운 뒤 스캔을 쓴다. 그때 짝이 없으면 내지 않고 센다.
  같은 시각 스캔의 도착 순서(앞 먼저·뒤 먼저)에 관계없이 짝을 짓는다. 출력 stamp는 앞 스캔 stamp다. 두 스캔 사이 차체 이동 보정은 하지 않는다(시험용 단순화).
- 범위 밖·NaN·±inf 거리는 반사점으로 쓰지 않는다(B1의 −inf 근접 표시도 제외).
"""
import math

import numpy as np

BEAMS = 720
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / BEAMS


# 한 스캔의 유효 반사점을 장착 자세(x, y, yaw)로 base_link 평면 좌표(N×2)로 옮긴다.
def scan_points(ranges, angle_min, angle_increment, range_min, range_max, mount):
    r = np.asarray(ranges, float)
    angles = angle_min + angle_increment * np.arange(len(r))

    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    lx, ly = r[valid] * np.cos(angles[valid]), r[valid] * np.sin(angles[valid])

    mx, my, myaw = mount
    c, s = math.cos(myaw), math.sin(myaw)
    return np.c_[mx + c * lx - s * ly, my + s * lx + c * ly]


# base_link 평면 점들을 원점 기준 각도 칸에 넣고 칸마다 가장 가까운 거리를 남긴다. 빈 칸은 +inf.
def polar_min(points, beams=BEAMS, angle_min=ANGLE_MIN):
    increment = 2.0 * math.pi / beams
    out = np.full(beams, np.inf)

    if len(points) == 0:
        return out

    distance = np.hypot(points[:, 0], points[:, 1])
    angle = np.arctan2(points[:, 1], points[:, 0])

    index = np.floor((angle - angle_min) / increment + 0.5).astype(int) % beams
    np.minimum.at(out, index, distance)
    return out


# 앞·뒤 스캔 사전(ranges, angle_min, angle_increment, range_min, range_max)과 장착 자세로 병합 거리 배열을 만든다.
def merge(front, rear, front_mount, rear_mount, beams=BEAMS):
    points = np.concatenate([
        scan_points(front["ranges"], front["angle_min"], front["angle_increment"],
                    front["range_min"], front["range_max"], front_mount),
        scan_points(rear["ranges"], rear["angle_min"], rear["angle_increment"],
                    rear["range_min"], rear["range_max"], rear_mount),
    ])
    return polar_min(points, beams)


# 뒤 스캔 후보 stamp 중 앞 stamp와 가장 가까운 것의 위치. slop 밖이면 None.
def closest_index(stamps, stamp, slop):
    if not len(stamps):
        return None

    k = int(np.argmin(np.abs(np.asarray(stamps) - stamp)))
    return k if abs(stamps[k] - stamp) <= slop else None


# 대기 중 앞 스캔들의 짝 결정: 뒤 흐름이 앞 시각까지 왔으면 (앞 위치, 뒤 위치 또는 None), 아직이면 결정하지 않는다.
def pair_decisions(front_stamps, rear_stamps, slop):
    if not len(rear_stamps):
        return []

    latest = max(rear_stamps)
    return [(i, closest_index(rear_stamps, f, slop)) for i, f in enumerate(front_stamps) if latest >= f]


# 쿼터니언 z축 회전각
def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ROS 노드를 만든다(rclpy.init 뒤 호출): 장착 TF를 읽고 앞 스캔마다 짝 뒤 스캔과 합쳐 발행한다.
def build_node():
    from collections import deque

    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    # 병합 노드 본체
    class ScanMerge(Node):
        # 파라미터·구독·발행·TF를 준비한다.
        def __init__(self):
            super().__init__("scan_merge")
            self.robot = self.declare_parameter("robot_id", "robot_2").value
            self.slop = float(self.declare_parameter("pair_slop_s", 0.06).value)
            self.mounts = {}
            self.front, self.rear = deque(maxlen=20), deque(maxlen=20)
            self.counts = {"front": 0, "rear": 0, "published": 0, "no_rear_pair": 0, "no_mount": 0}

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pub = self.create_publisher(LaserScan, "sensors/merged/scan_filtered", qos_profile_sensor_data)
            # rclpy는 콜백 서명을 검사하므로 내장 메서드(deque.append)를 직접 넘기지 않는다.
            self.create_subscription(LaserScan, "sensors/rear_lidar/scan_filtered", self.on_rear, qos_profile_sensor_data)
            self.create_subscription(LaserScan, "sensors/front_lidar/scan_filtered", self.on_front, qos_profile_sensor_data)
            self.create_timer(10.0, lambda: self.get_logger().info(f"scan_merge {self.counts}"))

        # 뒤 스캔을 대기열에 넣고 짝을 처리한다.
        def on_rear(self, message):
            self.counts["rear"] += 1
            self.rear.append(message)
            self.drain()

        # 앞 스캔을 대기열에 넣고 짝을 처리한다.
        def on_front(self, message):
            self.counts["front"] += 1
            self.front.append(message)
            self.drain()

        # 결정 가능한 앞 스캔을 짝 뒤 스캔과 합쳐 내고 대기열에서 뺀다.
        def drain(self):
            seconds = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9  # noqa: E731
            decisions = pair_decisions([seconds(m) for m in self.front], [seconds(m) for m in self.rear], self.slop)
            done = set()

            for i, k in decisions:
                done.add(i)
                if k is None:
                    self.counts["no_rear_pair"] += 1
                else:
                    self.publish_pair(self.front[i], self.rear[k])

            self.front = deque((m for i, m in enumerate(self.front) if i not in done), maxlen=20)

        # 장착 자세(x, y, yaw)를 tf_static에서 한 번 읽어 둔다.
        def mount(self, frame):
            if frame not in self.mounts:
                try:
                    t = self.tf_buffer.lookup_transform(f"{self.robot}/base_link", frame, Time()).transform
                except Exception:  # noqa: BLE001  아직 tf_static이 오지 않음
                    return None
                self.mounts[frame] = (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
                self.get_logger().info(f"장착 자세 {frame}: {self.mounts[frame]}")
            return self.mounts[frame]

        # 앞·뒤 한 쌍을 병합 스캔으로 낸다.
        def publish_pair(self, front, rear):
            front_mount, rear_mount = self.mount(front.header.frame_id), self.mount(rear.header.frame_id)

            if front_mount is None or rear_mount is None:
                self.counts["no_mount"] += 1
                return

            ranges = merge(as_dict(front), as_dict(rear), front_mount, rear_mount)

            out = LaserScan()
            out.header.stamp, out.header.frame_id = front.header.stamp, f"{self.robot}/base_link"
            out.angle_min, out.angle_increment = ANGLE_MIN, ANGLE_INCREMENT
            out.angle_max = ANGLE_MIN + ANGLE_INCREMENT * (BEAMS - 1)
            out.range_min = 0.0
            out.range_max = max(front.range_max + math.hypot(*front_mount[:2]), rear.range_max + math.hypot(*rear_mount[:2]))
            out.scan_time, out.time_increment = front.scan_time, 0.0
            out.ranges = [float(v) for v in ranges]
            self.pub.publish(out)
            self.counts["published"] += 1

    # LaserScan을 병합 함수 입력 사전으로 바꾼다.
    def as_dict(message):
        return {"ranges": message.ranges, "angle_min": message.angle_min, "angle_increment": message.angle_increment,
                "range_min": message.range_min, "range_max": message.range_max}

    return ScanMerge()


# 노드를 띄워 SIGINT까지 돈다.
def main():
    import rclpy

    rclpy.init()
    node = build_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT 뒤에는 rclpy 컨텍스트가 닫혀 rosout 로그가 실패하므로 표준 출력으로 남긴다.
        print(f"scan_merge 종료 {node.counts}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
