# B2 비교 시험용 앞·뒤 LiDAR 병합(scan_merge.py) 수학 시험. ROS 없이 순수 함수만 검사한다.
import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1].parent
SCRIPT = ROOT / "src/ax_localization/scripts/scan_merge.py"
spec = importlib.util.spec_from_file_location("scan_merge_under_test", SCRIPT)
sm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sm)

FRONT_MOUNT = (0.61, 0.0, 0.0)          # sensor_mounts.yaml ax2 front_lidar
REAR_MOUNT = (-0.57, 0.0, math.pi)      # sensor_mounts.yaml ax2 rear_lidar


# 한 방향에만 반사가 있는 스캔 사전을 만든다(나머지 +inf).
def scan(beams=360, angle_min=-math.pi / 2, increment=math.pi / 359, hits=None, rmin=0.1, rmax=10.0):
    ranges = [math.inf] * beams
    for index, value in (hits or {}).items():
        ranges[index] = value
    return {"ranges": ranges, "angle_min": angle_min, "angle_increment": increment, "range_min": rmin, "range_max": rmax}


# 병합 결과에서 base_link 각도에 해당하는 칸의 거리
def at(ranges, angle):
    index = int(math.floor((angle - sm.ANGLE_MIN) / sm.ANGLE_INCREMENT + 0.5)) % sm.BEAMS
    return ranges[index]


class ScanPoints(unittest.TestCase):
    # 앞 LiDAR 정면 2 m 반사는 base_link x = 0.61 + 2
    def test_front_forward_point(self):
        p = sm.scan_points([2.0], 0.0, 0.1, 0.1, 10.0, FRONT_MOUNT)
        np.testing.assert_allclose(p, [[2.61, 0.0]], atol=1e-12)

    # 뒤 LiDAR(yaw π) 정면 1 m 반사는 base_link x = −0.57 − 1
    def test_rear_mount_rotation(self):
        p = sm.scan_points([1.0], 0.0, 0.1, 0.1, 10.0, REAR_MOUNT)
        np.testing.assert_allclose(p, [[-1.57, 0.0]], atol=1e-12)

    # 뒤 LiDAR 왼쪽(+90°) 반사는 차체 오른쪽(−y)
    def test_rear_left_is_body_right(self):
        p = sm.scan_points([1.0], math.pi / 2, 0.1, 0.1, 10.0, REAR_MOUNT)
        np.testing.assert_allclose(p, [[-0.57, -1.0]], atol=1e-12)

    # 범위 밖·NaN·±inf는 반사점이 아니다
    def test_invalid_ranges_dropped(self):
        p = sm.scan_points([0.05, math.nan, math.inf, -math.inf, 11.0, 3.0], 0.0, 0.0, 0.1, 10.0, (0.0, 0.0, 0.0))
        np.testing.assert_allclose(p, [[3.0, 0.0]])


class PolarMin(unittest.TestCase):
    # 같은 칸에 두 점이 있으면 가까운 거리, 빈 칸은 +inf
    def test_min_per_bin_and_empty(self):
        out = sm.polar_min(np.array([[2.0, 0.0], [1.5, 0.0], [0.0, 3.0]]))
        self.assertAlmostEqual(at(out, 0.0), 1.5)
        self.assertAlmostEqual(at(out, math.pi / 2), 3.0)
        self.assertTrue(math.isinf(at(out, -math.pi / 2)))
        self.assertEqual(len(out), sm.BEAMS)

    # −π와 +π 근처 점은 같은 칸(0번)으로 감긴다
    def test_wraparound(self):
        out = sm.polar_min(np.array([[-2.0, 1e-9], [-3.0, -1e-9]]))
        self.assertAlmostEqual(out[0], 2.0)

    # 점이 없으면 모두 +inf
    def test_no_points(self):
        self.assertTrue(np.isinf(sm.polar_min(np.zeros((0, 2)))).all())


class Merge(unittest.TestCase):
    # 앞 정면 벽과 뒤 정면 벽이 base_link 0과 π 방향에 각각 나타난다
    def test_front_and_rear_walls(self):
        front = scan(hits={179: 2.0})            # 앞 스캔 0° 근처(−π/2 + 179·π/359 ≈ −0.0044 rad)
        rear = scan(hits={179: 1.0})
        out = sm.merge(front, rear, FRONT_MOUNT, REAR_MOUNT)

        self.assertAlmostEqual(at(out, 0.0), 2.61, places=2)
        self.assertAlmostEqual(at(out, math.pi), 1.57, places=2)
        self.assertEqual(int(np.isfinite(out).sum()), 2)

    # 앞 스캔만 있고 뒤가 비면 결과는 앞 반사점만 담는다
    def test_rear_empty(self):
        out = sm.merge(scan(hits={0: 1.0}), scan(), FRONT_MOUNT, REAR_MOUNT)
        self.assertEqual(int(np.isfinite(out).sum()), 1)


class ClosestIndex(unittest.TestCase):
    # slop 안의 가장 가까운 stamp를 고르고, 밖이면 None
    def test_pairing(self):
        self.assertEqual(sm.closest_index([1.0, 1.1, 1.2], 1.12, 0.06), 1)
        self.assertIsNone(sm.closest_index([1.0, 1.1], 1.5, 0.06))
        self.assertIsNone(sm.closest_index([], 1.0, 0.06))


class PairDecisions(unittest.TestCase):
    # 뒤 흐름이 앞 시각에 못 미치면 보류, 오면 가장 가까운 짝 또는 None
    def test_waits_for_rear_stream(self):
        self.assertEqual(sm.pair_decisions([1.0], [], 0.06), [])
        self.assertEqual(sm.pair_decisions([1.0, 1.1], [0.9, 1.0], 0.06), [(0, 1)])
        self.assertEqual(sm.pair_decisions([1.0, 1.1], [1.0, 1.2], 0.06), [(0, 0), (1, None)])


@unittest.skipIf(importlib.util.find_spec("rclpy") is None, "rclpy 없음")
class Node(unittest.TestCase):
    # 노드 생성(구독 콜백 서명 검사 포함)과 앞·뒤 콜백으로 병합 스캔 1개 발행
    def test_build_and_merge_callback(self):
        import rclpy
        from sensor_msgs.msg import LaserScan

        rclpy.init(args=[])
        try:
            node = sm.build_node()
            node.mounts = {"robot_2/front_lidar_link": FRONT_MOUNT, "robot_2/rear_lidar_link": REAR_MOUNT}
            sent = []
            node.pub.publish = sent.append

            def message(frame, sec):
                m = LaserScan()
                m.header.frame_id, m.header.stamp.sec = frame, sec
                m.angle_min, m.angle_increment, m.range_min, m.range_max = -math.pi / 2, math.pi / 359, 0.1, 10.0
                m.ranges = [2.0] + [math.inf] * 359
                return m

            node.on_front(message("robot_2/front_lidar_link", 5))     # 앞이 먼저 도착: 뒤 흐름이 오기 전엔 보류
            self.assertEqual(node.counts["published"], 0)
            node.on_rear(message("robot_2/rear_lidar_link", 5))
            node.on_front(message("robot_2/front_lidar_link", 9))     # 뒤 흐름이 9 s까지 오기 전엔 보류
            node.on_rear(message("robot_2/rear_lidar_link", 12))      # 9 s 앞 스캔은 짝 없음으로 결정

            self.assertEqual(node.counts, {"front": 2, "rear": 2, "published": 1, "no_rear_pair": 1, "no_mount": 0})
            self.assertEqual(len(node.front), 0)
            self.assertEqual(sent[0].header.frame_id, "robot_2/base_link")
            self.assertEqual(len(sent[0].ranges), sm.BEAMS)
            self.assertEqual(sum(math.isfinite(v) for v in sent[0].ranges), 2)
            node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    unittest.main()
