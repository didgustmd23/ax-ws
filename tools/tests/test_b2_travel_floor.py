# B2 이동 거리 비례 위치 std 하한(localization_health.TravelStdFloor·evaluate std_floor)을 ROS 없이 검사한다.
# 기본 설정은 하한 꺼짐(unconstrained_std_per_m 0, 통로 Gazebo 시험 기준 미달: log/b2_restart_impl/gazebo/corridor/table.md).
# 하한 동작은 켰을 때를 위해 명시 설정 0.025(시험값)로 계속 검사한다.
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_localization/scripts'))

import localization_health as lh  # noqa: E402

DEFAULT_CONFIG = lh.merge_config()
CONFIG = lh.merge_config({'health': {'unconstrained_std_per_m': 0.025}})


# 정상 입력(모든 입력 최신). odom 이동량 정체 검사가 걸리지 않게 amcl_odom_pose를 odom과 같게 둔다.
def inputs(std=0.05, yaw_std=0.03, now=10.0, odom=(1.0, 0.0, 0.0)):
    return lh.Inputs(
        scan_stamp=now - 0.1, odom_stamp=now - 0.02, odom_pose=odom,
        tf_valid_time=now - 0.1, map_odom=(0.0, 0.0, 0.0), tf_chain_ok=True,
        amcl_stamp=now - 0.1, amcl_pose=odom,
        amcl_covariance=tuple(lh.pose_covariance(std, yaw_std)), amcl_odom_pose=odom, amcl_updates=3)


# 하한 적용 판정
class FloorInEvaluate(unittest.TestCase):

    # 하한이 공분산 std보다 크면 그 값을 위치 std로 보고하고, 0.25 m를 넘으면 COVARIANCE_HIGH다.
    def test_floor_applied(self):
        health = lh.evaluate(inputs(std=0.05), 10.0, CONFIG, std_floor=0.2)
        self.assertEqual((health.state, health.position_std_m), (lh.OK, 0.2))

        health = lh.evaluate(inputs(std=0.05), 10.0, CONFIG, std_floor=0.3)
        self.assertEqual((health.state, health.reason, health.position_std_m), (lh.DEGRADED, 'COVARIANCE_HIGH', 0.3))

    # 하한이 작으면 공분산 std 그대로, 하한 0은 기존 판정과 같다.
    def test_floor_smaller(self):
        self.assertEqual(lh.evaluate(inputs(std=0.2), 10.0, CONFIG, std_floor=0.1).position_std_m, 0.2)
        self.assertEqual(lh.evaluate(inputs(std=0.2), 10.0, CONFIG, std_floor=0.0),
                         lh.evaluate(inputs(std=0.2), 10.0, CONFIG))

    # 공분산 붕괴 검사는 하한 적용 전 값으로 한다(하한이 있어도 붕괴는 COVARIANCE_COLLAPSED).
    def test_collapse_uses_raw(self):
        health = lh.evaluate(inputs(std=0.0, yaw_std=0.0), 10.0, CONFIG, std_floor=0.1)

        self.assertEqual(health.reason, 'COVARIANCE_COLLAPSED')


# 경로 누적과 재설정
class FloorTracking(unittest.TestCase):

    # 경로 × unconstrained_std_per_m. 잘 구속된(OK, std ≤ constrained_std_m) 판정이면 0으로 돌아간다.
    def test_accumulate_and_reset(self):
        floor = lh.TravelStdFloor(CONFIG)
        per_m = CONFIG['health']['unconstrained_std_per_m']

        for x in range(0, 11):
            floor.on_odom((float(x), 0.0, 0.0))

        self.assertAlmostEqual(floor.path_m, 10.0)
        self.assertAlmostEqual(floor.floor(), per_m * 10.0)

        # 구속되지 않은 판정(std 0.2 > 0.12)·DEGRADED·붕괴는 재설정하지 않는다.
        for raw in (lh.Health(lh.OK, True, '', 0.2, 0.05), lh.Health(lh.DEGRADED, True, 'COVARIANCE_HIGH', 0.1, 0.05),
                    lh.Health(lh.DEGRADED, True, 'COVARIANCE_COLLAPSED', 0.0, 0.0)):
            floor.observe(raw)
            self.assertAlmostEqual(floor.path_m, 10.0)

        floor.observe(lh.Health(lh.OK, True, '', 0.1, 0.05))
        self.assertEqual(floor.floor(), 0.0)

    # 왕복도 모두 센다(순변위가 아니라 누적 경로). 정지 중에는 늘지 않는다.
    def test_round_trip_and_stop(self):
        floor = lh.TravelStdFloor(CONFIG)

        for x in (0.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0):
            floor.on_odom((x, 0.0, 0.0))

        self.assertAlmostEqual(floor.path_m, 4.0)

    # odom 끊김 표본은 경로에 더하지 않고 기준만 다시 잡는다. 비유한 표본은 건너뛴다.
    def test_discontinuity(self):
        floor = lh.TravelStdFloor(CONFIG)
        floor.on_odom((0.0, 0.0, 0.0))
        floor.on_odom((5.0, 0.0, 0.0), 'ODOM_JUMP')
        floor.on_odom((float('nan'), 0.0, 0.0))
        floor.on_odom((5.5, 0.0, 0.0))

        self.assertAlmostEqual(floor.path_m, 0.5)

    # 재위치 성공 등으로 reset하면 0이다.
    def test_reset(self):
        floor = lh.TravelStdFloor(CONFIG)
        floor.on_odom((0.0, 0.0, 0.0))
        floor.on_odom((3.0, 0.0, 0.0))
        floor.reset()

        self.assertEqual(floor.floor(), 0.0)

    # 0이면 끈다.
    def test_disabled(self):
        config = lh.merge_config({'health': {'unconstrained_std_per_m': 0.0}})
        floor = lh.TravelStdFloor(config)
        floor.on_odom((0.0, 0.0, 0.0))
        floor.on_odom((30.0, 0.0, 0.0))

        self.assertEqual(floor.floor(), 0.0)

    # C3 재현 모양: 구속된 출발 뒤 통로 12 m에서 오차 0.38 m·AMCL std 0.22면 하한 0.30으로 DEGRADED(과신 아님),
    # C1 모양: 통로 출구(구속 뒤 7.5 m)에서 AMCL std 0.2면 하한 0.19로 OK 유지.
    def test_corridor_shapes(self):
        per_m = CONFIG['health']['unconstrained_std_per_m']

        c3 = lh.evaluate(inputs(std=0.22), 10.0, CONFIG, std_floor=per_m * 12.0)
        self.assertEqual(c3.state, lh.DEGRADED)
        self.assertLessEqual(0.38 / c3.position_std_m, 1.5)

        c1 = lh.evaluate(inputs(std=0.2), 10.0, CONFIG, std_floor=per_m * 7.5)
        self.assertEqual(c1.state, lh.OK)



# 기본 설정(하한 꺼짐)
class DefaultOff(unittest.TestCase):

    # 기본값은 0이고, 경로가 길어도 하한은 0이며 판정은 하한 없는 판정과 같다.
    def test_default_floor_zero(self):
        self.assertEqual(DEFAULT_CONFIG['health']['unconstrained_std_per_m'], 0.0)

        floor = lh.TravelStdFloor(DEFAULT_CONFIG)

        for x in range(0, 31):
            floor.on_odom((float(x), 0.0, 0.0))

        self.assertAlmostEqual(floor.path_m, 30.0)
        self.assertEqual(floor.floor(), 0.0)

        for std in (0.05, 0.2):
            self.assertEqual(lh.evaluate(inputs(std=std), 10.0, DEFAULT_CONFIG, std_floor=floor.floor()),
                             lh.evaluate(inputs(std=std), 10.0, DEFAULT_CONFIG))

    # 설치 설정 파일도 기본 꺼짐이다(코드 기본값과 파일 값이 어긋나지 않게).
    def test_config_file_zero(self):
        import yaml

        path = ROOT / 'src/ax_localization/config/localization_health.yaml'
        data = yaml.safe_load(path.read_text())

        self.assertEqual(data['health']['unconstrained_std_per_m'], 0.0)


if __name__ == '__main__':
    unittest.main()
