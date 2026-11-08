# B2 지도 작성·검증 계산(좌표 변환·자세 보간·점유 누적·정렬·양방향 일치·미관측·저장 실패)을 Gazebo 없이 검사한다.
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'src/ax_localization/scripts'
sys.path.insert(0, str(SCRIPTS))

import map_tools as mt  # noqa: E402


# 이름으로 스크립트 모듈을 읽는다. ROS 의존이 없으면 None
def load_script(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)

    try:
        spec.loader.exec_module(module)
    except ImportError:
        return None

    return module


# 사각 방(벽만 점유)의 격자. 내부는 빈 공간, 바깥은 미관측
def room_grid(width=60, height=40, wall=(5, 5, 55, 35)):
    grid = np.full((height, width), mt.UNKNOWN, np.int8)
    left, bottom, right, top = wall
    grid[bottom:top + 1, left:right + 1] = mt.FREE
    grid[bottom, left:right + 1] = mt.OCCUPIED
    grid[top, left:right + 1] = mt.OCCUPIED
    grid[bottom:top + 1, left] = mt.OCCUPIED
    grid[bottom:top + 1, right] = mt.OCCUPIED

    return grid


class Transforms(unittest.TestCase):
    def test_se2_round_trip_and_points(self):
        transform = mt.se2(1.0, -2.0, 0.7)
        self.assertEqual(tuple(round(v, 12) for v in mt.se2_params(transform)), (1.0, -2.0, 0.7))

        point = mt.transform_points(transform, [[1.0, 0.0]])[0]
        np.testing.assert_allclose(point, [1.0 + math.cos(0.7), -2.0 + math.sin(0.7)])

    def test_yaw_from_quaternion(self):
        self.assertAlmostEqual(mt.yaw_from_quaternion(0, 0, math.sin(-1.2), math.cos(-1.2)), -2.4)


class PoseInterpolation(unittest.TestCase):
    def setUp(self):
        self.times = np.array([0.0, 0.1, 0.2, 0.5])
        self.poses = np.array([[0, 0, 3.0], [1, 0, -3.0], [2, 0, -3.0], [3, 0, 0.0]])

    def test_midpoint_and_yaw_wrap(self):
        x, _, yaw = mt.interpolate_pose(self.times, self.poses, 0.05, 0.15)
        self.assertAlmostEqual(x, 0.5)

        # 3.0 → -3.0은 +π를 넘는 짧은 쪽(약 +0.28 rad)으로 돈다.
        self.assertAlmostEqual(abs(yaw), math.pi, delta=0.01)

    def test_gap_and_range_are_excluded(self):
        self.assertIsNone(mt.interpolate_pose(self.times, self.poses, 0.3, 0.15))
        self.assertIsNone(mt.interpolate_pose(self.times, self.poses, -0.01, 0.15))
        self.assertIsNone(mt.interpolate_pose(self.times, self.poses, 0.6, 0.15))
        self.assertEqual(mt.interpolate_pose(self.times, self.poses, 0.2, 0.15)[0], 2.0)


class ScanAndOccupancy(unittest.TestCase):
    def test_scan_rays_classes(self):
        ranges = [1.0, math.inf, -math.inf, math.nan, 0.05, 12.0]
        angles, distances, hit = mt.scan_rays(ranges, 0.0, 0.1, 0.12, 10.0)

        # 유한 1.0은 장애물, +inf와 10 m 초과는 빈 공간 광선, -inf·NaN·최소 미만은 버린다.
        np.testing.assert_allclose(angles, [0.0, 0.1, 0.5])
        np.testing.assert_allclose(distances, [1.0, 10.0, 10.0])
        self.assertEqual(list(hit), [True, False, False])

    def test_wall_free_and_unknown(self):
        geometry = mt.GridGeometry(-1.0, -1.0, 0.05, 80, 40)
        accumulator = mt.OccupancyAccumulator(geometry)

        # 원점 센서가 +x 방향 2 m 벽을 보는 좁은 부채꼴
        angles = np.linspace(-0.2, 0.2, 41)
        distances = 2.0 / np.cos(angles)
        accumulator.add_scan(mt.se2(0, 0, 0), angles, distances, np.ones_like(angles, bool))
        grid = accumulator.classify()

        rows, columns = geometry.cells([[2.0, 0.0], [1.0, 0.0], [2.5, 0.0], [0.0, 0.8]])
        self.assertEqual(grid[rows[0], columns[0]], mt.OCCUPIED)
        self.assertEqual(grid[rows[1], columns[1]], mt.FREE)
        self.assertEqual(grid[rows[2], columns[2]], mt.UNKNOWN)   # 벽 뒤
        self.assertEqual(grid[rows[3], columns[3]], mt.UNKNOWN)   # 시야 밖


class MapFiles(unittest.TestCase):
    def test_round_trip_keeps_unknown(self):
        grid = room_grid()
        geometry = mt.GridGeometry(-1.5, 2.0, 0.05, grid.shape[1], grid.shape[0])

        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'map.yaml'
            mt.write_map(path, grid, geometry)
            loaded, loaded_geometry, yaw = mt.read_map(path)

        np.testing.assert_array_equal(loaded, grid)
        self.assertEqual((loaded_geometry.origin_x, loaded_geometry.origin_y, yaw), (-1.5, 2.0, 0.0))


class Alignment(unittest.TestCase):
    def test_icp_recovers_offset(self):
        grid = room_grid()
        geometry = mt.GridGeometry(0, 0, 0.05, grid.shape[1], grid.shape[0])
        target = mt.occupied_points(grid, geometry)
        truth = mt.se2(0.12, -0.08, 0.03)
        source = mt.transform_points(np.linalg.inv(truth), target)

        found, info = mt.align_icp(source, target, np.eye(3))
        x, y, yaw = mt.se2_params(found)

        self.assertAlmostEqual(x, 0.12, delta=0.01)
        self.assertAlmostEqual(y, -0.08, delta=0.01)
        self.assertAlmostEqual(yaw, 0.03, delta=0.003)
        self.assertLess(info['rmse_m'], 0.01)


class BidirectionalMatch(unittest.TestCase):
    def test_identical_maps(self):
        grid = room_grid()
        metrics = mt.compare_grids(grid, grid.copy(), 0.05)

        self.assertEqual(metrics['obstacle_detection_ratio'], 1.0)
        self.assertEqual(metrics['reverse_match_ratio'], 1.0)
        self.assertEqual(metrics['wall_error_reference_to_compared']['max_m'], 0.0)

    def test_extra_obstacle_lowers_reverse_only(self):
        reference = room_grid()
        compared = reference.copy()
        compared[20, 20:30] = mt.OCCUPIED      # 참조에 없는 점유(빈 공간 위)
        metrics = mt.compare_grids(reference, compared, 0.05)

        self.assertEqual(metrics['obstacle_detection_ratio'], 1.0)
        self.assertLess(metrics['reverse_match_ratio'], 1.0)
        self.assertGreater(metrics['wall_error_compared_to_reference']['max_m'], 0.4)

    def test_unobserved_cells_are_not_compared(self):
        reference = room_grid()
        compared = reference.copy()
        compared[:, 30:] = mt.UNKNOWN          # 비교 지도가 오른쪽 절반을 보지 못함
        metrics = mt.compare_grids(reference, compared, 0.05)

        # 보지 못한 쪽 벽은 검출 실패로 세지 않고 참조만 관측한 면적으로 센다.
        self.assertEqual(metrics['obstacle_detection_ratio'], 1.0)
        self.assertGreater(metrics['coverage']['reference_only_area_m2'], 0.5)

    def test_one_cell_tolerance(self):
        reference = room_grid()
        shifted = np.roll(reference, 1, axis=1)
        self.assertEqual(mt.compare_grids(reference, shifted, 0.05, 1)['obstacle_detection_ratio'], 1.0)

        # 3칸 이동: 가로 벽은 길이 방향이라 겹치고 세로 벽만 놓친다. 벽 오차 최대는 3칸(0.15 m)이다.
        metrics = mt.compare_grids(reference, np.roll(reference, 3, axis=1), 0.05, 1)
        self.assertLess(metrics['obstacle_detection_ratio'], 1.0)
        self.assertAlmostEqual(metrics['wall_error_reference_to_compared']['max_m'], 0.15, places=6)


class ReferenceAndAdoption(unittest.TestCase):
    def setUp(self):
        self.tool = load_script('check_map_accuracy')

        if self.tool is None:
            self.skipTest('ROS·의존 패키지 필요')

    def test_reference_excludes_unsynchronized_scans(self):
        # 실제 자세 표본: 0~1 s는 0.05 s 간격, 1~2 s는 비어 있음, 2~3 s 다시 있음
        times = np.concatenate([np.arange(0, 1.0001, 0.05), np.arange(2.0, 3.0001, 0.05)])
        truth = np.stack([times, np.zeros_like(times), np.zeros_like(times), np.zeros_like(times)], axis=1)
        stamps = np.array([0.5, 1.5, 2.5, 3.5])
        recording = {
            'truth': truth,
            'scan_layout': np.array([-0.2, 0.01, 0.12, 10.0]),
            'front_mount': np.array([0.6, 0.0, 0.0]),
            'scan_stamps': stamps,
            'scan_ranges': np.full((4, 41), 3.0, np.float32),
        }

        _, _, stats, _ = self.tool.build_reference(recording, 0.1, 0.25)
        self.assertEqual(stats, {'scans': 4, 'used': 2, 'excluded_outside_truth': 1, 'excluded_gap': 1})

    def test_adopt_fails_without_files_and_keeps_existing(self):
        with tempfile.TemporaryDirectory() as folder:
            run, target = Path(folder) / 'run', Path(folder) / 'maps'
            run.mkdir()
            target.mkdir()
            (target / 'map.yaml').write_text('old', encoding='utf-8')
            (target / 'map_info.yaml').write_text('map_revision: 3\n', encoding='utf-8')

            # 지도 저장 파일만 있고 pose graph가 없으면 채택하지 않는다.
            grid = room_grid()
            mt.write_map(run / 'slam_map.yaml', grid, mt.GridGeometry(0, 0, 0.05, grid.shape[1], grid.shape[0]))

            with self.assertRaises(RuntimeError):
                self.tool.adopt_map(run, target, {})

            self.assertEqual((target / 'map.yaml').read_text(encoding='utf-8'), 'old')
            self.assertEqual(sorted(p.name for p in target.iterdir()), ['map.yaml', 'map_info.yaml'])

    def test_adopt_increments_revision(self):
        with tempfile.TemporaryDirectory() as folder:
            run, target = Path(folder) / 'run', Path(folder) / 'maps'
            run.mkdir()
            grid = room_grid()
            mt.write_map(run / 'slam_map.yaml', grid, mt.GridGeometry(0, 0, 0.05, grid.shape[1], grid.shape[0]))
            (run / 'pose_graph.posegraph').write_bytes(b'graph')
            (run / 'pose_graph.data').write_bytes(b'data')

            first = self.tool.adopt_map(run, target, {'note': 'a'})
            second = self.tool.adopt_map(run, target, {'note': 'b'})
            info = yaml.safe_load((target / 'map_info.yaml').read_text(encoding='utf-8'))

            self.assertEqual((first['map_revision'], second['map_revision'], info['map_revision']), (1, 2, 2))
            self.assertEqual(yaml.safe_load((target / 'map.yaml').read_text(encoding='utf-8'))['image'], 'map.pgm')
            self.assertEqual(info['files']['map.pgm'], mt.file_sha256(target / 'map.pgm'))
            self.assertFalse(any(p.name.startswith('.adopt_') for p in target.iterdir()))

    def test_trajectory_errors_separate_slam_and_odom(self):
        # 실제 = odom(오차 없음), SLAM의 map→odom에 0.3 rad 회전이 들어간 경우
        times = np.arange(0, 5.0001, 0.1)
        path = np.stack([times, 0.2 * times, np.zeros_like(times), np.zeros_like(times)], axis=1)
        recording = {
            'odom': np.c_[path, np.zeros((len(times), 2))],
            'map_odom': np.array([[0.0, 0.0, 0.0, 0.0], [2.0, 0.0, 0.0, 0.3]]),
        }
        world = mt.se2(2.5, -1.5, 0.0)
        truth = np.c_[times, mt.transform_points(world, path[:, 1:3]), path[:, 3]]

        result = self.tool.trajectory_errors(recording, truth, world, world, 0.15, period_s=0.5)

        self.assertAlmostEqual(result['odom_position_error_m']['max'], 0.0, places=9)
        self.assertAlmostEqual(result['slam_yaw_error_rad']['max'], 0.3, places=9)
        self.assertAlmostEqual(result['slam_yaw_error_rad']['mean'], 0.3 * 7 / 11, places=9)   # 2.0 s부터의 표본 7개만 회전

    def test_judgment_needs_criteria(self):
        metrics = mt.compare_grids(room_grid(), room_grid(), 0.05)
        self.assertEqual(self.tool.judge(metrics, None)['judgment'], 'not_judged')

        criteria = {'min_obstacle_detection_ratio': 0.9, 'min_reverse_match_ratio': 0.9,
                    'max_wall_error_p95_m': 0.1, 'min_comparable_area_m2': 1.0}
        self.assertEqual(self.tool.judge(metrics, criteria)['judgment'], 'PASS')


class DriveSafety(unittest.TestCase):
    def setUp(self):
        self.tool = load_script('mapping_drive')

        if self.tool is None:
            self.skipTest('ROS 환경 필요')

        self.footprint = {'front_m': 0.53, 'rear_m': 0.52, 'half_width_m': 0.5}

    def test_stop_distance(self):
        # 0.3 m/s, 감속 0.67 m/s², 지연 0.3 s, 여유 0.25 m
        self.assertAlmostEqual(self.tool.stop_distance(0.3, 0.67, 0.3, 0.25), 0.09 / 1.34 + 0.09 + 0.25)

    def test_forward_corridor(self):
        points = np.array([[1.0, 0.0], [1.0, 0.9], [2.5, 0.0], [-1.0, 0.0]])
        self.assertEqual(self.tool.corridor_blocked(points, self.footprint, 0.6, 0.25, 1), 1)
        self.assertEqual(self.tool.corridor_blocked(points, self.footprint, 0.6, 0.25, -1), 1)
        self.assertEqual(self.tool.corridor_blocked(np.zeros((0, 2)), self.footprint, 0.6, 0.25, 1), 0)

    def test_rotation_radius(self):
        radius = math.hypot(0.53, 0.5) + 0.15
        points = np.array([[radius - 0.01, 0.0], [0.0, radius + 0.01]])
        self.assertEqual(self.tool.rotation_blocked(points, self.footprint, 0.15), 1)

    def test_waypoint_command_rotates_first(self):
        limits = {'linear': 0.3, 'angular': 0.3}
        linear, angular, phase, _, _ = self.tool.waypoint_command((0, 0, 0), (0, 2), limits, 0.12)
        self.assertEqual((linear, phase), (0.0, 'rotate'))
        self.assertAlmostEqual(angular, 0.3)

        linear, _, phase, distance, _ = self.tool.waypoint_command((0, 0, math.pi / 2), (0, 2), limits, 0.12)
        self.assertEqual((phase, linear, distance), ('drive', 0.3, 2.0))

    def test_slam_health_items_are_separate(self):
        inputs = {'tf_receive_max_age_boottime_s': 2.0, 'slam_progress_distance_m': 1.0, 'slam_progress_heading_rad': 1.0}
        base = {'tf_receive_age_boottime_s': 0.02, 'travel_since_slam_update_m': 0.3, 'heading_since_slam_update_rad': 0.1}

        # 변환 시각이 늦어도(처리 간격 사이) 수신이 이어지고 이동이 작으면 문제 없음
        self.assertEqual(self.tool.slam_problems(dict(base, tf_stamp_lag_sim_s=1.7), inputs), [])

        # 수신 끊김과 처리 진척 없음은 따로 보고한다.
        self.assertEqual(len(self.tool.slam_problems(dict(base, tf_receive_age_boottime_s=2.5), inputs)), 1)
        self.assertIn('진척', self.tool.slam_problems(dict(base, travel_since_slam_update_m=1.2), inputs)[0])
        self.assertIn('진척', self.tool.slam_problems(dict(base, heading_since_slam_update_rad=1.3), inputs)[0])
        self.assertEqual(self.tool.slam_problems(dict(base, tf_receive_age_boottime_s=None), inputs), ['map→odom 미수신'])

    # 가짜 벽시계로 SlamMonitor를 만든다.
    def monitor(self):
        clock = {'t': 100.0}
        return self.tool.SlamMonitor('robot_2/odom', wall_clock=lambda: clock['t']), clock

    def test_monitor_ignores_other_transforms(self):
        monitor, clock = self.monitor()
        monitor.on_transform('map', 'robot_2/odom', 5.0, (0, 0, 0))
        clock['t'] += 3.0

        # 다른 로봇·다른 변환은 수신 시각을 갱신하지 않는다.
        self.assertFalse(monitor.on_transform('map', 'robot_1/odom', 6.0, (0, 0, 0)))
        self.assertFalse(monitor.on_transform('robot_2/odom', 'robot_2/base_link', 6.0, (0, 0, 0)))
        self.assertAlmostEqual(monitor.health(6.0, (0, 0, 0))['tf_receive_age_boottime_s'], 3.0)

    def test_monitor_repeated_old_transform_is_receipt_not_progress(self):
        monitor, clock = self.monitor()
        monitor.on_transform('map', 'robot_2/odom', 5.0, (0.0, 0.0, 0.0))

        # 같은 시각 변환이 계속 오면 수신은 신선하지만 처리 진척은 없다. 그동안 로봇이 1.5 m 움직였다.
        for step in range(20):
            clock['t'] += 0.02
            self.assertFalse(monitor.on_transform('map', 'robot_2/odom', 5.0, (0.0, 0.0, 0.0)))

        health = monitor.health(9.0, (1.5, 0.0, 0.0))
        inputs = {'tf_receive_max_age_boottime_s': 2.0, 'slam_progress_distance_m': 1.0, 'slam_progress_heading_rad': 1.0}
        problems = self.tool.slam_problems(health, inputs)

        self.assertLess(health['tf_receive_age_boottime_s'], 0.1)
        self.assertEqual((monitor.receipts, monitor.updates), (21, 1))
        self.assertEqual(len(problems), 1)
        self.assertIn('진척', problems[0])

    def test_monitor_detects_pause_by_boottime(self):
        monitor, clock = self.monitor()
        monitor.on_transform('map', 'robot_2/odom', 5.0, (0, 0, 0))

        # 시뮬레이션·SLAM이 멈춰 sim time은 그대로지만 벽시계는 흐른다.
        clock['t'] += 2.5
        health = monitor.health(5.0, (0, 0, 0))
        problems = self.tool.slam_problems(health, {'tf_receive_max_age_boottime_s': 2.0,
                                                    'slam_progress_distance_m': 1.0, 'slam_progress_heading_rad': 1.0})

        self.assertAlmostEqual(health['tf_receive_age_boottime_s'], 2.5)
        self.assertEqual(health['tf_stamp_lag_sim_s'], 0.0)
        self.assertIn('수신 끊김', problems[0])
        self.assertAlmostEqual(monitor.gaps[-1] if monitor.gaps else 0.0, 0.0)

    def test_route_file_is_valid(self):
        route = self.tool.load_route(ROOT / 'src/ax_localization/config/routes/aws_warehouse.yaml')
        self.assertEqual(route['frame'], 'robot_2/odom')
        self.assertEqual(route['waypoints'][-1][:2], route['waypoints'][route['loop_waypoint'] - 1][:2])   # 루프 기준점으로 복귀
        self.assertEqual(route['waypoints'][0], (-1.5, 0.0, True))

    def test_waypoint_command_reverse(self):
        limits = {'linear': 0.3, 'angular': 0.3}

        # 뒤쪽(-x)에 있는 경유점: 회전 없이 후진한다.
        linear, angular, phase, distance, _ = self.tool.waypoint_command((0, 0, 0), (-1.5, 0, True), limits, 0.12)
        self.assertEqual((phase, linear, angular, distance), ('reverse', -0.3, 0.0, 1.5))

        # 옆에 있으면 먼저 뒤쪽이 향하도록 돈다.
        _, angular, phase, _, error = self.tool.waypoint_command((0, 0, 0), (0, 2, True), limits, 0.12)
        self.assertEqual(phase, 'rotate')
        self.assertAlmostEqual(error, -math.pi / 2)


if __name__ == '__main__':
    unittest.main()
