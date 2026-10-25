"""arm_guard_logic의 명령 검사·보간 경로 검사·보호 판단을 ROS 없이 시험한다."""
import math
import random
import unittest

from ax_control import arm_guard_logic as logic
from ax_control.model_contracts import ARM_JOINTS

# 시험용 물리 한계(rad)와 계획 속도·가속도. 실제 모델과 같은 값을 쓴다.
PHYSICAL = {
    'column_yaw_joint': (-3.14159, 3.14159, 0.35),
    'shoulder_joint': (-1.5708, 1.5708, 0.50),
    'elbow_joint': (-2.0, 2.0, 0.50),
    'wrist_joint': (-1.57, 1.57, 0.70),
    'gripper_rotation_joint': (-3.14159, 3.14159, 0.70),
}
PLANNED = {
    'column_yaw_joint': {'max_velocity': 0.35, 'max_acceleration': 0.50},
    'shoulder_joint': {'max_velocity': 0.50, 'max_acceleration': 0.50},
    'elbow_joint': {'max_velocity': 0.50, 'max_acceleration': 0.70},
    'wrist_joint': {'max_velocity': 0.70, 'max_acceleration': 1.00},
    'gripper_rotation_joint': {'max_velocity': 0.70, 'max_acceleration': 1.00},
}
PARAMS = logic.ProtectionParams(reaction_time_s=0.1, tracking_allowance_rad=0.01,
                                min_clearance_rad=0.005, fault_clearance_rad=1e-6,
                                still_velocity_rad_s=0.01)
ZERO = [0.0] * 5


# 시험용 URDF 문자열을 만든다(팔 5축 limit만 포함).
def urdf():
    joints = ''.join(
        f'<joint name="{n}" type="revolute"><limit lower="{lo}" upper="{hi}" '
        f'effort="10" velocity="{v}"/></joint>'
        for n, (lo, hi, v) in PHYSICAL.items())
    return f'<robot name="t">{joints}</robot>'


LIMITS = logic.load_limits(urdf(), PLANNED, {n: 0.02 for n in ARM_JOINTS})


# 궤적 점 하나를 만든다.
def point(positions, time, velocities=(), accelerations=(), effort=()):
    return {'positions': list(positions), 'velocities': list(velocities),
            'accelerations': list(accelerations), 'effort': list(effort), 'time': time}


# 어깨 하나만 움직이는 궤적을 검사한다.
def shoulder(target, time=8.0, start=ZERO, **kwargs):
    return logic.validate_trajectory(['shoulder_joint'], [point([target], time, **kwargs)],
                                     True, False, list(start), ZERO, LIMITS)


# 명령 가능 범위 계산과 설정 오류를 시험한다.
class LimitTest(unittest.TestCase):
    # 명령 범위가 물리 한계에서 여유만큼 안쪽인지 확인한다.
    def test_command_range_is_inside_physical_limits(self):
        lim = LIMITS['shoulder_joint']
        self.assertAlmostEqual(lim.command_lower, -1.5508)
        self.assertAlmostEqual(lim.command_upper, 1.5508)
        self.assertEqual((lim.lower, lim.upper), (-1.5708, 1.5708))
        self.assertEqual(lim.max_acceleration, 0.5)

    # 가속도 계획값 누락이나 범위를 없애는 여유는 오류인지 확인한다.
    def test_missing_acceleration_or_too_large_margin_fails(self):
        planned = dict(PLANNED, wrist_joint={'max_velocity': 0.7})
        with self.assertRaises(ValueError):
            logic.load_limits(urdf(), planned, {n: 0.02 for n in ARM_JOINTS})
        with self.assertRaises(ValueError):
            logic.load_limits(urdf(), PLANNED, dict({n: 0.02 for n in ARM_JOINTS},
                                                    wrist_joint=1.6))


# 명령 가능 범위의 안쪽·경계·바깥 목표를 시험한다.
class RangeTest(unittest.TestCase):
    # 안쪽과 경계 목표는 통과하고 값을 바꾸지 않는지 확인한다.
    def test_inside_and_boundary_pass_unchanged(self):
        for target in (-1.0, -1.5508, 1.5508):
            verdict = shoulder(target)
            self.assertTrue(verdict.ok, verdict.reason)
            self.assertEqual(verdict.points[-1]['positions'][1], target)

    # 경계 바로 바깥과 물리 한계 목표는 거부하고 잘라서 실행하지 않는지 확인한다.
    def test_outside_is_rejected_not_clipped(self):
        for target in (-1.5509, -1.5708, 1.56, 2.0):
            verdict = shoulder(target)
            self.assertFalse(verdict.ok)
            self.assertIn('outside command range', verdict.reason)
            self.assertEqual(verdict.points, ())

    # 다른 관절도 같은 규칙인지 확인한다.
    def test_other_joints_use_their_own_ranges(self):
        ok = logic.validate_trajectory(['elbow_joint'], [point([1.97], 10.0)], True, False,
                                       ZERO, ZERO, LIMITS)
        bad = logic.validate_trajectory(['elbow_joint'], [point([1.99], 10.0)], True, False,
                                        ZERO, ZERO, LIMITS)
        self.assertTrue(ok.ok, ok.reason)
        self.assertFalse(bad.ok)


# 형식·수치 오류 입력을 시험한다.
class FormatTest(unittest.TestCase):
    # 잘못된 관절명·배열·시간·비정상 수치·지원하지 않는 형식을 사유와 함께 거부하는지 확인한다.
    def test_invalid_inputs_are_rejected_with_reason(self):
        cases = [
            (['bad_joint'], [point([0.1], 4.0)], 'unknown joint'),
            (['shoulder_joint', 'shoulder_joint'], [point([0.1, 0.1], 4.0)], 'duplicate'),
            ([], [point([], 4.0)], 'empty joint_names'),
            (['shoulder_joint'], [], 'no points'),
            (['shoulder_joint'], [point([0.1, 0.2], 4.0)], 'positions length'),
            (['shoulder_joint'], [point([0.1], 4.0, velocities=[0, 0])], 'velocities length'),
            (['shoulder_joint'], [point([math.nan], 4.0)], 'non-finite'),
            (['shoulder_joint'], [point([math.inf], 4.0)], 'non-finite'),
            (['shoulder_joint'], [point([0.1], 0.0)], 'time_from_start'),
            (['shoulder_joint'], [point([0.1], 4.0), point([0.2], 3.0)], 'time_from_start'),
            (['shoulder_joint'], [point([0.1], 4.0, effort=[1.0])], 'effort'),
            (['shoulder_joint'], [point([0.1], 4.0, accelerations=[0.0])], 'accelerations'),
            (['shoulder_joint'], [point([0.1], 4.0), point([0.2], 8.0, velocities=[0.0])],
             'same fields'),
        ]
        for names, points, expected in cases:
            with self.subTest(expected=expected):
                verdict = logic.validate_trajectory(names, points, True, False,
                                                    ZERO, ZERO, LIMITS)
                self.assertFalse(verdict.ok)
                self.assertIn(expected, verdict.reason)

    # 지연 시작·multi-DOF·비정상 현재 상태를 거부하는지 확인한다.
    def test_unsupported_forms_and_bad_state(self):
        points = [point([0.1], 4.0)]
        delayed = logic.validate_trajectory(['shoulder_joint'], points, False, False,
                                            ZERO, ZERO, LIMITS)
        multi = logic.validate_trajectory(['shoulder_joint'], points, True, True,
                                          ZERO, ZERO, LIMITS)
        state = logic.validate_trajectory(['shoulder_joint'], points, True, False,
                                          [math.nan] + ZERO[1:], ZERO, LIMITS)
        self.assertIn('delayed start', delayed.reason)
        self.assertIn('multi-DOF', multi.reason)
        self.assertIn('not finite', state.reason)


# 보간 경로의 범위·속도·가속도 검사를 시험한다.
class PathTest(unittest.TestCase):
    # 위치만 있는 궤적은 속도 0 경계의 3차식으로 바뀌고 피크 속도·가속도가 해석값과 같은지 확인한다.
    def test_positions_only_is_normalized_and_peaks_match(self):
        verdict = shoulder(-1.0, time=4.0)
        self.assertTrue(verdict.ok, verdict.reason)
        self.assertIn('cubic', verdict.normalized)
        self.assertAlmostEqual(verdict.summary['peak_velocity']['shoulder_joint'], 0.375, 3)
        self.assertAlmostEqual(verdict.summary['peak_acceleration']['shoulder_joint'], 0.375, 3)
        self.assertEqual(verdict.points[-1]['velocities'], [0.0] * 5)

    # 끝점이 범위 안이어도 3차 보간이 범위를 넘으면 거부하는지 확인한다.
    def test_interpolation_overshoot_is_rejected(self):
        # 경계 근처 점을 바깥쪽 속도로 지나면 다음 구간에서 경로가 경계를 넘는다
        # (손계산: 두 번째 구간 최대 이탈 4/27 × 3 s × 0.3 rad/s ≈ 0.13 rad).
        points = [point([-1.54], 8.0, velocities=[-0.3]), point([-1.54], 11.0, velocities=[0.0])]
        verdict = logic.validate_trajectory(['shoulder_joint'], points, True, False,
                                            ZERO, ZERO, LIMITS)
        self.assertFalse(verdict.ok)
        self.assertIn('interpolated', verdict.reason)

    # 너무 짧은 시간의 목표는 속도 또는 가속도 제한으로 거부하는지 확인한다.
    def test_velocity_and_acceleration_limits(self):
        fast = shoulder(-1.5, time=4.4)        # 피크 속도 1.5×1.5/4.4 = 0.511 > 0.5, 가속도 0.465
        sharp = shoulder(-0.6, time=2.0)       # 피크 가속도 6×0.6/4 = 0.9 > 0.5
        self.assertIn('velocity', fast.reason)
        self.assertIn('acceleration', sharp.reason)

    # 5차(가속도 포함) 궤적도 경로 검사가 적용되는지 확인한다.
    def test_quintic_path_is_checked(self):
        ok = shoulder(-1.0, time=6.0, velocities=[0.0], accelerations=[0.0])
        bad = shoulder(-1.0, time=2.0, velocities=[0.0], accelerations=[0.0])
        self.assertTrue(ok.ok, ok.reason)
        self.assertFalse(bad.ok)

    # 명령 범위 밖(여유 구역)에서 시작한 관절은 안쪽으로 돌아가는 경로만 허용하는지 확인한다.
    def test_start_outside_range_allows_only_inward(self):
        start = [0.0, -1.56, 0.0, 0.0, 0.0]
        inward = shoulder(-1.0, time=6.0, start=start)
        hold_out = logic.validate_trajectory(['elbow_joint'], [point([0.1], 4.0)], True, False,
                                             start, ZERO, LIMITS)
        # 시작 속도가 바깥(아래)쪽이면 경로가 시작 위치보다 바깥으로 나가므로 거부한다.
        outward = logic.validate_trajectory(['shoulder_joint'], [point([-1.0], 6.0)], True, False,
                                            start, [0.0, -0.1, 0.0, 0.0, 0.0], LIMITS)
        self.assertTrue(inward.ok, inward.reason)
        self.assertTrue(hold_out.ok, hold_out.reason)   # 어깨는 시작 위치에 머묾
        self.assertFalse(outward.ok)


# 실제 상태 기반 보호 판단과 정지 궤적을 시험한다.
class ProtectionTest(unittest.TestCase):
    # 하한 쪽 상태 하나를 판단한다.
    def check(self, position, velocity):
        positions = [0.0, position, 0.0, 0.0, 0.0]
        velocities = [0.0, velocity, 0.0, 0.0, 0.0]
        return {(n, side, kind) for n, side, kind, _ in
                logic.protection_check(positions, velocities, LIMITS, PARAMS)}

    # 경계에 멈춰 있거나 멀리서 접근하면 보호하지 않는지 확인한다.
    def test_rest_at_boundary_and_far_approach_are_free(self):
        self.assertEqual(self.check(-1.5508, 0.0), set())
        self.assertEqual(self.check(-1.0, -0.5), set())

    # 속도가 클수록 더 먼 곳에서 감속을 시작하는지 확인한다(방향 포함).
    def test_braking_distance_depends_on_speed_and_direction(self):
        # 0.5 rad/s로 하한에 접근: 제동 거리 0.05 + 0.25 + 0.01 = 0.31 rad
        self.assertIn(('shoulder_joint', 'lower', 'protect'), self.check(-1.30, -0.5))
        self.assertEqual(self.check(-1.20, -0.5), set())
        # 같은 위치에서 안쪽(위)으로 움직이면 보호하지 않는다.
        self.assertEqual(self.check(-1.30, 0.5), set())
        # 느린 접근은 한계 가까이에서야 보호한다.
        self.assertEqual(self.check(-1.53, -0.05), set())
        self.assertIn(('shoulder_joint', 'lower', 'protect'), self.check(-1.55, -0.05))

    # 상한 쪽도 같은 규칙인지 확인한다.
    def test_upper_side(self):
        self.assertIn(('shoulder_joint', 'upper', 'protect'), self.check(1.30, 0.5))
        self.assertEqual(self.check(1.30, -0.5), set())

    # 물리 한계 도달·초과는 fault, 멈춘 채 아주 가까우면 near인지 확인한다.
    def test_fault_and_near(self):
        self.assertIn(('shoulder_joint', 'lower', 'fault'), self.check(-1.5708, 0.0))
        self.assertIn(('shoulder_joint', 'lower', 'fault'), self.check(-1.5708000000000202, 0.0))
        self.assertIn(('shoulder_joint', 'lower', 'near'), self.check(-1.568, 0.0))

    # 하한 쪽 0.5 rad/s 접근의 판단 기준이 실제 정지 궤적 거리와 같은 식인지 확인한다.
    def test_braking_distance_matches_stop_plan(self):
        plan = logic.stop_trajectory([0.0, -1.3, 0.0, 0.0, 0.0], [0.0, -0.5, 0.0, 0.0, 0.0],
                                     LIMITS)
        travel = abs(plan.stop_positions['shoulder_joint'] + 1.3)
        expected = (logic.braking_distance(0.5, 0.5, PARAMS)
                    - 0.5 * PARAMS.reaction_time_s - PARAMS.tracking_allowance_rad)
        self.assertAlmostEqual(travel, expected)          # v²/(2a) = 0.25


# 다축 보호 정지 궤적을 시험한다.
class StopTrajectoryTest(unittest.TestCase):
    # 정지 궤적을 JTC와 같은 3차 보간으로 펼쳐 관절별 극값을 구한다.
    def extremes(self, positions, velocities, plan):
        return logic.path_extremes(positions, velocities, plan.points, False)

    # 정지 궤적의 공통 조건: 관절별 가속도 이내, 마지막 속도 0, 정지 거리 = v²/(2a)(최소 시간 적용
    # 관절은 v·T/2), 먼저 멈춘 관절은 뒤 점에서 위치 유지, 이동이 단조(되돌아가지 않음).
    def assert_valid_stop(self, positions, velocities, plan, minimum=0.01):
        extremes = self.extremes(positions, velocities, plan)
        for i, name in enumerate(ARM_JOINTS):
            lim, q, v = LIMITS[name], positions[i], velocities[i]
            q_min, q_max, v_max, a_max, _ = extremes[name]
            self.assertLessEqual(a_max, lim.max_acceleration * (1 + 1e-9), name)
            self.assertLessEqual(v_max, abs(v) + 1e-12, name)
            self.assertEqual(plan.points[-1]['velocities'][i], 0.0)

            stop_time = max(abs(v) / lim.max_acceleration, minimum) if v else 0.0
            self.assertAlmostEqual(plan.stop_times[name], stop_time, msg=name)
            self.assertAlmostEqual(plan.stop_positions[name], q + v * stop_time / 2, msg=name)
            self.assertAlmostEqual(min(q_min, q), min(q, plan.stop_positions[name]), msg=name)
            self.assertAlmostEqual(max(q_max, q), max(q, plan.stop_positions[name]), msg=name)

            for p in plan.points:
                if p['time'] >= stop_time:
                    self.assertAlmostEqual(p['positions'][i], plan.stop_positions[name])

    # 재현 사례: 어깨 0 rad·0.5 rad/s, 손목 1.53 rad·0.2 rad/s. 이전 구현은 공통 1.0 s를 써서
    # 손목 목표가 1.63 rad(물리 상한 1.57 초과)였다. 이제 손목은 0.2 s에 1.55 rad에서 멈춘다.
    def test_reported_case_wrist_stops_inside_limit(self):
        positions, velocities = [0.0, 0.0, 0.0, 1.53, 0.0], [0.0, 0.5, 0.0, 0.2, 0.0]
        plan = logic.stop_trajectory(positions, velocities, LIMITS)

        self.assertAlmostEqual(plan.stop_times['wrist_joint'], 0.2)
        self.assertAlmostEqual(plan.stop_positions['wrist_joint'], 1.55)
        self.assertAlmostEqual(plan.stop_times['shoulder_joint'], 1.0)
        self.assertAlmostEqual(plan.stop_positions['shoulder_joint'], 0.25)
        self.assertEqual([p['time'] for p in plan.points], [0.2, 1.0])
        self.assertEqual(plan.limit_violations, [])
        self.assertLess(self.extremes(positions, velocities, plan)['wrist_joint'][1], 1.57)
        self.assert_valid_stop(positions, velocities, plan)

    # 여러 축의 속도·방향이 모두 다른 경우에도 관절마다 자기 감속도로 멈추는지 확인한다.
    def test_mixed_speeds_and_directions(self):
        positions = [1.0, -0.4, 0.5, -0.3, 2.0]
        velocities = [-0.35, 0.5, -0.45, 0.7, 0.3]
        plan = logic.stop_trajectory(positions, velocities, LIMITS)
        self.assertEqual(len(plan.points), len(set(plan.stop_times.values())))
        self.assert_valid_stop(positions, velocities, plan)
        self.assertAlmostEqual(plan.duration, 1.0)                 # 어깨 0.5/0.5

    # 최소 정지 시간은 그보다 빨리 멈출 느린 관절에만 적용되고 다른 관절의 시간을 늘리지 않는지,
    # 모두 멈춰 있으면 최소 시간의 유지 점 하나인지 확인한다.
    def test_minimum_time_affects_only_slow_joints(self):
        positions, velocities = [0.0] * 5, [0.002, 0.3, 0.0, 0.0, -0.004]
        plan = logic.stop_trajectory(positions, velocities, LIMITS, minimum_time_s=0.01)
        self.assertAlmostEqual(plan.stop_times['column_yaw_joint'], 0.01)   # 0.004 s → 0.01 s
        self.assertAlmostEqual(plan.stop_times['gripper_rotation_joint'], 0.01)
        self.assertAlmostEqual(plan.stop_times['shoulder_joint'], 0.6)      # 늘어나지 않음
        self.assertEqual(plan.stop_times['elbow_joint'], 0.0)
        self.assert_valid_stop(positions, velocities, plan)

        still = logic.stop_trajectory([0.1] * 5, ZERO, LIMITS, minimum_time_s=0.01)
        self.assertEqual(len(still.points), 1)
        self.assertEqual(still.points[0]['time'], 0.01)
        self.assertEqual(still.points[0]['positions'], [0.1] * 5)

    # 최대 감속으로도 물리 한계 안에서 멈출 수 없으면 잘라내지 않고 그 사실을 알리는지 확인한다.
    def test_unreachable_stop_is_reported_not_clipped(self):
        positions, velocities = [0.0, 0.0, 0.0, 1.56, 0.0], [0.0, 0.0, 0.0, 0.2, 0.0]
        plan = logic.stop_trajectory(positions, velocities, LIMITS)
        self.assertEqual(plan.limit_violations, ['wrist_joint'])
        self.assertAlmostEqual(plan.stop_positions['wrist_joint'], 1.58)    # 자르지 않음
        self.assert_valid_stop(positions, velocities, plan)


# 보간 구간 내부 극값 검사를 시험한다.
class InteriorExtremaTest(unittest.TestCase):
    # 보고된 사례: 0.005 s에 속도 0.5, 0.010 s에 정지(가속도 0 지정). 구간 내부 가속도 약 394 rad/s².
    def test_short_quintic_segment_acceleration_is_rejected(self):
        points = [point([0.0], 0.005, velocities=[0.5], accelerations=[0.0]),
                  point([0.0], 0.010, velocities=[0.0], accelerations=[0.0])]
        verdict = logic.validate_trajectory(['shoulder_joint'], points, True, False,
                                            ZERO, ZERO, LIMITS)
        self.assertFalse(verdict.ok)
        self.assertIn('acceleration', verdict.reason)
        extremes = logic.path_extremes(ZERO, ZERO, [
            {'time': 0.005, 'positions': ZERO, 'velocities': [0, 0.5, 0, 0, 0],
             'accelerations': ZERO},
            {'time': 0.010, 'positions': ZERO, 'velocities': ZERO, 'accelerations': ZERO}], True)
        self.assertAlmostEqual(extremes['shoulder_joint'][3], 394.0, delta=1.0)

    # 0.01 s보다 짧은 구간 안에서 위치가 허용 범위를 벗어나면 거부하는지 확인한다.
    # 경계(-1.5508)에서 바깥쪽 0.5 rad/s로 시작해 0.008 s 뒤 같은 위치로 돌아오는 3차 구간:
    # 내부 최저점은 경계보다 4/27 × 0.008 × 0.5 ≈ 0.0006 rad 바깥이다.
    def test_short_segment_position_excursion_is_rejected(self):
        start = [0.0, -1.5508, 0.0, 0.0, 0.0]
        points = [point([-1.5508], 0.008, velocities=[0.0]),
                  point([-1.0], 8.0, velocities=[0.0])]
        verdict = logic.validate_trajectory(['shoulder_joint'], points, True, False,
                                            start, [0.0, -0.5, 0.0, 0.0, 0.0], LIMITS)
        self.assertFalse(verdict.ok)
        self.assertIn('outside allowed range', verdict.reason)

    # 양 끝 속도는 한계 이내지만 구간 중간 속도가 한계를 넘으면 거부하는지 확인한다.
    # 3차 구간 중간 속도 = 1.5 × 2.4/4 − 0.25 × (0.4 + 0.4) = 0.7 > 0.5, 가속도는 0.3 이내.
    def test_interior_velocity_peak_is_rejected(self):
        start = [0.0, -1.2, 0.0, 0.0, 0.0]
        points = [point([1.2], 4.0, velocities=[0.4])]
        verdict = logic.validate_trajectory(['shoulder_joint'], points, True, False,
                                            start, [0.0, 0.4, 0.0, 0.0, 0.0], LIMITS)
        self.assertFalse(verdict.ok)
        self.assertIn('velocity', verdict.reason)

    # 제한 이내의 다점 5차 궤적은 통과하고 피크가 한계 이하인지 확인한다.
    def test_multi_point_quintic_within_limits_passes(self):
        points = [point([-0.5], 4.0, velocities=[-0.2], accelerations=[0.0]),
                  point([-1.0], 8.0, velocities=[0.0], accelerations=[0.0])]
        verdict = logic.validate_trajectory(['shoulder_joint'], points, True, False,
                                            ZERO, ZERO, LIMITS)
        self.assertTrue(verdict.ok, verdict.reason)
        self.assertLessEqual(verdict.summary['peak_velocity']['shoulder_joint'], 0.5)
        self.assertLessEqual(verdict.summary['peak_acceleration']['shoulder_joint'], 0.5)

    # 극값 계산이 촘촘한 표본(구간당 2만 점)의 최대·최소와 일치하는지 무작위 구간으로 확인한다.
    def test_extremes_match_dense_sampling(self):
        rng = random.Random(7)
        for _ in range(200):
            T = rng.choice([0.004, 0.03, 0.5, 3.0])
            p0, p1 = rng.uniform(-1, 1), rng.uniform(-1, 1)
            v0, v1 = rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)
            a0, a1 = rng.uniform(-1, 1), rng.uniform(-1, 1)
            for coefficients in (logic._cubic_coefficients(p0, v0, p1, v1, T),
                                 logic._quintic_coefficients(p0, v0, a0, p1, v1, a1, T)):
                q_min, q_max, v_max, a_max = logic.segment_extremes(coefficients, T)
                velocity = logic._derivative(coefficients)
                acceleration = logic._derivative(velocity)
                ts = [T * k / 20000 for k in range(20001)]
                qs = [logic._evaluate(coefficients, t) for t in ts]
                vs = [abs(logic._evaluate(velocity, t)) for t in ts]
                accs = [abs(logic._evaluate(acceleration, t)) for t in ts]

                # 표본 최대는 해석 극값을 넘지 않고, 해석 극값과의 차이는 표본 간격 수준이다.
                for analytic, sampled in ((q_max, max(qs)), (-q_min, -min(qs)),
                                          (v_max, max(vs)), (a_max, max(accs))):
                    self.assertGreaterEqual(analytic + 1e-9 * max(1, abs(analytic)), sampled)
                    self.assertLessEqual(analytic - sampled, 1e-6 * max(1, abs(analytic)))


if __name__ == '__main__':
    unittest.main()
