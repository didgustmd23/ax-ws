"""
팔 명령 보호(개발용)의 순수 판정 로직. ROS 없이 시험할 수 있게 노드와 분리한다.

물리 위치 한계(URDF)는 그대로 두고, 그보다 안쪽의 명령 가능 범위로 목표·보간 경로를 검사한다.
실제 관절 상태로 한계까지 남은 거리와 이동 방향을 판단해 감속 정지를 정한다.
단위: 위치 rad, 속도 rad/s, 가속도 rad/s², 시간 s. 개발용 관절 보호이며 안전 게이트가 아니다.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from ax_control.model_contracts import ARM_JOINTS


# 관절 하나의 물리 한계·명령 가능 범위·속도·가속도 제한.
@dataclass(frozen=True)
class JointLimit:
    name: str
    lower: float
    upper: float
    max_velocity: float
    max_acceleration: float
    margin: float

    # 명령 가능 범위의 하한(물리 하한 + 여유).
    @property
    def command_lower(self):
        return self.lower + self.margin

    # 명령 가능 범위의 상한(물리 상한 − 여유).
    @property
    def command_upper(self):
        return self.upper - self.margin


# 실제 상태 감시의 판단 기준(설정 파일 값).
@dataclass(frozen=True)
class ProtectionParams:
    reaction_time_s: float
    tracking_allowance_rad: float
    min_clearance_rad: float
    fault_clearance_rad: float
    still_velocity_rad_s: float


# 검사 결과: 통과 여부, 거부 사유, 실행할(정규화한) 궤적, 경로 요약.
@dataclass
class Verdict:
    ok: bool
    reason: str = ''
    joint_names: tuple = ()
    points: tuple = ()
    normalized: str = ''
    summary: dict = None


# URDF 문자열과 계획용 가속도·속도 표에서 팔 5축의 제한을 만든다.
# 속도는 URDF velocity와 계획값 중 작은 값을 쓰고, 가속도 계획값이 없으면 오류로 본다.
def load_limits(urdf_text, planned_limits, margins):
    root = ET.fromstring(urdf_text)
    result = {}

    for name in ARM_JOINTS:
        limit = root.find(f"joint[@name='{name}']/limit")
        if limit is None:
            raise ValueError(f'joint {name} has no <limit> in robot_description')

        planned = planned_limits.get(name)
        if not planned or 'max_acceleration' not in planned:
            raise ValueError(f'joint {name} has no planned max_acceleration')

        margin = float(margins[name])
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if not (margin >= 0 and lower + margin < upper - margin):
            raise ValueError(f'joint {name}: margin {margin} leaves no command range')

        velocity = float(limit.get('velocity'))
        if 'max_velocity' in planned:
            velocity = min(velocity, float(planned['max_velocity']))

        result[name] = JointLimit(name, lower, upper, velocity,
                                  float(planned['max_acceleration']), margin)
    return result


# 값 목록이 모두 유한한 실수인지 확인한다.
def _finite(values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


# 3차 Hermite 구간(시작·끝 위치·속도, 길이 T)을 t의 거듭제곱 계수 [c0..c3]로 바꾼다.
def _cubic_coefficients(p0, v0, p1, v1, T):
    return [p0, v0,
            (3 * (p1 - p0) - (2 * v0 + v1) * T) / T**2,
            (2 * (p0 - p1) + (v0 + v1) * T) / T**3]


# 5차 다항식 구간(시작·끝 위치·속도·가속도, 길이 T)을 t의 거듭제곱 계수 [c0..c5]로 바꾼다.
def _quintic_coefficients(p0, v0, a0, p1, v1, a1, T):
    return [p0, v0, a0 / 2,
            (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T**2) / (2 * T**3),
            (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T**2) / (2 * T**4),
            (12 * (p1 - p0) - (6 * v1 + 6 * v0) * T - (a0 - a1) * T**2) / (2 * T**5)]


# 오름차순 계수 다항식의 x 값(Horner 방식).
def _evaluate(coefficients, x):
    value = 0.0
    for c in reversed(coefficients):
        value = value * x + c
    return value


# 오름차순 계수 다항식의 도함수 계수.
def _derivative(coefficients):
    return [k * c for k, c in enumerate(coefficients)][1:]


# 다항식의 (0, 1) 안 실근을 모두 찾는다. 도함수의 근으로 단조 구간을 나눈 뒤 부호가 바뀌는
# 구간만 이분법으로 좁힌다(근이 단조 구간마다 최대 하나이므로 빠짐없이 찾는다).
def _roots_in_unit(coefficients):
    while len(coefficients) > 1 and coefficients[-1] == 0.0:
        coefficients = coefficients[:-1]
    if len(coefficients) <= 1:
        return []
    if len(coefficients) == 2:
        root = -coefficients[0] / coefficients[1]
        return [root] if 0.0 < root < 1.0 else []

    edges = [0.0] + _roots_in_unit(_derivative(coefficients)) + [1.0]
    roots = []
    for a, b in zip(edges, edges[1:]):
        fa, fb = _evaluate(coefficients, a), _evaluate(coefficients, b)
        if fa == 0.0 and 0.0 < a < 1.0:
            roots.append(a)
        if fa * fb >= 0.0:
            continue

        for _ in range(80):
            middle = (a + b) / 2
            fm = _evaluate(coefficients, middle)
            if (fm < 0.0) == (fa < 0.0):
                a, fa = middle, fm
            else:
                b = middle
        roots.append((a + b) / 2)
    return roots


# 구간 다항식(t 계수)의 [0, T] 위 위치 최소·최대와 속도·가속도 절대 최대를 극값으로 구한다.
# 정규화 변수 s = t/T로 바꿔 도함수의 근(내부 극값)과 양 끝을 모두 확인한다(표본화하지 않음).
def segment_extremes(coefficients, T):
    position = [c * T**k for k, c in enumerate(coefficients)]          # q(s)
    velocity = _derivative(position)                                   # T·v(s)
    acceleration = _derivative(velocity)                               # T²·a(s)

    # 각 함수의 극값 후보 = 양 끝 + 도함수의 내부 근.
    def extreme_values(poly):
        candidates = [0.0, 1.0] + _roots_in_unit(_derivative(poly))
        return [_evaluate(poly, s) for s in candidates]

    q = extreme_values(position)
    v = [abs(x) / T for x in extreme_values(velocity)] if len(velocity) else [0.0]
    a = [abs(x) / T**2 for x in extreme_values(acceleration)] if len(acceleration) else [0.0]
    return min(q), max(q), max(v), max(a)


# 시작 상태와 궤적 점(5축 전체 벡터)으로 JTC 방식 보간 경로의 관절별 극값을 구한다.
# 결과: {관절: (최소 위치, 최대 위치, 최대 |속도|, 최대 |가속도|, 구간 번호별 같은 값 목록)}.
# with_acc가 참이면 5차, 아니면 3차 Hermite로 본다(JTC와 같은 규칙, 시작 가속도 0).
def path_extremes(start_positions, start_velocities, points, with_acc):
    result = {}
    for i, name in enumerate(ARM_JOINTS):
        state = (0.0, start_positions[i], start_velocities[i], 0.0)
        segments = []
        for point in points:
            t0, p0, v0, a0 = state
            t1, p1 = point['time'], point['positions'][i]
            v1 = point['velocities'][i] if point['velocities'] else 0.0
            a1 = point['accelerations'][i] if with_acc else 0.0
            T = t1 - t0
            if with_acc:
                coefficients = _quintic_coefficients(p0, v0, a0, p1, v1, a1, T)
            else:
                coefficients = _cubic_coefficients(p0, v0, p1, v1, T)
            segments.append(segment_extremes(coefficients, T))
            state = (t1, p1, v1, a1)

        result[name] = (min(x[0] for x in segments), max(x[1] for x in segments),
                        max(x[2] for x in segments), max(x[3] for x in segments), segments)
    return result


# 궤적 형식·수치를 검사한다. 통과하면 None, 아니면 거부 사유를 돌려준다.
def _check_format(names, points, header_stamp_zero, has_multi_dof):
    if has_multi_dof:
        return 'multi-DOF trajectories are not supported'
    if not header_stamp_zero:
        return 'delayed start (non-zero header.stamp) is not supported'
    if not names:
        return 'empty joint_names'
    if len(set(names)) != len(names):
        return 'duplicate joint names'

    unknown = [n for n in names if n not in ARM_JOINTS]
    if unknown:
        return f'unknown joint names: {unknown}'
    if not points:
        return 'trajectory has no points'

    kinds = set()
    previous = 0.0
    for index, point in enumerate(points):
        if len(point['positions']) != len(names):
            return f'point {index}: positions length != joint_names length'

        for field in ('velocities', 'accelerations'):
            if len(point[field]) not in (0, len(names)):
                return f'point {index}: {field} length must be 0 or {len(names)}'
        if point['effort']:
            return f'point {index}: effort fields are not supported'
        if point['accelerations'] and not point['velocities']:
            return f'point {index}: accelerations without velocities are not supported'

        values = point['positions'] + point['velocities'] + point['accelerations']
        if not _finite(values + [point['time']]):
            return f'point {index}: non-finite value'

        if point['time'] <= previous:
            return f'point {index}: time_from_start must increase and be > 0'
        previous = point['time']
        kinds.add((bool(point['velocities']), bool(point['accelerations'])))

    if len(kinds) != 1:
        return 'all points must provide the same fields (positions/velocities/accelerations)'
    return None


# 형식 검사 후, 현재 상태에서 시작하는 JTC 방식 보간 경로의 범위·속도·가속도를 검사한다.
# 각 구간 다항식의 내부 극값(도함수의 근)까지 확인하므로 짧은 구간 안의 초과도 놓치지 않는다.
# 위치만 있는 궤적은 각 점에서 속도 0인 3차 Hermite로 바꿔 실행한다(가속도를 유한하게 하기 위함).
# 명령 범위 밖에서 시작한 관절은 시작 위치보다 바깥으로 가지 않는 경로만 허용한다(안쪽 복귀).
def validate_trajectory(names, points, header_stamp_zero, has_multi_dof,
                        start_positions, start_velocities, limits):
    reason = _check_format(names, points, header_stamp_zero, has_multi_dof)
    if reason:
        return Verdict(False, reason)
    if not (_finite(start_positions) and _finite(start_velocities)):
        return Verdict(False, 'current joint state is not finite')

    # 입력을 팔 5축 전체 벡터로 펼친다. 빠진 관절은 현재 위치에 머문다.
    index = {name: i for i, name in enumerate(ARM_JOINTS)}
    with_vel = bool(points[0]['velocities'])
    with_acc = bool(points[0]['accelerations'])
    normalized = '' if with_vel else 'positions-only -> cubic with zero velocity at each point'

    full = []
    for point in points:
        positions, velocities, accelerations = list(start_positions), [0.0] * 5, [0.0] * 5
        for k, name in enumerate(names):
            i = index[name]
            positions[i] = point['positions'][k]
            if with_vel:
                velocities[i] = point['velocities'][k]
            if with_acc:
                accelerations[i] = point['accelerations'][k]
        full.append((point['time'], positions, velocities, accelerations))

    # 관절별 허용 범위. 범위 밖에서 시작했으면 시작 위치까지를 허용한다(더 바깥은 금지).
    bounds = {}
    for name in ARM_JOINTS:
        lim, start = limits[name], start_positions[index[name]]
        bounds[name] = (min(lim.command_lower, start), max(lim.command_upper, start))

    # 명령한 관절의 끝점(웨이포인트)은 반드시 명령 가능 범위 안이어야 한다.
    # 명령하지 않은 관절은 현재 위치에 머물며 아래 경로 검사(시작 위치보다 바깥 금지)를 받는다.
    for number, (_, positions, _, _) in enumerate(full):
        for name in names:
            lim, q = limits[name], positions[index[name]]
            if not (lim.command_lower <= q <= lim.command_upper):
                return Verdict(False, f'point {number}: {name}={q:.6f} outside command range '
                                      f'[{lim.command_lower:.6f}, {lim.command_upper:.6f}]')

    # 구간마다 위치·속도·가속도의 극값(양 끝 + 내부 극값)을 검사한다.
    path = [{'time': t, 'positions': p, 'velocities': v, 'accelerations': a}
            for t, p, v, a in full]
    extremes = path_extremes(start_positions, start_velocities, path, with_acc)

    for name in ARM_JOINTS:
        lim, (low, high) = limits[name], bounds[name]
        for number, (q_min, q_max, v_max, a_max) in enumerate(extremes[name][4]):
            if q_min < low - 1e-12 or q_max > high + 1e-12:
                return Verdict(False, f'segment {number}: interpolated {name} '
                                      f'[{q_min:.6f}, {q_max:.6f}] outside allowed range '
                                      f'[{low:.6f}, {high:.6f}]')
            if v_max > lim.max_velocity * (1 + 1e-9):
                return Verdict(False, f'segment {number}: {name} velocity {v_max:.4f} > '
                                      f'{lim.max_velocity:.4f} rad/s')
            if a_max > lim.max_acceleration * (1 + 1e-9):
                return Verdict(False, f'segment {number}: {name} acceleration '
                                      f'{a_max:.4f} > {lim.max_acceleration:.4f} rad/s^2')

    peak_v = {name: extremes[name][2] for name in ARM_JOINTS}
    peak_a = {name: extremes[name][3] for name in ARM_JOINTS}
    executed = tuple({'time': t, 'positions': p, 'velocities': v,
                      'accelerations': a if with_acc else []}
                     for t, p, v, a in full)
    return Verdict(True, '', tuple(ARM_JOINTS), executed, normalized,
                   {'peak_velocity': peak_v, 'peak_acceleration': peak_a})


# 한계 방향 속도로 필요한 제동 거리(반응 시간 이동 + 등감속 정지 + 추종 오차 여유).
def braking_distance(speed_toward, max_acceleration, params):
    speed = max(0.0, speed_toward)
    return (speed * params.reaction_time_s + speed**2 / (2 * max_acceleration)
            + params.tracking_allowance_rad)


# 실제 상태로 관절별 보호 판단을 한다. 결과는 [(관절, 쪽, 종류, 사유)]이다.
# - 'fault': 물리 한계 도달·초과(일반 작업 재개 차단 대상)
# - 'protect': 한계 쪽으로 움직이고(속도 > 정지 판정 속도) 제동 거리가 남은 거리를 넘는 접근
# - 'near': 멈춘 상태로 최소 여유보다 한계에 가까움(정보. 감속 대상 아님)
def protection_check(positions, velocities, limits, params):
    findings = []
    for i, name in enumerate(ARM_JOINTS):
        lim, q, v = limits[name], positions[i], velocities[i]
        sides = (('lower', q - lim.lower, -v), ('upper', lim.upper - q, v))

        for side, remaining, toward in sides:
            if remaining <= params.fault_clearance_rad:
                findings.append((name, side, 'fault',
                                 f'{name} at/over physical {side} limit '
                                 f'(remaining {remaining:.6f} rad)'))
                continue

            needed = braking_distance(toward, lim.max_acceleration, params)
            moving_toward = toward > params.still_velocity_rad_s

            if moving_toward and remaining - needed < params.min_clearance_rad:
                findings.append((name, side, 'protect',
                                 f'{name} approaching {side} limit: '
                                 f'remaining {remaining:.4f} rad, '
                                 f'velocity toward {toward:.4f} rad/s, '
                                 f'braking distance {needed:.4f} rad'))
            elif not moving_toward and remaining < params.min_clearance_rad:
                findings.append((name, side, 'near',
                                 f'{name} resting {remaining:.4f} rad from physical {side} limit'))
    return findings


# 정지 계획: 실행할 궤적 점, 관절별 정지 시간·정지 위치, 물리 한계 안에 멈추지 못하는 관절.
@dataclass
class StopPlan:
    points: list
    stop_times: dict
    stop_positions: dict
    limit_violations: list

    # 마지막 관절이 멈추는 시간.
    @property
    def duration(self):
        return self.points[-1]['time']


# 관절별로 자기 최대 가속도로 등감속해 멈추는 정지 궤적을 만든다(먼저 멈춘 관절은 그 위치 유지).
# 관절 i의 정지 시간 T_i = |v_i|/a_i, 정지 거리 v_i²/(2a_i)로 braking_distance의 등감속 항과 같다.
# 관절이 멈추는 시각마다 궤적 점(위치·속도)을 두므로 JTC의 3차 Hermite 보간이 이 등감속
# 프로파일을 그대로 재현한다(구간마다 각 관절이 2차식 또는 정지이므로 보간 오차 없음).
# minimum_time_s보다 빨리 멈출 관절(|v| < a·minimum_time_s)만 그 시간에 멈추게 하며, 이때
# 감속도는 |v|/minimum_time_s < a라서 제한 이내다. 다른 관절의 정지 시간은 늘리지 않는다.
# 정지 위치가 물리 한계(fault_clearance 포함)에 닿거나 넘는 관절은 limit_violations에 담는다.
# 목표를 잘라 가속도 제한을 넘기지 않으며, 이때도 가능한 가장 짧은 정지를 그대로 낸다.
def stop_trajectory(positions, velocities, limits, minimum_time_s=0.01, fault_clearance=1e-6):
    stop_times, decelerations, stop_positions = {}, {}, {}
    for name, q, v in zip(ARM_JOINTS, positions, velocities):
        if v == 0.0:
            stop_times[name], decelerations[name], stop_positions[name] = 0.0, 0.0, q
            continue

        duration = max(abs(v) / limits[name].max_acceleration, minimum_time_s)
        stop_times[name] = duration
        decelerations[name] = -v / duration
        stop_positions[name] = q + v * duration / 2

    times = sorted({t for t in stop_times.values() if t > 0.0}) or [minimum_time_s]

    # 각 시각의 관절별 위치·속도(정지 전에는 등감속, 정지 후에는 정지 위치 유지).
    points = []
    for t in times:
        point_positions, point_velocities = [], []
        for name, q, v in zip(ARM_JOINTS, positions, velocities):
            if t < stop_times[name]:
                point_positions.append(q + v * t + decelerations[name] * t**2 / 2)
                point_velocities.append(v + decelerations[name] * t)
            else:
                point_positions.append(stop_positions[name])
                point_velocities.append(0.0)
        points.append({'time': t, 'positions': point_positions,
                       'velocities': point_velocities, 'accelerations': []})

    # 등감속 정지는 단조 이동이므로 정지 위치가 경로의 끝(극값)이다.
    violations = [
        name for name in ARM_JOINTS
        if not (limits[name].lower + fault_clearance < stop_positions[name]
                < limits[name].upper - fault_clearance)]
    return StopPlan(points, stop_times, stop_positions, violations)
