"""C3 suction-surface candidate generation; never claims heuristic IK success."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math


@dataclass(frozen=True)
class Pose:
    position: tuple
    orientation: tuple  # x, y, z, w; TCP +Z points along approach toward the surface


@dataclass(frozen=True)
class Surface:
    # C2가 실제 관측으로 제공할 표면입니다. 후보 생성기가 물체 위치를 임의로 만들어 넣지 않습니다.
    robot_id: str
    object_id: str
    scene_revision: int
    stamp_s: float
    frame_id: str
    center: tuple
    normal: tuple       # outward normal
    tangent: tuple      # observed in-plane width direction
    width_m: float
    depth_m: float
    confidence: float
    suction_suitable: bool
    flatness_error_m: float
    mass_kg: float


@dataclass(frozen=True)
class Constraints:
    # 공구 크기·하중·접근 거리는 검증한 값을 전달합니다. 값이 바뀌면 제약 버전도 관리해야 합니다.
    version: int
    contact_width_m: float
    contact_depth_m: float
    max_mass_kg: float
    approach_m: float = 0.10
    retreat_m: float = 0.10
    edge_margin_m: float = 0.005
    min_confidence: float = 0.8
    max_flatness_error_m: float = 0.002
    observation_ttl_s: float = 1.0
    candidate_ttl_s: float = 3.0


@dataclass(frozen=True)
class Candidate:
    # 후보와 접근·이탈 자세를 보관합니다. 이 자료를 만드는 것과 실제 팔을 실행하는 것은 다릅니다.
    candidate_id: str
    target: Pose
    approach: Pose
    retreat: Pose
    rule_score: float
    success_probability: float = -1.0
    reachable: bool = True
    collision_free: bool = True


@dataclass(frozen=True)
class Plan:
    robot_id: str
    object_id: str
    scene_revision: int
    constraint_version: int
    frame_id: str
    stamp_s: float
    valid_for_s: float
    candidate_set_id: str
    candidates: tuple
    reason: str = ''

    @property
    def valid(self):
        return bool(self.candidates) and not self.reason


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(vector):
    if len(vector) != 3 or not all(math.isfinite(value) for value in vector):
        raise ValueError('finite 3D vector required')
    length = math.sqrt(dot(vector, vector))
    if length < 1e-9:
        raise ValueError('nonzero vector required')
    return tuple(value / length for value in vector)


def quaternion(x, y, z):
    """Convert a right-handed orthonormal TCP basis to a normalized quaternion."""
    matrix = [[x[i], y[i], z[i]] for i in range(3)]
    trace = sum(matrix[i][i] for i in range(3))
    if trace > 0:
        scale = math.sqrt(trace + 1) * 2
        values = ((matrix[2][1] - matrix[1][2]) / scale,
                  (matrix[0][2] - matrix[2][0]) / scale,
                  (matrix[1][0] - matrix[0][1]) / scale, scale / 4)
    else:
        i = max(range(3), key=lambda index: matrix[index][index])
        j, k = (i + 1) % 3, (i + 2) % 3
        scale = math.sqrt(1 + matrix[i][i] - matrix[j][j] - matrix[k][k]) * 2
        result = [0.0] * 4
        result[i] = scale / 4
        result[j] = (matrix[j][i] + matrix[i][j]) / scale
        result[k] = (matrix[k][i] + matrix[i][k]) / scale
        result[3] = (matrix[k][j] - matrix[j][k]) / scale
        values = tuple(result)
    length = math.sqrt(sum(value * value for value in values))
    return tuple(value / length for value in values)


def identifier(prefix, value):
    payload = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return prefix + hashlib.sha256(payload.encode()).hexdigest()[:24]


def checked(candidate, reachability, collision):
    """Require explicit True for every pose and the entire approach/exit path."""
    # 실제 IK와 전체 경로 충돌 검증기가 모두 True를 줘야 합니다. 검증기 없음·예외는 통과가 아닙니다.
    if reachability is None or collision is None:
        return False
    try:
        return (all(reachability(pose) is True for pose in
                    (candidate.approach, candidate.target, candidate.retreat))
                and collision(candidate.approach, candidate.target, candidate.retreat) is True)
    except Exception:
        # 검증기 실패를 도달 가능·충돌 없음으로 바꾸지 않습니다.
        return False


def plan_candidates(surface, constraints, now_s, *, reachability=None, collision=None):
    """Create deterministic top-surface suction candidates from C2 observations."""
    # 관측·공구 조건 → 후보 생성 → 도달·충돌 필터 → 규칙 점수 순으로 계산합니다.
    def invalid(reason):
        return Plan(surface.robot_id, surface.object_id, surface.scene_revision,
                    constraints.version, surface.frame_id, surface.stamp_s,
                    constraints.candidate_ttl_s, '', (), reason)

    scalar_values = [getattr(constraints, name) for name in asdict(constraints)]
    scalar_values += [surface.width_m, surface.depth_m, surface.confidence,
                      surface.flatness_error_m, surface.mass_kg, surface.stamp_s, now_s]
    finite_scalars = all(isinstance(v, (int, float)) and math.isfinite(v) for v in scalar_values)
    if (not finite_scalars
            or len(surface.center) != 3 or not all(math.isfinite(x) for x in surface.center)):
        return invalid('INVALID_GEOMETRY')
    if (not surface.robot_id or not surface.object_id or not surface.frame_id
            or surface.scene_revision <= 0 or constraints.version <= 0
            or surface.stamp_s <= 0
            or not 0 <= now_s - surface.stamp_s <= constraints.observation_ttl_s):
        return invalid('OBSERVATION_STALE_OR_INVALID')
    if (min(constraints.contact_width_m, constraints.contact_depth_m, constraints.max_mass_kg,
            constraints.approach_m, constraints.retreat_m, constraints.observation_ttl_s,
            constraints.candidate_ttl_s) <= 0 or constraints.edge_margin_m < 0
            or not 0 <= constraints.min_confidence <= 1
            or constraints.max_flatness_error_m < 0):
        return invalid('CONSTRAINTS_INVALID')
    if (not surface.suction_suitable or not 0 <= surface.confidence <= 1
            or surface.confidence < constraints.min_confidence
            or not 0 <= surface.mass_kg <= constraints.max_mass_kg
            or not 0 <= surface.flatness_error_m <= constraints.max_flatness_error_m):
        return invalid('TOOL_CONDITIONS_NOT_MET')
    if reachability is None or collision is None:
        return invalid('VALIDATORS_UNAVAILABLE')
    try:
        normal = unit(surface.normal)
        tangent = unit(surface.tangent)
        if abs(dot(normal, tangent)) > 1e-5:
            return invalid('SURFACE_AXES_INVALID')
        across = unit(cross(normal, tangent))
    except (ValueError, TypeError):
        return invalid('SURFACE_AXES_INVALID')
    room_x = (surface.width_m - constraints.contact_width_m) / 2 - constraints.edge_margin_m
    # TCP 중심만 표면 안에 있으면 부족합니다. 흡착 헤드 전체 크기와 가장자리 여유를 뺍니다.
    room_y = (surface.depth_m - constraints.contact_depth_m) / 2 - constraints.edge_margin_m
    if room_x < -1e-9 or room_y < -1e-9:
        return invalid('CONTACT_SURFACE_TOO_SMALL')
    room_x, room_y = max(0.0, room_x), max(0.0, room_y)
    tcp_z = tuple(-value for value in normal)
    # 표면 법선은 바깥 방향이고 TCP +Z는 표면으로 향합니다. 실제 공구 축 보정은 통합 전에 확인합니다.
    rotation = quaternion(tangent, cross(tcp_z, tangent), tcp_z)
    result = []
    # 기본은 중심과 네 방향의 안쪽 후보입니다. 학습 성공률이 없으면 규칙 점수만 사용합니다.
    offsets = list(dict.fromkeys([(0.0, 0.0), (room_x / 2, 0.0), (-room_x / 2, 0.0),
                                 (0.0, room_y / 2), (0.0, -room_y / 2)]))
    for dx, dy in offsets:
        position = tuple(surface.center[i] + dx * tangent[i] + dy * across[i] for i in range(3))
        target = Pose(position, rotation)
        approach = Pose(tuple(position[i] + constraints.approach_m * normal[i]
                              for i in range(3)), rotation)
        retreat = Pose(tuple(position[i] + constraints.retreat_m * normal[i]
                             for i in range(3)), rotation)
        score = surface.confidence * 100 - math.hypot(dx, dy) * 100
        candidate_data = [surface.object_id, surface.scene_revision,
                          asdict(target), asdict(constraints)]
        candidate_id = identifier('cand-', candidate_data)
        candidate = Candidate(candidate_id, target, approach, retreat, score)
        if checked(candidate, reachability, collision):
            result.append(candidate)
    result.sort(key=lambda candidate: (-candidate.rule_score, candidate.candidate_id))
    # 학습 모델이 없으므로 성공 확률은 -1입니다. 같은 점수는 후보 ID로 정렬해 결과를 일정하게 합니다.
    if not result:
        return invalid('NO_VALID_CANDIDATES')
    set_id = identifier(
        'cset-', [asdict(surface), asdict(constraints), [asdict(c) for c in result]])
    return Plan(surface.robot_id, surface.object_id, surface.scene_revision,
                constraints.version, surface.frame_id, surface.stamp_s,
                constraints.candidate_ttl_s, set_id, tuple(result))


def select_candidate(plan, now_s, *, scene_revision, constraint_version,
                     reachability, collision, preference=None):
    """Recheck candidates at execution boundary; a stale preference is only ignored."""
    # 실행 직전에 현재 장면·제약 버전·기한을 다시 확인합니다. 예전 후보를 그대로 신뢰하지 않습니다.
    if (not plan.valid or scene_revision != plan.scene_revision
            or constraint_version != plan.constraint_version or not math.isfinite(now_s)
            or not 0 <= now_s - plan.stamp_s < plan.valid_for_s):
        return None
    ordered = list(plan.candidates)
    if (preference is not None and preference.candidate_set_id == plan.candidate_set_id
            and preference.scene_revision == plan.scene_revision and preference.proposal_id):
        # 선호는 등록된 후보의 검토 순서만 바꿉니다. 부적합하면 유효한 기본 후보로 넘어갑니다.
        ordered.sort(key=lambda candidate: candidate.candidate_id != preference.candidate_id)
    # 새 의견을 기다리거나 실행 중 의견으로 이미 고른 결과를 바꾸지 않습니다.
    return next((candidate for candidate in ordered
                 if checked(candidate, reachability, collision)), None)
