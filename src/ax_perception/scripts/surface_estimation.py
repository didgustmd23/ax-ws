#!/usr/bin/env python3
"""C2 관측 표면 계산(순수 numpy·cv2, ROS 노드 없음).

같은 RGB-D 카메라의 깊이(m)·내부 파라미터·명시 ROI에서 보이는 표면 1개를 계산한다.
출력은 관측 시각에 보인 평면의 중심·법선·면 내 축·관측 크기와 계산 가능한 품질 지표다.
물체 전체 크기·전체 6D 자세·가려진 부분은 복원하지 않는다.

VALID의 의미(validity_scope='plane_fit')
  VALID는 ROI 안 주 평면 적합이 입력·분리·기하 검사를 통과했다는 뜻뿐이다.
  등록 물체와의 동일성, 물체 바깥 방향 법선, 흡착 가능·성공을 보증하지 않는다.
  ROI 대부분이 배경이면 배경 평면도 VALID가 될 수 있다(목표 물체 판정 없음).
  관측과 등록 물체의 대응은 외부 판정(계약상 C, 실행 단계 C4) 몫이며 이 모듈에는 없다.

규약
  좌표: 입력 frame은 카메라 optical(X 오른쪽, Y 아래, Z 전방). 단위 m, rad.
  법선: camera_facing 관측 규약. 카메라 쪽을 향하도록 부호를 정한다. 물체 바깥 방향이 검증됐다는 뜻이 아니다.
  면 내 축: tangent는 관측 점 분포의 주축(긴 쪽). 180° 부호는 optical +X 성분 ≥ 0(같으면 +Y 성분 ≥ 0)으로 고정한다.
    가로·세로 관측 크기가 비슷하면 90° 모호성을 axis_ambiguous로 표시한다(해결하지 않는다).
  품질: 유효 점 수·비율, 평면 잔차 RMS/p95, 평면 분리 비율 등은 관측 기하 설명값이다. 성공 확률·보정된 불확실성이 아니며
    공분산은 만들지 않는다.
  시각·frame: 깊이·내부 파라미터·ROI는 같은 카메라·같은 처리 단계(source)·같은 frame·같은 stamp여야 한다.
    이 정확 일치 검사는 이 라이브러리의 입력 조건이며 B1 동기화 허용(sync_slop 0.03 s) 계약을 바꾸지 않는다.
  시각 식별자: 권위 값은 정수 stamp_ns(header sec×10^9 + nanosec)다. stamp_s는 표시·기하용 파생값이며 식별·일치·
    계보 비교에 쓰지 않는다. stamp_origin='header'만 정확 시각이다. float 초만 남은 기록은 legacy_stamp_ns()로
    stamp_origin='legacy_float'(비정확)로만 들어오며, 원래 ns라고 주장하지 않고 정확 관측으로 바뀌지 않는다.
  변환: 상위 호출자가 관측 stamp의 강체 변환을 명시 주입한다. TF 조회·최신 TF 대체·map/위치 신뢰 정책은 없다.

사용(ROS 없이)
  sys.path에 src/ax_perception/scripts를 넣고 import surface_estimation.
  stamp = stamp_ns_from_header(sec, nanosec)            # 정수 ns(권위 시각)
  intrinsics = CameraIntrinsics.from_camera_info(width, height, k, p, d, frame_id, stamp, HEADER, source)
  depth = DepthFrame(array, '32FC1', frame_id, stamp, HEADER, source)
  roi = Roi(u_min, v_min, u_max, v_max, frame_id, stamp, HEADER, origin='test_fixture')
  observation = estimate_surface('robot_1', depth, intrinsics, roi, settings)
  in_base = transform_observation(observation, matrix_4x4, 'robot_1/base_link',
                                  observation.frame_id, observation.stamp_ns)
"""

from dataclasses import dataclass, field, replace
import hashlib
import json
import math

import cv2
import numpy as np

VALID = 'VALID'
REJECTED = 'REJECTED'
NORMAL_CONVENTION = 'camera_facing'
VALIDITY_SCOPE = 'plane_fit'   # VALID가 보증하는 범위: 평면 적합. 물체 동일성·바깥 법선·흡착 성공 아님

# 변환 행렬 강체 검사 허용(수치 오차). 운영 정확도 기준이 아니다.
RIGID_TOLERANCE = 1e-6

# 시각 출처: header(정수 sec·nanosec에서 만든 정확 시각) / legacy_float(float 초에서 만든 비정확 시각)
HEADER = 'header'
LEGACY_FLOAT = 'legacy_float'
STAMP_ORIGINS = (HEADER, LEGACY_FLOAT)
NS_PER_S = 1_000_000_000
# 관측 ID 접두어: 정확 시각과 비정확 시각의 ID 공간을 나눈다.
ID_PREFIX = {HEADER: 'obs-', LEGACY_FLOAT: 'obsL-'}


# 정수(bool 제외)인지
def is_integer(value):
    return isinstance(value, (int, np.integer)) and not isinstance(value, (bool, np.bool_))


# header의 sec·nanosec를 정수 ns로 만든다. 범위 밖이면 ValueError('STAMP_INVALID')
def stamp_ns_from_header(sec, nanosec):
    if not (is_integer(sec) and is_integer(nanosec)) or sec < 0 or not 0 <= nanosec < NS_PER_S:
        raise ValueError('STAMP_INVALID: header sec/nanosec')

    return int(sec) * NS_PER_S + int(nanosec)


# 명시적 비정확 경로: float 초만 남은 기록용. 결과는 반드시 stamp_origin=LEGACY_FLOAT와 함께 쓴다(원래 ns 아님)
def legacy_stamp_ns(stamp_s):
    if isinstance(stamp_s, bool) or not isinstance(stamp_s, (int, float)) or not math.isfinite(stamp_s) or stamp_s < 0:
        raise ValueError('STAMP_INVALID: legacy float')

    return int(round(stamp_s * NS_PER_S))


# 시각 필드 검사(생성 시). 문제가 있으면 ValueError
def check_stamp(stamp_ns, stamp_origin):
    if not is_integer(stamp_ns) or stamp_ns < 0:
        raise ValueError('STAMP_INVALID: stamp_ns는 0 이상 정수')

    if stamp_origin not in STAMP_ORIGINS:
        raise ValueError('STAMP_INVALID: stamp_origin')


@dataclass(frozen=True)
class CameraIntrinsics:
    # 무왜곡 단일 카메라 내부 파라미터. frame·stamp·source는 CameraInfo 출처 대조용이다.
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str
    stamp_ns: int
    stamp_origin: str
    source: str

    def __post_init__(self):
        check_stamp(self.stamp_ns, self.stamp_origin)

    # 표시·기하용 초(파생값, 식별에 쓰지 않음)
    @property
    def stamp_s(self):
        return self.stamp_ns / NS_PER_S

    # CameraInfo의 k(9)·p(12)·d 배열에서 만든다. B1 checked_camera_info와 같은 조건(무왜곡, P=K|0)만 받는다.
    @classmethod
    def from_camera_info(cls, width, height, k, p, d, frame_id, stamp_ns, stamp_origin, source):
        k = np.asarray(k, dtype=float)
        p = np.asarray(p, dtype=float)
        d = np.asarray(d, dtype=float)

        if k.size != 9 or p.size != 12 or not np.isfinite(np.r_[k, p, d]).all():
            raise ValueError('CAMERA_MODEL_INVALID: 비유한 값 또는 크기 오류')

        if int(width) <= 0 or int(height) <= 0:
            raise ValueError('CAMERA_MODEL_INVALID: 영상 크기')

        if k[0] <= 0 or k[4] <= 0 or not np.allclose(k[[1, 3, 6, 7, 8]], [0, 0, 0, 0, 1]):
            raise ValueError('CAMERA_MODEL_UNSUPPORTED: 내부 파라미터 형식')

        expected = [k[0], 0, k[2], 0, 0, k[4], k[5], 0, 0, 0, 1, 0]

        if not np.allclose(p, expected, rtol=1e-5, atol=1e-6) or not np.allclose(d, 0):
            raise ValueError('CAMERA_MODEL_UNSUPPORTED: 왜곡 또는 투영 행렬 불일치')

        return cls(int(width), int(height), float(k[0]), float(k[4]), float(k[2]), float(k[5]),
                   frame_id, stamp_ns, stamp_origin, source)


@dataclass(frozen=True)
class DepthFrame:
    # 깊이 영상(H×W, m). encoding은 32FC1만 받는다. 무효 깊이는 NaN 등 비유한 값이다.
    depth: np.ndarray
    encoding: str
    frame_id: str
    stamp_ns: int
    stamp_origin: str
    source: str

    def __post_init__(self):
        check_stamp(self.stamp_ns, self.stamp_origin)

    @property
    def stamp_s(self):
        return self.stamp_ns / NS_PER_S


@dataclass(frozen=True)
class Roi:
    # 픽셀 ROI [u_min, u_max) × [v_min, v_max). origin은 ROI 출처(예: 'test_fixture')이며 C1 검출이라고 표시하지 않는 한 검출이 아니다.
    u_min: int
    v_min: int
    u_max: int
    v_max: int
    frame_id: str
    stamp_ns: int
    stamp_origin: str
    origin: str

    def __post_init__(self):
        check_stamp(self.stamp_ns, self.stamp_origin)

    @property
    def stamp_s(self):
        return self.stamp_ns / NS_PER_S


@dataclass(frozen=True)
class EstimationSettings:
    # 계산 설정. 기본값을 두지 않는다. 호출자가 의미·출처를 정해 넘긴다(운영 승인 기준 아님).
    depth_min_m: float              # 유효 깊이 하한(m)
    depth_max_m: float              # 유효 깊이 상한(m)
    min_valid_points: int           # ROI 안 유효 깊이 점 최소 수
    min_valid_fraction: float       # ROI 픽셀 중 유효 깊이 비율 하한(0~1)
    inlier_distance_m: float        # 평면 inlier 거리(m)
    ransac_iterations: int          # RANSAC 반복 수
    ransac_seed: int                # 난수 시드(같은 입력 같은 결과)
    min_inlier_fraction: float      # 유효 점 중 주 평면 inlier 비율 하한. 미만이면 배경 혼입으로 거부
    max_second_plane_fraction: float  # 남은 점에서 찾은 두 번째 평면이 유효 점 중 이 비율 이상이면 복수 평면으로 거부
    min_component_fraction: float   # 주 평면 inlier 중 가장 큰 연결 덩어리 비율 하한. 미만이면 분리 근거 부족
    min_extent_m: float             # 관측 짧은 변 하한(m). 미만이면 퇴화(선 모양)
    min_extent_ratio: float         # 짧은 변/긴 변 하한. 미만이면 퇴화
    extent_percentile: float        # 관측 크기 계산 백분위(예: 1이면 1~99 백분위 범위)
    axis_ambiguity_ratio: float     # 긴 변/짧은 변 ≤ 1 + 이 값이면 90° 축 모호
    max_incidence_deg: float        # 시선과 법선 사이 각 상한(도). 초과면 비스듬한 관측으로 거부


@dataclass(frozen=True)
class SurfaceObservation:
    # C2 관측 표면 1개. status가 VALID가 아니면 기하 필드는 None일 수 있다.
    # VALID는 validity_scope('plane_fit') 범위의 유효이며 등록 물체 동일성·바깥 법선·흡착 성공 보증이 아니다.
    # observed_frame_id는 처음 관측한 카메라 frame이고, 변환 뒤에도 바뀌지 않는다(transforms가 계보).
    status: str
    reasons: tuple
    observation_id: str
    robot_id: str
    frame_id: str
    stamp_ns: int
    stamp_origin: str
    roi_origin: str
    normal_convention: str = NORMAL_CONVENTION
    center: tuple = None
    normal: tuple = None
    tangent: tuple = None
    across: tuple = None
    width_m: float = None
    depth_m: float = None
    axis_ambiguous: bool = None
    quality: dict = field(default_factory=dict)
    transforms: tuple = ()
    observed_frame_id: str = None
    validity_scope: str = VALIDITY_SCOPE

    # 생성·replace 때마다 시각과 ID 공간이 맞는지 본다(비정확 관측을 정확 관측으로 바꾸는 재포장 차단)
    def __post_init__(self):
        check_stamp(self.stamp_ns, self.stamp_origin)

        if not isinstance(self.observation_id, str) or not self.observation_id.startswith(ID_PREFIX[self.stamp_origin]):
            raise ValueError('STAMP_ORIGIN_ID_MISMATCH')

        if self.stamp_origin == HEADER and self.observation_id.startswith(ID_PREFIX[LEGACY_FLOAT]):
            raise ValueError('STAMP_ORIGIN_ID_MISMATCH')

    # 평면 적합 유효 여부(목표 물체 관측 유효가 아니다)
    @property
    def valid(self):
        return self.status == VALID

    # 표시·기하용 초(파생값)
    @property
    def stamp_s(self):
        return self.stamp_ns / NS_PER_S

    # 정확 시각(header 정수) 관측인지
    @property
    def stamp_exact(self):
        return self.stamp_origin == HEADER


# 깊이 영상 내용 지문. 같은 stamp·ROI라도 깊이 내용이 다르면 다른 관측 ID가 되게 한다
def depth_digest(depth):
    array = depth.depth

    if not isinstance(array, np.ndarray):
        return ''

    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()[:16]


# 관측 식별자. robot·정수 stamp_ns·시각 출처·frame·ROI·깊이 내용 지문으로 정하며 지속 object_id가 아니다.
def observation_identifier(robot_id, frame_id, stamp_ns, stamp_origin, roi, content=''):
    payload = json.dumps([robot_id, frame_id, int(stamp_ns), stamp_origin, roi.u_min, roi.v_min, roi.u_max,
                          roi.v_max, content], separators=(',', ':'), default=str)

    return ID_PREFIX[stamp_origin] + hashlib.sha256(payload.encode()).hexdigest()[:20]


# 거부 관측을 만든다
def rejected_observation(robot_id, frame_id, stamp_ns, stamp_origin, roi, reasons, quality=None, content=''):
    return SurfaceObservation(
        REJECTED, tuple(reasons), observation_identifier(robot_id, frame_id, stamp_ns, stamp_origin, roi, content),
        robot_id, frame_id, stamp_ns, stamp_origin, roi.origin, quality=dict(quality or {}),
        observed_frame_id=frame_id,
    )


# 설정 값이 유한하고 범위가 맞는지 본다. 문제가 있으면 ValueError
def check_settings(settings):
    values = [getattr(settings, name) for name in settings.__dataclass_fields__]

    if not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
               for value in values):
        raise ValueError('SETTINGS_INVALID: 비유한 값')

    fractions = (settings.min_valid_fraction, settings.min_inlier_fraction,
                 settings.max_second_plane_fraction, settings.min_component_fraction, settings.min_extent_ratio)

    if (not 0 < settings.depth_min_m < settings.depth_max_m or settings.min_valid_points < 3
            or settings.inlier_distance_m <= 0 or settings.ransac_iterations < 1
            or not all(0 <= value <= 1 for value in fractions) or settings.min_extent_m < 0
            or not 0 <= settings.extent_percentile < 50 or settings.axis_ambiguity_ratio < 0
            or not 0 < settings.max_incidence_deg <= 90):
        raise ValueError('SETTINGS_INVALID: 범위')


# 깊이·내부 파라미터·ROI가 같은 카메라·처리 단계·frame·stamp인지 본다. 문제 사유 목록(정상이면 빈 목록)
def capture_mismatches(depth, intrinsics, roi):
    reasons = []

    if depth.source != intrinsics.source:
        reasons.append('SOURCE_MISMATCH')

    if not depth.frame_id or len({depth.frame_id, intrinsics.frame_id, roi.frame_id}) != 1:
        reasons.append('FRAME_MISMATCH')

    # 정수 ns로만 비교한다. 시각 출처(정확/비정확)가 섞이면 거부한다.
    stamps = (depth.stamp_ns, intrinsics.stamp_ns, roi.stamp_ns)

    if not all(is_integer(value) and value > 0 for value in stamps):
        reasons.append('STAMP_INVALID')
    elif len(set(stamps)) != 1:
        reasons.append('STAMP_MISMATCH')

    if len({depth.stamp_origin, intrinsics.stamp_origin, roi.stamp_origin}) != 1:
        reasons.append('STAMP_ORIGIN_MISMATCH')

    return reasons


# 깊이 배열 형식을 본다. 문제 사유 목록
def depth_problems(depth, intrinsics):
    array = depth.depth

    if depth.encoding != '32FC1':
        return ['DEPTH_ENCODING_UNSUPPORTED']

    if not isinstance(array, np.ndarray) or array.ndim != 2 or array.dtype != np.float32:
        return ['DEPTH_FORMAT_INVALID']

    if array.shape != (intrinsics.height, intrinsics.width):
        return ['SIZE_MISMATCH']

    return []


# ROI를 영상 안으로 자른다. (잘린 ROI 경계, 경계 잘림 여부) 또는 무효면 None
def clip_roi(roi, intrinsics):
    bounds = (roi.u_min, roi.v_min, roi.u_max, roi.v_max)

    if not all(isinstance(value, (int, np.integer)) and not isinstance(value, bool) for value in bounds):
        return None

    u_min, v_min = max(roi.u_min, 0), max(roi.v_min, 0)
    u_max, v_max = min(roi.u_max, intrinsics.width), min(roi.v_max, intrinsics.height)

    if u_max - u_min < 1 or v_max - v_min < 1:
        return None

    # 영상 경계에 닿으면 물체가 화면 밖으로 이어질 수 있어 잘림으로 기록한다.
    touches = u_min == 0 or v_min == 0 or u_max == intrinsics.width or v_max == intrinsics.height
    clipped = (u_min, v_min, u_max, v_max) != bounds or touches

    return (u_min, v_min, u_max, v_max), clipped


# ROI 안의 유효 깊이를 optical 점으로 역투영한다. (점 N×3, 픽셀 (v, u) N×2, ROI 픽셀 수)
def back_project(depth, intrinsics, bounds, settings):
    u_min, v_min, u_max, v_max = bounds
    patch = depth.depth[v_min:v_max, u_min:u_max].astype(np.float64)
    valid = np.isfinite(patch) & (patch >= settings.depth_min_m) & (patch <= settings.depth_max_m)
    rows, columns = np.nonzero(valid)
    z = patch[rows, columns]
    u = columns + u_min
    v = rows + v_min
    points = np.column_stack(((u - intrinsics.cx) * z / intrinsics.fx, (v - intrinsics.cy) * z / intrinsics.fy, z))

    return points, np.column_stack((v, u)), patch.size


# 세 점 평면(단위 법선, 원점 거리)을 만든다. 퇴화면 None
def plane_from_points(a, b, c):
    normal = np.cross(b - a, c - a)
    length = np.linalg.norm(normal)

    if length < 1e-12:
        return None

    normal = normal / length

    return normal, float(normal @ a)


# RANSAC으로 inlier가 가장 많은 평면을 찾는다. inlier 불리언 배열 또는 None
def ransac_plane(points, settings, seed):
    if len(points) < 3:
        return None

    generator = np.random.default_rng(seed)
    best = None
    best_count = 0

    for _ in range(settings.ransac_iterations):
        a, b, c = points[generator.choice(len(points), 3, replace=False)]
        plane = plane_from_points(a, b, c)

        if plane is None:
            continue

        inliers = np.abs(points @ plane[0] - plane[1]) <= settings.inlier_distance_m
        count = int(inliers.sum())

        if count > best_count:
            best, best_count = inliers, count

    return best


# 픽셀 마스크에서 가장 큰 8-연결 덩어리에 속한 점 표시를 돌려준다
def largest_component(pixels, shape):
    mask = np.zeros(shape, dtype=np.uint8)
    mask[pixels[:, 0], pixels[:, 1]] = 1
    count, labels = cv2.connectedComponents(mask, connectivity=8)

    if count <= 1:
        return np.zeros(len(pixels), dtype=bool)

    sizes = np.bincount(labels.ravel())[1:]
    label = int(np.argmax(sizes)) + 1

    return labels[pixels[:, 0], pixels[:, 1]] == label


# 점들에 최소제곱 평면(SVD)을 맞춘다. (중심, 단위 법선, 면 내 주축 두 개, 특이값)
def fit_plane(points):
    centroid = points.mean(axis=0)
    _, singular, vectors = np.linalg.svd(points - centroid, full_matrices=False)

    return centroid, vectors[2], vectors[0], vectors[1], singular


# tangent 180° 부호 규칙: optical +X 성분 ≥ 0, 같으면 +Y 성분 ≥ 0
def fixed_sign(vector):
    if abs(vector[0]) > 1e-12:
        return vector if vector[0] > 0 else -vector

    return vector if vector[1] >= 0 else -vector


# 보이는 표면 1개를 계산한다. 입력 문제·분리 근거 부족은 REJECTED와 사유로 돌려준다(예외로 숨기지 않음).
def estimate_surface(robot_id, depth, intrinsics, roi, settings):
    check_settings(settings)
    frame_id, stamp_ns, stamp_origin = depth.frame_id, depth.stamp_ns, depth.stamp_origin
    content = depth_digest(depth)

    # 이 호출의 거부 관측(같은 깊이 지문·시각 출처를 쓴다)
    def rejected(robot_id, frame_id, stamp_ns, roi, reasons, quality=None):
        return rejected_observation(robot_id, frame_id, stamp_ns, stamp_origin, roi, reasons, quality, content)
    reasons = capture_mismatches(depth, intrinsics, roi) + depth_problems(depth, intrinsics)

    if not robot_id:
        reasons.append('ROBOT_ID_MISSING')

    if reasons:
        return rejected(robot_id, frame_id, stamp_ns, roi, reasons)

    clipped_roi = clip_roi(roi, intrinsics)

    if clipped_roi is None:
        return rejected(robot_id, frame_id, stamp_ns, roi, ['ROI_INVALID'])

    bounds, edge_clipped = clipped_roi
    points, pixels, roi_pixels = back_project(depth, intrinsics, bounds, settings)
    quality = {
        'roi_pixel_count': int(roi_pixels),
        'valid_point_count': int(len(points)),
        'valid_fraction': len(points) / roi_pixels,
        'edge_clipped': bool(edge_clipped),
        'roi_used': list(bounds),
    }

    # 표본 부족·유효 비율 부족
    if len(points) < settings.min_valid_points or quality['valid_fraction'] < settings.min_valid_fraction:
        return rejected(robot_id, frame_id, stamp_ns, roi, ['INSUFFICIENT_POINTS'], quality)

    # 주 평면(inlier가 가장 많은 평면). 가장 가까운 평면을 목표로 가정하지 않는다.
    inliers = ransac_plane(points, settings, settings.ransac_seed)

    if inliers is None or inliers.sum() < 3:
        return rejected(robot_id, frame_id, stamp_ns, roi, ['PLANE_FIT_FAILED'], quality)

    quality['inlier_fraction'] = float(inliers.mean())

    # 남은 점에서 두 번째 평면을 찾아 복수 표면 여부를 본다.
    rest = points[~inliers]
    second = ransac_plane(rest, settings, settings.ransac_seed + 1) if len(rest) >= 3 else None
    quality['second_plane_fraction'] = float(second.sum()) / len(points) if second is not None else 0.0

    reasons = []

    if quality['inlier_fraction'] < settings.min_inlier_fraction:
        reasons.append('BACKGROUND_MIXED')

    if quality['second_plane_fraction'] >= settings.max_second_plane_fraction:
        reasons.append('MULTIPLE_SURFACES')

    # 주 평면 inlier가 한 덩어리인지(좌우로 떨어진 바닥 등) 본다.
    component = largest_component(pixels[inliers] - [bounds[1], bounds[0]],
                                  (bounds[3] - bounds[1], bounds[2] - bounds[0]))
    quality['component_fraction'] = float(component.mean())

    if quality['component_fraction'] < settings.min_component_fraction:
        reasons.append('SURFACE_FRAGMENTED')

    if reasons:
        return rejected(robot_id, frame_id, stamp_ns, roi, reasons, quality)

    surface_points = points[inliers][component]

    if len(surface_points) < settings.min_valid_points:
        return rejected(robot_id, frame_id, stamp_ns, roi, ['INSUFFICIENT_POINTS'], quality)

    centroid, normal, major, minor, singular = fit_plane(surface_points)

    # camera_facing 부호: 카메라(원점)에서 표면으로 가는 시선과 반대쪽을 향하게 한다.
    if normal @ centroid > 0:
        normal = -normal

    residual = (surface_points - centroid) @ normal
    quality['plane_rms_m'] = float(np.sqrt(np.mean(residual ** 2)))
    quality['plane_p95_m'] = float(np.percentile(np.abs(residual), 95))
    quality['surface_point_count'] = int(len(surface_points))
    incidence = math.degrees(math.acos(min(1.0, abs(float(normal @ (-centroid / np.linalg.norm(centroid)))))))
    quality['incidence_deg'] = incidence

    # 면 내 축과 관측 크기(백분위 범위). 보인 범위이며 물체 전체 크기가 아니다.
    tangent = fixed_sign(major - (major @ normal) * normal)
    tangent = tangent / np.linalg.norm(tangent)
    across = np.cross(normal, tangent)
    low, high = settings.extent_percentile, 100 - settings.extent_percentile
    along = (surface_points - centroid) @ tangent
    side = (surface_points - centroid) @ across
    along_range = np.percentile(along, [low, high])
    side_range = np.percentile(side, [low, high])
    width, depth_extent = float(np.diff(along_range)[0]), float(np.diff(side_range)[0])
    center = centroid + tangent * along_range.mean() + across * side_range.mean()

    reasons = []

    if incidence > settings.max_incidence_deg:
        reasons.append('NORMAL_GRAZING')

    if depth_extent < settings.min_extent_m or depth_extent < settings.min_extent_ratio * width:
        reasons.append('DEGENERATE_SURFACE')

    if reasons:
        return rejected(robot_id, frame_id, stamp_ns, roi, reasons, quality)

    return SurfaceObservation(
        VALID, (), observation_identifier(robot_id, frame_id, stamp_ns, stamp_origin, roi, content),
        robot_id, frame_id, stamp_ns, stamp_origin, roi.origin,
        center=tuple(float(x) for x in center), normal=tuple(float(x) for x in normal),
        tangent=tuple(float(x) for x in tangent), across=tuple(float(x) for x in across),
        width_m=width, depth_m=depth_extent,
        axis_ambiguous=bool(width <= (1 + settings.axis_ambiguity_ratio) * depth_extent),
        quality=quality, observed_frame_id=frame_id,
    )


# 4×4 강체 변환 유효성. 문제가 있으면 사유 문자열, 정상이면 None
def rigid_problem(matrix):
    try:
        matrix = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError):
        return 'TRANSFORM_INVALID'

    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        return 'TRANSFORM_INVALID'

    rotation = matrix[:3, :3]

    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=RIGID_TOLERANCE)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=RIGID_TOLERANCE)
            or abs(np.linalg.det(rotation) - 1) > RIGID_TOLERANCE):
        return 'TRANSFORM_NOT_RIGID'

    return None


# 관측을 주입한 강체 변환(source_frame → target_frame, 관측 stamp_ns)으로 옮긴다.
# 중심은 회전+이동, 법선·축은 회전만. 거부 관측은 그대로 돌려준다. 문제가 있으면 REJECTED 관측.
# transform_stamp_ns는 변환이 실제로 나타내는 시각(정수 ns)이며 관측 stamp_ns와 정수로 같아야 한다.
def transform_observation(observation, matrix, target_frame, source_frame, transform_stamp_ns):
    if not observation.valid:
        return observation

    reasons = []

    if source_frame != observation.frame_id:
        reasons.append('TRANSFORM_FRAME_MISMATCH')

    if not is_integer(transform_stamp_ns):
        reasons.append('TRANSFORM_STAMP_INVALID')
    elif transform_stamp_ns != observation.stamp_ns:
        reasons.append('TRANSFORM_STAMP_MISMATCH')

    if not target_frame:
        reasons.append('TRANSFORM_TARGET_MISSING')

    problem = rigid_problem(matrix)

    if problem:
        reasons.append(problem)

    if reasons:
        return replace(observation, status=REJECTED, reasons=tuple(reasons),
                       center=None, normal=None, tangent=None, across=None)

    matrix = np.asarray(matrix, dtype=float)
    rotation, translation = matrix[:3, :3], matrix[:3, 3]

    def direction(vector):
        return tuple(float(x) for x in rotation @ np.asarray(vector))

    return replace(
        observation,
        frame_id=target_frame,
        center=tuple(float(x) for x in rotation @ np.asarray(observation.center) + translation),
        normal=direction(observation.normal),
        tangent=direction(observation.tangent),
        across=direction(observation.across),
        transforms=observation.transforms + ((source_frame, target_frame, int(transform_stamp_ns)),),
    )


# 관측 frame 계보가 일관적인지 본다. 문제가 있으면 사유 문자열, 정상이면 None.
# 처음 관측 frame → transforms 연쇄 → 현재 frame이 이어지고, 모든 변환 stamp_ns가 관측 stamp_ns와 정수로 같아야 한다
def lineage_problem(observation):
    frame = observation.observed_frame_id

    if not frame:
        return 'LINEAGE_INVALID'

    for step in observation.transforms:
        if (len(step) != 3 or step[0] != frame or not is_integer(step[2]) or step[2] != observation.stamp_ns
                or not step[1]):
            return 'LINEAGE_INVALID'

        frame = step[1]

    if frame != observation.frame_id:
        return 'LINEAGE_INVALID'

    return None
