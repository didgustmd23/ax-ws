"""C2 관측 표면 계산·C3 어댑터를 합성 깊이로 검사한다. ROS·Gazebo 없이 실행한다."""
# [검사 범위] 합성 입력 기반 계산·거부 검증이다. 합성 ROI는 시험 fixture이며 C1 실검출이 아니다.
# 허용오차는 무잡음 합성 렌더의 수치·픽셀 이산화 오차 범위이며 운영 정확도 기준이 아니다.
# Gazebo 물체 pose는 추정 입력에 쓰지 않는다(이 시험은 Gazebo를 쓰지 않는다).
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_perception/scripts'))
sys.path.insert(0, str(ROOT / 'src/ax_grasp_planner'))

import surface_estimation as se  # noqa: E402
import c2_surface_adapter as adapter  # noqa: E402
from ax_grasp_planner.planner import Constraints, plan_candidates, Surface  # noqa: E402

FRAME = 'robot_1/front_rgbd_optical_frame'
SOURCE = 'robot_1/sensors/front_rgbd/processed'
NS = se.NS_PER_S
STAMP = 100 * NS   # 정수 ns(권위 시각). 100 s
WIDTH, HEIGHT = 640, 480
FOCAL = 320 / math.tan(math.pi / 6)   # sensors.xacro 기본 hfov π/3과 같은 식

# 시험용 설정(fixture). 의미·단위는 EstimationSettings 주석. 운영 승인 기준이 아니다.
TEST_SETTINGS = se.EstimationSettings(
    depth_min_m=0.05, depth_max_m=5.0,          # B1 클립 값과 같게 둔 시험값
    min_valid_points=50, min_valid_fraction=0.3,
    inlier_distance_m=0.005, ransac_iterations=200, ransac_seed=7,
    min_inlier_fraction=0.8, max_second_plane_fraction=0.15, min_component_fraction=0.9,
    min_extent_m=0.02, min_extent_ratio=0.1, extent_percentile=0.0,
    axis_ambiguity_ratio=0.05, max_incidence_deg=75.0,
)


# 어댑터 문자열 길이 상한(시험값, 운영값 아님)
ADAPTER_TEXT_LIMIT = 128


# 시험 카메라 내부 파라미터(무왜곡)
def intrinsics(frame=FRAME, stamp=STAMP, source=SOURCE, origin=se.HEADER):
    k = [FOCAL, 0, 320, 0, FOCAL, 240, 0, 0, 1]
    p = [FOCAL, 0, 320, 0, 0, FOCAL, 240, 0, 0, 0, 1, 0]

    return se.CameraIntrinsics.from_camera_info(WIDTH, HEIGHT, k, p, [0] * 5, frame, stamp, origin, source)


# 어댑터 호출(시험 문자열 상한)
def to_surface(observation, inputs):
    return adapter.to_surface(observation, inputs, ADAPTER_TEXT_LIMIT)


# 단위 벡터
def unit(vector):
    vector = np.asarray(vector, dtype=float)

    return vector / np.linalg.norm(vector)


# 직사각 평면 조각을 깊이 영상에 그린다(가까운 쪽 우선). 카메라는 optical 원점이다
def draw_rectangle(depth, center, normal, tangent, width, height):
    u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    rays = np.stack(((u - 320) / FOCAL, (v - 240) / FOCAL, np.ones_like(u, dtype=float)), axis=-1)
    normal, tangent = unit(normal), unit(tangent)
    across = np.cross(normal, tangent)
    denominator = rays @ normal
    with np.errstate(divide='ignore', invalid='ignore'):
        distance = (np.asarray(center) @ normal) / denominator
    points = rays * distance[..., None]
    offset = points - np.asarray(center)
    inside = ((np.abs(offset @ tangent) <= width / 2) & (np.abs(offset @ across) <= height / 2)
              & (distance > 0) & np.isfinite(distance))
    z = points[..., 2]
    closer = inside & (~np.isfinite(depth) | (z < depth))
    depth[closer] = z[closer]

    return depth


# 빈(전부 NaN) 깊이 영상
def blank():
    return np.full((HEIGHT, WIDTH), np.nan, dtype=np.float32)


# 깊이 배열로 DepthFrame을 만든다
def frame_of(depth, frame=FRAME, stamp=STAMP, source=SOURCE, encoding='32FC1', origin=se.HEADER):
    return se.DepthFrame(depth.astype(np.float32) if depth.dtype != np.float32 else depth, encoding, frame,
                         stamp, origin, source)


# 시험 fixture ROI
def roi(u_min=200, v_min=150, u_max=440, v_max=330, frame=FRAME, stamp=STAMP, stamp_origin=se.HEADER):
    return se.Roi(u_min, v_min, u_max, v_max, frame, stamp, stamp_origin, origin='test_fixture')


# 그린 유한 깊이를 감싸는 fixture ROI(검출기처럼 물체에 맞춘 상자 + 여유 픽셀). C1 검출이 아니다
def tight_roi(depth, margin=3):
    rows, columns = np.nonzero(np.isfinite(depth))

    return roi(int(columns.min()) - margin, int(rows.min()) - margin,
               int(columns.max()) + 1 + margin, int(rows.max()) + 1 + margin)


# 표면 하나를 계산한다
def estimate(depth, region=None, settings=TEST_SETTINGS, camera=None):
    return se.estimate_surface('robot_1', frame_of(depth), camera or intrinsics(), region or roi(), settings)


# 두 방향 사이 각(rad)
def angle(a, b):
    return math.acos(max(-1.0, min(1.0, float(unit(a) @ unit(b)))))


# 정면 30×20 cm 평면(중심 (0.05, -0.02, 1.0))
def front_plane():
    return draw_rectangle(blank(), (0.05, -0.02, 1.0), (0, 0, -1), (1, 0, 0), 0.30, 0.20)


class KnownPlanes(unittest.TestCase):
    def test_front_plane_center_normal_axes_and_size(self):
        result = estimate(front_plane())
        self.assertEqual(result.status, 'VALID', result.reasons)
        self.assertEqual(result.normal_convention, 'camera_facing')
        np.testing.assert_allclose(result.center, (0.05, -0.02, 1.0), atol=0.003)
        np.testing.assert_allclose(result.normal, (0, 0, -1), atol=1e-6)
        np.testing.assert_allclose(result.tangent, (1, 0, 0), atol=1e-6)
        self.assertAlmostEqual(result.width_m, 0.30, delta=0.004)
        self.assertAlmostEqual(result.depth_m, 0.20, delta=0.004)
        self.assertFalse(result.axis_ambiguous)
        self.assertEqual(result.roi_origin, 'test_fixture')
        self.assertLess(result.quality['plane_rms_m'], 1e-4)
        self.assertEqual(result.frame_id, FRAME)
        self.assertEqual(result.stamp_ns, STAMP)
        self.assertEqual(result.stamp_s, 100.0)
        self.assertTrue(result.stamp_exact)

    def test_tilted_plane_normal_faces_camera(self):
        # X축으로 30° 기운 면(위쪽이 멀어짐). 법선은 카메라 쪽(−Z 성분)이어야 한다.
        tilt = math.radians(30)
        normal = (0, -math.sin(tilt), -math.cos(tilt))
        tangent = (1, 0, 0)
        depth = draw_rectangle(blank(), (0.0, 0.0, 1.2), normal, tangent, 0.30, 0.20)
        result = estimate(depth, tight_roi(depth))
        self.assertEqual(result.status, 'VALID', result.reasons)
        self.assertLess(angle(result.normal, normal), 1e-3)
        self.assertLess(result.normal[2], 0)
        np.testing.assert_allclose(result.center, (0.0, 0.0, 1.2), atol=0.004)
        self.assertAlmostEqual(result.width_m, 0.30, delta=0.005)
        self.assertAlmostEqual(result.depth_m, 0.20, delta=0.005)
        self.assertAlmostEqual(result.quality['incidence_deg'], 30.0, delta=0.5)

    def test_axes_are_unit_orthogonal_and_sign_deterministic(self):
        first = estimate(front_plane())
        flipped = estimate(draw_rectangle(blank(), (0.05, -0.02, 1.0), (0, 0, -1), (-1, 0, 0), 0.30, 0.20))
        np.testing.assert_allclose(first.tangent, flipped.tangent, atol=1e-9)
        self.assertGreaterEqual(first.tangent[0], 0)

        for vector in (first.normal, first.tangent, first.across):
            self.assertAlmostEqual(np.linalg.norm(vector), 1.0, places=9)

        self.assertLess(abs(np.dot(first.normal, first.tangent)), 1e-9)
        np.testing.assert_allclose(np.cross(first.normal, first.tangent), first.across, atol=1e-9)

        # 같은 입력은 같은 관측 ID·결과를 낸다.
        again = estimate(front_plane())
        self.assertEqual(first.observation_id, again.observation_id)
        self.assertEqual(first.center, again.center)
        self.assertTrue(first.observation_id.startswith('obs-'))

    def test_square_face_marks_axis_ambiguity(self):
        depth = draw_rectangle(blank(), (0.0, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.20, 0.20)
        result = estimate(depth, tight_roi(depth))
        self.assertEqual(result.status, 'VALID', result.reasons)
        self.assertTrue(result.axis_ambiguous)


class RigidTransform(unittest.TestCase):
    def setUp(self):
        self.observation = estimate(front_plane())
        yaw = math.radians(90)
        self.matrix = np.eye(4)
        self.matrix[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
        self.matrix[:3, 3] = (1.0, 2.0, 3.0)

    def move(self, matrix=None, source=FRAME, stamp=STAMP, target='robot_1/base_link'):
        return se.transform_observation(self.observation, self.matrix if matrix is None else matrix,
                                        target, source, stamp)

    def test_center_moves_directions_rotate_only(self):
        moved = self.move()
        rotation = self.matrix[:3, :3]
        self.assertEqual(moved.status, 'VALID')
        self.assertEqual(moved.frame_id, 'robot_1/base_link')
        np.testing.assert_allclose(moved.center, rotation @ self.observation.center + (1, 2, 3), atol=1e-12)
        np.testing.assert_allclose(moved.normal, rotation @ self.observation.normal, atol=1e-12)
        np.testing.assert_allclose(moved.tangent, rotation @ self.observation.tangent, atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(moved.normal), 1.0, places=12)
        self.assertLess(abs(np.dot(moved.normal, moved.tangent)), 1e-9)
        self.assertEqual(moved.transforms, ((FRAME, 'robot_1/base_link', STAMP),))
        self.assertEqual((moved.width_m, moved.depth_m), (self.observation.width_m, self.observation.depth_m))

    def test_invalid_transforms_are_rejected(self):
        scaled = self.matrix.copy()
        scaled[:3, :3] *= 2
        bottom = self.matrix.copy()
        bottom[3] = (0, 0, 1, 1)
        nan = self.matrix.copy()
        nan[0, 3] = np.nan
        mirror = np.diag([1.0, 1.0, -1.0, 1.0])
        cases = {
            'TRANSFORM_NOT_RIGID': [self.move(scaled), self.move(bottom), self.move(mirror)],
            'TRANSFORM_INVALID': [self.move(nan), self.move(np.eye(3))],
            'TRANSFORM_FRAME_MISMATCH': [self.move(source='robot_1/front_rgb_optical_frame')],
            'TRANSFORM_STAMP_MISMATCH': [self.move(stamp=STAMP + 10_000_000), self.move(stamp=STAMP + 1)],
            'TRANSFORM_STAMP_INVALID': [self.move(stamp=100.0), self.move(stamp=True)],
            'TRANSFORM_TARGET_MISSING': [self.move(target='')],
        }

        for reason, results in cases.items():
            for result in results:
                self.assertEqual(result.status, 'REJECTED', reason)
                self.assertIn(reason, result.reasons)
                self.assertIsNone(result.center)

    def test_rejected_observation_passes_through(self):
        bad = estimate(blank())
        self.assertIs(se.transform_observation(bad, self.matrix, 'robot_1/base_link', FRAME, STAMP), bad)


class InputRejection(unittest.TestCase):
    def test_capture_mismatches_are_not_valid(self):
        depth = front_plane()
        cases = {
            'STAMP_MISMATCH': dict(region=roi(stamp=STAMP + 30_000_000)),
            'STAMP_ORIGIN_MISMATCH': dict(region=roi(stamp_origin=se.LEGACY_FLOAT)),
            'FRAME_MISMATCH': dict(region=roi(frame='robot_1/front_rgb_optical_frame')),
            'SOURCE_MISMATCH': dict(camera=intrinsics(source='robot_1/sensors/front_rgbd/raw')),
        }

        for reason, arguments in cases.items():
            result = estimate(depth, **arguments)
            self.assertEqual(result.status, 'REJECTED', reason)
            self.assertIn(reason, result.reasons)

        # 깊이 stamp만 다른 경우(검출 시각과 다른 묶음)
        result = se.estimate_surface('robot_1', frame_of(depth, stamp=STAMP - 500_000_000), intrinsics(), roi(),
                                     TEST_SETTINGS)
        self.assertIn('STAMP_MISMATCH', result.reasons)

    def test_depth_format_and_size(self):
        depth = front_plane()
        cases = {
            'DEPTH_ENCODING_UNSUPPORTED': frame_of(depth, encoding='16UC1'),
            'DEPTH_FORMAT_INVALID': se.DepthFrame(depth.astype(np.float64), '32FC1', FRAME, STAMP, se.HEADER, SOURCE),
            'SIZE_MISMATCH': frame_of(depth[:, :320].copy()),
        }

        for reason, frame in cases.items():
            result = se.estimate_surface('robot_1', frame, intrinsics(), roi(), TEST_SETTINGS)
            self.assertIn(reason, result.reasons)

    def test_invalid_intrinsics_raise(self):
        k = [FOCAL, 0, 320, 0, FOCAL, 240, 0, 0, 1]
        p = [FOCAL, 0, 320, 0, 0, FOCAL, 240, 0, 0, 0, 1, 0]

        with self.assertRaisesRegex(ValueError, 'CAMERA_MODEL_UNSUPPORTED'):
            se.CameraIntrinsics.from_camera_info(WIDTH, HEIGHT, k, p, [0.1, 0, 0, 0, 0], FRAME, STAMP, se.HEADER,
                                                 SOURCE)

        with self.assertRaisesRegex(ValueError, 'CAMERA_MODEL_INVALID'):
            se.CameraIntrinsics.from_camera_info(WIDTH, HEIGHT, [np.nan] + k[1:], p, [0] * 5, FRAME, STAMP,
                                                 se.HEADER, SOURCE)

        with self.assertRaisesRegex(ValueError, 'CAMERA_MODEL_UNSUPPORTED'):
            se.CameraIntrinsics.from_camera_info(WIDTH, HEIGHT, k, [FOCAL + 1] + p[1:], [0] * 5, FRAME, STAMP,
                                                 se.HEADER, SOURCE)

    def test_roi_validity_and_edge_clipping(self):
        depth = front_plane()

        for region in (roi(700, 0, 800, 100), roi(300, 200, 300, 260), roi(200.5, 150, 440, 330)):
            self.assertIn('ROI_INVALID', estimate(depth, region).reasons)

        # 영상 밖으로 나간 ROI는 잘라 쓰고 경계 잘림을 기록한다.
        wide = draw_rectangle(blank(), (0.0, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 1.4, 0.2)
        result = estimate(wide, roi(-50, 150, 700, 330))
        self.assertEqual(result.status, 'VALID', result.reasons)
        self.assertTrue(result.quality['edge_clipped'])
        self.assertEqual(result.quality['roi_used'], [0, 150, WIDTH, 330])
        self.assertFalse(estimate(depth).quality['edge_clipped'])

    def test_missing_nonfinite_and_out_of_range_depth(self):
        self.assertIn('INSUFFICIENT_POINTS', estimate(blank()).reasons)

        depth = front_plane()
        noisy = depth.copy()
        noisy[160:170, :] = np.inf
        noisy[170:175, :] = 10.0     # 상한 밖
        noisy[175:180, :] = 0.01     # 하한 밖
        clean = estimate(depth)
        result = estimate(noisy)
        self.assertEqual(result.status, 'VALID', result.reasons)
        self.assertLess(result.quality['valid_point_count'], clean.quality['valid_point_count'])
        self.assertLess(result.quality['valid_fraction'], clean.quality['valid_fraction'])

        # 유효 비율이 설정 하한 미만이면 표본 부족이다.
        sparse = depth.copy()
        mask = np.random.default_rng(1).random(sparse.shape) < 0.9
        sparse[mask] = np.nan
        self.assertIn('INSUFFICIENT_POINTS', estimate(sparse).reasons)


class SegmentationRejection(unittest.TestCase):
    def test_far_background_plane_is_not_target(self):
        # ROI에 뒤쪽 벽(1.6 m)과 물체 면이 비슷하게 섞이면 어느 평면이 목표인지 근거가 없어 거부한다.
        # 가까운 면을 목표로 고르지 않는다.
        depth = draw_rectangle(blank(), (0.0, 0.0, 1.6), (0, 0, -1), (1, 0, 0), 3.0, 3.0)
        depth = draw_rectangle(depth, (-0.04, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.20, 0.20)
        result = estimate(depth)
        self.assertEqual(result.status, 'REJECTED')
        self.assertIn('MULTIPLE_SURFACES', result.reasons)

    def test_known_limit_small_target_in_background_roi(self):
        # 알려진 한계: 물체 면이 설정 비율보다 작고 ROI 대부분이 배경 벽이면 배경 평면을 VALID로 낸다.
        # 라이브러리는 어느 면이 등록 물체인지 판정하지 않는다. 남은 점 비율을 품질로 남겨 호출자가 볼 수 있게 한다.
        depth = draw_rectangle(blank(), (0.0, 0.0, 1.6), (0, 0, -1), (1, 0, 0), 3.0, 3.0)
        depth = draw_rectangle(depth, (-0.06, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.10, 0.20)
        result = estimate(depth)
        self.assertEqual(result.status, 'VALID')
        self.assertAlmostEqual(result.center[2], 1.6, delta=0.01)
        self.assertGreater(result.quality['second_plane_fraction'], 0.1)
        self.assertLess(result.quality['inlier_fraction'], 0.9)

    def test_box_top_and_front_face_together(self):
        # 윗면(비스듬)과 정면이 함께 보이면 복수 평면으로 거부한다.
        tilt = math.radians(60)
        depth = draw_rectangle(blank(), (0.0, 0.05, 1.0), (0, 0, -1), (1, 0, 0), 0.30, 0.10)
        depth = draw_rectangle(depth, (0.0, -0.05, 1.05), (0, -math.sin(tilt), -math.cos(tilt)), (1, 0, 0),
                               0.30, 0.12)
        result = estimate(depth)
        self.assertEqual(result.status, 'REJECTED')
        self.assertIn('MULTIPLE_SURFACES', result.reasons)

    def test_scattered_clutter_is_background_mixed(self):
        depth = front_plane()
        rng = np.random.default_rng(3)
        region = depth[150:330, 200:440]
        mask = np.isfinite(region) & (rng.random(region.shape) < 0.3)
        region[mask] = rng.uniform(1.2, 3.0, size=int(mask.sum())).astype(np.float32)
        result = estimate(depth)
        self.assertEqual(result.status, 'REJECTED')
        self.assertIn('BACKGROUND_MIXED', result.reasons)

    def test_separated_patches_are_fragmented(self):
        depth = draw_rectangle(blank(), (-0.08, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.08, 0.15)
        depth = draw_rectangle(depth, (0.08, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.08, 0.15)
        result = estimate(depth, tight_roi(depth))
        self.assertEqual(result.status, 'REJECTED')
        self.assertIn('SURFACE_FRAGMENTED', result.reasons)

    def test_degenerate_strip_and_grazing_view(self):
        strip = draw_rectangle(blank(), (0.0, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.30, 0.012)
        self.assertIn('DEGENERATE_SURFACE', estimate(strip, tight_roi(strip, margin=0)).reasons)

        tilt = math.radians(80)
        grazing = draw_rectangle(blank(), (0.0, 0.0, 1.0), (0, -math.sin(tilt), -math.cos(tilt)), (1, 0, 0),
                                 0.30, 0.60)
        result = estimate(grazing, tight_roi(grazing))
        self.assertEqual(result.status, 'REJECTED')
        self.assertIn('NORMAL_GRAZING', result.reasons)

    def test_settings_have_no_silent_defaults(self):
        with self.assertRaises(TypeError):
            se.EstimationSettings()

        with self.assertRaisesRegex(ValueError, 'SETTINGS_INVALID'):
            estimate(front_plane(), settings=replace(TEST_SETTINGS, min_inlier_fraction=1.5))


# 대응 판정 fixture(합성 시험 전용). 실제 C4 판정이 아니며 정확성을 검증하지 않는다
def fixture_association(observation, **changes):
    values = dict(
        observation_id=observation.observation_id, robot_id=observation.robot_id, object_id='obj-test-0001',
        observation_stamp_ns=observation.stamp_ns, observation_frame_id=observation.observed_frame_id,
        judged_by='test_fixture', method='test_fixture', evidence_ref='test_fixture', result='MATCH',
    )
    values.update(changes)

    return adapter.TargetAssociation(**values)


# 어댑터 외부 값 fixture(출처는 시험 표시이며 실제 근거 아님). observation을 주면 그 관측에 결합한 판정을 넣는다
def fixture_inputs(observation=None, **changes):
    values = dict(
        target_association=fixture_association(observation) if observation is not None else None,
        object_id=adapter.Sourced('obj-test-0001', 'test_fixture'),
        scene_revision=adapter.Sourced(1, 'test_fixture'),
        mass_kg=adapter.Sourced(0.5, 'test_fixture'),
        confidence=adapter.Sourced(0.9, 'test_fixture'),
        suction_suitable=adapter.Sourced(True, 'test_fixture'),
        flatness_definition=adapter.Sourced('plane_p95_m', 'test_fixture'),
        normal_outward=adapter.Sourced(True, 'test_fixture'),
        accepted_frames=adapter.Sourced(['robot_1/base_link'], 'test_fixture'),
    )
    values.update(changes)

    return adapter.AdapterInputs(**values)


class C3Adapter(unittest.TestCase):
    def setUp(self):
        observation = estimate(front_plane())
        matrix = np.eye(4)
        self.observation = se.transform_observation(observation, matrix, 'robot_1/base_link', FRAME, STAMP)

    def test_all_undefined_holds_without_defaults(self):
        result = to_surface(self.observation, adapter.AdapterInputs())
        self.assertIsNone(result.surface)

        for name in ('OBJECT_ID', 'SCENE_REVISION', 'MASS_KG', 'CONFIDENCE', 'SUCTION_SUITABLE',
                     'FLATNESS_DEFINITION', 'NORMAL_OUTWARD', 'ACCEPTED_FRAMES'):
            self.assertIn(f'{name}_UNDEFINED', result.reasons)

        self.assertIn('TARGET_ASSOCIATION_UNDEFINED', result.reasons)

    def test_each_missing_field_alone_holds(self):
        for name in ('object_id', 'scene_revision', 'mass_kg', 'confidence', 'suction_suitable',
                     'flatness_definition', 'normal_outward', 'accepted_frames', 'target_association'):
            if name == 'object_id':
                # object_id가 없으면 대응 판정의 물체 대조도 함께 실패한다(판정이 미정 물체를 대신하지 않음).
                result = to_surface(self.observation, fixture_inputs(self.observation, object_id=None))
                self.assertEqual(result.reasons, ('OBJECT_ID_UNDEFINED', 'TARGET_ASSOCIATION_OBJECT_MISMATCH'))
                continue

            result = to_surface(self.observation, fixture_inputs(self.observation, **{name: None}))
            self.assertIsNone(result.surface, name)
            self.assertEqual(result.reasons, (f'{name.upper()}_UNDEFINED',))

    def test_invalid_and_unsourced_values(self):
        cases = {
            'MASS_KG_INVALID': dict(mass_kg=adapter.Sourced(-1.0, 'test_fixture')),
            'CONFIDENCE_INVALID': dict(confidence=adapter.Sourced(1.5, 'test_fixture')),
            'SCENE_REVISION_INVALID': dict(scene_revision=adapter.Sourced(True, 'test_fixture')),
            'FLATNESS_DEFINITION_INVALID': dict(flatness_definition=adapter.Sourced('max', 'test_fixture')),
            'SUCTION_SUITABLE_INVALID': dict(suction_suitable=adapter.Sourced(1, 'test_fixture')),
            'CONFIDENCE_SOURCE_MISSING': dict(confidence=adapter.Sourced(0.9, ' ')),
            'OBJECT_ID_IS_OBSERVATION_ID': dict(object_id=adapter.Sourced(self.observation.observation_id, 'x')),
            'NORMAL_OUTWARD_NOT_CONFIRMED': dict(normal_outward=adapter.Sourced(False, 'test_fixture')),
            'FRAME_NOT_ACCEPTED': dict(accepted_frames=adapter.Sourced(['map'], 'test_fixture')),
        }

        for reason, change in cases.items():
            result = to_surface(self.observation, fixture_inputs(self.observation, **change))
            self.assertIsNone(result.surface, reason)
            self.assertIn(reason, result.reasons)

    def test_quality_metric_is_not_used_as_confidence(self):
        # confidence가 없으면 plane RMS 등 품질 지표로 대신하지 않고 보류한다.
        result = to_surface(self.observation, fixture_inputs(self.observation, confidence=None))
        self.assertEqual(result.reasons, ('CONFIDENCE_UNDEFINED',))
        self.assertNotIn('confidence', result.provenance)

    def test_rejected_observation_and_axis_ambiguity(self):
        result = to_surface(estimate(blank()), fixture_inputs())
        self.assertEqual(result.reasons[:2], ('OBSERVATION_REJECTED', 'INSUFFICIENT_POINTS'))

        depth = draw_rectangle(blank(), (0.0, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.20, 0.20)
        square = se.transform_observation(estimate(depth, tight_roi(depth)), np.eye(4), 'robot_1/base_link',
                                          FRAME, STAMP)
        self.assertTrue(square.axis_ambiguous)
        self.assertEqual(to_surface(square, fixture_inputs(square)).reasons,
                         ('ALLOW_AXIS_AMBIGUOUS_UNDEFINED',))
        denied = fixture_inputs(square, allow_axis_ambiguous=adapter.Sourced(False, 'test_fixture'))
        self.assertEqual(to_surface(square, denied).reasons, ('AXIS_AMBIGUOUS',))

    def test_complete_fixture_matches_c3_surface_checks(self):
        result = to_surface(self.observation, fixture_inputs(self.observation))
        self.assertEqual(result.reasons, ())
        self.assertIn('미검증', result.provenance['target_association'])
        surface = result.surface
        self.assertIsInstance(surface, Surface)
        self.assertEqual(surface.object_id, 'obj-test-0001')
        self.assertEqual(surface.frame_id, 'robot_1/base_link')
        self.assertEqual(surface.flatness_error_m, self.observation.quality['plane_p95_m'])
        self.assertIn('plane_p95_m', result.provenance['flatness_error_m'])
        self.assertEqual(result.provenance['confidence'], 'test_fixture')
        self.assertLessEqual(abs(np.dot(surface.normal, surface.tangent)), 1e-5)

        # C3 실제 검사: 관측·공구 조건은 통과하고 검증기가 없으므로 후보 성공을 만들지 않는다.
        plan = plan_candidates(surface, Constraints(1, 0.17, 0.13, 2.0), STAMP / NS + 0.1)
        self.assertFalse(plan.valid)
        self.assertEqual(plan.reason, 'VALIDATORS_UNAVAILABLE')

    def test_negative_suction_is_passed_through_not_flipped(self):
        result = to_surface(self.observation, fixture_inputs(
            self.observation, suction_suitable=adapter.Sourced(False, 'test_fixture')))
        self.assertIs(result.surface.suction_suitable, False)
        plan = plan_candidates(result.surface, Constraints(1, 0.17, 0.13, 2.0), STAMP / NS + 0.1)
        self.assertEqual(plan.reason, 'TOOL_CONDITIONS_NOT_MET')


# 배경 벽이 ROI 대부분인 깊이(물체 면 0.10×0.20 m, 설정 비율 미만)
def background_dominated():
    depth = draw_rectangle(blank(), (0.0, 0.0, 1.6), (0, 0, -1), (1, 0, 0), 3.0, 3.0)

    return draw_rectangle(depth, (-0.06, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.10, 0.20)


class TargetAssociationBoundary(unittest.TestCase):
    def setUp(self):
        self.observation = se.transform_observation(estimate(front_plane()), np.eye(4), 'robot_1/base_link',
                                                    FRAME, STAMP)

    def test_validity_scope_is_plane_fit_and_survives_transform(self):
        raw = estimate(front_plane())
        self.assertEqual(raw.validity_scope, 'plane_fit')
        self.assertEqual(raw.observed_frame_id, FRAME)
        self.assertEqual(self.observation.validity_scope, 'plane_fit')
        self.assertEqual(self.observation.observed_frame_id, FRAME)
        self.assertEqual(self.observation.observation_id, raw.observation_id)
        self.assertIsNone(se.lineage_problem(self.observation))
        self.assertIn('plane_fit', se.__doc__)
        self.assertIn('동일성', se.__doc__)

    def test_background_plane_with_metadata_but_no_association_is_held(self):
        # 실제 한계 사례(배경 벽 VALID)에 메타정보를 모두 채워도 대응 판정이 없으면 보류된다.
        wall = se.transform_observation(estimate(background_dominated()), np.eye(4), 'robot_1/base_link',
                                        FRAME, STAMP)
        self.assertEqual(wall.status, 'VALID')
        self.assertAlmostEqual(wall.center[2], 1.6, delta=0.01)
        result = to_surface(wall, fixture_inputs(target_association=None))
        self.assertIsNone(result.surface)
        self.assertEqual(result.reasons, ('TARGET_ASSOCIATION_UNDEFINED',))

    def test_association_bound_to_other_observation_robot_object_time_or_frame(self):
        wall = se.transform_observation(estimate(background_dominated()), np.eye(4), 'robot_1/base_link',
                                        FRAME, STAMP)
        other = fixture_association(wall)
        cases = {
            'TARGET_ASSOCIATION_OBSERVATION_MISMATCH': fixture_association(self.observation,
                                                                           observation_id=wall.observation_id),
            'TARGET_ASSOCIATION_ROBOT_MISMATCH': fixture_association(self.observation, robot_id='robot_3'),
            'TARGET_ASSOCIATION_OBJECT_MISMATCH': fixture_association(self.observation, object_id='obj-other'),
            'TARGET_ASSOCIATION_STAMP_MISMATCH': fixture_association(self.observation,
                                                                     observation_stamp_ns=STAMP - 1),
            # 변환 뒤 frame(base_link)이 아니라 처음 관측 frame과 대조한다.
            'TARGET_ASSOCIATION_FRAME_MISMATCH': fixture_association(self.observation,
                                                                     observation_frame_id='robot_1/base_link'),
        }

        for reason, association in cases.items():
            result = to_surface(self.observation, fixture_inputs(target_association=association))
            self.assertIsNone(result.surface, reason)
            self.assertIn(reason, result.reasons)

        # 다른 관측(배경 벽)에 대한 판정을 현재 관측에 쓰지 못한다.
        result = to_surface(self.observation, fixture_inputs(target_association=other))
        self.assertIn('TARGET_ASSOCIATION_OBSERVATION_MISMATCH', result.reasons)

    def test_negative_unknown_incomplete_judgments_are_rejected(self):
        cases = {
            'TARGET_ASSOCIATION_NOT_MATCH': dict(result='NO_MATCH'),
            'TARGET_ASSOCIATION_UNCONFIRMED': dict(result='UNKNOWN'),
            'TARGET_ASSOCIATION_INVALID': dict(result='true'),
            'TARGET_ASSOCIATION_INCOMPLETE': dict(evidence_ref=' '),
        }

        for reason, change in cases.items():
            association = fixture_association(self.observation, **change)
            result = to_surface(self.observation, fixture_inputs(target_association=association))
            self.assertIsNone(result.surface, reason)
            self.assertIn(reason, result.reasons)

        # 판정 객체가 아닌 값(예: 사용자 True)은 대응 근거가 아니다.
        result = to_surface(self.observation, fixture_inputs(target_association=True))
        self.assertEqual(result.reasons, ('TARGET_ASSOCIATION_INVALID',))

    def test_broken_lineage_is_held(self):
        broken = replace(self.observation, transforms=(('robot_1/front_rgb_optical_frame', 'robot_1/base_link',
                                                        STAMP),))
        result = to_surface(broken, fixture_inputs(broken))
        self.assertIn('LINEAGE_INVALID', result.reasons)

        relabeled = replace(self.observation, frame_id='map')
        self.assertIn('LINEAGE_INVALID', to_surface(relabeled, fixture_inputs(relabeled)).reasons)

        lost = replace(self.observation, observed_frame_id=None)
        self.assertIn('LINEAGE_INVALID', to_surface(lost, fixture_inputs(lost)).reasons)

    def test_observation_id_binds_depth_content(self):
        # 같은 로봇·frame·stamp·ROI라도 깊이 내용이 다르면 다른 관측 ID다(다른 관측 판정 재사용 방지).
        first = estimate(front_plane())
        second = estimate(background_dominated())
        self.assertNotEqual(first.observation_id, second.observation_id)
        self.assertEqual(first.observation_id, estimate(front_plane()).observation_id)

    def test_only_bound_fixture_passes(self):
        result = to_surface(self.observation, fixture_inputs(self.observation))
        self.assertEqual(result.reasons, ())
        self.assertIsNotNone(result.surface)


# epoch 크기 시각(정수 ns). 1 ns 차이는 float 초로는 구분되지 않는다
EPOCH = 1_794_923_490 * NS + 123_456_789


# 지정 stamp·시각 출처로 정면 평면 관측을 만든다
def observe_at(stamp, origin=se.HEADER, depth=None):
    depth = front_plane() if depth is None else depth

    return se.estimate_surface('robot_1', frame_of(depth, stamp=stamp, origin=origin),
                               intrinsics(stamp=stamp, origin=origin), roi(stamp=stamp, stamp_origin=origin),
                               TEST_SETTINGS)


class IntegerStampIdentity(unittest.TestCase):
    def test_epoch_one_nanosecond_apart_is_distinct(self):
        first, second = observe_at(EPOCH), observe_at(EPOCH + 1)
        # float 초로는 같다(보완 전 결함의 원인).
        self.assertEqual(first.stamp_s, second.stamp_s)
        self.assertEqual((first.status, second.status), ('VALID', 'VALID'))
        self.assertNotEqual(first.observation_id, second.observation_id)
        self.assertEqual((first.stamp_ns, second.stamp_ns), (EPOCH, EPOCH + 1))

        # 깊이·ROI가 1 ns 다르면 정확 일치가 아니다.
        mixed = se.estimate_surface('robot_1', frame_of(front_plane(), stamp=EPOCH), intrinsics(stamp=EPOCH),
                                    roi(stamp=EPOCH + 1), TEST_SETTINGS)
        self.assertIn('STAMP_MISMATCH', mixed.reasons)

    def test_transform_lineage_and_association_use_integer_ns(self):
        observation = observe_at(EPOCH)
        off = se.transform_observation(observation, np.eye(4), 'robot_1/base_link', FRAME, EPOCH + 1)
        self.assertIn('TRANSFORM_STAMP_MISMATCH', off.reasons)
        moved = se.transform_observation(observation, np.eye(4), 'robot_1/base_link', FRAME, EPOCH)
        self.assertEqual(moved.transforms, ((FRAME, 'robot_1/base_link', EPOCH),))
        self.assertIsNone(se.lineage_problem(moved))

        # 계보 stamp가 1 ns 다르거나 float이면 계보 불일치
        for bad in (EPOCH + 1, EPOCH / NS):
            broken = replace(moved, transforms=((FRAME, 'robot_1/base_link', bad),))
            self.assertEqual(se.lineage_problem(broken), 'LINEAGE_INVALID')

        # 대응 판정도 정수 ns로 결합한다.
        result = to_surface(moved, fixture_inputs(target_association=fixture_association(
            moved, observation_stamp_ns=EPOCH + 1)))
        self.assertIn('TARGET_ASSOCIATION_STAMP_MISMATCH', result.reasons)
        result = to_surface(moved, fixture_inputs(target_association=fixture_association(
            moved, observation_stamp_ns=EPOCH / NS)))
        self.assertEqual(result.reasons, ('TARGET_ASSOCIATION_INVALID',))
        self.assertEqual(to_surface(moved, fixture_inputs(moved)).reasons, ())
        self.assertEqual(to_surface(moved, fixture_inputs(moved)).provenance['stamp_ns'], EPOCH)

    def test_invalid_stamp_values_are_refused(self):
        for bad in (100.0, True, -1, None):
            with self.assertRaisesRegex(ValueError, 'STAMP_INVALID'):
                frame_of(front_plane(), stamp=bad)

            with self.assertRaisesRegex(ValueError, 'STAMP_INVALID'):
                roi(stamp=bad)

        with self.assertRaisesRegex(ValueError, 'STAMP_INVALID'):
            frame_of(front_plane(), origin='guess')

        self.assertEqual(se.stamp_ns_from_header(3, 250_000_000), 3_250_000_000)

        for sec, nanosec in ((3, NS), (-1, 0), (3, -1), (True, 0), (3.0, 0)):
            with self.assertRaisesRegex(ValueError, 'STAMP_INVALID'):
                se.stamp_ns_from_header(sec, nanosec)

        # 0 ns는 만들 수 있지만 유효 관측 시각이 아니다.
        self.assertIn('STAMP_INVALID', observe_at(0).reasons)


class LegacyFloatStamp(unittest.TestCase):
    def setUp(self):
        self.stamp = se.legacy_stamp_ns(100.25)
        self.legacy = observe_at(self.stamp, se.LEGACY_FLOAT)

    def test_legacy_is_marked_inexact_and_separate_id_space(self):
        self.assertEqual(self.legacy.status, 'VALID')
        self.assertFalse(self.legacy.stamp_exact)
        self.assertTrue(self.legacy.observation_id.startswith('obsL-'))
        exact = observe_at(self.stamp)
        self.assertTrue(exact.stamp_exact)
        self.assertNotEqual(exact.observation_id, self.legacy.observation_id)

        for bad in (float('nan'), -1.0, True, '1'):
            with self.assertRaisesRegex(ValueError, 'STAMP_INVALID'):
                se.legacy_stamp_ns(bad)

    def test_inexact_survives_transform_and_cannot_be_promoted(self):
        moved = se.transform_observation(self.legacy, np.eye(4), 'robot_1/base_link', FRAME, self.stamp)
        self.assertEqual(moved.status, 'VALID')
        self.assertFalse(moved.stamp_exact)
        self.assertEqual(moved.observation_id, self.legacy.observation_id)

        # 재포장으로 정확 관측이 되지 않는다.
        with self.assertRaisesRegex(ValueError, 'STAMP_ORIGIN_ID_MISMATCH'):
            replace(moved, stamp_origin=se.HEADER)

        # 정확 깊이와 비정확 ROI를 섞지 않는다.
        mixed = se.estimate_surface('robot_1', frame_of(front_plane(), stamp=self.stamp),
                                    intrinsics(stamp=self.stamp), roi(stamp=self.stamp, stamp_origin=se.LEGACY_FLOAT),
                                    TEST_SETTINGS)
        self.assertIn('STAMP_ORIGIN_MISMATCH', mixed.reasons)

    def test_adapter_holds_legacy_even_with_match(self):
        moved = se.transform_observation(self.legacy, np.eye(4), 'robot_1/base_link', FRAME, self.stamp)
        result = to_surface(moved, fixture_inputs(moved))
        self.assertIsNone(result.surface)
        self.assertEqual(result.reasons, ('STAMP_NOT_EXACT', 'TARGET_ASSOCIATION_STAMP_NOT_EXACT'))


class AdapterTextLimits(unittest.TestCase):
    def setUp(self):
        self.observation = se.transform_observation(estimate(front_plane()), np.eye(4), 'robot_1/base_link',
                                                    FRAME, STAMP)

    def test_long_external_texts_are_held(self):
        long = 'x' * (ADAPTER_TEXT_LIMIT + 1)
        cases = {
            'CONFIDENCE_TEXT_LIMIT': dict(confidence=adapter.Sourced(0.9, long)),
            'OBJECT_ID_TEXT_LIMIT': dict(object_id=adapter.Sourced(long, 'test_fixture')),
            'ACCEPTED_FRAMES_INVALID': dict(accepted_frames=adapter.Sourced([long], 'test_fixture')),
        }

        for reason, change in cases.items():
            result = to_surface(self.observation, fixture_inputs(self.observation, **change))
            self.assertIsNone(result.surface, reason)
            self.assertIn(reason, result.reasons)

        association = fixture_association(self.observation, evidence_ref=long)
        result = to_surface(self.observation, fixture_inputs(target_association=association))
        self.assertEqual(result.reasons, ('TARGET_ASSOCIATION_TEXT_LIMIT',))

        # 상한과 같은 길이는 받는다.
        exact = fixture_association(self.observation, evidence_ref='x' * ADAPTER_TEXT_LIMIT)
        self.assertEqual(to_surface(self.observation, fixture_inputs(target_association=exact)).reasons, ())

    def test_limit_setting_is_required(self):
        for bad in (0, -1, 1.5, True, None):
            with self.assertRaisesRegex(ValueError, 'ADAPTER_SETTINGS_INVALID'):
                adapter.to_surface(self.observation, fixture_inputs(self.observation), bad)


if __name__ == '__main__':
    unittest.main()
