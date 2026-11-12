"""B3 영역 파일·전체 외곽·불명 처리, 두 창고 월드 영역의 SDF 대조, 회전 외곽·접근 방향 판정을 검증한다."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_environment'))
from ax_environment.regions import (bounds_polygon, checked_bounds, load_regions, observe_corridor, observe_robot,
                                    outline_polygon, polygons_overlap)
from ax_environment.outline import model_outline
sys.path.insert(0, str(ROOT / 'tools'))
import generate_b3_regions as gen  # noqa: E402

WAREHOUSE = ROOT / 'src/ax_environment/config/ax_warehouse_regions.json'
PLAIN_WORLD = ROOT / 'src/ax_description/worlds/planned/ax_warehouse_plain.world'
DESCRIPTION_CONFIG = ROOT / 'src/ax_description/config'
# 충돌 형상 검사에서 제외할 바닥 모델(영역은 바닥 위에 있으므로 겹치는 것이 정상)
FLOORS = ('floor_source', 'floor_destination', 'floor_transfer_corridor', 'floor_transfer_corridor_1',
          'floor_transfer_corridor_2')


class RegionTests(unittest.TestCase):
    def setUp(self):
        self.test_world = load_regions(ROOT / 'src/ax_environment/config/ax_test_regions.json')
        self.aws = load_regions(ROOT / 'src/ax_environment/config/aws_warehouse_regions.json')

    def test_empty_world_does_not_claim_removed_workstations(self):
        sys.path.insert(0, str(ROOT / 'tools'))
        from empty_test_fixture import empty_world
        world = empty_world().find('world')
        self.assertEqual({m.get('name') for m in world.findall('model')}, {'ground_plane'})
        self.assertFalse(any(r['kind'] == 'workstation' for r in self.test_world['regions']))

    # AWS 영역 파일은 월드가 저장소에 없어 좌표가 비어 있고 미승인이다. 판정은 항상 UNKNOWN이어야 한다(파일은 보존).
    def test_unadopted_aws_cannot_be_used_for_release(self):
        self.assertFalse(self.aws['approved'])
        self.assertTrue(all(r['bounds'] is None for r in self.aws['regions']))
        result = self.observe(self.aws, [20, 20, 21, 21])
        self.assertEqual(result.state, 'UNKNOWN')
        self.assertFalse(result.position_valid)

    def observe(self, catalog, bounds, **changes):
        arguments = dict(frame_id='map', map_revision=1, position_valid=True,
                         outline_complete=True, age_s=0.1)
        arguments.update(changes)
        return observe_corridor(catalog, 'corridor_main', bounds, **arguments)

    def approved_test_catalog(self):
        # 단위 시험 전용 자료. 실제 설정 파일의 미승인 표시를 바꾸지 않는다.
        catalog = deepcopy(self.test_world)
        catalog.update(approved=True, frame_id='map', map_revision=1)
        return catalog

    def test_full_footprint_boundary_stale_and_coordinate_checks(self):
        catalog = self.approved_test_catalog()
        self.assertEqual(self.observe(catalog, [5.5, 0, 6.5, 0.5]).state, 'INSIDE')
        self.assertEqual(self.observe(catalog, [6, 0, 7, 0.5]).state, 'INSIDE')
        self.assertEqual(self.observe(catalog, [6.1, 0, 7, 0.5]).state, 'OUTSIDE')
        self.assertEqual(self.observe(catalog, [6.1, 0, 7, 0.5], uncertainty_m=0.2).state, 'INSIDE')
        for changes in (dict(frame_id='world'), dict(map_revision=2),
                        dict(position_valid=False), dict(outline_complete=False),
                        dict(age_s=2), dict(age_s=-1), dict(age_s=float('nan'))):
            with self.subTest(changes=changes):
                result = self.observe(catalog, [8, 8, 9, 9], **changes)
                self.assertEqual(result.state, 'UNKNOWN')
                self.assertFalse(result.position_valid)

    def test_bad_geometry_and_false_approval_rejected(self):
        for bounds in ([1, 0, 0, 1], [0, 0, float('nan'), 1], [0, 1]):
            with self.assertRaises(ValueError):
                checked_bounds(bounds)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'invalid.json'
            data = deepcopy(self.aws)
            data['approved'] = True
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                load_regions(path)


# 영역 좌표는 소수 6자리로 반올림해 저장하므로 1e-5 m보다 얇은 겹침은 반올림 차로 보고 닿음으로 취급한다.
ROUNDING_M = 1e-5


# 두 사각형의 겹친 넓이(닿거나 반올림 차만큼만 겹치면 0)
def overlap_area(a, b):
    width = min(a[2], b[2]) - max(a[0], b[0])
    height = min(a[3], b[3]) - max(a[1], b[1])
    if width <= ROUNDING_M or height <= ROUNDING_M:
        return 0.0
    return width * height


# 두 사각형 사이 최소 거리(겹치면 0)
def gap(a, b):
    dx = max(b[0] - a[2], a[0] - b[2], 0.0)
    dy = max(b[1] - a[3], a[1] - b[3], 0.0)
    return (dx * dx + dy * dy) ** 0.5


class WarehouseRegionTests(unittest.TestCase):
    """두 창고 월드(planned/ax_warehouse.world) 영역 파일을 SDF 형상과 대조한다."""

    # 영역 파일과 SDF 상자를 읽는다.
    def setUp(self):
        self.catalog = load_regions(WAREHOUSE)
        self.regions = {r['id']: r for r in self.catalog['regions']}
        self.boxes = gen.model_boxes()

    # 종류별 영역 목록
    def kind(self, name):
        return [r for r in self.catalog['regions'] if r['kind'] == name]

    # 파일이 생성기 결과와 같고, 단색 보존 월드에서 만들어도 같다(구조 동일).
    def test_file_matches_generator_and_plain_world(self):
        self.assertEqual(json.loads(WAREHOUSE.read_text(encoding='utf-8')), gen.build())
        self.assertEqual(gen.build(PLAIN_WORLD)['regions'], gen.build()['regions'])

    # 미승인·개발용 표시, map revision 1, 기준 월드 이름
    def test_development_flags(self):
        self.assertFalse(self.catalog['approved'])
        self.assertIn('DEVELOPMENT_ONLY', self.catalog['reason'])
        self.assertEqual((self.catalog['frame_id'], self.catalog['map_revision']), ('map', 1))
        self.assertEqual(self.catalog['world'], 'planned/ax_warehouse.world')

    # world→map 변환이 채택 지도 README의 T_world_map 초기값과 같고, 모든 영역이 그 규칙을 따른다.
    def test_world_to_map_transform(self):
        readme = (ROOT / 'src/ax_localization/maps/ax_warehouse/README.md').read_text(encoding='utf-8')
        self.assertIn('초기 (−6.800, 0, 0)', readme)
        self.assertEqual(self.catalog['transform']['T_world_map'], [-6.8, 0.0, 0.0])
        for region in self.catalog['regions']:
            with self.subTest(region=region['id']):
                expected = [region['world_bounds'][0] + 6.8, region['world_bounds'][1],
                            region['world_bounds'][2] + 6.8, region['world_bounds'][3]]
                for value, target in zip(region['bounds'], expected):
                    self.assertAlmostEqual(value, target, places=5)

    # 통로 경계가 SDF 벽 안쪽 면·문 벽 바깥 면과 같고, 문 벽에 통로 폭만큼 열린 곳이 있다(독립 계산).
    def test_corridors_match_sdf_walls_and_doors(self):
        self.assertEqual({r['id'] for r in self.kind('corridor')}, {'corridor_1', 'corridor_2', 'corridor_3'})
        doors = [name for name in self.boxes if 'door_section' in name]
        for number in (1, 2, 3):
            region = self.regions[f'corridor_{number}']['world_bounds']
            north, south = self.boxes[f'corridor_{number}_wall_north'], self.boxes[f'corridor_{number}_wall_south']
            with self.subTest(corridor=number):
                self.assertAlmostEqual(region[3], north['y'] - north['sy'] / 2, places=5)
                self.assertAlmostEqual(region[1], south['y'] + south['sy'] / 2, places=5)
                self.assertAlmostEqual(region[0], -4.2, places=5)
                self.assertAlmostEqual(region[2], 4.2, places=5)
                self.assertAlmostEqual(region[2] - region[0], north['sx'], places=5)
                # 통로 안에는 문 벽이 없다(문이 열려 있음)
                for name in doors:
                    self.assertEqual(overlap_area(region, gen.box_bounds(self.boxes[name])), 0.0, name)

    # 접근 구역: 통로마다 서·동 2개, 소속 통로·진입 방향, 통로 입구에 붙어 있고 폭이 같다.
    def test_approach_zones_attach_to_corridor_entrances(self):
        approaches = self.kind('approach')
        self.assertEqual(len(approaches), 6)
        for zone in approaches:
            corridor = self.regions[zone['corridor_id']]['bounds']
            bounds = zone['bounds']
            with self.subTest(zone=zone['id']):
                self.assertEqual((bounds[1], bounds[3]), (corridor[1], corridor[3]))
                self.assertAlmostEqual(bounds[2] - bounds[0], self.catalog['approach_depth_m'], places=6)
                if zone['entry_heading_rad'] == 0.0:
                    self.assertAlmostEqual(bounds[2], corridor[0], places=6)
                else:
                    self.assertAlmostEqual(zone['entry_heading_rad'], 3.141593, places=6)
                    self.assertAlmostEqual(bounds[0], corridor[2], places=6)

    # 접근 구역은 통로·다른 접근 구역·작업대·대기 지점과 넓이로 겹치지 않는다(입구 경계에서 닿는 것만 허용).
    def test_approach_zones_do_not_overlap_other_regions(self):
        for zone in self.kind('approach'):
            for other in self.catalog['regions']:
                if other['id'] == zone['id']:
                    continue
                with self.subTest(zone=zone['id'], other=other['id']):
                    self.assertEqual(overlap_area(zone['bounds'], other['bounds']), 0.0)

    # 대기 지점은 접근 구역에서 0.8 m 이상 떨어지고 통로·작업대와 겹치지 않으며, 로봇 외곽(AX_1)이 들어간다.
    def test_waiting_regions_clear_of_approach(self):
        ax1, _ = model_outline('AX_1', DESCRIPTION_CONFIG)
        for wait in self.kind('waiting'):
            bounds = wait['bounds']
            self.assertGreaterEqual(bounds[2] - bounds[0], ax1[2] - ax1[0])
            self.assertGreaterEqual(bounds[3] - bounds[1], ax1[3] - ax1[1])
            for zone in self.kind('approach'):
                self.assertGreaterEqual(gap(bounds, zone['bounds']), 0.8, (wait['id'], zone['id']))
            for other in self.kind('corridor') + self.kind('workstation'):
                self.assertEqual(overlap_area(bounds, other['bounds']), 0.0, (wait['id'], other['id']))

    # 접근 구역·대기 지점·통로 안에 충돌 형상(벽·랙·상자·표본)이 없다. 경계에서 닿는 벽은 허용한다.
    def test_regions_free_of_collision_geometry(self):
        for region in self.kind('approach') + self.kind('waiting') + self.kind('corridor'):
            for name, box in self.boxes.items():
                if not box['collision'] or name in FLOORS:
                    continue
                with self.subTest(region=region['id'], model=name):
                    self.assertEqual(overlap_area(region['world_bounds'], gen.box_bounds(box)), 0.0)

    # 작업대는 SDF 픽업·하역 구역 상자와 같다.
    def test_workstations_match_sdf_zones(self):
        for station_id, model in gen.STATIONS.items():
            self.assertEqual(self.regions[station_id]['world_bounds'], gen.rounded(gen.box_bounds(self.boxes[model])))

    # 승인 사본에서 실제 출발 위치의 로봇은 어느 통로에도 INSIDE·APPROACHING이 아니다.
    def test_spawn_positions_are_outside_every_corridor(self):
        catalog = deepcopy(self.catalog)
        catalog['approved'] = True
        spawns = {'robot_1': ('AX_1', -10.0, -2.4), 'robot_2': ('AX_2', -6.8, 0.0),
                  'robot_3': ('AX_1', -17.0, -2.4), 'robot_4': ('AX_2', -13.0, -2.4)}
        for robot_id, (model, x, y) in spawns.items():
            outline, _ = model_outline(model, DESCRIPTION_CONFIG)
            result = observe_robot(catalog, outline, (x + 6.8, y, 0.0), frame_id='map', map_revision=1,
                                   position_valid=True, outline_complete=True, age_s=0.1, uncertainty_m=0.1)
            with self.subTest(robot=robot_id):
                self.assertEqual({o.state for o in result.values()}, {'OUTSIDE'})

    # 접근 구역의 corridor_id·진입 방향 검사
    def test_approach_validation(self):
        cases = [('corridor_id', 'corridor_9'), ('corridor_id', None), ('entry_heading_rad', 4.0),
                 ('entry_heading_rad', float('nan')), ('entry_heading_rad', True)]
        for key, value in cases:
            data = deepcopy(json.loads(WAREHOUSE.read_text(encoding='utf-8')))
            zone = next(r for r in data['regions'] if r['kind'] == 'approach')
            zone[key] = value
            with self.subTest(key=key, value=value), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'bad.json'
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    load_regions(path)

    # 미승인 파일은 모든 통로 UNKNOWN(REGIONS_NOT_APPROVED)
    def test_unapproved_file_is_unknown(self):
        outline, _ = model_outline('AX_2', DESCRIPTION_CONFIG)
        result = observe_robot(self.catalog, outline, (5.0, 0.0, 0.0), frame_id='map', map_revision=1,
                               position_valid=True, outline_complete=True, age_s=0.1)
        self.assertEqual({(o.state, o.reason, o.position_valid) for o in result.values()},
                         {('UNKNOWN', 'REGIONS_NOT_APPROVED', False)})


class RotatedOutlineTests(unittest.TestCase):
    """회전 외곽·경계·접근 방향·불확실성 판정(승인 사본, map 좌표)."""

    # 승인 사본과 AX_2 외곽
    def setUp(self):
        self.catalog = deepcopy(load_regions(WAREHOUSE))
        self.catalog['approved'] = True
        self.outline, _ = model_outline('AX_2', DESCRIPTION_CONFIG)
        self.corridor = next(r for r in self.catalog['regions'] if r['id'] == 'corridor_2')['bounds']

    # 한 자세의 corridor_2 판정
    def state(self, pose, **changes):
        arguments = dict(frame_id='map', map_revision=1, position_valid=True, outline_complete=True, age_s=0.1)
        arguments.update(changes)
        return observe_robot(self.catalog, self.outline, pose, **arguments)['corridor_2']

    # 135° 회전 외곽: 포위 사각형은 통로 모서리와 겹치지만 실제 다각형은 떨어져 있어 OUTSIDE(축 정렬 근사 과대 판정 방지).
    # 방향 135°는 진입 방향(+X)에서 60° 밖이라 접근 구역과 겹쳐도 APPROACHING이 아니다.
    def test_rotated_outline_not_overinflated(self):
        x0, y0 = self.corridor[0], self.corridor[3]       # 통로 서북쪽 모서리 (2.6, 0.9)
        pose = (x0 - 0.62, y0 + 0.62, 2.356194)
        polygon = outline_polygon(self.outline, pose)
        xs, ys = [p[0] for p in polygon], [p[1] for p in polygon]
        aabb = [min(xs), min(ys), max(xs), max(ys)]
        self.assertGreater(overlap_area(aabb, self.corridor), 0.0)
        self.assertFalse(polygons_overlap(polygon, bounds_polygon(self.corridor)))
        self.assertEqual(self.state(pose).state, 'OUTSIDE')
        # 같은 경우 축 정렬 근사(observe_corridor)는 INSIDE로 과대 판정한다.
        legacy = deepcopy(self.catalog)
        legacy['regions'] = [dict(r, id='corridor_main') if r['id'] == 'corridor_2' else r for r in legacy['regions']]
        old = observe_corridor(legacy, 'corridor_main', aabb, frame_id='map', map_revision=1, position_valid=True,
                               outline_complete=True, age_s=0.1)
        self.assertEqual(old.state, 'INSIDE')

    # 경계: 앞 끝이 통로 서쪽 면에 닿으면 INSIDE, 1 mm 떨어지면 접근 구역 안 APPROACHING
    def test_boundary_touch_is_inside(self):
        front = self.outline[2]
        self.assertEqual(self.state((self.corridor[0] - front, 0.0, 0.0)).state, 'INSIDE')
        self.assertEqual(self.state((self.corridor[0] - front - 0.001, 0.0, 0.0)).state, 'APPROACHING')

    # 접근 방향: 진입 방향(+X)에서 60° 안이면 APPROACHING, 밖이면 OUTSIDE, 방향 불확실성은 허용 각을 넓힌다.
    def test_approach_heading(self):
        x = 1.6                                           # 접근 구역 서쪽(1.1~2.6) 안, 통로와 닿지 않는 위치
        self.assertEqual(self.state((x, 0.0, 0.0)).state, 'APPROACHING')
        self.assertEqual(self.state((x, 0.0, 0.8)).state, 'APPROACHING')
        self.assertEqual(self.state((x, 0.0, 1.2)).state, 'OUTSIDE')
        self.assertEqual(self.state((x, 0.0, 3.14159)).state, 'OUTSIDE')
        self.assertEqual(self.state((x, 0.0, 1.2), yaw_uncertainty_rad=0.2).state, 'APPROACHING')
        # 동쪽 입구는 −X(π)로 진입
        self.assertEqual(self.state((12.0, 0.0, 3.14159)).state, 'APPROACHING')
        self.assertEqual(self.state((12.0, 0.0, 0.0)).state, 'OUTSIDE')

    # 위치 불확실성만큼 넓히면 가까운 OUTSIDE가 INSIDE로 바뀐다(보수적).
    def test_uncertainty_inflates(self):
        rear = -self.outline[0]                            # 통로를 등지면(π) 뒤 끝이 통로 쪽이다
        pose = (self.corridor[0] - rear - 0.05, 0.0, 3.14159)   # 뒤 끝이 통로 서쪽 면에서 5 cm 앞
        self.assertEqual(self.state(pose).state, 'OUTSIDE')
        self.assertEqual(self.state(pose, uncertainty_m=0.06).state, 'INSIDE')
        far = (self.corridor[0] - rear - 0.30, 0.0, 3.14159)
        self.assertEqual(self.state(far, yaw_uncertainty_rad=0.05).state, 'OUTSIDE')

    # 위치 나이·미래 stamp·무효 입력은 UNKNOWN
    def test_invalid_inputs_unknown(self):
        pose = (5.0, 0.0, 0.0)
        self.assertEqual(self.state(pose, age_s=-0.5, future_tolerance_s=1.0).state, 'INSIDE')
        for changes in (dict(age_s=-1.5, future_tolerance_s=1.0), dict(age_s=1.5), dict(position_valid=False),
                        dict(outline_complete=False), dict(map_revision=2), dict(frame_id='world'),
                        dict(uncertainty_m=-0.1), dict(age_s=float('nan'))):
            with self.subTest(changes=changes):
                result = self.state(pose, **changes)
                self.assertEqual(result.state, 'UNKNOWN')
                self.assertFalse(result.position_valid)
        self.assertEqual(self.state((float('nan'), 0.0, 0.0)).state, 'UNKNOWN')

    # 통로 하나만 INSIDE이고 나머지 통로는 OUTSIDE(모든 통로를 판정)
    def test_all_corridors_reported(self):
        result = observe_robot(self.catalog, self.outline, (5.0, 0.0, 0.0), frame_id='map', map_revision=1,
                               position_valid=True, outline_complete=True, age_s=0.1)
        self.assertEqual({k: v.state for k, v in result.items()},
                         {'corridor_1': 'OUTSIDE', 'corridor_2': 'INSIDE', 'corridor_3': 'OUTSIDE'})

    # DEGRADED: 넓힌 외곽이 통로·접근 구역과 떨어지면 OUTSIDE(유효), 겹치면 UNKNOWN. INSIDE·APPROACHING은 확정하지 않는다.
    def test_degraded_mode(self):
        margin = 0.04 + 0.5                                # k·std + degraded_margin_m(노드 기본값 기준)

        def degraded(pose):
            return observe_robot(self.catalog, self.outline, pose, frame_id='map', map_revision=1, position_valid=True,
                                 outline_complete=True, age_s=0.1, uncertainty_m=margin, degraded=True)

        far = degraded((-2.0, 0.0, 0.0))
        self.assertEqual({(o.state, o.position_valid) for o in far.values()}, {('OUTSIDE', True)})
        # 접근 구역 근처(앞 끝 0.67 + 0.54 > 1.1)와 통로 안은 UNKNOWN
        for pose in ((0.0, 0.0, 0.0), (1.6, 0.0, 0.0), (5.0, 0.0, 0.0), (5.0, 0.0, 1.57)):
            with self.subTest(pose=pose):
                result = degraded(pose)
                self.assertEqual((result['corridor_2'].state, result['corridor_2'].position_valid), ('UNKNOWN', False))
                self.assertEqual(result['corridor_2'].reason, 'DEGRADED_NEAR')
                self.assertEqual(result['corridor_1'].state, 'OUTSIDE')
        # 같은 자세의 OK 판정은 이전과 같다(회귀)
        self.assertEqual(self.state((0.0, 0.0, 0.0)).state, 'OUTSIDE')
        self.assertEqual(self.state((1.6, 0.0, 0.0)).state, 'APPROACHING')
        self.assertEqual(self.state((5.0, 0.0, 0.0)).state, 'INSIDE')
        # 공통 판정 불가는 DEGRADED여도 UNKNOWN
        result = observe_robot(self.catalog, self.outline, (-2.0, 0.0, 0.0), frame_id='map', map_revision=1,
                               position_valid=True, outline_complete=True, age_s=5.0, uncertainty_m=margin, degraded=True)
        self.assertEqual({o.state for o in result.values()}, {'UNKNOWN'})

    # 모델 외곽: 생성 자료 합집합, 알 수 없는 모델 거절
    def test_model_outline(self):
        self.assertEqual(model_outline('AX_1', DESCRIPTION_CONFIG)[0], [-0.78, -0.42, 0.82, 0.613364])
        self.assertEqual(model_outline('AX_2', DESCRIPTION_CONFIG)[0], [-0.63, -0.53, 0.67, 0.53])
        self.assertEqual(model_outline('AX_2', DESCRIPTION_CONFIG, 0.05)[0], [-0.68, -0.58, 0.72, 0.58])
        for model in ('', 'robot_2', 'AX_3'):
            with self.assertRaises(ValueError):
                model_outline(model, DESCRIPTION_CONFIG)


if __name__ == '__main__':
    unittest.main()
