"""B3 영역 파일·전체 외곽·불명 처리와 시험 월드 좌표를 검증한다."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_environment'))
from ax_environment.regions import checked_bounds, load_regions, observe_corridor


class RegionTests(unittest.TestCase):
    def setUp(self):
        self.test_world = load_regions(ROOT / 'src/ax_environment/config/ax_test_regions.json')
        self.aws = load_regions(ROOT / 'src/ax_environment/config/aws_warehouse_regions.json')

    def test_workstations_match_actual_world_not_invented_map_coordinates(self):
        world = ET.parse(ROOT / 'src/ax_description/worlds/ax_test.world').getroot().find('world')
        for region in self.test_world['regions']:
            if region['kind'] != 'workstation':
                continue
            model = world.find(f"model[@name='{region['id']}']")
            x, y, z, *_ = map(float, model.findtext('pose').split())
            width, depth, height = map(float, model.findtext('link/collision/geometry/box/size').split())
            self.assertEqual(region['bounds'], [x-width/2, y-depth/2, x+width/2, y+depth/2])
            self.assertEqual(region['surface_height_m'], z+height/2)

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


if __name__ == '__main__':
    unittest.main()
