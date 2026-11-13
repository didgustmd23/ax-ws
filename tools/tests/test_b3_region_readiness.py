"""승인 전 점검이 원본을 바꾸지 않고 정적 오류와 실제 검증 미완료를 구분하는지 검사합니다."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import check_b3_region_readiness as readiness


class RegionReadinessTests(unittest.TestCase):
    """현재 저장된 모델과 지도 근거에 대한 사전 점검을 검사합니다."""

    def test_static_pass_is_not_approval_and_does_not_change_source(self):
        # 핵심: 계산이 통과해도 approved를 바꾸지 않았음을 원본 바이트까지 비교합니다.
        """정적 계산 통과를 운영 승인으로 잘못 바꾸지 않습니다."""
        path = ROOT / 'src/ax_environment/config/ax_warehouse_regions.json'
        before = path.read_bytes()
        report = readiness.audit()
        self.assertTrue(report['static_passed'])
        self.assertEqual(len(report['boundary_cases']), 72)
        self.assertFalse(report['approved'])
        self.assertFalse(report['ready_for_approval'])
        self.assertTrue(report['pending'])
        self.assertEqual(path.read_bytes(), before)

    def test_manually_shifted_boundary_is_detected(self):
        # 핵심: 개발 경계만 임의로 바꿨을 때 월드 형상과의 불일치를 검출해야 합니다.
        """SDF 근거에서 벗어난 수동 경계 수정을 탐지합니다."""
        original = readiness.load_regions(ROOT / 'src/ax_environment/config/ax_warehouse_regions.json')
        modified = deepcopy(original)
        modified['regions'][0]['bounds'][0] += 0.5
        with patch.object(readiness, 'load_regions', return_value=modified):
            report = readiness.audit()
        self.assertFalse(report['static_passed'])
        self.assertFalse(report['checks'][0]['passed'])

    def test_map_revision_mismatch_is_detected(self):
        """관측 장부와 채택 지도 revision이 다르면 정적 점검을 통과시키지 않습니다."""
        original = readiness.load_regions(ROOT / 'src/ax_environment/config/ax_warehouse_regions.json')
        modified = deepcopy(original)
        modified['map_revision'] += 1
        with patch.object(readiness, 'load_regions', return_value=modified):
            report = readiness.audit()
        revision_check = next(row for row in report['checks'] if row['name'] == 'map_frame_revision')
        self.assertFalse(revision_check['passed'])
        self.assertFalse(report['ready_for_approval'])


if __name__ == '__main__':
    unittest.main()
