"""Check measurement failures stay visible and frozen pose geometry stays faithful."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_geometry import ArmGeometry, origin
from check_w2_arm_visibility import freeze_pose
from check_w2_calibration import finite_result
from check_w2_sensor_quality import nearest_skew_s, stream_metrics


class W2Measurements(unittest.TestCase):
    def test_invalid_sample_is_retained_as_failure_in_strict_json(self):
        measured = {'passed': False, 'samples': [{'passed': False, 'depth': float('nan'),
                                                'point': [1., float('inf'), -float('inf')]}]}
        saved = json.loads(json.dumps(finite_result(measured), allow_nan=False))
        self.assertFalse(saved['passed'])
        self.assertFalse(saved['samples'][0]['passed'])
        self.assertIsNone(saved['samples'][0]['depth'])
        self.assertEqual(saved['samples'][0]['point'], [1., None, None])

    def test_stream_report_exposes_reordering_and_duplicates(self):
        row = stream_metrics([0, 100000000, 100000000, 50000000, 200000000], 10.)
        self.assertEqual(row['duplicates'], 1)
        self.assertEqual(row['reversed'], 1)
        self.assertAlmostEqual(row['max_gap_sim_s'], .1)
        self.assertIsNone(stream_metrics([], 10.)['sim_rate_hz'])

    def test_synchronization_keeps_missing_and_boundary_measurements_visible(self):
        self.assertEqual(nearest_skew_s([0, 100000000], [20000000, 140000000]), [.02, .04])
        self.assertEqual(nearest_skew_s([0], []), [])

    def test_frozen_model_preserves_configured_joint_transforms(self):
        geometry = ArmGeometry()
        for positions in ([0.] * 5, [.35, -.30, .35, -.20, .25], [-.35, .30, -.35, .20, -.25]):
            expected = geometry.frames(positions)
            frozen = freeze_pose(deepcopy(geometry.robot), positions)
            frames, pending = {'base_link': np.eye(4)}, list(frozen.findall('joint'))
            while pending:
                progressed = False
                for joint in pending[:]:
                    parent = joint.find('parent').get('link')
                    if parent in frames:
                        self.assertEqual(joint.get('type'), 'fixed')
                        frames[joint.find('child').get('link')] = frames[parent] @ origin(joint.find('origin'))
                        pending.remove(joint)
                        progressed = True
                self.assertTrue(progressed, 'Disconnected frozen model')
            for link, matrix in expected.items():
                np.testing.assert_allclose(frames[link], matrix, atol=1e-10)


if __name__ == '__main__':
    unittest.main()
