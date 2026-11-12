"""python3 -m unittest discover -s tools/tests -p 'test_b4_risk_rules.py' -v"""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'/'ax_safety'))
from ax_safety.b4.risk_rules import RiskInputs, assess_risk, CLEAR, STOP_REQUIRED, UNKNOWN, SLOWDOWN_REQUIRED

class TestB4(unittest.TestCase):
    def data(self,**changes):
        kwargs=dict(front_sensor_valid=True,rear_sensor_valid=True,sensor_fresh=True,
                    localization_valid=True,localization_fresh=True,corridor_valid=True,
                    obstacle_distance_m=3.0,rear_distance_m=3.0)
        kwargs.update(changes)
        return RiskInputs(**kwargs)
    def test_clear(self):
        r=assess_risk(self.data())
        self.assertEqual(r.level,CLEAR)
        self.assertTrue(r.allow_forward and r.allow_backward and r.allow_rotate)
    def test_stop_near(self):
        self.assertEqual(assess_risk(self.data(obstacle_distance_m=0.4)).level,STOP_REQUIRED)
    def test_slowdown(self):
        r=assess_risk(self.data(obstacle_distance_m=0.8))
        self.assertEqual(r.level,SLOWDOWN_REQUIRED)
        self.assertLessEqual(r.max_linear_speed,0.2)
    def test_no_front(self):
        self.assertEqual(assess_risk(self.data(front_sensor_valid=False)).level,UNKNOWN)
    def test_sensor_stale(self):
        self.assertEqual(assess_risk(self.data(sensor_fresh=False)).level,UNKNOWN)
    def test_localization_lost(self):
        self.assertEqual(assess_risk(self.data(localization_valid=False)).level,UNKNOWN)
    def test_corridor_stale(self):
        self.assertEqual(assess_risk(self.data(corridor_valid=False)).level,UNKNOWN)
    def test_nan(self):
        self.assertEqual(assess_risk(self.data(obstacle_distance_m=float('nan'))).level,UNKNOWN)
    def test_rear_missing(self):
        r=assess_risk(self.data(rear_sensor_valid=False,rear_distance_m=None))
        self.assertFalse(r.allow_backward)
        self.assertEqual(r.level,SLOWDOWN_REQUIRED)
    def test_emergency(self):
        self.assertEqual(assess_risk(self.data(emergency_stop=True)).level,STOP_REQUIRED)
    def test_invalid_threshold(self):
        self.assertEqual(assess_risk(self.data(stop_distance_m=2.0,slowdown_distance_m=1.0)).level,UNKNOWN)

if __name__=='__main__': unittest.main()
