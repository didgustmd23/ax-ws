import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_safety'))
from ax_safety.b4.battery_rules import (CRITICAL, LOW, NORMAL, UNKNOWN,
                                         assess_battery)
from ax_safety.b4.risk_rules import (RiskInputs, STOP_REQUIRED, UNKNOWN as
                                      RISK_UNKNOWN, assess_risk)


def battery(percent=0.8, status=2, stamp=100, present=True):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)),
        percentage=percent, power_supply_status=status, present=present)


class BatteryRulesTests(unittest.TestCase):
    def test_missing_required_state_is_unknown(self):
        result = assess_battery(None, 100, 0.5)
        self.assertEqual((result.valid, result.risk, result.reason_code),
                         (False, UNKNOWN, 'SENSOR_STALE'))

    def test_invalid_nan_and_out_of_range_soc_are_blocked(self):
        self.assertFalse(assess_battery(battery(float('nan')), 100, .5).valid)
        self.assertFalse(assess_battery(battery(1.1), 100, .5).valid)
        self.assertFalse(assess_battery(battery(present=False), 100, .5).valid)

    def test_stale_and_future_samples_are_blocked(self):
        self.assertFalse(assess_battery(battery(stamp=99), 100, .5).fresh)
        self.assertFalse(assess_battery(battery(stamp=101), 100, .5).fresh)

    def test_robot_states_are_independent_inputs(self):
        one = assess_battery(battery(.08), 100, .5, low_percent=20,
                             critical_percent=10)
        two = assess_battery(battery(.50), 100, .5, low_percent=20,
                             critical_percent=10)
        self.assertEqual(one.risk, CRITICAL)
        self.assertEqual(two.risk, NORMAL)

    def test_low_boundary_is_22_percent(self):
        at_boundary = assess_battery(battery(.22), 100, .5)
        above_boundary = assess_battery(battery(.2201), 100, .5)
        self.assertEqual(at_boundary.risk, LOW)
        self.assertEqual(above_boundary.risk, NORMAL)
        low = assess_battery(battery(.18), 100, .5, low_percent=20,
                             critical_percent=10)
        self.assertEqual(low.risk, LOW)

    def test_charging_and_full_status_are_valid(self):
        charging = assess_battery(battery(.8, status=1), 100, .5)
        full = assess_battery(battery(1.0, status=4), 100, .5)
        self.assertEqual(charging.risk, NORMAL)
        self.assertEqual(full.risk, NORMAL)

    def test_invalid_required_battery_reaches_unknown_risk_decision(self):
        result = assess_risk(RiskInputs(
            front_sensor_valid=True, rear_sensor_valid=True,
            sensor_fresh=True, localization_valid=True,
            localization_fresh=True, corridor_valid=True,
            obstacle_distance_m=3.0, rear_distance_m=3.0,
            battery_required=True, battery_valid=False,
            battery_reason_code='SENSOR_STALE'))
        self.assertEqual(result.level, RISK_UNKNOWN)
        self.assertFalse(result.allow_forward)

    def test_explicit_battery_action_is_applied_without_defaulting_policy(self):
        result = assess_risk(RiskInputs(
            front_sensor_valid=True, rear_sensor_valid=True,
            sensor_fresh=True, localization_valid=True,
            localization_fresh=True, corridor_valid=True,
            obstacle_distance_m=3.0, rear_distance_m=3.0,
            battery_required=True, battery_valid=True,
            battery_risk=CRITICAL, battery_reason_code='BATTERY_CRITICAL',
            battery_action_level=STOP_REQUIRED))
        self.assertEqual(result.level, STOP_REQUIRED)
        self.assertFalse(result.allow_forward or result.allow_backward)


if __name__ == '__main__':
    unittest.main()
