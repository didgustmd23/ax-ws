"""내 B3 출력과 상대 B4 배터리 판단을 함께 검사합니다. 실제 센서·정책 승인은 아닙니다."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_team_consumers_offline as consumer
from ax_interfaces.msg import SafetyState
from sensor_msgs.msg import BatteryState


class B3BatteryHandoffTests(unittest.TestCase):
    """상대 B4 코드를 바꾸지 않고 통로·배터리 두 조건이 함께 유지되는지 확인합니다."""

    def battery(self, percentage=0.8):
        """시험 시각의 표준 BatteryState를 만들며 실제 측정값으로 사용하지 않습니다."""
        message = BatteryState(present=True, percentage=percentage,
                               power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        message.header.stamp.sec = 100
        return message

    def test_valid_battery_does_not_override_unapproved_corridor(self):
        """배터리가 정상이어도 내 영역 미승인을 정상 통로로 바꾸면 안 됩니다."""
        fixture = consumer.B3B4ConsumerTests()
        result = fixture.consume(fixture.produce(False), battery=self.battery(), battery_required=True)
        self.assertFalse(result.meta.valid)
        self.assertFalse(result.allow_forward)

    def test_required_missing_stale_or_invalid_battery_blocks_clear_corridor(self):
        """내 통로가 정상이어도 상대가 필요한 배터리 근거를 잃으면 주행 허가가 나지 않습니다."""
        fixture = consumer.B3B4ConsumerTests()
        stale = self.battery()
        stale.header.stamp.sec = 90
        for battery in (None, stale, self.battery(float('nan'))):
            result = fixture.consume(fixture.produce(True), battery=battery, battery_required=True)
            self.assertFalse(result.allow_forward)
            self.assertNotEqual(result.level, SafetyState.CLEAR)

    def test_low_warning_does_not_invent_unapproved_stop_policy(self):
        """기본 LOW 경고 정책을 내 연결 시험에서 임의의 강제 정지로 바꾸지 않습니다."""
        fixture = consumer.B3B4ConsumerTests()
        for percentage in (0.8, 0.1):
            result = fixture.consume(fixture.produce(True), battery=self.battery(percentage),
                                     battery_required=True)
            self.assertTrue(result.meta.valid)
            self.assertEqual(result.level, SafetyState.CLEAR)
