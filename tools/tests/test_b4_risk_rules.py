"""python3 -m unittest discover -s tools/tests -p 'test_b4_risk_rules.py' -v"""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'/'ax_safety'))
from ax_safety.b4.risk_rules import RiskInputs, assess_risk, CLEAR, STOP_REQUIRED, UNKNOWN, SLOWDOWN_REQUIRED



# 순수 B4 규칙 엔진만 검사하며 ROS/Gazebo 센서나 A4 노드는 실행하지 않는다.
class TestB4(unittest.TestCase):
    def data(self,**changes):
        # 정상 운용 입력을 기본으로 만든 뒤 각 시험에서 한 조건만 변경한다.
        kwargs=dict(front_sensor_valid=True,rear_sensor_valid=True,sensor_fresh=True,
                    localization_valid=True,localization_fresh=True,corridor_valid=True,
                    obstacle_distance_m=3.0,rear_distance_m=3.0)
        kwargs.update(changes)
        return RiskInputs(**kwargs)


    def test_clear(self):
        # 필수 입력이 모두 최신이면 세 방향과 원래 속도 상한을 허용한다.
        r=assess_risk(self.data())
        self.assertEqual(r.level,CLEAR)
        self.assertTrue(r.allow_forward and r.allow_backward and r.allow_rotate)


    def test_stop_near(self):
        # 정지거리 안의 장애물은 감속이 아니라 즉시 정지 판정이어야 한다.
        self.assertEqual(assess_risk(self.data(obstacle_distance_m=0.4)).level,STOP_REQUIRED)


    def test_slowdown(self):
        # 감속거리 안에서는 주행을 제한하고 선속도 상한을 낮춘다.
        r=assess_risk(self.data(obstacle_distance_m=0.8))
        self.assertEqual(r.level,SLOWDOWN_REQUIRED)
        self.assertLessEqual(r.max_linear_speed,0.2)


    def test_no_front(self):
        # 전방 거리를 알 수 없으면 안전한 거리 비교가 불가능하므로 UNKNOWN이다.
        self.assertEqual(assess_risk(self.data(front_sensor_valid=False)).level,UNKNOWN)


    def test_sensor_stale(self):
        # 값이 있어도 최신성이 없으면 현재 장애물 상태를 보장하지 못한다.
        self.assertEqual(assess_risk(self.data(sensor_fresh=False)).level,UNKNOWN)


    def test_localization_lost(self):
        # 위치 기준이 없으면 센서 거리와 통로 관측을 안전 판정에 사용할 수 없다.
        self.assertEqual(assess_risk(self.data(localization_valid=False)).level,UNKNOWN)


    def test_corridor_stale(self):
        # B3 통로 관측이 오래되면 통로 점유·장애물 상태를 신뢰하지 않는다.
        self.assertEqual(assess_risk(self.data(corridor_valid=False)).level,UNKNOWN)


    def test_nan(self):
        # NaN 거리는 비교 결과를 오염시키므로 입력 오류로 차단한다.
        self.assertEqual(assess_risk(self.data(obstacle_distance_m=float('nan'))).level,UNKNOWN)


    def test_rear_missing(self):
        # 후방 센서가 없으면 감속 상태를 유지하고 후진은 허용하지 않는다.
        r=assess_risk(self.data(rear_sensor_valid=False,rear_distance_m=None))
        self.assertFalse(r.allow_backward)
        self.assertEqual(r.level,SLOWDOWN_REQUIRED)


    def test_emergency(self):
        # 비상 정지는 다른 거리·신선도 조건보다 먼저 정지 판정을 낸다.
        self.assertEqual(assess_risk(self.data(emergency_stop=True)).level,STOP_REQUIRED)


    def test_invalid_threshold(self):
        # stop_distance가 slowdown_distance보다 크면 임계값 계약이 깨진 상태다.
        self.assertEqual(assess_risk(self.data(stop_distance_m=2.0,slowdown_distance_m=1.0)).level,UNKNOWN)

if __name__=='__main__': unittest.main()
