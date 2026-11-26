"""A2 suction_tool_node의 AttachState 재발행 타이머가 STEADY 시계인지 확인한다(spin 없음).

재발행 주기는 /clock 정지·시스템 시계 역행과 무관해야 하므로 STEADY_TIME이어야 한다. stamp용 노드 시계는
ROS 시간(use_sim_time이면 sim)을 그대로 쓴다. Gazebo·A3·A4는 쓰지 않는다.
실행(ax_interfaces를 빌드한 install을 source한 셸):
  python3 -m pytest -q -p no:cacheprovider -p no:launch_testing -p no:launch_ros \
      tools/tests/test_a2_timer_clock.py
"""
import importlib.util
from pathlib import Path
import sys
import unittest

import rclpy
from rclpy.clock import ClockType

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[2] / 'src/ax_control/scripts'
ROBOT = 'robot_timertest'
_DOMAIN_STATE = None


# 모듈 전체에 시험용 domain 하나를 잡는다(같은 이름 공구 노드가 있으면 시작하지 않는다)
def setUpModule():
    global _DOMAIN_STATE
    _DOMAIN_STATE = ros_test_domain.enter_module_domain(ros_test_domain.A3_DOMAINS, rf'^/ax/{ROBOT}/')


# 모듈이 끝나면 domain 환경변수를 되돌린다
def tearDownModule():
    ros_test_domain.leave_module_domain(_DOMAIN_STATE)


# suction_tool_node 스크립트를 모듈로 읽는다(실행 경로와 같은 코드)
def load_tool_module():
    spec = importlib.util.spec_from_file_location('suction_tool_node_timer', SCRIPTS / 'suction_tool_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


TOOL = load_tool_module()


class AttachStateTimerClockTest(unittest.TestCase):
    """use_sim_time true/false 모두에서 타이머는 STEADY, stamp 시계는 ROS 시간."""

    # 노드를 만들어 (타이머 시계 종류 목록, 타이머 주기 목록, 노드 시계 종류)를 돌려준다
    def inspect(self, use_sim_time):
        rclpy.init(args=['--ros-args', '-p', f'robot_id:={ROBOT}',
                         '-p', f'use_sim_time:={str(use_sim_time).lower()}'])

        try:
            node = TOOL.SuctionToolNode()
            clocks = [timer.clock.clock_type for timer in node.timers]
            periods = [timer.timer_period_ns for timer in node.timers]
            stamp_clock = node.get_clock().clock_type
            node.destroy_node()
        finally:
            rclpy.shutdown()

        return clocks, periods, stamp_clock

    # 각 시계 설정에서 같은 조건을 확인한다
    def test_attach_state_timer_is_steady(self):
        for use_sim_time in (False, True):
            with self.subTest(use_sim_time=use_sim_time):
                clocks, periods, stamp_clock = self.inspect(use_sim_time)

                # 이 노드의 타이머는 AttachState 재발행 하나뿐이다
                self.assertEqual(clocks, [ClockType.STEADY_TIME])
                self.assertEqual(periods, [int(TOOL.ATTACH_STATE_PERIOD_SECONDS * 1e9)])
                self.assertEqual(stamp_clock, ClockType.ROS_TIME)


if __name__ == '__main__':
    unittest.main()
