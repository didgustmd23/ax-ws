"""D2 개발 노드의 재발행·폴링 타이머가 STEADY 시계인지 확인한다(격리 install 필요, spin 없음).

실행(격리 install을 source한 셸, 사용자 세션과 다른 ROS_DOMAIN_ID):
  python3 -m pytest -q -p no:cacheprovider tools/tests/test_d2_timer_clock_ros.py
"""
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.clock import ClockType  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / 'src' / 'ax_fleet_manager' / 'config' / 'dev_fixture_transfer.json'


# 노드를 만들고 타이머 시계 종류·stamp 시계 종류를 돌려준다(spin하지 않고 바로 정리)
def timer_clocks(factory, params, use_sim_time=False):
    args = ['--ros-args'] + [x for k, v in params.items() for x in ('-p', f'{k}:={v}')]
    args += ['-p', f'use_sim_time:={str(use_sim_time).lower()}']
    rclpy.init(args=args)

    try:
        node = factory()
        clocks = [timer.clock.clock_type for timer in node.timers]
        stamp_clock = node.get_clock().clock_type
        node.destroy_node()
    finally:
        rclpy.shutdown()

    return clocks, stamp_clock


# 출력 노드: 1 Hz 재발행·원장 폴링 타이머 모두 STEADY, stamp 시계는 ROS 시간(sim이면 sim)
@pytest.mark.parametrize('use_sim_time', [False, True])
def test_group_node_timers_are_steady(tmp_path, use_sim_time):
    from ax_fleet_manager import group_model as gm
    from ax_fleet_manager.dev_fixtures import make_spec
    from ax_fleet_manager.group_node import GroupTaskDevPublisher
    from ax_fleet_manager.task_store import TaskStore

    store_path = tmp_path / 'd2.sqlite'
    store = TaskStore(str(store_path))
    store.create_task(gm.new_group_state(make_spec()))
    store.close()

    clocks, stamp_clock = timer_clocks(
        GroupTaskDevPublisher,
        {'fixture_file': FIXTURE, 'store_path': store_path}, use_sim_time)

    assert clocks == [ClockType.STEADY_TIME, ClockType.STEADY_TIME]
    assert stamp_clock == ClockType.ROS_TIME


# 입고 노드: 재고 1 Hz 재발행 타이머가 STEADY
@pytest.mark.parametrize('use_sim_time', [False, True])
def test_intake_node_timer_is_steady(tmp_path, use_sim_time):
    from ax_fleet_manager.intake_node import ReceiptIntakeDevNode

    clocks, stamp_clock = timer_clocks(
        ReceiptIntakeDevNode, {'store_path': tmp_path / 'd2.sqlite'}, use_sim_time)

    assert clocks == [ClockType.STEADY_TIME]
    assert stamp_clock == ClockType.ROS_TIME
