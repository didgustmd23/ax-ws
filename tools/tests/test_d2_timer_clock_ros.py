"""D2 개발 노드의 재발행·폴링 타이머가 STEADY 시계인지, 세션 재발급이 sim 시간 역행에서만 일어나는지
확인한다(격리 install 필요, spin 없음).

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


# 노드를 만들고 시계를 직접 조정하며 발행 함수를 불러 세션 재발급 횟수를 센다(spin 없음).
# sim이면 set_ros_time_override로 200 s → 100 s 역행을 만들고, 시스템 시계면 직전 stamp를 10 s 미래로
# 둔다(시험이 시스템 시계 역행을 일으킬 수 없어 같은 판정 조건을 직접 만든다)
def renewals_after_regression(factory, params, publish_name, use_sim_time):
    from rclpy.time import Time

    args = ['--ros-args'] + [x for k, v in params.items() for x in ('-p', f'{k}:={v}')]
    args += ['-p', f'use_sim_time:={str(use_sim_time).lower()}']
    rclpy.init(args=args)

    try:
        node = factory()
        publish = getattr(node, publish_name)
        before = node.session_id

        if use_sim_time:
            node.get_clock().set_ros_time_override(Time(seconds=200))
            publish()
            node.get_clock().set_ros_time_override(Time(seconds=100))
        else:
            publish()
            node.last_stamp_ns = node.get_clock().now().nanoseconds + 10 * 10**9

        publish()
        result = (node.session_renewals, node.session_id != before)
        node.destroy_node()
    finally:
        rclpy.shutdown()

    return result


# 세션 재발급은 sim 시간 역행에서만: sim 1회, 시스템 시계(같은 크기 이상 역행 조건)에서는 0회
@pytest.mark.parametrize('use_sim_time,expected', [(True, (1, True)), (False, (0, False))])
@pytest.mark.parametrize('which', ['group', 'intake'])
def test_session_renewal_only_on_sim_time(tmp_path, which, use_sim_time, expected):
    if which == 'group':
        from ax_fleet_manager.group_node import GroupTaskDevPublisher
        factory, params, name = GroupTaskDevPublisher, {'fixture_file': FIXTURE}, '_publish'
    else:
        from ax_fleet_manager.intake_node import ReceiptIntakeDevNode
        factory, params = ReceiptIntakeDevNode, {'store_path': tmp_path / 'd2.sqlite'}
        name = '_publish_inventory'

    assert renewals_after_regression(factory, params, name, use_sim_time) == expected
