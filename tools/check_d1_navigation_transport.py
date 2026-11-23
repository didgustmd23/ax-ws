#!/usr/bin/env python3
"""Isolated fake Nav2/B2/scan check. No velocity publishers or actual driving."""
import json
from pathlib import Path
import sys
import tempfile
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from ax_interfaces.msg import ActuationState, LocalizationHealth
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/ax_navigation/scripts'))
from navigation_client import NavigationClient


def main():
    rclpy.init()
    server_node = rclpy.create_node('d1_fake_nav2')
    client_node = rclpy.create_node('d1_transport_check')
    received = []
    async def execute(handle):
        received.append(bytes(handle.goal_id.uuid).hex())
        if handle.request.pose.pose.position.x == 2.:
            while not handle.is_cancel_requested:
                wake = Future()
                timer = server_node.create_timer(.02, lambda: wake.set_result(True)
                    if not wake.done() else None)
                await wake
                server_node.destroy_timer(timer)
            handle.canceled()
            return NavigateToPose.Result()
        handle.succeed()
        return NavigateToPose.Result()
    server = ActionServer(server_node, NavigateToPose, '/ax/robot_1/navigate_to_pose',
        execute_callback=execute,
        callback_group=ReentrantCallbackGroup(),
        cancel_callback=lambda request: CancelResponse.ACCEPT,
        goal_callback=lambda goal: GoalResponse.REJECT if goal.pose.pose.position.x < 0
        else GoalResponse.ACCEPT)
    executor = SingleThreadedExecutor()
    executor.add_node(server_node)
    executor.add_node(client_node)
    publishers = {
        'actuation': server_node.create_publisher(ActuationState,
            '/ax/robot_1/safety/actuation_state', QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        'localization': server_node.create_publisher(LocalizationHealth,
            '/ax/robot_1/localization/health', QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        **{source: server_node.create_publisher(LaserScan,
            '/ax/robot_1/sensors/' + source + '_lidar/scan_filtered', qos_profile_sensor_data)
            for source in ('front', 'rear')}}
    health_state = [LocalizationHealth.OK]
    enabled = dict(front=True, rear=True, localization=True)
    def publish():
        stamp = server_node.get_clock().now().to_msg()
        health = LocalizationHealth()
        health.robot_id, health.state, health.meta.valid = 'robot_1', health_state[0], True
        health.meta.header.stamp = stamp
        if enabled['localization']:
            publishers['localization'].publish(health)
        for source in ('front', 'rear'):
            if enabled[source]:
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp, 'robot_1/' + source + '_lidar_link'
                scan.range_min, scan.range_max, scan.ranges = .1, 10., [2., 3., float('inf')]
                publishers[source].publish(scan)
    telemetry = server_node.create_timer(.05, publish)
    def until(predicate):
        deadline = time.monotonic() + 15
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('D1 fake Nav2 transport timed out')
            executor.spin_once(timeout_sec=.05)
    try:
        with tempfile.TemporaryDirectory(prefix='d1-transport-') as directory:
            adapter = NavigationClient(client_node, 'robot_1', Path(directory) / 'ledger.sqlite')
            try:
                until(adapter.client.server_is_ready)
                until(lambda: not adapter._input_reason())
                request = dict(execution_id='transport-arrival', assignment_id='fake-assignment',
                    assignment_generation=1, x=1., y=0., yaw=0.)
                stop = ActuationState(robot_id='robot_1', state=ActuationState.STOPPED)
                stop.meta.source_id = 'a4_safety_gate'
                stop.meta.source_session_id = 'transport-test'
                stop.meta.schema_version, stop.meta.revision, stop.meta.valid = 2, 1, True
                stop.meta.header.stamp = server_node.get_clock().now().to_msg()
                publishers['actuation'].publish(stop)
                until(lambda: adapter.actuation is not None)
                row = adapter.submit(**request)
                until(lambda: adapter.ledger.get(request['execution_id'])['state'] == 'ARRIVED')
                assert not adapter.stopped_after_arrival(request['execution_id'])
                stop.meta.revision += 1
                stop.meta.header.stamp = server_node.get_clock().now().to_msg()
                publishers['actuation'].publish(stop)
                until(lambda: adapter.stopped_after_arrival(request['execution_id']))
                assert not adapter.stopped_after_arrival('other-execution')
                until(lambda: not adapter.stopped_after_arrival(request['execution_id']))
                stop.meta.source_session_id = 'transport-restarted'
                stop.meta.revision = 1
                stop.meta.header.stamp = server_node.get_clock().now().to_msg()
                publishers['actuation'].publish(stop)
                until(lambda: adapter.stopped_after_arrival(request['execution_id']))
                stop.meta.source_session_id = 'transport-test'
                stop.meta.revision = 3
                stop.meta.header.stamp = server_node.get_clock().now().to_msg()
                publishers['actuation'].publish(stop)
                deadline = time.monotonic() + .1
                until(lambda: time.monotonic() >= deadline)
                assert adapter.actuation_cursor[0] == 'transport-restarted'
                stop.meta.source_session_id = 'transport-restarted'
                stop.meta.schema_version = 1
                stop.meta.header.stamp = server_node.get_clock().now().to_msg()
                publishers['actuation'].publish(stop)
                until(lambda: adapter.actuation is None)
                assert not adapter.stopped_after_arrival(request['execution_id'])
                stop.meta.schema_version = 2
                stop.meta.revision = 2
                stop.state = ActuationState.RUNNING
                stop.meta.header.stamp = server_node.get_clock().now().to_msg()
                publishers['actuation'].publish(stop)
                until(lambda: adapter.actuation is not None
                      and adapter.actuation[0].state == ActuationState.RUNNING)
                stop.state = ActuationState.STOPPED
                stop.meta.header.stamp = server_node.get_clock().now().to_msg()
                publishers['actuation'].publish(stop)
                until(lambda: adapter.actuation is None)
                assert not adapter.stopped_after_arrival(request['execution_id'])
                assert received == [row['goal_uuid']], (received, row)
                adapter.submit(**request)
                assert received == [row['goal_uuid']], 'Duplicate execution sent'
                rejected = request | dict(execution_id='transport-reject', x=-1.)
                adapter.submit(**rejected)
                until(lambda: adapter.ledger.get(rejected['execution_id'])['state'] == 'REJECTED')
                assert len(received) == 1
                moving = request | dict(execution_id='transport-lost', x=2.)
                adapter.submit(**moving)
                until(lambda: adapter.ledger.get(moving['execution_id'])['state'] == 'ACTIVE')
                health_state[0] = LocalizationHealth.LOST
                until(lambda: adapter.ledger.get(moving['execution_id'])['state'] == 'CANCELED')
                assert adapter.ledger.held('robot_1')['reason'] == 'LOCALIZATION_INVALID'
                health_state[0] = LocalizationHealth.OK
                until(lambda: not adapter._input_reason())
                try:
                    adapter.submit(**(moving | dict(execution_id='must-not-resume')))
                    raise AssertionError('Input recovery resumed a held robot')
                except RuntimeError:
                    pass
                expiry_results = []
                for source in ('front', 'rear', 'localization'):
                    adapter.close()
                    adapter = NavigationClient(client_node, 'robot_1',
                        Path(directory) / (source + '.sqlite'))
                    until(adapter.client.server_is_ready)
                    until(lambda: not adapter._input_reason())
                    expiring = moving | dict(execution_id='expire-' + source)
                    adapter.submit(**expiring)
                    until(lambda: adapter.ledger.get(expiring['execution_id'])['state'] == 'ACTIVE')
                    enabled[source] = False
                    until(lambda: adapter.ledger.get(expiring['execution_id'])['state'] == 'CANCELED')
                    assert adapter.ledger.held('robot_1')['reason'] == source.upper() + '_STALE'
                    enabled[source] = True
                    until(lambda: not adapter._input_reason())
                    expiry_results.append(source)
                print(json.dumps(dict(fake_nav2=True, uuid_match=True, arrival=True,
                    post_arrival_stop_and_expiry=True, retired_and_invalid_stop_rejected=True,
                    conflicting_stop_revision_rejected=True,
                    duplicate_not_sent=True, rejection=True, lost_cancels=True,
                    expiry_cancels=expiry_results, recovery_stays_held=True,
                    actual_driving_tested=False)))
            finally:
                adapter.close()
    finally:
        server.destroy()
        server_node.destroy_timer(telemetry)
        executor.shutdown()
        server_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
