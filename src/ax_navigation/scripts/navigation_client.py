#!/usr/bin/env python3
"""D1 Nav2 adapter. Caller supplies approved IDs; no D2/A4 policy is implemented here."""
import json
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from unique_identifier_msgs.msg import UUID
from ax_interfaces.msg import ActuationState, LocalizationHealth
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType

from navigation_execution import ExecutionLedger, NavigationExecution
from navigation_watchdog import InputWatchdog


class NavigationClient:
    """One robot per adapter. Executor must spin the supplied node.

    This adapter is deliberately not launched automatically: D2's authorized
    dispatch and independent A4 stop verification must be connected before operation.
    """

    STOP_STATE_MAX_AGE_NS = 300_000_000

    def __init__(self, node, robot, database):
        self.node, self.robot = node, robot
        self.ledger = ExecutionLedger(database)
        self.manager = NavigationExecution(self.ledger)
        self.client = ActionClient(node, NavigateToPose, '/ax/' + robot + '/navigate_to_pose')
        self.handles = {}
        self.current = None
        self.watchdog = InputWatchdog()
        self.arrival = None
        self.actuation = None
        self.actuation_cursor = None
        self.retired_actuation_sessions = set()
        self.last_stop_clock = None
        root = '/ax/' + robot
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [node.create_subscription(LocalizationHealth,
            root + '/localization/health', self._health, state_qos),
            node.create_subscription(ActuationState, root + '/safety/actuation_state',
                                     self._actuation, state_qos)]
        for source in ('front', 'rear'):
            self.subscriptions.append(node.create_subscription(LaserScan,
                root + '/sensors/' + source + '_lidar/scan_filtered',
                lambda message, source=source: self._scan(source, message), qos_profile_sensor_data))
        self.timer = node.create_timer(.05, self._supervise,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _boot():
        return time.clock_gettime(time.CLOCK_BOOTTIME)

    @staticmethod
    def _stamp(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _health(self, message):
        valid = (message.robot_id == self.robot and message.meta.valid
                 and message.state == LocalizationHealth.OK)
        self.watchdog.observe('localization', self._stamp(message.meta.header.stamp),
                              self._boot(), valid)
        self._supervise()

    def _scan(self, source, message):
        valid = (message.header.frame_id.startswith(self.robot + '/')
                 and bool(message.ranges) and math.isfinite(message.range_min)
                 and math.isfinite(message.range_max) and 0 <= message.range_min < message.range_max
                 and all(not math.isnan(v) and v != -math.inf and v >= 0 for v in message.ranges))
        self.watchdog.observe(source, self._stamp(message.header.stamp), self._boot(), valid)
        self._supervise()

    def _input_reason(self):
        return self.watchdog.reason(self.node.get_clock().now().nanoseconds * 1e-9, self._boot())

    def _stop_clock(self):
        now = self.node.get_clock().now().nanoseconds
        if now <= 0 or (self.last_stop_clock is not None and now < self.last_stop_clock):
            self.arrival = None
            self.actuation = None
            self.actuation_cursor = None
        self.last_stop_clock = now
        return now

    def _actuation(self, message):
        now = self._stop_clock()
        meta = message.meta
        stamp = meta.header.stamp.sec * 1_000_000_000 + meta.header.stamp.nanosec
        if message.robot_id != self.robot or meta.source_id != 'a4_safety_gate':
            return
        session = meta.source_session_id
        if session in self.retired_actuation_sessions:
            return
        if (meta.schema_version != 2 or not session.strip() or not meta.revision
                or not 0 < stamp <= now):
            self.actuation = None
            return
        stop_fields = (meta.valid, message.state)
        if self.actuation_cursor is not None:
            last_session, last_revision, last_stamp, last_stop_fields = self.actuation_cursor
            if stamp < last_stamp:
                return
            if (session == last_session and meta.revision == last_revision
                    and stop_fields != last_stop_fields):
                self.actuation = None
                return
            if session == last_session and (meta.revision < last_revision
                                            or (stamp, meta.revision) == (last_stamp, last_revision)):
                return
            if session != last_session:
                self.retired_actuation_sessions.add(last_session)
        self.actuation_cursor = (session, meta.revision, stamp, stop_fields)
        self.actuation = (message, stamp, self._boot())

    def stopped_after_arrival(self, execution_id):
        """Check current A4 stop evidence for this process's most recent Nav2 arrival."""
        now = self._stop_clock()
        if (self.arrival is None or self.arrival[0] != execution_id
                or self.current != execution_id or self.actuation is None):
            return False
        _, arrived, received_after = self.arrival
        message, stamp, received = self.actuation
        return (message.meta.valid and message.state == ActuationState.STOPPED
                and arrived < stamp <= now and received > received_after
                and now - stamp <= self.STOP_STATE_MAX_AGE_NS
                and 0 <= self._boot() - received <= self.STOP_STATE_MAX_AGE_NS / 1e9)

    def _supervise(self):
        self._stop_clock()
        if not self.current:
            return
        row = self.ledger.get(self.current)
        if row['state'] not in ('SENDING', 'ACTIVE', 'CANCEL_REQUESTED'):
            return
        reason = self._input_reason()
        if reason:
            self.ledger.hold(self.robot, self.current, reason)
        if self.ledger.held(self.robot) and row['state'] == 'ACTIVE':
            try:
                self.cancel(self.current)
            except Exception as error:
                self._unknown(self.current, error)

    def submit(self, *, execution_id, assignment_id, assignment_generation, x, y, yaw):
        reason = self._input_reason()
        if reason or self.ledger.held(self.robot):
            raise RuntimeError('D1 inputs unavailable or robot held: ' + reason)
        # Readiness failure before reservation is safe to retry: nothing was sent.
        if not self.client.server_is_ready():
            raise RuntimeError('Nav2 action server unavailable')
        return self.manager.submit(self._send, execution_id=execution_id, robot=self.robot,
            assignment_id=assignment_id, assignment_generation=assignment_generation,
            x=x, y=y, yaw=yaw)

    def _send(self, row):
        self.current = row['execution_id']
        self.arrival = None
        request = json.loads(row['request'])
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = request['x'], request['y']
        goal.pose.pose.orientation.z = math.sin(request['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(request['yaw'] / 2)
        future = self.client.send_goal_async(goal,
            goal_uuid=UUID(uuid=list(bytes.fromhex(row['goal_uuid']))))
        future.add_done_callback(lambda done: self._accepted(row['execution_id'], done))

    def _unknown(self, key, error):
        row = self.ledger.get(key)
        if row and row['state'] not in self.ledger.TERMINAL:
            self.ledger.transition(key, row['state'], 'UNKNOWN', str(error))
        self.node.get_logger().error('D1 execution unresolved: ' + key + ': ' + str(error))

    def _accepted(self, key, future):
        try:
            handle = future.result()
            self.manager.accepted(key, handle.accepted)
            if handle.accepted:
                self.handles[key] = handle
                handle.get_result_async().add_done_callback(lambda done: self._result(key, done))
                self._supervise()  # Covers input loss while goal acceptance was pending.
        except Exception as error:
            self._unknown(key, error)

    def _result(self, key, future):
        try:
            result = future.result()
            self.manager.result(key, result.status)
            # ARRIVED is Nav2 success only. No STOPPED or manipulation permit is issued.
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                now = self._stop_clock()
                if now > 0:
                    self.arrival = (key, now, self._boot())
                self.node.get_logger().info('D1 ARRIVED; independent A4 stop confirmation required')
            self.handles.pop(key, None)
        except Exception as error:
            self._unknown(key, error)

    def cancel(self, execution_id):
        if execution_id not in self.handles:
            raise ValueError('No live owned goal handle; reconcile persisted UUID first')
        self.manager.cancel(execution_id, self._send_cancel)

    def _send_cancel(self, row):
        self.handles[row['execution_id']].cancel_goal_async().add_done_callback(
            lambda done: self._cancel_response(row['execution_id'], done))

    def _cancel_response(self, key, future):
        try:
            row = self.ledger.get(key)
            # Terminal result can race the cancellation acknowledgment.
            if row['state'] == 'CANCEL_REQUESTED':
                expected = row['goal_uuid']
                accepted = any(bytes(item.goal_id.uuid).hex() == expected
                               for item in future.result().goals_canceling)
                self.manager.cancel_response(key, accepted)
                if not accepted and self.ledger.held(self.robot):
                    self._unknown(key, 'INPUT_CANCEL_REJECTED')
        except Exception as error:
            self._unknown(key, error)

    def close(self):
        self.node.destroy_timer(self.timer)
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.client.destroy()
        self.ledger.close()
