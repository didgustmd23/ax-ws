"""
팔 명령 보호 노드(개발용): 공개 팔 명령을 검사해 내부 JTC action 하나로만 전달한다.

공개 입력(기존 계약 유지):
  {ns}/arm_controller/joint_trajectory          trajectory_msgs/JointTrajectory
  {ns}/arm_controller/follow_joint_trajectory   control_msgs/FollowJointTrajectory
실제 제어기 입력(launch가 arm_controller의 이름을 바꿈):
  {ns}/arm_controller/internal/follow_joint_trajectory
기록: ~/events(사유 JSON), ~/status(현재 모드 JSON, transient local).
이 경로 정리는 정상 명령 경로를 하나로 만드는 것이며 접근 차단(보안)을 보장하지 않는다.
토크가 부족해 처지는 관절은 명령 제한만으로 막을 수 없다.
"""
import json
import math
from pathlib import Path
import queue
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from ax_control import arm_guard_logic as logic
from ax_control.model_contracts import ARM_JOINTS
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

PUBLIC_TOPIC = 'arm_controller/joint_trajectory'
PUBLIC_ACTION = 'arm_controller/follow_joint_trajectory'
INTERNAL_ACTION = 'arm_controller/internal/follow_joint_trajectory'

# 내부 목표 결과 기록용 상태 이름. 수락 거부는 GoalStatus에 없어서 별도 값(-1)으로 둔다.
STATUS_REJECTED = -1
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_UNKNOWN: 'failed',
                STATUS_REJECTED: 'rejected'}


# Duration 메시지를 초로 바꾼다.
def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


# 초를 Duration 메시지로 바꾼다.
def to_duration(value):
    sec = int(math.floor(value))
    return Duration(sec=sec, nanosec=int(round((value - sec) * 1e9)))


# JointTrajectory를 검사 모듈이 쓰는 점 목록으로 바꾼다.
def trajectory_points(trajectory):
    return [{'positions': list(p.positions), 'velocities': list(p.velocities),
             'accelerations': list(p.accelerations), 'effort': list(p.effort),
             'time': seconds(p.time_from_start)}
            for p in trajectory.points]


# 공개 명령을 검사하고, 통과한 명령만 내부 JTC action으로 보내며, 실제 관절 상태로 보호 동작을 하는 노드.
class ArmCommandGuard(Node):
    # 설정·구독·내부 action 클라이언트를 준비한다. 공개 입력은 준비가 끝난 뒤 만든다.
    # kwargs는 Node 생성 인자(예: 시험의 parameter_overrides)로 그대로 넘긴다.
    def __init__(self, **kwargs):
        super().__init__('arm_command_guard', **kwargs)

        self.declare_parameter('margins_rad', [0.02] * len(ARM_JOINTS))
        self.declare_parameter('joint_limits_file', '')
        self.declare_parameter('reaction_time_s', 0.1)
        self.declare_parameter('tracking_allowance_rad', 0.01)
        self.declare_parameter('min_clearance_rad', 0.005)
        self.declare_parameter('fault_clearance_rad', 1.0e-6)
        self.declare_parameter('still_velocity_rad_s', 0.01)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('velocity_plausibility_factor', 1.25)
        self.declare_parameter('stop_minimum_time_s', 0.01)
        self.declare_parameter('stop_confirm_velocity_rad_s', 0.01)
        self.declare_parameter('stop_confirm_timeout_s', 1.0)

        get = self.get_parameter
        self.params = logic.ProtectionParams(
            reaction_time_s=get('reaction_time_s').value,
            tracking_allowance_rad=get('tracking_allowance_rad').value,
            min_clearance_rad=get('min_clearance_rad').value,
            fault_clearance_rad=get('fault_clearance_rad').value,
            still_velocity_rad_s=get('still_velocity_rad_s').value)
        self.stale_timeout = get('stale_timeout_s').value
        self.velocity_plausibility = get('velocity_plausibility_factor').value
        self.stop_minimum_time = get('stop_minimum_time_s').value
        self.stop_confirm_velocity = get('stop_confirm_velocity_rad_s').value
        self.stop_confirm_timeout = get('stop_confirm_timeout_s').value

        margins = list(get('margins_rad').value)
        if len(margins) != len(ARM_JOINTS):
            raise ValueError(f'margins_rad needs {len(ARM_JOINTS)} values')
        self.margins = dict(zip(ARM_JOINTS, margins))
        self.planned = self._planned_limits(get('joint_limits_file').value)

        # 공유 상태. 모든 콜백은 이 잠금 아래에서 상태를 바꾼다.
        self.lock = threading.RLock()
        self.group = ReentrantCallbackGroup()
        self.limits = None
        self.state = None          # (positions, velocities, 수신 단조 시각)
        self.mode = 'init'
        self.mode_reason = 'waiting for robot_description and joint_states'
        self.public_ready = False
        self.last_stamp = None     # 마지막으로 처리한 joint_states 시각(ns)
        self.active = None         # 진행 중 공개 action 목표의 제어 정보
        self.internal = None       # 현재 내부 목표 {'id', 'owner', 'handle', 'result', ...}
        self.internal_count = 0
        self.fault_reason = ''
        self.resume_mode = 'ready'   # 상태가 다시 정상일 때 돌아갈 모드
        self.pending = {}            # 수락 직전 검사 결과(목표 요청 객체 id별)
        self.epoch = 0               # 모드가 바뀔 때마다 늘어나는 상태 변경 번호

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.events = self.create_publisher(String, '~/events', 50)
        self.status = self.create_publisher(String, '~/status', latched)

        # 이벤트·상태 발행과 로그는 송출 스레드가 순서대로 한다. 잠금을 쥔 채 로그를 쓰면
        # (rclpy 로그의 호출 위치 확인 등) 시험에서 다른 콜백이 0.5 s까지 잠금을 기다렸다.
        self.outbox = queue.Queue()
        self.emitter = threading.Thread(target=self._emit_loop, name='arm_guard_emitter',
                                        daemon=True)
        self.emitter.start()

        self.create_subscription(String, 'robot_description', self.on_description, latched,
                                 callback_group=self.group)
        # 관절 상태는 도착 순서대로 하나씩 처리한다(여러 스레드 실행기에서 순서가 뒤바뀌지 않게).
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 20,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.internal_client = ActionClient(self, FollowJointTrajectory, INTERNAL_ACTION,
                                            callback_group=self.group)
        self.create_service(Trigger, '~/clear_fault', self.on_clear_fault,
                            callback_group=self.group)
        self._publish_status()

        # 상태 만료·정지 확인 제한시간 감시는 실행기 타이머 대신 전용 스레드로 0.05 s마다 한다.
        # 시험에서 rclpy 여러 스레드 실행기가 준비된 타이머를 약 3 s 동안 꺼내지 않은 일이 있어
        # (작업 스레드는 비어 있었음), 감시 주기가 실행기 일정에 묶이지 않게 한다.
        self.watchdog_stop = threading.Event()
        self.watchdog = threading.Thread(target=self._watchdog_loop,
                                         name='arm_guard_watchdog', daemon=True)
        self.watchdog.start()

    # 감시 스레드: 정리 요청 전까지 주기적으로 감시를 실행하고, 예외는 로그로 남기고 계속한다.
    def _watchdog_loop(self):
        while not self.watchdog_stop.wait(0.05):
            try:
                self.on_timer()
            except Exception as error:      # noqa: B902 (감시가 멈추지 않게 모든 예외를 기록)
                self.get_logger().error(f'watchdog error: {error!r}')

    # 송출 스레드: 큐에 들어온 순서대로 이벤트·상태를 발행하고 로그에 남긴다(None이면 끝냄).
    def _emit_loop(self):
        while True:
            item = self.outbox.get()
            if item is None:
                return
            try:
                publisher, record, level, text = item
                publisher.publish(String(data=json.dumps(record, ensure_ascii=False)))
                if level == 'warning':
                    self.get_logger().warning(text)
                elif level == 'info':
                    self.get_logger().info(text)
            except Exception as error:      # noqa: B902 (송출이 멈추지 않게 모든 예외를 기록)
                self.get_logger().error(f'emitter error: {error!r}')

    # 감시·송출 스레드를 멈춘 뒤 노드를 정리한다(남은 이벤트는 먼저 내보낸다).
    def destroy_node(self):
        self.watchdog_stop.set()
        self.outbox.put(None)
        for thread in (self.watchdog, self.emitter):
            if thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=1.0)
        return super().destroy_node()

    # 계획용 속도·가속도 표(arm_joint_limits.yaml)를 읽는다. 경로가 없으면 ax_description 설치본을 쓴다.
    def _planned_limits(self, path):
        if not path:
            share = Path(get_package_share_directory('ax_description'))
            path = share / 'config' / 'arm_joint_limits.yaml'
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        return data['joint_limits']

    # 이벤트(JSON)를 송출 큐에 넣는다. 발행·로그는 송출 스레드가 한다.
    def _event(self, kind, **fields):
        record = dict(fields, kind=kind, mode=self.mode,
                      time=self.get_clock().now().nanoseconds * 1e-9)
        text = f'{kind}: ' + json.dumps(fields, ensure_ascii=False)
        warn = kind in ('rejected', 'protect', 'fault', 'state_invalid', 'stop_failed',
                        'stop_reaches_limit')
        self.outbox.put((self.events, record, 'warning' if warn else 'info', text))

    # 현재 모드와 사유를 상태 토픽 송출 큐에 넣는다.
    def _publish_status(self):
        record = {'mode': self.mode, 'reason': self.mode_reason,
                  'public_interfaces_ready': self.public_ready}
        self.outbox.put((self.status, record, None, ''))

    # 모드를 바꾸고 상태 토픽을 갱신한다. 모드가 실제로 바뀌면 상태 변경 번호를 늘려,
    # 그 전에 검사한 목표가 뒤늦게 실행되지 않게 한다.
    def _set_mode(self, mode, reason):
        if mode != self.mode:
            self.epoch += 1
        if (mode, reason) != (self.mode, self.mode_reason):
            self.mode, self.mode_reason = mode, reason
            self._publish_status()

    # robot_description에서 물리 한계를 읽어 명령 가능 범위를 만든다.
    def on_description(self, msg):
        try:
            limits = logic.load_limits(msg.data, self.planned, self.margins)
        except (ValueError, KeyError) as error:
            self._event('config_error', reason=str(error))
            return

        with self.lock:
            self.limits = limits
        self._event('limits_loaded', limits={
            n: {'physical': [lim.lower, lim.upper],
                'command': [lim.command_lower, lim.command_upper],
                'max_velocity': lim.max_velocity, 'max_acceleration': lim.max_acceleration}
            for n, lim in limits.items()})

    # 관절 상태를 갱신하고 보호 판단을 한다.
    def on_joint_states(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(n in index and index[n] < len(msg.position) for n in ARM_JOINTS):
            return

        positions = [msg.position[index[n]] for n in ARM_JOINTS]
        velocities = [msg.velocity[index[n]] if index[n] < len(msg.velocity) else math.nan
                      for n in ARM_JOINTS]
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

        with self.lock:
            # 이미 처리한 것보다 오래된 상태는 버린다(늦게 도착한 메시지로 판단하지 않음).
            # 다만 stale_timeout_s보다 더 과거로 돌아간 시각은 발행 쪽 시계가 되돌아간 것으로 보고
            # 받아들인다(시험 환경 WSL2에서 시스템 시계가 약 3 s씩 뒤로 보정되는 것을 확인함).
            # 그대로 버리면 시계가 따라잡을 때까지 새 상태를 모두 버려 상태 만료가 된다.
            if self.last_stamp is not None and stamp < self.last_stamp:
                behind = (self.last_stamp - stamp) * 1e-9
                if behind <= self.stale_timeout:
                    return
                self._event('stamp_jump_back', seconds=behind)
            self.last_stamp = stamp
            self.state = (positions, velocities, time.monotonic())
            if self.limits is None:
                return

            if not logic._finite(positions + velocities):
                self._state_invalid('joint_states contains non-finite values')
                return

            # DART가 URDF velocity를 강제하므로 관절 속도 한계를 크게 넘는 값은 실제 운동이 아닌
            # 비정상 표본으로 본다(Gazebo 시험에서 제어기 활성 직후 손목 2.10 rad/s 한 표본을 관측).
            # 이 값으로 정지 궤적을 만들면 정지 궤적이 오히려 관절을 한계 쪽으로 움직인다.
            implausible = [
                f'{n} {abs(v):.3f} rad/s' for n, v in zip(ARM_JOINTS, velocities)
                if abs(v) > self.limits[n].max_velocity * self.velocity_plausibility]
            if implausible:
                self._state_invalid('joint velocity above plausible bound '
                                    f'({self.velocity_plausibility:g} x max_velocity): '
                                    + ', '.join(implausible))
                return
            if self.mode == 'state_invalid':
                self._set_mode(self.resume_mode, 'joint_states fresh again')

            self._maybe_open_public()
            self._protect(positions, velocities)
            self._confirm_stop()

    # 준비가 끝나면 공개 토픽·action을 한 번만 만든다.
    def _maybe_open_public(self):
        if self.public_ready or not self.internal_client.server_is_ready():
            return

        # 토픽 궤적은 도착 순서대로 하나씩 처리한다(검사가 잠금 밖이라 순서가 뒤바뀌지 않게).
        self.create_subscription(JointTrajectory, PUBLIC_TOPIC, self.on_topic, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        ActionServer(self, FollowJointTrajectory, PUBLIC_ACTION,
                     execute_callback=self.execute, goal_callback=self.on_goal,
                     handle_accepted_callback=self.on_accepted, cancel_callback=self.on_cancel,
                     callback_group=self.group)
        self.public_ready = True
        self._set_mode('ready', 'accepting commands')
        self._event('ready', public_topic=PUBLIC_TOPIC, public_action=PUBLIC_ACTION,
                    internal_action=INTERNAL_ACTION)

    # 보호 판단 결과에 따라 고장 잠금 또는 감속 정지를 시작한다.
    def _protect(self, positions, velocities):
        findings = logic.protection_check(positions, velocities, self.limits, self.params)
        faults = [f for f in findings if f[2] == 'fault']
        protects = [f for f in findings if f[2] == 'protect']

        if faults and self.mode != 'fault':
            self.fault_reason = '; '.join(f[3] for f in faults)
            self._set_mode('fault', self.fault_reason)
            self._interrupt_public(f'fault: {self.fault_reason}')
            self._hold(force=True)
            self._event('fault', reason=self.fault_reason, positions=positions)
            return

        if protects and self.mode in ('ready', 'protected_hold'):
            reason = '; '.join(f[3] for f in protects)
            plan = logic.stop_trajectory(positions, velocities, self.limits,
                                         self.stop_minimum_time, self.params.fault_clearance_rad)

            self._set_mode('protecting', reason)
            self._interrupt_public(f'protective stop: {reason}')
            deadline = time.monotonic() + plan.duration + self.stop_confirm_timeout
            self._send_internal(plan.points, 'protection', deadline=deadline)

            # 최대 감속으로도 물리 한계 안에서 멈추지 못하는 관절이 있으면 그대로 기록한다
            # (목표를 잘라 가속도 제한을 넘기지 않는다. 실제 한계 도달은 고장 잠금으로 처리된다).
            if plan.limit_violations:
                self._event('stop_reaches_limit', joints=plan.limit_violations,
                            stop_positions=plan.stop_positions)
            self._event('protect', reason=reason, positions=positions, velocities=velocities,
                        stop_targets=[plan.stop_positions[n] for n in ARM_JOINTS],
                        stop_times_s=plan.stop_times, stop_duration_s=plan.duration)

    # 보호 정지 결과를 판정한다. 내부 정지 목표가 성공으로 끝나고, 그 뒤 받은 실제 관절 속도가
    # 모두 확인 속도 이하일 때만 보호 유지로 바꾼다. 수락 거부·중단·취소·통신 예외나 제한시간
    # (정지 시간 + 확인 제한시간) 안에 확인하지 못하면 정지 실패로 잠근다.
    def _confirm_stop(self):
        record = self.internal
        if record is None or record['owner'] != 'protection' or record.get('closed'):
            return
        if self.mode not in ('protecting', 'state_invalid'):
            return

        result = record['result']
        if result is not None and result['status'] != GoalStatus.STATUS_SUCCEEDED:
            status = STATUS_NAMES.get(result['status'], str(result['status']))
            detail = result['error_string']
            return self._stop_failed(record, f'internal stop goal {status}: {detail}')

        if result is not None and self.mode == 'protecting' \
                and self.state[2] > record['result_time'] \
                and all(abs(v) <= self.stop_confirm_velocity for v in self.state[1]):
            record['closed'] = True
            self._set_mode('protected_hold', 'protective stop finished; holding inside limits')
            self._event('protected_hold', result=result, velocities=self.state[1])
            return

        if record['deadline'] is not None and time.monotonic() > record['deadline']:
            what = 'no result' if result is None else 'joint velocities not below ' \
                f'{self.stop_confirm_velocity:g} rad/s'
            self._stop_failed(record, f'protective stop not confirmed in time ({what})')

    # 보호 정지 실패: 고장 잠금으로 일반 작업 재개를 막는다. 정지 목표가 거부돼 이전 내부 목표가
    # 계속 진행 중일 수 있으면 그 목표를 취소해 JTC가 현재 위치를 유지하게 한다(제어기 비활성화 없음).
    def _stop_failed(self, record, reason):
        record['closed'] = True
        previous = record.get('previous')
        if previous is not None and previous['result'] is None:
            if previous['handle'] is None:
                previous['cancel_pending'] = True
            else:
                previous['handle'].cancel_goal_async()

        self.fault_reason = f'protective stop failed: {reason}'
        self._set_mode('fault', self.fault_reason)
        self._event('stop_failed', reason=reason, result=record['result'])

    # 상태가 오래되거나 비정상이면 신규 조작을 막고, 진행 중 일반 동작은 중단해 현재 위치를 유지한다.
    def _state_invalid(self, reason):
        if self.mode in ('state_invalid', 'fault', 'init'):
            return

        # 감속 정지 중이었으면 상태가 돌아온 뒤에도 정지 확인을 이어 가야 하므로 그 모드로 돌아간다.
        protecting = self._protection_active()
        if protecting:
            self.resume_mode = 'protecting'
        else:
            self.resume_mode = 'protected_hold' if self.mode == 'protected_hold' else 'ready'
        self._set_mode('state_invalid', reason)

        # 진행 중인 감속 정지는 JTC가 끝까지 수행하게 둔다. 일반 동작은 취소해 JTC가 유지하게 한다.
        if not protecting:
            self._interrupt_public(f'state invalid: {reason}')
            self._hold()
        self._event('state_invalid', reason=reason, protective_stop_kept=protecting)

    # 주기적으로 관절 상태가 오래되지 않았는지, 감속 정지 확인 제한시간이 지났는지 확인한다.
    def on_timer(self):
        with self.lock:
            if self.state is None or self.limits is None:
                return
            age = time.monotonic() - self.state[2]
            if age > self.stale_timeout:
                self._state_invalid(f'joint_states older than {self.stale_timeout:g} s '
                                    f'(age {age:.2f} s)')
            self._confirm_stop()

    # 내부 JTC action으로 목표를 보낸다. 내부 목표는 항상 하나이며 새 목표가 이전 목표를 대체한다.
    # deadline은 감속 정지의 확인 제한 시각(단조 시계)이다.
    def _send_internal(self, points, owner, tolerances=None, deadline=None):
        self.internal_count += 1
        record = {'id': self.internal_count, 'owner': owner, 'handle': None, 'result': None,
                  'cancel_pending': False, 'result_time': None,
                  'deadline': deadline,
                  # 감속 정지가 실패하면 취소할 수 있게 대체된 이전 목표를 기억한다.
                  'previous': self.internal if owner == 'protection' else None}
        self.internal = record

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        for p in points:
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=[float(v) for v in p['positions']],
                velocities=[float(v) for v in p['velocities']],
                accelerations=[float(v) for v in p['accelerations']],
                time_from_start=to_duration(p['time'])))

        if tolerances is not None:
            goal.path_tolerance, goal.goal_tolerance, goal.goal_time_tolerance = tolerances

        try:
            future = self.internal_client.send_goal_async(goal)
        except Exception as error:      # noqa: B902 (통신 계층의 모든 예외를 실패로 기록)
            self._set_result(record, GoalStatus.STATUS_UNKNOWN,
                             f'internal goal request failed: {error!r}')
            return record
        future.add_done_callback(lambda f: self._on_internal_goal(record, f))
        return record

    # 내부 목표 결과를 기록하고 끝난 목표를 처리한다.
    def _set_result(self, record, status, error_string, error_code=-1):
        record['result'] = {'status': status, 'error_code': error_code,
                            'error_string': error_string}
        record['result_time'] = time.monotonic()
        self._internal_finished(record)

    # 내부 목표 수락 결과를 기록하고 결과를 기다린다. 수락 거부·통신 예외는 실패 결과로 남긴다.
    def _on_internal_goal(self, record, future):
        try:
            handle = future.result()
        except Exception as error:      # noqa: B902
            handle, failure = None, f'internal goal request failed: {error!r}'
        else:
            failure = 'no response from internal controller' if handle is None else None

        with self.lock:
            record['handle'] = handle
            if failure is not None:
                self._set_result(record, GoalStatus.STATUS_UNKNOWN, failure)
                return
            if not handle.accepted:
                self._set_result(record, STATUS_REJECTED, 'internal controller rejected the goal')
                return

            # 수락 응답 전에 유지(취소) 요청이 있었으면 지금 취소한다.
            if record['cancel_pending']:
                handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f: self._on_internal_result(record, f))

    # 내부 목표 결과를 기록한다. 결과 수신 예외도 실패 결과로 남긴다.
    def _on_internal_result(self, record, future):
        try:
            response = future.result()
        except Exception as error:      # noqa: B902
            response, failure = None, f'internal result request failed: {error!r}'
        else:
            failure = 'no result from internal controller' if response is None else None

        with self.lock:
            if failure is not None:
                self._set_result(record, GoalStatus.STATUS_UNKNOWN, failure)
                return
            self._set_result(record, response.status, response.result.error_string,
                             response.result.error_code)

    # 내부 목표가 끝났을 때 감속 정지였다면 정지 결과를 판정한다.
    def _internal_finished(self, record):
        if record is self.internal and record['owner'] == 'protection':
            self._confirm_stop()

    # 현재 내부 목표를 취소해 JTC가 현재 위치를 유지하게 한다(제어기는 비활성화하지 않음).
    # 내부 목표의 수락 응답이 아직 없으면 취소 요청을 기록해 두고 수락 즉시 취소한다.
    # 진행 중인 감속 정지(결과 확인 전)는 force일 때만 취소한다(물리 한계 도달 고장 잠금). 빈 궤적·
    # 공개 취소·상태 이상 처리는 보호 정지를 취소하지 못한다.
    def _hold(self, force=False):
        record = self.internal
        if record is None or record['result'] is not None:
            return
        if self._protection_active() and not force:
            return
        if record['handle'] is None:
            record['cancel_pending'] = True
        else:
            record['handle'].cancel_goal_async()

    # 감속 정지가 진행 중인지(내부 목표가 보호 정지이고 정지 판정이 아직 끝나지 않음) 돌려준다.
    # 모드(protecting 또는 감속 중 상태 이상)와 관계없이 이 기준 하나로 판단한다.
    def _protection_active(self):
        record = self.internal
        return (record is not None and record['owner'] == 'protection'
                and not record.get('closed'))

    # 진행 중 공개 action 목표를 사유와 함께 중단 대상으로 표시한다.
    def _interrupt_public(self, reason):
        if self.active is not None:
            self.active['interrupt'] = reason
            self.active = None

    # 잠금 안에서 모드·관절 상태를 확인하고, 검사에 쓸 상태를 (상태 변경 번호, 위치, 속도)로
    # 복사한다. 받을 수 없는 상태면 (None, 사유)를 돌려준다.
    def _snapshot(self):
        if self.mode == 'fault':
            return None, f'fault latched: {self.fault_reason}'
        if self.mode in ('init', 'state_invalid', 'protecting'):
            return None, f'not accepting commands in mode {self.mode}: {self.mode_reason}'
        if self._protection_active():
            return None, 'protective stop in progress'
        if self.state is None or time.monotonic() - self.state[2] > self.stale_timeout:
            return None, 'joint_states missing or stale'
        return (self.epoch, list(self.state[0]), list(self.state[1])), ''

    # 잠금 안에서, 복사 이후 모드가 바뀌지 않았고 관절 상태가 아직 최신인지 확인한다.
    # 문제가 없으면 None, 있으면 사유를 돌려준다.
    def _still_valid(self, snapshot):
        if self.epoch != snapshot[0]:
            return (f'guard state changed during check (now {self.mode}: '
                    f'{self.mode_reason})')
        if self.state is None or time.monotonic() - self.state[2] > self.stale_timeout:
            return 'joint_states missing or stale'
        return None

    # 궤적을 검사해 (Verdict, 복사한 상태)를 돌려준다. 잠금 밖에서 호출한다. 상태 복사만 잠금
    # 안에서 하고, 긴 궤적의 경로 검사가 관절 상태 처리·보호 판단을 막지 않게 잠금 밖에서 한다.
    # 호출자는 결과를 쓰기 전에 잠금 안에서 _still_valid로 다시 확인해야 한다.
    def _check(self, trajectory):
        with self.lock:
            snapshot, reason = self._snapshot()
            limits = self.limits
        if snapshot is None:
            return logic.Verdict(False, reason), None

        stamp_zero = trajectory.header.stamp.sec == 0 and trajectory.header.stamp.nanosec == 0
        verdict = logic.validate_trajectory(
            list(trajectory.joint_names), trajectory_points(trajectory), stamp_zero,
            False, snapshot[1], snapshot[2], limits)
        return verdict, snapshot

    # 공개 토픽 궤적: 비어 있으면 현재 위치 유지, 통과하면 내부로 보내고, 아니면 사유를 기록한다.
    def on_topic(self, msg):
        if not msg.points:
            with self.lock:
                # 감속 정지 중(상태 이상으로 모드가 바뀐 경우 포함)에는 유지 요청이 감속 정지를
                # 취소하지 않게 무시한다.
                if self.mode in ('fault', 'init') or self._protection_active():
                    what = 'protective stop in progress' if self._protection_active() \
                        else f'mode {self.mode}'
                    self._event('rejected', source='topic',
                                reason=f'hold request ignored: {what}')
                    return
                self._interrupt_public('hold requested by empty trajectory')
                self._hold()
                self._event('hold', source='topic')
            return

        verdict, snapshot = self._check(msg)
        with self.lock:
            if verdict.ok:
                changed = self._still_valid(snapshot)
                if changed is not None:
                    verdict = logic.Verdict(False, changed)
            if not verdict.ok:
                self._event('rejected', source='topic', reason=verdict.reason)
                return

            self._interrupt_public('preempted by topic trajectory')
            self._set_mode('ready', 'executing topic trajectory')
            self._send_internal(verdict.points, 'topic')
            self._event('accepted', source='topic', normalized=verdict.normalized,
                        summary=verdict.summary)

    # 공개 action 목표를 수락 전에 검사한다. 거부 사유는 이벤트와 로그에 남긴다.
    def on_goal(self, goal_request):
        has_multi_dof = bool(goal_request.multi_dof_trajectory.points
                             or goal_request.multi_dof_trajectory.joint_names)
        if has_multi_dof:
            verdict = logic.Verdict(False, 'multi-DOF trajectories are not supported')
            snapshot = None
        else:
            verdict, snapshot = self._check(goal_request.trajectory)

        with self.lock:
            if verdict.ok:
                changed = self._still_valid(snapshot)
                if changed is not None:
                    verdict = logic.Verdict(False, changed)
            if not verdict.ok:
                self._event('rejected', source='action', reason=verdict.reason)
                return GoalResponse.REJECT

            # 수락 시점의 상태 변경 번호를 함께 기억해 실행 직전에 비교한다.
            tolerances = (goal_request.path_tolerance, goal_request.goal_tolerance,
                          goal_request.goal_time_tolerance)
            self.pending[id(goal_request)] = (verdict, tolerances, snapshot[0])
            return GoalResponse.ACCEPT

    # 수락한 목표를 등록하고 실행을 시작한다.
    def on_accepted(self, goal_handle):
        self._register_goal(goal_handle)
        goal_handle.execute()

    # 수락한 목표가 이전 목표를 선점하게 하고 진행 중 목표로 등록한다.
    def _register_goal(self, goal_handle):
        with self.lock:
            self._interrupt_public('preempted by newer action goal')
            verdict, tolerances, epoch = self.pending.pop(id(goal_handle.request))
            context = {'handle': goal_handle, 'verdict': verdict, 'tolerances': tolerances,
                       'epoch': epoch, 'interrupt': None, 'internal': None}
            self.active = context
            goal_handle.context = context

    # 공개 목표 취소 요청은 받아들인다(실제 처리는 실행 콜백에서 내부 목표를 취소해 유지).
    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    # 수락한 목표를 내부로 보내기 직전에 다시 확인한다. 수락 검사 뒤 모드가 바뀌었거나(상태 변경
    # 번호 비교) 지금 상태로 다시 검사해 통과하지 못하면 보내지 않고 중단 사유를 돌려준다.
    def _start_goal(self, context):
        verdict, snapshot = self._check(context['handle'].request.trajectory)

        with self.lock:
            if context['interrupt'] is not None:
                return context['interrupt']
            if self.epoch != context['epoch']:
                reason = (f'guard state changed after acceptance (now {self.mode}: '
                          f'{self.mode_reason}); goal not executed')
                self._event('rejected', source='action', reason=reason)
                return reason

            if verdict.ok:
                changed = self._still_valid(snapshot)
                if changed is not None:
                    verdict = logic.Verdict(False, changed)
            if not verdict.ok:
                reason = f'recheck before execution failed: {verdict.reason}'
                self._event('rejected', source='action', reason=reason)
                return reason

            self._set_mode('ready', 'executing action goal')
            context['epoch'] = self.epoch
            context['internal'] = self._send_internal(verdict.points, 'action',
                                                      context['tolerances'])
            self._event('accepted', source='action', normalized=verdict.normalized,
                        summary=verdict.summary)
            return None

    # 공개 목표를 내부로 보내고 끝날 때까지 기다리며 선점·보호·취소·결과를 반영한다.
    def execute(self, goal_handle):
        context = goal_handle.context
        refused = self._start_goal(context)
        if refused is not None:
            with self.lock:
                self.active = None if self.active is context else self.active
            return self._finish(goal_handle, 'abort', -1, refused)

        while rclpy.ok():
            with self.lock:
                record = context['internal']

                if context['interrupt'] is not None:
                    return self._finish(goal_handle, 'abort', -1, context['interrupt'])

                if goal_handle.is_cancel_requested:
                    self._hold()
                    self.active = None if self.active is context else self.active
                    return self._finish(goal_handle, 'cancel', 0, 'canceled; joints hold')

                if record is not None and record['result'] is not None:
                    self.active = None if self.active is context else self.active
                    result = record['result']
                    kind = 'succeed' if result['status'] == GoalStatus.STATUS_SUCCEEDED \
                        else 'abort'
                    return self._finish(goal_handle, kind, result['error_code'],
                                        result['error_string'])
            time.sleep(0.01)

        return self._finish(goal_handle, 'abort', -1, 'shutdown')

    # 공개 목표를 끝내고 결과 메시지를 돌려준다.
    def _finish(self, goal_handle, kind, error_code, error_string):
        result = FollowJointTrajectory.Result(error_code=int(error_code),
                                              error_string=error_string)
        {'succeed': goal_handle.succeed, 'abort': goal_handle.abort,
         'cancel': goal_handle.canceled}[kind]()
        self._event('action_result', outcome=kind, error_code=int(error_code),
                    error_string=error_string)
        return result

    # 고장 해제를 막는 사유를 돌려준다(없으면 None). 상태 존재, 팔 5축 데이터 길이, 위치·속도의
    # 유한성, 최신성을 먼저 확인한다. NaN은 비교가 모두 거짓이라 아래 여유·정지 검사를 그대로
    # 통과하므로 반드시 먼저 걸러야 한다. 그 뒤 위치 여유와 정지 속도를 검사한다.
    def _clear_fault_problem(self):
        if self.state is None or self.limits is None:
            return 'joint_states missing'
        positions, velocities, received = self.state
        if len(positions) != len(ARM_JOINTS) or len(velocities) != len(ARM_JOINTS):
            return 'joint_states incomplete for arm joints'

        bad = [n for n, q, v in zip(ARM_JOINTS, positions, velocities)
               if not logic._finite([q, v])]
        if bad:
            return f'joint_states not finite (or velocity missing): {bad}'
        if time.monotonic() - received > self.stale_timeout:
            return 'joint_states missing or stale'

        close = [n for n, q in zip(ARM_JOINTS, positions)
                 if min(q - self.limits[n].lower, self.limits[n].upper - q)
                 < self.params.min_clearance_rad]
        if close:
            return f'joints still at/near physical limits: {close}'

        moving = [n for n, v in zip(ARM_JOINTS, velocities)
                  if abs(v) > self.stop_confirm_velocity]
        if moving:
            return f'joints still moving: {moving}'
        return None

    # 고장 해제 요청: 관절 상태가 정상·최신이고 모든 관절이 물리 한계에서 최소 여유 이상 떨어져
    # 멈춰 있을 때만 보호 유지 모드로 돌린다. 거부 사유는 응답과 이벤트에 남긴다.
    def on_clear_fault(self, request, response):
        with self.lock:
            if self.mode != 'fault':
                response.success, response.message = False, f'not in fault (mode {self.mode})'
                return response

            reason = self._clear_fault_problem()
            if reason is not None:
                response.success, response.message = False, reason
                self._event('clear_fault_rejected', reason=reason)
                return response

            self._set_mode('protected_hold', 'fault cleared by request')
            self._event('fault_cleared')
            response.success, response.message = True, 'fault cleared'
            return response


# 노드를 여러 스레드 실행기로 실행하고 종료 시 정리한다.
def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandGuard()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
