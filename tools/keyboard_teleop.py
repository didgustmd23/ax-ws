#!/usr/bin/env python3
# 키보드와 웹 리모컨이 공용으로 쓰는 ROS 제어부. 주행은 TwistStamped, 팔은 FollowJointTrajectory Action을 사용한다.
"""Terminal keyboard control for the AX simulation (not collision avoidance)."""
import argparse
import curses
import math
from pathlib import Path
import time
import unicodedata

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

JOINTS = ['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']
JOINT_LABELS = ['기둥 회전', '어깨', '팔꿈치', '손목', '집게 전체 회전']

# 한글은 보통 두 칸을 차지하므로 문자 수가 아니라 터미널 표시 폭으로 잘라낸다.
def fit_columns(text, columns):
    result, used = [], 0
    for char in text:
        size = 0 if unicodedata.combining(char) else 2 if unicodedata.east_asian_width(char) in ('W', 'F') else 1
        if used + size > columns:
            break
        result.append(char)
        used += size
    return ''.join(result)



# 현재 관절 상태에서 한 축만 이동하고 위치 한계를 적용한다. SI 단위는 rad, rad/s, rad/s²다.
def jog_target(positions, index, direction, limits, step=.04):
    if len(positions) != 5 or not all(math.isfinite(p) for p in positions):
        raise ValueError('Invalid arm state')
    target = list(positions)
    spec = limits[JOINTS[index]]
    target[index] = min(spec['max_position'], max(spec['min_position'], positions[index] + direction * step))
    distance = abs(target[index] - positions[index])
    # Zero-velocity/acceleration endpoints give a smooth quintic trajectory.
    duration = max(1.0, 1.875 * distance / spec['max_velocity'],
                   math.sqrt(5.8 * distance / spec['max_acceleration']))
    return target, duration


class KeyboardTeleop(Node):
    # 시뮬레이션 시계를 쓰며, 두 주행 제어기와 AX_1 팔 Action만 연결한다.
    def __init__(self, max_drive_scale=2.5, max_arm_step=.15):
        super().__init__('ax_keyboard_teleop', parameter_overrides=[Parameter('use_sim_time', value=True)])
        share = Path(get_package_share_directory('ax_description'))
        self.limits = yaml.safe_load((share / 'config/arm_joint_limits.yaml').read_text())['joint_limits']
        self.robot = 'robot_1'
        self.joint = 0
        self.adjust_mode = 'drive'
        self.drive_scale = 1.0
        self.max_drive_scale = max_drive_scale
        self.arm_step = .04
        self.max_arm_step = max_arm_step
        self.positions = None
        self.state_time = -math.inf
        self.velocity = (0., 0.)
        self.key_deadline = 0.
        self.pending = False
        self.goal = None
        self.cancel_requested = False
        self.notice = '준비 완료. 이 터미널을 클릭한 뒤 영문 소문자로 조종하세요.'
        self.drive_publishers = {robot: self.create_publisher(TwistStamped,
            f'/ax/{robot}/diff_drive_controller/cmd_vel', 10) for robot in ('robot_1', 'robot_2')}
        self.create_subscription(JointState, '/ax/robot_1/joint_states', self.on_state, qos_profile_sensor_data)
        self.arm = ActionClient(self, FollowJointTrajectory, '/ax/robot_1/arm_controller/follow_joint_trajectory')

    # 팔 5축이 모두 들어온 상태만 저장한다. 최신성 판정에는 실제 경과 시간(monotonic)을 쓴다.
    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values for j in JOINTS):
            self.positions = [values[j] for j in JOINTS]
            self.state_time = time.monotonic()

    # 선택 로봇에만 속도를 보내고 나머지는 0으로 유지한다. 메시지 stamp는 ROS 시뮬레이션 시각이다.
    def publish_drive(self):
        moving = time.monotonic() < self.key_deadline and not self.pending
        for robot, publisher in self.drive_publishers.items():
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = robot + '/base_link'
            if robot == self.robot and moving:
                msg.twist.linear.x, msg.twist.angular.z = self.velocity
            publisher.publish(msg)

    # 양쪽 베이스를 정지시키고 이 인스턴스가 보낸 팔 Action만 취소한다.
    def stop(self):
        self.velocity = (0., 0.)
        self.key_deadline = 0.
        self.publish_drive()
        self.cancel_requested = True
        if self.goal is not None:
            self.goal.cancel_goal_async()
        self.notice = '정지 요청: 두 로봇 주행 정지 / 진행 중인 팔 동작 취소'

    # 상태 최신성·서버 준비·위치 범위를 확인한 뒤 0 속도/가속도 끝점을 가진 작은 팔 궤적을 보낸다.
    def jog(self, direction):
        if self.robot != 'robot_1':
            self.notice = 'AX_2에는 팔이 없습니다. r을 눌러 AX_1을 선택하세요.'
            return
        if self.pending:
            self.notice = '팔 이동 중입니다. 완료 후 다시 입력하거나 스페이스로 취소하세요.'
            return
        if self.positions is None or time.monotonic() - self.state_time > 1.:
            self.notice = 'AX_1의 최신 관절 상태를 기다리는 중입니다.'
            return
        if not self.arm.server_is_ready():
            self.notice = '팔 제어기가 아직 준비되지 않았습니다.'
            return
        target, duration = jog_target(self.positions, self.joint, direction, self.limits, self.arm_step)
        if abs(target[self.joint] - self.positions[self.joint]) < .001:
            self.notice = '관절 이동 한계에 도달했습니다.'
            return
        self.stop()
        self.cancel_requested = False
        self.pending = True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        start = JointTrajectoryPoint()
        start.positions = list(self.positions)
        start.velocities = start.accelerations = [0.] * 5
        end = JointTrajectoryPoint()
        end.positions = target
        end.velocities = end.accelerations = [0.] * 5
        end.time_from_start.sec = int(duration)
        end.time_from_start.nanosec = round((duration % 1) * 1e9)
        goal.trajectory.points = [start, end]
        self.arm.send_goal_async(goal).add_done_callback(self.on_goal)
        change = target[self.joint] - self.positions[self.joint]
        self.notice = f'{JOINT_LABELS[self.joint]} 이동 중: {math.degrees(change):+.1f}도'

    # 주행 속도 배율 또는 다음 팔 이동 각도만 변경한다. 진행 중인 팔 궤적의 속도를 바꾸지는 않는다.
    def adjust(self, direction):
        if self.adjust_mode == 'arm':
            self.arm_step = min(self.max_arm_step, round(max(.01, self.arm_step + direction * .01), 2))
            self.notice = f'다음 팔 이동 각도: {math.degrees(self.arm_step):.1f}도'
        else:
            self.drive_scale = round(min(self.max_drive_scale, max(.25, self.drive_scale + direction * .25)), 2)
            # Changing a setting never starts or prolongs motion.
            self.velocity = (0., 0.)
            self.key_deadline = 0.
            self.publish_drive()
            self.notice = f'주행 {self.drive_scale * .1:.3f} m/s / 회전 {self.drive_scale * .2:.2f} rad/s'

    # 수락 응답보다 먼저 정지를 눌렀다면 수락 직후 취소해 지연된 동작 시작을 막는다.
    def on_goal(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.pending = False
                self.notice = '팔 제어기가 이동 명령을 거부했습니다.'
                return
            self.goal = handle
            handle.get_result_async().add_done_callback(self.on_result)
            if self.cancel_requested:
                handle.cancel_goal_async()
        except Exception as exc:
            self.pending = False
            self.notice = str(exc)

    # Action 결과를 UI 상태로 바꾸고 다음 팔 입력을 받을 수 있게 pending을 해제한다.
    def on_result(self, future):
        try:
            response = future.result()
            self.notice = ('팔 동작을 취소했습니다.' if response.status == 5 else
                           '팔 이동 완료. 다음 조작을 입력하세요.' if response.result.error_code == 0 else
                           '팔 오류: ' + response.result.error_string)
        except Exception as exc:
            self.notice = str(exc)
        self.goal = None
        self.pending = False

    # UI 입력의 공용 라우터. 팔 동작 중 주행을 막고 관절 선택/로봇 변경 때는 먼저 정지한다.
    def key(self, key):
        if key in (ord('q'), ord('Q'), 3):
            return False
        if key == ord(' '):
            self.stop()
        elif key in (ord('+'), ord('='), ord('-')):
            self.adjust(-1 if key == ord('-') else 1)
        elif key in (ord('r'), ord('R')):
            self.stop()
            self.robot = 'robot_2' if self.robot == 'robot_1' else 'robot_1'
            self.adjust_mode = 'drive'
            self.notice = '선택: ' + self.robot
        elif ord('1') <= key <= ord('5'):
            self.stop()
            self.joint = key - ord('1')
            self.adjust_mode = 'arm'
            self.notice = '선택: ' + JOINTS[self.joint]
        elif key in (ord('i'), ord('k')):
            self.adjust_mode = 'arm'
            self.jog(1 if key == ord('i') else -1)
        elif key in (ord('w'), ord('s'), ord('a'), ord('d')):
            self.adjust_mode = 'drive'
            if self.pending:
                self.notice = '팔 이동 중에는 주행할 수 없습니다. 스페이스로 먼저 정지하세요.'
            else:
                linear, angular = .1 * self.drive_scale, .2 * self.drive_scale
                self.velocity = {ord('w'): (linear, 0.), ord('s'): (-linear, 0.),
                                 ord('a'): (0., angular), ord('d'): (0., -angular)}[key]
                self.key_deadline = time.monotonic() + .2
                self.notice = '주행 대상: ' + self.robot
        return True

    # 터미널 키 입력·ROS 콜백·제로 속도 갱신을 같은 루프에서 처리한다.
    def screen(self, screen):
        screen.nodelay(True)
        curses.curs_set(0)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            key = screen.getch()
            if key != -1 and not self.key(key):
                break
            self.publish_drive()
            lines = [
                'AX 키보드 조종 | 시뮬레이션',
                f"선택 로봇: {'AX_1 (팔 로봇)' if self.robot == 'robot_1' else 'AX_2 (운반 로봇)'}  [r: 전환]",
                f'주행 {self.drive_scale * .1:.3f} m/s | 회전 {self.drive_scale * .2:.2f} rad/s | 팔 1회 {math.degrees(self.arm_step):.1f}도',
                f"+ / - 조절 대상: {'주행·회전 속도' if self.adjust_mode == 'drive' else '팔 1회 이동 각도'}",
                '--- 주행: 키를 누르고 있으면 이동 / 떼면 약 0.2초 후 정지 ---',
                '              w 전진',
                '    a 좌회전             d 우회전',
                '              s 후진',
                '--- AX_1 팔: 숫자로 관절 선택 후 i / k ---',
                '1 기둥 회전 | 2 어깨 | 3 팔꿈치 | 4 손목 | 5 집게 전체 회전',
                f'현재 관절: {self.joint + 1}번 {JOINT_LABELS[self.joint]}',
                'i: +방향 / k: -방향으로 위에 표시된 각도만큼 이동',
                '+ / -: 값 증가·감소 (=도 증가) | 주행 키→속도, 관절 번호→각도',
                '스페이스: 두 로봇 주행 정지 + 팔 동작 취소',
                'q 또는 Ctrl+C: 정지 후 조종 종료 (Gazebo는 유지)',
                '이 터미널을 클릭하고 영문 소문자로 입력하세요.',
                '손가락 개폐 / 자동 충돌 회피는 지원하지 않습니다.',
                '상태: ' + self.notice]
            if self.positions is not None:
                lines.append('관절 각도(rad): ' + ' '.join(f'{p:+.3f}' for p in self.positions))
            screen.erase()
            height, width = screen.getmaxyx()
            for row, text in enumerate(lines[:max(0, height - 1)]):
                if width > 1:
                    try:
                        screen.addstr(row, 0, fit_columns(text, width - 1))
                    except curses.error:
                        pass  # Terminal may have resized between measuring and drawing.
            screen.refresh()
            time.sleep(.025)

    # 종료 전 잠시 ROS 응답을 처리해 정지·취소 요청이 전송될 시간을 확보한다.
    def close_safely(self):
        self.stop()
        deadline = time.monotonic() + 2.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            self.publish_drive()
            if not self.pending and time.monotonic() > deadline - 1.7:
                break


# 터미널 종료나 Ctrl+C에서도 finally 경로로 정지 요청을 수행한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    try:
        curses.wrapper(node.screen)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_safely()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
