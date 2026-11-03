#!/usr/bin/env python3
"""Terminal keyboard control for the AX simulation (not collision avoidance)."""
import argparse
import curses
import math
from pathlib import Path
import time

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
    def __init__(self):
        super().__init__('ax_keyboard_teleop', parameter_overrides=[Parameter('use_sim_time', value=True)])
        share = Path(get_package_share_directory('ax_description'))
        self.limits = yaml.safe_load((share / 'config/arm_joint_limits.yaml').read_text())['joint_limits']
        self.robot = 'robot_1'
        self.joint = 0
        self.positions = None
        self.state_time = -math.inf
        self.velocity = (0., 0.)
        self.key_deadline = 0.
        self.pending = False
        self.goal = None
        self.cancel_requested = False
        self.notice = 'Ready. Focus this terminal; Space = stop, Q = quit.'
        self.drive_publishers = {robot: self.create_publisher(TwistStamped,
            f'/ax/{robot}/diff_drive_controller/cmd_vel', 10) for robot in ('robot_1', 'robot_2')}
        self.create_subscription(JointState, '/ax/robot_1/joint_states', self.on_state, qos_profile_sensor_data)
        self.arm = ActionClient(self, FollowJointTrajectory, '/ax/robot_1/arm_controller/follow_joint_trajectory')

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values for j in JOINTS):
            self.positions = [values[j] for j in JOINTS]
            self.state_time = time.monotonic()

    def publish_drive(self):
        moving = time.monotonic() < self.key_deadline and not self.pending
        for robot, publisher in self.drive_publishers.items():
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = robot + '/base_link'
            if robot == self.robot and moving:
                msg.twist.linear.x, msg.twist.angular.z = self.velocity
            publisher.publish(msg)

    def stop(self):
        self.velocity = (0., 0.)
        self.key_deadline = 0.
        self.publish_drive()
        self.cancel_requested = True
        if self.goal is not None:
            self.goal.cancel_goal_async()
        self.notice = 'Stop requested.'

    def jog(self, direction):
        if self.robot != 'robot_1':
            self.notice = 'robot_2 has no arm. Press R to select robot_1.'
            return
        if self.pending:
            self.notice = 'Arm step in progress; Space cancels it.'
            return
        if self.positions is None or time.monotonic() - self.state_time > 1.:
            self.notice = 'Waiting for fresh robot_1 joint states.'
            return
        if not self.arm.server_is_ready():
            self.notice = 'Arm controller is not ready.'
            return
        target, duration = jog_target(self.positions, self.joint, direction, self.limits)
        if abs(target[self.joint] - self.positions[self.joint]) < .001:
            self.notice = 'Joint limit reached.'
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
        self.notice = f'Jogging {JOINTS[self.joint]} by {direction * .04:+.2f} rad.'

    def on_goal(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.pending = False
                self.notice = 'Arm goal rejected.'
                return
            self.goal = handle
            handle.get_result_async().add_done_callback(self.on_result)
            if self.cancel_requested:
                handle.cancel_goal_async()
        except Exception as exc:
            self.pending = False
            self.notice = str(exc)

    def on_result(self, future):
        try:
            response = future.result()
            self.notice = ('Arm stopped.' if response.status == 5 else
                           'Arm step complete.' if response.result.error_code == 0 else
                           'Arm error: ' + response.result.error_string)
        except Exception as exc:
            self.notice = str(exc)
        self.goal = None
        self.pending = False

    def key(self, key):
        if key in (ord('q'), ord('Q'), 3):
            return False
        if key == ord(' '):
            self.stop()
        elif key in (ord('r'), ord('R')):
            self.stop()
            self.robot = 'robot_2' if self.robot == 'robot_1' else 'robot_1'
            self.notice = 'Selected ' + self.robot
        elif ord('1') <= key <= ord('5'):
            self.stop()
            self.joint = key - ord('1')
            self.notice = 'Selected ' + JOINTS[self.joint]
        elif key in (ord('i'), ord('k')):
            self.jog(1 if key == ord('i') else -1)
        elif key in (ord('w'), ord('s'), ord('a'), ord('d')):
            if self.pending:
                self.notice = 'Stop the arm with Space before driving.'
            else:
                self.velocity = {ord('w'): (.1, 0.), ord('s'): (-.1, 0.),
                                 ord('a'): (0., .2), ord('d'): (0., -.2)}[key]
                self.key_deadline = time.monotonic() + .2
                self.notice = 'Driving ' + self.robot
        return True

    def screen(self, screen):
        screen.nodelay(True)
        curses.curs_set(0)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            key = screen.getch()
            if key != -1 and not self.key(key):
                break
            self.publish_drive()
            lines = ['AX keyboard control - SIMULATION',
                     f'Robot: {self.robot}    Joint: {self.joint + 1} {JOINTS[self.joint]}',
                     'R: switch robot | W/S: forward/back | A/D: turn left/right',
                     '1..5: select arm joint | I/K: + / - 0.04 rad step',
                     'SPACE: stop both bases + cancel arm | Q: stop and quit',
                     'Hold driving key (OS key repeat); release stops within ~0.2s.',
                     'Arm: robot_1 only. No finger open/close or collision avoidance.',
                     self.notice]
            if self.positions is not None:
                lines.append('Arm rad: ' + ' '.join(f'{p:+.3f}' for p in self.positions))
            screen.erase()
            height, width = screen.getmaxyx()
            for row, text in enumerate(lines[:max(0, height - 1)]):
                screen.addnstr(row, 0, text, max(0, width - 1))
            screen.refresh()
            time.sleep(.025)

    def close_safely(self):
        self.stop()
        deadline = time.monotonic() + 2.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            self.publish_drive()
            if not self.pending and time.monotonic() > deadline - 1.7:
                break


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
