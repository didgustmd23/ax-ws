"""격리된 실제 Gazebo·배터리·A4/B4와 내 D3 수신을 관측합니다. 주행 허가는 켜지 않습니다."""
import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from ax_interfaces.msg import ActuationState, SafetyState
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/tests'))
sys.path.insert(0, str(ROOT / 'src/ax_corridor_manager'))
import ros_test_domain
from ax_corridor_manager.node import CorridorNode, STATE_QOS


class ObservedD3(CorridorNode):
    """원래 D3 수신 처리를 그대로 실행하고 실제 DDS 정지 수락 횟수만 셉니다."""

    def __init__(self):
        self.accepted_stops = {'robot_1': 0, 'robot_2': 0}
        super().__init__()

    def on_stop(self, message, robot):
        """시험 관측을 위해 처리 결과만 집계하며 판정이나 권한은 바꾸지 않습니다."""
        super().on_stop(message, robot)
        confirmed, deadline = self.manager.stops.get(robot, (False, 0.0))
        if confirmed and deadline > self.manager.clock():
            self.accepted_stops[robot] += 1


def stop_process(process):
    """이번 시험이 시작한 프로세스 그룹만 종료합니다."""
    if process is None or process.poll() is not None:
        return
    for action, wait_s in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, action)
            process.wait(timeout=wait_s)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


@contextmanager
def isolated_ros_home(get_process):
    """자식 프로세스의 종료 로그가 끝난 뒤 임시 ROS_HOME을 지웁니다."""
    folder = tempfile.TemporaryDirectory(prefix='kim_gazebo_ros_home_')
    try:
        yield folder.name
    finally:
        stop_process(get_process())
        folder.cleanup()


def main():
    """실제 생산자와 수신부의 관측 결과 및 제한을 원본 로그와 함께 보관합니다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=120)
    parser.add_argument('--check-charging', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = {'passed': False, 'scope': '실제 Gazebo 정지 상태·배터리·Schema 2·D3 DDS 수신; '
              '주행/조작/영역 승인/LOW 강제정지 검증 아님'}
    domain, lock = ros_test_domain.reserve_domain(ros_test_domain.A3_DOMAINS)
    process = None
    monitor = d3 = executor = None
    try:
        # 이미 내게 배정된 domain만 예약하고, 기본 domain 및 팀 Gazebo 범위는 사용하지 않는다.
        os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                          GZ_PARTITION=f'kim_schema2_gazebo_{os.getpid()}')
        nodes = ros_test_domain.conflicting_nodes(domain, r'^(?!/ax_test_domain_probe$).*', 2.0)
        services = ros_test_domain.conflicting_services(domain, r'^/ax/', 2.0)
        if nodes or services:
            raise RuntimeError(f'ROS graph conflict: {nodes + services}')
        result.update(domain=domain, gz_partition=os.environ['GZ_PARTITION'])
        safety = Path(get_package_share_directory('ax_safety')) / 'config'
        params = yaml.safe_load((safety / 'safety_limits.yaml').read_text())
        params['/**/b4_risk_node']['ros__parameters']['battery_required'] = True
        params_path = output / 'test_safety.yaml'
        params_path.write_text(yaml.safe_dump(params, allow_unicode=True))
        database = output / 'd3.sqlite3'
        command = ['ros2', 'launch', 'ax_control', 'simulation.launch.py', 'gui:=false',
                   'sensors:=false', 'safety:=true', 'robot_ids:=robot_1,robot_2',
                   f'safety_params_file:={params_path}',
                   f'safety_stop_evidence_files:=robot_1={safety}/stop_evidence_ax1.yaml,'
                   f'robot_2={safety}/stop_evidence_ax2.yaml']
        result['command'] = command
        with isolated_ros_home(lambda: process) as home:
            environment = dict(os.environ, ROS_HOME=home)
            with (output / 'simulation.log').open('w') as log:
                process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', f'database:={database}'])
                monitor = Node('kim_schema2_observer')
                d3 = ObservedD3()
                executor = SingleThreadedExecutor()
                executor.add_node(monitor)
                executor.add_node(d3)
                stats = {robot: {'battery_count': 0, 'battery_latest': {}, 'battery_status_counts': {},
                                'actuation_count': 0, 'schema_versions': [], 'stopped_count': 0,
                                'unsafe_permission_count': 0, 'safety_count': 0}
                         for robot in ('robot_1', 'robot_2')}
                result['robots'] = stats
                handles, parameter_futures = [], {}

                def battery(message, robot):
                    row = stats[robot]
                    row['battery_count'] += 1
                    status = str(message.power_supply_status)
                    row['battery_status_counts'][status] = row['battery_status_counts'].get(status, 0) + 1
                    row['battery_latest'] = {'percentage': message.percentage,
                                             'present': message.present,
                                             'status': message.power_supply_status,
                                             'stamp_sec': message.header.stamp.sec}

                def actuation(message, robot):
                    row = stats[robot]
                    row['actuation_count'] += 1
                    if message.meta.schema_version not in row['schema_versions']:
                        row['schema_versions'].append(message.meta.schema_version)
                    row['stopped_count'] += int(message.meta.valid and message.state == message.STOPPED)
                    row['unsafe_permission_count'] += int(message.drive_allowed or message.arm_allowed)
                    row['block_reasons'] = list(message.block_reasons)
                    row['source_id'] = message.meta.source_id
                    row['actuation_latest'] = {
                        'state': message.state, 'valid': message.meta.valid,
                        'validity_reason': message.meta.validity_reason,
                        'linear_speed': message.linear_speed, 'angular_speed': message.angular_speed,
                        'max_joint_speed': message.max_joint_speed}

                def safety_state(message, robot):
                    stats[robot]['safety_count'] += 1
                    stats[robot]['safety_latest'] = {'valid': message.meta.valid, 'level': message.level}

                for robot in stats:
                    for kind, suffix, callback, qos in (
                            (BatteryState, 'battery_state', battery, qos_profile_sensor_data),
                            (ActuationState, 'safety/actuation_state', actuation, STATE_QOS),
                            (SafetyState, 'safety/state', safety_state, STATE_QOS)):
                        handles.append(monitor.create_subscription(
                            kind, f'/ax/{robot}/{suffix}', lambda msg, r=robot, cb=callback: cb(msg, r), qos))
                    client = monitor.create_client(GetParameters, f'/ax/{robot}/b4_risk_node/get_parameters')
                    handles.append(client)
                    parameter_futures[robot] = [client, None]
                deadline = time.monotonic() + args.seconds
                moved_to_pad = False
                while time.monotonic() < deadline and process.poll() is None:
                    executor.spin_once(timeout_sec=0.05)
                    for robot, pair in parameter_futures.items():
                        client, future = pair
                        if future is None and client.service_is_ready():
                            pair[1] = client.call_async(GetParameters.Request(names=['battery_required']))
                        elif future is not None and future.done() and future.exception() is None:
                            stats[robot]['battery_required'] = future.result().values[0].bool_value
                    if all(row['battery_count'] >= 10 and row['stopped_count'] >= 10
                           and d3.accepted_stops[robot] >= 5 and row.get('battery_required')
                           and row['safety_count'] >= 5 for robot, row in stats.items()):
                        if not args.check_charging:
                            break
                        if not moved_to_pad:
                            # 격리된 Gazebo의 robot_2 위치만 충전 패드로 옮긴다. 실제 주행 허가는 켜지 않는다.
                            catalog = yaml.safe_load((Path(get_package_share_directory('ax_description'))
                                                      / 'config/worlds.yaml').read_text())
                            world = catalog['worlds'][catalog['default']]['gz_world']
                            move = subprocess.run([
                                'gz', 'service', '-s', f'/world/{world}/set_pose',
                                '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                                '--timeout', '5000', '--req',
                                'name: "robot_2" position: {x: -15.0 y: -6.5 z: 0.30} '
                                'orientation: {w: 1.0}'], env=environment,
                                capture_output=True, text=True, timeout=10)
                            result['move_to_pad'] = {'returncode': move.returncode,
                                                     'stdout': move.stdout, 'stderr': move.stderr}
                            moved_to_pad = True
                        charging = stats['robot_2']['battery_status_counts']
                        if charging.get('1', 0) >= 3 or charging.get('4', 0) >= 3:
                            result['charging_observed'] = True
                            break
                for robot, row in stats.items():
                    row['d3_accepted_stop_count'] = d3.accepted_stops[robot]
                    row['battery_subscribers'] = [info.node_name for info in
                        monitor.get_subscriptions_info_by_topic(f'/ax/{robot}/battery_state')]
                    confirmed, stop_deadline = d3.manager.stops.get(robot, (False, 0.0))
                    row['d3_stop_currently_valid'] = confirmed and stop_deadline > d3.manager.clock()
                result['passed'] = all(
                    row['battery_count'] >= 10 and row['stopped_count'] >= 10
                    and row['schema_versions'] == [2] and row['unsafe_permission_count'] == 0
                    and row['d3_accepted_stop_count'] >= 5 and row.get('battery_required')
                    and 'b4_risk_node' in row['battery_subscribers']
                    and row['safety_count'] >= 5 for row in stats.values())
                if args.check_charging:
                    result['passed'] = result['passed'] and result.get('charging_observed', False)
                # 전송 성공과 마지막 순간의 정지 유지 여부를 분리해, 과거 한 번의 STOPPED로 과장하지 않는다.
                result['transport_and_battery_passed'] = result['passed']
                result['both_stopped_at_end'] = all(row['d3_stop_currently_valid'] for row in stats.values())
                result['gazebo_process_exit_before_cleanup'] = process.poll()
                print(json.dumps(result, ensure_ascii=False), flush=True)
    except Exception as error:
        result['error'] = repr(error)
        print(repr(error), flush=True)
    finally:
        if executor is not None:
            executor.shutdown()
        for node in (d3, monitor):
            if node is not None:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_process(process)
        ros_test_domain.release(lock)
        (output / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
