#!/usr/bin/env python3
"""Run the existing simulation in an isolated domain; preserve raw evidence.

Usage: source ROS/workspace setup, then python3 tools/verify_handoff.py.
Does not modify the robot, install packages, or terminate other simulations.
"""
import concurrent.futures
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
STAMP = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S')
OUT = ROOT / 'docs' / 'evidence' / STAMP
OUT.mkdir(parents=True, exist_ok=False)
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['GZ_PARTITION'] = f'ax_handoff_{os.getpid()}'
RESULT = {'date_kst': STAMP, 'ros_domain_id': 87,
          'gz_partition': os.environ['GZ_PARTITION'], 'checks': {}}


def run(name, args, timeout=20):
    try:
        p = subprocess.run(args, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=timeout)
        text = p.stdout
        rc = p.returncode
    except subprocess.TimeoutExpired as e:
        raw = e.stdout or b''
        text = raw.decode(errors='replace') if isinstance(raw, bytes) else raw
        rc = 124
    (OUT / f'{name}.txt').write_text(text, encoding='utf-8')
    return rc, text


def snapshot(name):
    rc, raw = run(name, ['gz', 'topic', '-e', '-n', '1', '--json-output',
                         '-t', '/world/empty/pose/info'], 15)
    if rc:
        return {}
    try:
        obj, _ = json.JSONDecoder().raw_decode(raw[raw.index('{'):])
        return {p['name'].split('::')[-1]: p for p in obj.get('pose', [])}
    except (ValueError, KeyError):
        return {}


def quat(p):
    q = p.get('orientation', {})
    return [float(q.get(k, 0 if k != 'w' else 1)) for k in ('w', 'x', 'y', 'z')]


def mul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def relative(poses, parent, child):
    a, b = poses[parent], poses[child]
    qa = quat(a)
    inv = [qa[0], -qa[1], -qa[2], -qa[3]]
    delta = [float(b.get('position', {}).get(k, 0))-float(a.get('position', {}).get(k, 0))
             for k in ('x', 'y', 'z')]
    return mul(inv, quat(b)), mul(mul(inv, [0]+delta), qa)[1:]


launch = None
node = None
log = None
from ament_index_python.packages import get_package_prefix, PackageNotFoundError
dependencies = {}
for manifest in sorted((ROOT / 'src').glob('*/package.xml')):
    package = ET.parse(manifest).getroot()
    names = sorted({d.text for tag in ('depend', 'exec_depend', 'buildtool_depend')
                    for d in package.findall(tag)})
    dependencies[package.findtext('name')] = {}
    for name in names:
        try:
            prefix = get_package_prefix(name)
        except PackageNotFoundError:
            prefix = None
        dependencies[package.findtext('name')][name] = prefix
RESULT['checks']['declared_dependency_prefixes'] = dependencies
if '--dependencies-only' in sys.argv:
    (OUT / 'results.json').write_text(json.dumps(RESULT, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(RESULT, ensure_ascii=False, indent=2))
    print('Evidence:', OUT)
    sys.exit(0 if all(v for deps in dependencies.values() for v in deps.values()) else 1)
try:
    rc, _ = run('xacro', ['xacro', str(ROOT / 'src/ax_description/urdf/ax.urdf.xacro'),
                          '-o', str(OUT / 'ax.urdf')])
    RESULT['checks']['xacro'] = {'returncode': rc}
    if rc:
        raise RuntimeError('Xacro conversion failed')
    rc, _ = run('check_urdf', ['check_urdf', str(OUT / 'ax.urdf')])
    RESULT['checks']['check_urdf'] = {'returncode': rc}
    tree = ET.parse(OUT / 'ax.urdf').getroot()
    RESULT['links'] = len(tree.findall('link'))
    RESULT['joints'] = len(tree.findall('joint'))
    log = (OUT / 'simulation.log').open('w', encoding='utf-8')
    launch = subprocess.Popen(['ros2', 'launch', 'ax_control', 'simulation.launch.py'],
                              cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float64
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy
    rclpy.init()
    node = rclpy.create_node('handoff_verifier')
    scans, static_frames = [], set()

    def scan_cb(msg):
        if len(scans) < 300:
            scans.append({'received_monotonic': time.monotonic(),
                          'frame_id': msg.header.frame_id, 'count': len(msg.ranges),
                          'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
                          'range_min': msg.range_min, 'range_max': msg.range_max,
                          'finite_ranges': sum(math.isfinite(v) for v in msg.ranges)})

    subscriptions = [node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data),
                     node.create_subscription(TFMessage, '/tf_static',
                         lambda m: static_frames.update(t.child_frame_id for t in m.transforms),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    # Gazebo merges the fixed support into base_link and the fixed upper mount
    # into column_link. Their fixed-origin rotations are identity.
    channels = [('column_yaw', 'column_yaw_joint', 'base_link', 'column_link', 0.3),
                ('shoulder', 'shoulder_joint', 'column_link', 'upper_arm_link', -0.4),
                ('elbow', 'elbow_joint', 'upper_arm_link', 'forearm_link', 0.5),
                ('wrist', 'wrist_joint', 'forearm_link', 'wrist_link', 0.3),
                ('gripper_rotation', 'gripper_rotation_joint', 'wrist_link', 'gripper_base_link', 0.3),
                ('left_finger', 'left_finger_joint', 'gripper_base_link', 'left_finger_link', 0.015),
                ('right_finger', 'right_finger_joint', 'gripper_base_link', 'right_finger_link', 0.015)]
    pubs = {c: node.create_publisher(Float64, f'/{c}/command', 10) for c, *_ in channels}
    pubs['gripper'] = node.create_publisher(Float64, '/gripper/command', 10)
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        spin(1)
        if all(p.get_subscription_count() for p in pubs.values()):
            break
        if launch.poll() is not None:
            raise RuntimeError('Simulation launch exited during startup')
    RESULT['checks']['ros_subscribers'] = {c: p.get_subscription_count() for c, p in pubs.items()}
    for c, *_ in channels:
        pubs[c].publish(Float64(data=0.0))
    spin(5)
    before = snapshot('poses_before')
    with concurrent.futures.ThreadPoolExecutor(max_workers=7) as pool:
        receivers = {c: pool.submit(run, 'forward_' + c,
                     ['gz', 'topic', '-e', '-n', '1', '-t', f'/model/ax/{j}/position'], 15)
                     for c, j, *_ in channels}
        spin(2)
        for c, j, parent, child, target in channels:
            if c not in ('left_finger', 'right_finger'):
                pubs[c].publish(Float64(data=target))
        pubs['gripper'].publish(Float64(data=0.015))
        spin(6)
        RESULT['checks']['command_forwarding'] = {c: {'returncode': f.result()[0],
              'message': f.result()[1].strip()} for c, f in receivers.items()}
    after = snapshot('poses_after')
    motion = {}
    for c, j, parent, child, target in channels:
        if not all(n in poses for poses in (before, after) for n in (parent, child)):
            motion[j] = {'status': 'unverified', 'reason': 'parent/child poses unavailable'}
            continue
        qa, pa = relative(before, parent, child)
        qb, pb = relative(after, parent, child)
        dot = abs(sum(a*b for a, b in zip(qa, qb)))
        angle = 2*math.acos(min(1.0, dot))
        distance = math.sqrt(sum((a-b)**2 for a, b in zip(pa, pb)))
        moved = distance > 0.001 if 'finger' in c else angle > 0.01
        motion[j] = {'status': 'motion_observed' if moved else 'no_motion_observed',
                     'target': target, 'relative_angle_change_rad': angle,
                     'relative_position_change_m': distance,
                     'before_relative_position': pa, 'after_relative_position': pb,
                     'before_relative_quaternion_wxyz': qa, 'after_relative_quaternion_wxyz': qb}
        if 'finger' in c:
            actual = pb[1]-0.025 if c == 'left_finger' else -pb[1]-0.025
            unit, tolerance = 'm', 0.003
        else:
            component = 3 if c == 'column_yaw' else 1
            actual = 2*math.atan2(qb[component], qb[0])
            actual = (actual + math.pi) % (2*math.pi) - math.pi
            unit, tolerance = 'rad', 0.1
        motion[j].update({'measured_position': actual, 'unit': unit,
                          'absolute_target_error': abs(actual-target),
                          'review_tolerance': tolerance,
                          'within_review_tolerance': abs(actual-target) <= tolerance})
    RESULT['checks']['motion'] = motion
    spin(3)
    elapsed = scans[-1]['received_monotonic']-scans[0]['received_monotonic'] if len(scans)>1 else 0
    RESULT['checks']['scan'] = {'received': len(scans), 'observed_hz': (len(scans)-1)/elapsed if elapsed else None,
                                'first': scans[0] if scans else None}
    RESULT['checks']['tf_static'] = sorted(static_frames)
    RESULT['checks']['ros_topics'] = node.get_topic_names_and_types()
    RESULT['checks']['ros_publisher_counts'] = {
        topic: node.count_publishers(topic) for topic in ('/joint_states', '/clock', '/odom', '/scan')}
    run('gz_topics', ['gz', 'topic', '-l'])
    run('ros_nodes', ['ros2', 'node', 'list', '--no-daemon'])
    RESULT['launch_alive_at_end'] = launch.poll() is None
except Exception as e:
    RESULT['error'] = repr(e)
finally:
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    if launch is not None and launch.poll() is None:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGTERM)
            launch.wait(timeout=10)
    if log is not None:
        log.close()
    (OUT / 'results.json').write_text(json.dumps(RESULT, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(RESULT, ensure_ascii=False, indent=2), flush=True)
    print('Evidence:', OUT, flush=True)

checks = RESULT['checks']
complete = ('error' not in RESULT and RESULT.get('launch_alive_at_end')
            and checks.get('check_urdf', {}).get('returncode') == 0
            and checks.get('scan', {}).get('received', 0) > 0
            and len(checks.get('motion', {})) == 7
            and all(v.get('status') == 'motion_observed' for v in checks.get('motion', {}).values()))
if not complete:
    sys.exit(1)
# A distinct exit code keeps motion success separate from position accuracy.
sys.exit(0 if all(v['within_review_tolerance'] for v in checks['motion'].values()) else 2)
