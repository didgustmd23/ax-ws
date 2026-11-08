# robot_2 회전 병진 후보 비교 시험(임시 조사 도구). 실행 중인 격리 시뮬레이션(ax_test, robot_2, 접촉 센서 사본)에 붙는다.
# 사용: python3 slip_eval.py <조건 이름> <결과 json>
import json, math, subprocess, sys, threading, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

sys.path.insert(0, '<workspace>/src/ax_control/scripts')
from check_wheel_calibration import rotation_translation, accumulated_rotation   # noqa: E402
from check_tool_attach import spawn_box, remove_model                              # noqa: E402

CONDITION, OUT = sys.argv[1], sys.argv[2]
WORLD, ROBOT = 'ax_test', 'robot_2'
WHEELS = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
CARGO_TOP_ABOVE_BASE = 0.8956
lock = threading.Lock()
truth = {}            # 모델 이름 → [(t, x, y, z, yaw)]
contacts = []         # (t, wheel, points, normal_sum, axle_force_sum)
phase = {'contact': False}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def on_pose(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    with lock:
        for p in msg.pose:
            if p.name == ROBOT or p.name.startswith('load_box'):
                truth.setdefault(p.name, []).append((t, p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))


def contact_handler(wheel):
    def handler(msg):
        if not phase['contact']:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with lock:
            yaw = truth[ROBOT][-1][4] if truth.get(ROBOT) else 0.0
        c_, s_ = math.cos(yaw), math.sin(yaw)
        n = normal = axle = 0.0
        for c in msg.contact:
            for i in range(len(c.position)):
                w = c.wrench[i]
                f = w.body_1_wrench.force if wheel in c.collision1.name else w.body_2_wrench.force
                n += 1; normal += f.z; axle += -s_ * f.x + c_ * f.y
        with lock:
            contacts.append((t, wheel, n, normal, axle))
    return handler


gz = GzNode()
gz.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', on_pose)
for w in WHEELS:
    gz.subscribe(Contacts, f'/world/{WORLD}/model/{ROBOT}/link/{w}/sensor/{w}_contact/contact', contact_handler(w))

rclpy.init()
node = Node('slip_eval', parameter_overrides=[Parameter('use_sim_time', value=True)])
odom = []
node.create_subscription(Odometry, f'/ax/{ROBOT}/diff_drive_controller/odom', lambda m: odom.append((
    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
    yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z)), 50)
pub = node.create_publisher(TwistStamped, f'/ax/{ROBOT}/diff_drive_controller/cmd_vel', 10)
executor = SingleThreadedExecutor(); executor.add_node(node)
spinner = threading.Thread(target=executor.spin, daemon=True); spinner.start()
now = lambda: node.get_clock().now().nanoseconds * 1e-9


def command(linear, angular, seconds):
    end = now() + seconds
    while now() < end:
        m = TwistStamped(); m.header.stamp = node.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.angular.z = float(linear), float(angular); pub.publish(m); time.sleep(0.05)


def window(name, t0, t1):
    with lock:
        return [r for r in truth.get(name, []) if t0 <= r[0] <= t1]


def odom_window(t0, t1):
    return [r for r in odom if t0 <= r[0] <= t1]


def wait_after(t):
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        with lock:
            if truth.get(ROBOT) and truth[ROBOT][-1][0] >= t and odom and odom[-1][0] >= t:
                return
        time.sleep(0.05)


def rotation_trial(direction, seconds=10.0, contact=False):
    command(0, 0, 2.0); t0 = now(); phase['contact'] = contact
    command(0, 0.3 * direction, seconds); phase['contact'] = False
    command(0, 0, 3.0); t1 = now(); wait_after(t1)
    tr = [(r[0], r[1], r[2], r[4]) for r in window(ROBOT, t0, t1)]
    od = [(r[0], r[1], r[2], r[3]) for r in odom_window(t0, t1)]
    result = rotation_translation(tr, od)
    result['truth_turn_rad'] = accumulated_rotation([(r[0], r[3]) for r in tr])
    result['odom_turn_rad'] = accumulated_rotation([(r[0], r[3]) for r in od])
    return result, (t0, t1)


def straight_trial(speed, seconds=3.0):
    command(0, 0, 2.0); t0 = now()
    command(speed, 0, seconds)
    command(0, 0, 3.0); t1 = now(); wait_after(t1)
    tr = window(ROBOT, t0, t1); od = odom_window(t0, t1)
    h = tr[0][4]; c, s = math.cos(h), math.sin(h)
    dx, dy = tr[-1][1] - tr[0][1], tr[-1][2] - tr[0][2]
    oh = od[0][3]; oc, osn = math.cos(oh), math.sin(oh)
    odx, ody = od[-1][1] - od[0][1], od[-1][2] - od[0][2]
    true_along, true_lateral = c * dx + s * dy, -s * dx + c * dy
    odom_along = oc * odx + osn * ody
    return {'truth_distance_m': true_along, 'odom_distance_m': odom_along, 'lateral_m': true_lateral,
            'ratio_odom_over_truth': odom_along / true_along if abs(true_along) > 0.1 else None,
            'slip': 1 - true_along / odom_along if abs(odom_along) > 0.1 else None}


def first_stable(samples, threshold, hold=0.5):
    start = None
    for t, v in samples:
        if abs(v) < threshold:
            start = start if start is not None else t
            if t - start >= hold:
                return start
        else:
            start = None
    return None


def stop_trial(linear, angular, seconds):
    command(0, 0, 2.0)
    command(linear, angular, seconds); cut = now()
    command(0, 0, 4.0); t1 = now(); wait_after(t1)
    od = odom_window(cut - 0.5, t1)
    tr = window(ROBOT, cut - 1.0, t1)
    # 실제 속도: 0.1 s 차분
    tv = []
    for a, b in zip(tr, tr[10:]):
        dt = b[0] - a[0]
        if dt > 0:
            lin = math.hypot(b[1] - a[1], b[2] - a[2]) / dt
            ang = abs(math.atan2(math.sin(b[4] - a[4]), math.cos(b[4] - a[4]))) / dt
            tv.append((b[0], lin, ang))
    rotate = angular != 0
    odom_stop = first_stable([(r[0], r[5] if rotate else r[4]) for r in od if r[0] >= cut], 0.02 if rotate else 0.01)
    true_stop = first_stable([(r[0], r[2] if rotate else r[1]) for r in tv if r[0] >= cut], 0.02 if rotate else 0.01)
    slide = None
    if odom_stop is not None:
        after = [r for r in tr if r[0] >= odom_stop]
        slide = math.hypot(after[-1][1] - after[0][1], after[-1][2] - after[0][2]) if len(after) > 1 else None
    return {'odom_stop_s': odom_stop - cut if odom_stop else None, 'truth_stop_s': true_stop - cut if true_stop else None,
            'slide_after_odom_stop_m': slide}


def load_trial(mass, offset):
    with lock:
        t, x, y, z, yaw = truth[ROBOT][-1]
    base_z = z
    position = [x + offset * math.cos(yaw), y + offset * math.sin(yaw), base_z + CARGO_TOP_ABOVE_BASE + 0.15 + 0.01]
    name = f'load_box_{mass:g}_{"front" if offset > 0 else "rear"}'
    spawn_box(name, position, [0.3, 0.3, 0.3], False, mass)
    command(0, 0, 3.0)
    result, (t0, t1) = rotation_trial(1)
    box = window(name, t0, t1); robot = window(ROBOT, t0, t1)
    def rel(b, r):
        c, s = math.cos(r[4]), math.sin(r[4]); dx, dy = b[1] - r[1], b[2] - r[2]
        return (c * dx + s * dy, -s * dx + c * dy)
    if box and robot:
        a, b = rel(box[0], robot[0]), rel(box[-1], robot[-1])
        result['box_slip_on_cargo_m'] = math.hypot(b[0] - a[0], b[1] - a[1])
        result['box_start_offset_m'] = a
    remove_model(name); command(0, 0, 2.0)
    result.update({'mass_kg': mass, 'offset_m': offset})
    return result


report = {'condition': CONDITION, 'world': WORLD}
time.sleep(3.0); command(0, 0, 3.0)
start_wall, start_sim = time.monotonic(), now()

report['rotation'] = []
for i, d in enumerate((1, -1, 1, -1, 1, -1)):
    r, _ = rotation_trial(d, contact=(i == 0)); r['direction'] = 'left' if d > 0 else 'right'; report['rotation'].append(r)
    print(CONDITION, 'rot', r['direction'], round(r['truth_turn_rad'], 3), round(r['truth_translation_m'], 3), r['rotation_centre_body_m'], flush=True)

report['straight'] = [dict(straight_trial(v), direction='forward' if v > 0 else 'backward') for v in (0.3, -0.3, 0.3, -0.3)]
report['stop'] = {'linear_0.67': stop_trial(0.67, 0, 4.0), 'angular_0.5': stop_trial(0, 0.5, 6.0)}
report['load'] = [load_trial(m, o) for m in (2.0, 5.0) for o in (0.2, -0.2)]

with lock:
    rows = list(contacts)
still = {}
summary = {}
for w in WHEELS:
    rs = [r for r in rows if r[1] == w]
    if rs:
        summary[w] = {'messages': len(rs), 'min_points': min(r[2] for r in rs), 'normal_mean_N': float(np.mean([r[3] for r in rs])),
                      'normal_min_N': float(min(r[3] for r in rs)), 'axle_force_mean_N': float(np.mean([r[4] for r in rs]))}
report['contact_during_first_left_turn'] = summary
report['clock'] = {'sim_s': now() - start_sim, 'wall_s': time.monotonic() - start_wall,
                   'rtf': (now() - start_sim) / (time.monotonic() - start_wall)}

gz.unsubscribe(f'/world/{WORLD}/dynamic_pose/info')
for w in WHEELS:
    gz.unsubscribe(f'/world/{WORLD}/model/{ROBOT}/link/{w}/sensor/{w}_contact/contact')
executor.shutdown(); spinner.join(timeout=3)
json.dump(report, open(OUT, 'w'), indent=1, default=float)
print('saved', OUT, flush=True)
