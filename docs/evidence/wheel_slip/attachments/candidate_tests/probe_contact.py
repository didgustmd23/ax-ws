# 바퀴별 접촉 조사(임시): 정지·왼쪽 회전 중 접촉 개수·위치(바퀴 중심 기준)·수직 하중·마찰력
import json, math, sys, threading, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

WORLD = 'ax_test'; WHEELS = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
OUT = sys.argv[1]
lock = threading.Lock(); phase = {'name': 'settle'}; data = {}; wheel_pose = {}

def on_contact(wheel):
    def handler(msg):
        with lock:
            for c in msg.contact:
                for i, pos in enumerate(c.position):
                    w = c.wrench[i] if i < len(c.wrench) else None
                    # body_1이 바퀴인지 땅인지에 따라 부호가 다르므로 바퀴 쪽 힘을 고른다.
                    force = None
                    if w is not None:
                        b1 = c.collision1.name; f1 = w.body_1_wrench.force; f2 = w.body_2_wrench.force
                        force = (f1.x, f1.y, f1.z) if wheel in b1 else (f2.x, f2.y, f2.z)
                    data.setdefault((phase['name'], wheel), []).append((msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, (pos.x, pos.y, pos.z), force, c.depth[i] if i < len(c.depth) else None))
    return handler

yaw_hist = []
def on_pose(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    for p in msg.pose:
        if p.name == 'robot_2':
            q = p.orientation; yaw_hist.append((t, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), p.position.x, p.position.y))
    for p in msg.pose:
        if p.name in WHEELS or p.name == 'robot_2':
            wheel_pose[p.name] = (p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

gz = GzNode()
for w in WHEELS: gz.subscribe(Contacts, f'/world/{WORLD}/model/robot_2/link/{w}/sensor/{w}_contact/contact', on_contact(w))
gz.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', on_pose)
rclpy.init(); node = Node('probe_contact', parameter_overrides=[Parameter('use_sim_time', value=True)])
pub = node.create_publisher(TwistStamped, '/ax/robot_2/diff_drive_controller/cmd_vel', 10)
ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(node); th = threading.Thread(target=ex.spin, daemon=True); th.start()
now = lambda: node.get_clock().now().nanoseconds * 1e-9

def command(w, seconds):
    end = now() + seconds
    while now() < end:
        m = TwistStamped(); m.header.stamp = node.get_clock().now().to_msg(); m.twist.angular.z = w; pub.publish(m); time.sleep(0.05)

time.sleep(3); command(0.0, 3.0)
phase['name'] = 'still'; command(0.0, 2.0)
phase['name'] = 'turn_ramp'; command(0.3, 3.0)
phase['name'] = 'turning'; command(0.3, 4.0)
phase['name'] = 'end'; command(0.0, 3.0)
for w in WHEELS: gz.unsubscribe(f'/world/{WORLD}/model/robot_2/link/{w}/sensor/{w}_contact/contact')
gz.unsubscribe(f'/world/{WORLD}/dynamic_pose/info')
ex.shutdown(); th.join(timeout=3)

Y = np.array(yaw_hist)
def body(t, f):
    i = min(np.searchsorted(Y[:, 0], t), len(Y) - 1); yaw = Y[i, 1]
    c, s_ = math.cos(yaw), math.sin(yaw)
    return (c*f[0] + s_*f[1], -s_*f[0] + c*f[1])      # (구름 방향, 축 방향)
def body_point(t, p):
    i = min(np.searchsorted(Y[:, 0], t), len(Y) - 1); yaw, x, y = Y[i, 1:4]
    c, s_ = math.cos(yaw), math.sin(yaw); dx, dy = p[0]-x, p[1]-y
    return (c*dx + s_*dy, -s_*dx + c*dy)
report = {}
for (ph, w), rows in sorted(data.items()):
    if ph not in ('still', 'turning'): continue
    forces = np.array([r[2] for r in rows if r[2] is not None]) if any(r[2] is not None for r in rows) else None
    stamps = sorted({r[0] for r in rows})
    per_step = len(rows) / max(len(stamps), 1)
    pos = np.array([r[1] for r in rows])
    report.setdefault(ph, {})[w] = {
        'messages': len(stamps), 'contacts_per_message': round(per_step, 2),
        'contact_z_world_mean': round(float(pos[:, 2].mean()), 4),
        'normal_force_sum_per_step_N': round(float(forces[:, 2].sum() / max(len(stamps), 1)), 2) if forces is not None else None,
        'lateral_force_abs_mean_N': round(float(np.abs(forces[:, :2]).sum(axis=1).mean()), 3) if forces is not None else None,
        'force_rolling_mean_N': round(float(np.mean([body(r[0], r[2])[0] for r in rows if r[2] is not None])), 3),
        'force_axle_mean_N': round(float(np.mean([body(r[0], r[2])[1] for r in rows if r[2] is not None])), 3),
        'contact_points_body_mean_m': [round(float(v), 3) for v in np.mean([body_point(r[0], r[1]) for r in rows], axis=0)],
        'depth_mean_m': round(float(np.mean([r[3] for r in rows if r[3] is not None])), 5) if any(r[3] is not None for r in rows) else None,
    }
json.dump(report, open(OUT, 'w'), indent=1)
print(json.dumps(report, indent=1))
