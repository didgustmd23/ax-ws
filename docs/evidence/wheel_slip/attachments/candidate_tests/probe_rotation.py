# robot_2 제자리 회전 중 병진 조사(임시). 1) 모델 원점-base_link 관계 2) 시각 맞춘 실제 이동-odom 비교
# 3) 좌·우 반복 회전의 회전 중심·병진 4) 네 바퀴 회전 방향·속도·바퀴 중심 높이
import json, math, sys, threading, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
sys.path.insert(0, '<workspace>/src/ax_localization/scripts')
import map_tools as mt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.empty_pb2 import Empty
from gz.transport13 import Node as GzNode

ROBOT, WORLD = 'robot_2', sys.argv[1] if len(sys.argv) > 1 else 'ax_test'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/rotation.json'
WHEELS = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
lock = threading.Lock()
truth, links = [], {w: [] for w in WHEELS + ['base_link']}

def q2m(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def pose4(p):
    T = np.eye(4); T[:3, :3] = q2m(p.orientation); T[:3, 3] = [p.position.x, p.position.y, p.position.z]; return T

def on_pose(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    model = None; rel = {}
    for p in msg.pose:
        if p.name == ROBOT: model = pose4(p)
        elif p.name in links: rel[p.name] = pose4(p)
    if model is None: return
    with lock:
        truth.append((t, model))
        for k, v in rel.items(): links[k].append((t, model @ v, v))

gz = GzNode(); gz.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', on_pose)

# 1) scene/info: base_link의 모델 기준 자세
ok, scene = gz.request(f'/world/{WORLD}/scene/info', Empty(), Empty, Scene, 5000)
result = {'world': WORLD}
for m in scene.model:
    if m.name == ROBOT:
        result['scene_model_pose'] = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
        for l in m.link:
            if l.name in ('base_link',) + tuple(WHEELS):
                result.setdefault('scene_links_in_model', {})[l.name] = [round(l.pose.position.x, 4), round(l.pose.position.y, 4), round(l.pose.position.z, 4)]

rclpy.init(); node = Node('probe_rotation', parameter_overrides=[Parameter('use_sim_time', value=True)])
odom, joints = [], []
node.create_subscription(Odometry, f'/ax/{ROBOT}/diff_drive_controller/odom', lambda m: odom.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, m.pose.pose.position.x, m.pose.pose.position.y, mt.yaw_from_quaternion(m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w), m.twist.twist.linear.x, m.twist.twist.angular.z)), 50)
node.create_subscription(JointState, f'/ax/{ROBOT}/joint_states', lambda m: joints.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, dict(zip(m.name, m.velocity)))), 50)
pub = node.create_publisher(TwistStamped, f'/ax/{ROBOT}/diff_drive_controller/cmd_vel', 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
now = lambda: node.get_clock().now().nanoseconds * 1e-9

def command(w, seconds):
    end = now() + seconds; start = now()
    while now() < end:
        m = TwistStamped(); m.header.stamp = node.get_clock().now().to_msg(); m.twist.angular.z = w; pub.publish(m); time.sleep(0.05)

time.sleep(3.0)
while now() < 1.0: time.sleep(0.1)

trials = []
for direction in ((1, -1) if len(sys.argv) > 3 else (1, -1, 1, -1, 1, -1)):
    command(0.0, 3.0); t0 = now()
    command(0.3 * direction, 10.0)       # 0.3 rad/s, 가속 0.2 rad/s² → 약 2.7 rad
    command(0.0, 4.0); t1 = now()
    trials.append((direction, t0, t1))

time.sleep(1.0)
with lock:
    tr = sorted(truth, key=lambda x: x[0]); lk = {k: list(v) for k, v in links.items()}
gz.unsubscribe(f'/world/{WORLD}/dynamic_pose/info')

T = np.array([x[0] for x in tr]); P = np.array([[x[1][0, 3], x[1][1, 3], math.atan2(x[1][1, 0], x[1][0, 0])] for x in tr])
od = np.array(odom); radius = 0.149900663548
summary = []
for direction, t0, t1 in trials:
    a = mt.interpolate_pose(T, P, t0, 0.1); b = mt.interpolate_pose(T, P, t1, 0.1)
    i0, i1 = np.searchsorted(od[:, 0], t0), np.searchsorted(od[:, 0], t1) - 1
    oa, ob = od[i0], od[i1]
    # 2) 같은 시각(odom 표본 시각)의 실제 자세와 비교
    pa = mt.interpolate_pose(T, P, oa[0], 0.1); pb = mt.interpolate_pose(T, P, ob[0], 0.1)
    true_move = math.hypot(pb[0] - pa[0], pb[1] - pa[1]); odom_move = math.hypot(ob[1] - oa[1], ob[2] - oa[2])
    true_turn = mt.wrap_angle(pb[2] - pa[2]); odom_turn = mt.wrap_angle(ob[3] - oa[3])
    # 3) 회전 중심(실제 궤적 원 맞춤) → 시작 차체 좌표계
    sel = (T >= t0) & (T <= t1); x, y = P[sel, 0], P[sel, 1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]; cx, cy, c = np.linalg.lstsq(A, x**2 + y**2, rcond=None)[0]
    centre_body = mt.transform_points(np.linalg.inv(mt.se2(*pa)), [[cx, cy]])[0]
    # 4) 바퀴 속도·높이
    js = [j for j in joints if t0 + 3 < j[0] < t0 + 9]
    speeds = {w: round(float(np.mean([j[1].get(w + '_joint', np.nan) for j in js])), 3) for w in WHEELS}
    heights = {}
    for w in WHEELS:
        z = np.array([v[1][2, 3] for v in lk[w] if t0 < v[0] < t1])
        heights[w] = [round(float(z.min()), 4), round(float(z.max()), 4)] if len(z) else None
    summary.append({'direction': 'left' if direction > 0 else 'right', 'true_turn_rad': round(true_turn, 3), 'odom_turn_rad': round(odom_turn, 3),
                    'true_translation_m': round(true_move, 3), 'odom_translation_m': round(odom_move, 3),
                    'centre_in_start_body_m': [round(float(v), 3) for v in centre_body], 'centre_radius_m': round(float(np.sqrt(c + cx*cx + cy*cy)), 3),
                    'wheel_speed_rad_s': speeds, 'wheel_centre_z_world_m': heights})
base_rel = [v[2][:3, 3] for v in lk['base_link']]
result['dynamic_base_link_in_model'] = np.round(np.mean(base_rel, axis=0), 5).tolist() if base_rel else 'base_link 링크 자세가 dynamic_pose에 없음'
result['trials'] = summary
json.dump(result, open(OUT, 'w'), indent=1, ensure_ascii=False)
print(json.dumps(result, indent=1, ensure_ascii=False))
