#!/usr/bin/env python3
"""B2 AMCL 위치 추정 개발 검증 기록·평가. 실행 중인 시뮬레이션·B1·localization.launch.py에 붙어 기록만 한다.

  python3 src/ax_localization/scripts/check_localization.py --robot-ids robot_1,robot_2 \
      --duration 60 --world-map 2.4927,-1.4957,-0.0003 --output <결과 폴더>

기록(로봇별): map→<robot>/odom TF(stamp, 위치), odom, amcl_pose, initialpose 입력, 스캔 수신 시각,
Gazebo 실제 자세(검증 전용, 위치 추정 입력으로 쓰지 않음), /tf 발행 노드 목록, 끝날 때 TF 연결 조회 결과.

평가: 같은 시각의 AMCL 추정 차체 자세(map→odom · odom→base_link)와 실제 자세(T_world_map⁻¹ · world 자세)의
위치·방향 오차. T_world_map은 지도 검증 기록(accuracy.json)의 값을 쓰는 검증 전용 변환이다.
AMCL TF stamp는 처리한 스캔 시각 + transform_tolerance라서 그만큼 빼서 유효 시각으로 쓴다.
정확도 기준이 없으므로 judgment는 not_judged다.

status(검증 가능 여부)와 judgment(정확도 판정)는 따로 본다.
- completed: 요청한 기록 시간을 채웠고, 요청한 모든 로봇이 검증 가능하다(아래 필수 입력·연결이 모두 있음).
- incomplete: 기록 시간 부족 또는 한 로봇이라도 필수 입력·연결이 없다. 로봇별 사유를 problems에 남기고 종료 코드 1.
  확보한 기록·계산 결과는 그대로 남긴다.
- 로봇별 필수: 스캔·odom·map→odom 수신, Gazebo 실제 자세 표본, 오차 계산 표본, map→odom→base_link TF 연결.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from map_tools import interpolate_pose, se2, se2_params, wrap_angle, yaw_from_quaternion  # noqa: E402

# 평가 표본 간격(시뮬레이션 s)과 실제 자세 보간 허용 간격
SAMPLE_PERIOD_S = 0.5
TRUTH_MAX_GAP_S = 0.1


# 표본 배열의 평균·95백분위수·최대(절댓값). 비어 있으면 None
def stats(values):
    values = np.abs(np.asarray(values, float))

    if len(values) == 0:
        return None

    return {"mean": float(values.mean()), "p95": float(np.percentile(values, 95)), "max": float(values.max())}


# 한 로봇의 기록으로 AMCL 추정 자세와 실제 자세의 오차 시계열·요약을 만든다.
# map_odom: (유효 시각, x, y, yaw), odom: (시각, x, y, yaw, v, w), truth: (시각, x, y, yaw) world 기준.
def pose_errors(map_odom, odom, truth, world_map, period_s=SAMPLE_PERIOD_S, max_gap=TRUTH_MAX_GAP_S):
    map_odom = np.asarray(map_odom, float).reshape(-1, 4)
    odom = np.asarray(odom, float).reshape(-1, 6)
    truth = np.asarray(truth, float).reshape(-1, 4)
    map_world = np.linalg.inv(world_map)
    rows = []
    last = -math.inf

    for sample in odom:
        if sample[0] - last < period_s:
            continue

        earlier = map_odom[map_odom[:, 0] <= sample[0]]
        actual = interpolate_pose(truth[:, 0], truth[:, 1:4], sample[0], max_gap) if len(truth) else None

        if len(earlier) == 0 or actual is None:
            continue

        last = sample[0]
        estimate = se2(*earlier[-1][1:4]) @ se2(*sample[1:4])
        expected = map_world @ se2(*actual)
        error = se2_params(np.linalg.inv(expected) @ estimate)
        pose = se2_params(estimate)
        rows.append([sample[0], *pose, *se2_params(expected), math.hypot(*error[:2]), error[2],
                     math.hypot(sample[4], 0.0), abs(sample[5])])

    if not rows:
        return None

    rows = np.array(rows)

    return {
        "samples": int(len(rows)),
        "position_error_m": stats(rows[:, 7]),
        "yaw_error_rad": stats(rows[:, 8]),
        "final": {"sim_time_s": float(rows[-1, 0]), "position_error_m": float(rows[-1, 7]),
                  "yaw_error_rad": float(rows[-1, 8])},
        "series": {"columns": ["sim_time_s", "est_x", "est_y", "est_yaw", "true_x", "true_y", "true_yaw",
                               "position_error_m", "yaw_error_rad", "odom_speed_m_s", "odom_yaw_rate_rad_s"],
                   "rows": np.round(rows, 4).tolist()},
    }


# 시각 구간 안에서 map→odom 변환이 얼마나 바뀌었는지(위치·방향 최대 변화)를 구한다.
def map_odom_change(map_odom, start, end):
    map_odom = np.asarray(map_odom, float).reshape(-1, 4)
    before = map_odom[map_odom[:, 0] <= start]
    window = map_odom[(map_odom[:, 0] > start) & (map_odom[:, 0] <= end)]

    if len(before) == 0 or len(window) == 0:
        return None

    base = before[-1]

    return {
        "samples": int(len(window)),
        "max_position_change_m": float(max(math.hypot(row[1] - base[1], row[2] - base[2]) for row in window)),
        "max_yaw_change_rad": float(max(abs(wrap_angle(row[3] - base[3])) for row in window)),
    }


# 초기 위치 입력이 다른 로봇에 영향을 주는지 판단할 자료를 만든다.
# 각 initialpose 수신 시각 앞뒤로 모든 로봇의 map→odom 변화와 amcl_pose 수신 수를 센다.
def initial_pose_effects(records, window_s=5.0):
    effects = []

    for source, record in records.items():
        for event in record["initialpose"]:
            stamp = event[0]
            entry = {"input_robot": source, "sim_time_s": stamp, "pose": event[1:], "robots": {}}

            for robot_id, other in records.items():
                poses = [row for row in other["amcl_pose"] if stamp < row[0] <= stamp + window_s]
                entry["robots"][robot_id] = {
                    "map_odom_change": map_odom_change(other["map_odom"], stamp, stamp + window_s),
                    "amcl_pose_messages": len(poses),
                    "last_amcl_pose": poses[-1][1:4] if poses else None,
                }

            effects.append(entry)

    return effects


# 메시지 수신 시각 목록으로 수신 주기 요약을 만든다.
def rate_summary(stamps):
    stamps = np.asarray(stamps, float)

    if len(stamps) < 2:
        return {"messages": int(len(stamps)), "rate_hz": None, "max_gap_s": None}

    gaps = np.diff(stamps)

    return {"messages": int(len(stamps)), "rate_hz": float((len(stamps) - 1) / (stamps[-1] - stamps[0])),
            "max_gap_s": float(gaps.max())}


# 여러 로봇의 Gazebo 실제 자세를 기록한다(검증 전용).
class GazeboTruth:
    # 기록할 모델 이름과 월드 이름
    def __init__(self, models, world):
        self.models = set(models)
        self.topic = f"/world/{world}/dynamic_pose/info"
        self.lock = threading.Lock()
        self.samples = {model: [] for model in models}
        self.node = None

    # 자세 토픽을 구독한다.
    def start(self):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        self.node = GzNode()

        if not self.node.subscribe(Pose_V, self.topic, self.on_pose):
            raise RuntimeError(f"Gazebo 자세 구독 실패: {self.topic}")

    # 대상 모델의 (시뮬레이션 시각, x, y, yaw)를 저장한다.
    def on_pose(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9

        for pose in message.pose:
            if pose.name in self.models:
                q = pose.orientation

                with self.lock:
                    self.samples[pose.name].append(
                        (stamp, pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    # 로봇별 기록 사본
    def arrays(self):
        with self.lock:
            return {model: np.array(rows, float).reshape(-1, 4) for model, rows in self.samples.items()}

    # 구독 해제
    def stop(self):
        if self.node is not None:
            self.node.unsubscribe(self.topic)
            self.node = None


# ROS 입력 기록 노드를 만든다(rclpy를 쓰는 부분만 함수 안에서 불러 단위 시험이 ROS 없이 돌게 한다).
def make_recorder(robot_ids, scan, tf_future_s):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener

    # 로봇별 입력을 시각과 함께 쌓는 노드
    class Recorder(Node):
        # 구독을 만든다. 모든 콜백은 같은 잠금으로 기록한다.
        def __init__(self):
            super().__init__("ax_localization_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.lock = threading.Lock()
            self.records = {robot_id: {"map_odom": [], "odom": [], "amcl_pose": [], "initialpose": [], "scan": []}
                            for robot_id in robot_ids}
            self.odom_children = {f"{robot_id}/odom": robot_id for robot_id in robot_ids}
            self.foreign_map_parents = []
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            suffix = "scan_filtered" if scan == "filtered" else "scan"

            for robot_id in robot_ids:
                namespace = f"/ax/{robot_id}"
                self.create_subscription(Odometry, f"{namespace}/diff_drive_controller/odom",
                                         lambda msg, r=robot_id: self.on_odom(r, msg), 50)
                self.create_subscription(PoseWithCovarianceStamped, f"{namespace}/amcl_pose",
                                         lambda msg, r=robot_id: self.on_pose(r, "amcl_pose", msg), latched)
                self.create_subscription(PoseWithCovarianceStamped, f"{namespace}/initialpose",
                                         lambda msg, r=robot_id: self.on_pose(r, "initialpose", msg), 10)
                self.create_subscription(LaserScan, f"{namespace}/sensors/front_lidar/{suffix}",
                                         lambda msg, r=robot_id: self.on_scan(r, msg), qos_profile_sensor_data)

        # 현재 시뮬레이션 시각(s)
        def now_seconds(self):
            return self.get_clock().now().nanoseconds * 1e-9

        # map→<robot>/odom만 골라 유효 시각(stamp − transform_tolerance)으로 기록한다.
        # 다른 부모에서 <robot>/odom으로 가는 변환이 있으면 따로 남긴다(발행자 중복 확인).
        def on_tf(self, message):
            for transform in message.transforms:
                robot_id = self.odom_children.get(transform.child_frame_id)

                if robot_id is None:
                    continue

                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                t, q = transform.transform.translation, transform.transform.rotation

                with self.lock:
                    if transform.header.frame_id == "map":
                        self.records[robot_id]["map_odom"].append(
                            (stamp - tf_future_s, t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))
                    else:
                        self.foreign_map_parents.append((transform.header.frame_id, transform.child_frame_id))

        # odom 자세·속도
        def on_odom(self, robot_id, message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            p, q = message.pose.pose.position, message.pose.pose.orientation
            twist = message.twist.twist

            with self.lock:
                self.records[robot_id]["odom"].append(
                    (stamp, p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), twist.linear.x, twist.angular.z))

        # amcl_pose·initialpose(map 기준 자세와 xy 공분산 대각합)
        def on_pose(self, robot_id, kind, message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            p, q = message.pose.pose.position, message.pose.pose.orientation
            covariance = message.pose.covariance

            with self.lock:
                self.records[robot_id][kind].append(
                    [self.now_seconds() if kind == "initialpose" else stamp, p.x, p.y,
                     yaw_from_quaternion(q.x, q.y, q.z, q.w), covariance[0] + covariance[7], message.header.frame_id])

        # 스캔 수신 시각(stamp)과 frame
        def on_scan(self, robot_id, message):
            with self.lock:
                self.records[robot_id]["scan"].append(
                    (message.header.stamp.sec + message.header.stamp.nanosec * 1e-9, message.header.frame_id))

        # /tf 발행 노드 목록
        def tf_publishers(self):
            return sorted(f"{info.node_namespace.rstrip('/')}/{info.node_name}"
                          for info in self.get_publishers_info_by_topic("/tf"))

        # 로봇별 TF 연결(map→odom→base_link) 조회 결과
        def tf_chain(self):
            from rclpy.time import Time

            result = {}

            for robot_id in robot_ids:
                entry = {}

                for parent, child in (("map", f"{robot_id}/odom"), (f"{robot_id}/odom", f"{robot_id}/base_link"),
                                      ("map", f"{robot_id}/base_link")):
                    try:
                        transform = self.tf_buffer.lookup_transform(parent, child, Time())
                        t = transform.transform.translation
                        entry[f"{parent}->{child}"] = {"ok": True, "x": t.x, "y": t.y}
                    except Exception as error:
                        entry[f"{parent}->{child}"] = {"ok": False, "error": type(error).__name__}

                result[robot_id] = entry

            return result

    return Recorder()


# 한 로봇 결과에서 검증에 필요한 입력·연결이 빠진 사유 목록을 만든다. 비어 있으면 검증 가능하다.
def robot_problems(entry):
    problems = []

    for key, label in (("scan", "스캔"), ("odom", "odom"), ("map_odom", "map→odom")):
        if not entry[key]["messages"]:
            problems.append(f"{label} 수신 없음")

    if not entry["truth_samples"]:
        problems.append("Gazebo 실제 자세 표본 없음")

    if entry["errors"] is None:
        problems.append("오차 계산 표본 없음")

    chain = entry.get("tf_chain") or {}

    if not chain:
        problems.append("TF 연결 조회 결과 없음")

    for name, link in chain.items():
        if not link.get("ok"):
            problems.append(f"TF 연결 실패 {name}({link.get('error', '원인 미기록')})")

    return problems


# 기록 시간 충족과 로봇별 검증 가능 여부로 status를 정한다. 판정(judgment)은 바꾸지 않는다.
# start·end: 실제 기록한 시뮬레이션 시각, duration: 요청 시간, wall_limit: 벽시계 상한
def finalize(result, start, end, duration, wall_limit):
    duration_met = end - start >= duration
    result["recording"] = {"requested_sim_s": duration, "recorded_sim_s": round(end - start, 3),
                           "window_sim_s": [start, end], "duration_met": duration_met}
    problems = []

    if not duration_met:
        problems.append(f"기록 시간 부족: 벽시계 상한 {wall_limit} s 안에 시뮬레이션 {duration} s 중 "
                        f"{end - start:.2f} s만 기록")

    for robot_id, entry in result["robots"].items():
        entry["problems"] = robot_problems(entry)
        entry["verifiable"] = not entry["problems"]
        problems.extend(f"{robot_id}: {problem}" for problem in entry["problems"])

    result["verifiable"] = all(entry["verifiable"] for entry in result["robots"].values()) and bool(result["robots"])
    result["status"] = "completed" if duration_met and result["verifiable"] else "incomplete"
    result["problems"] = problems

    if not result["robots"]:
        problems.append("요청한 로봇 없음")

    return result


# 기록을 로봇별 결과로 정리한다. status는 finalize가 정한다.
def summarize(records, truth, world_map, tf_chain, tf_publishers, foreign, arguments):
    robots = {}

    for robot_id, record in records.items():
        scan_frames = sorted({row[1] for row in record["scan"]})
        poses = record["amcl_pose"]
        robots[robot_id] = {
            "scan": {**rate_summary([row[0] for row in record["scan"]]), "frames": scan_frames},
            "map_odom": rate_summary([row[0] for row in record["map_odom"]]),
            "odom": rate_summary([row[0] for row in record["odom"]]),
            "amcl_pose": {"messages": len(poses), "frames": sorted({row[5] for row in poses}),
                          "last": poses[-1][:5] if poses else None},
            "initialpose_inputs": [row[:5] for row in record["initialpose"]],
            "truth_samples": int(len(truth.get(robot_id, []))),
            "tf_chain": tf_chain.get(robot_id),
            "errors": pose_errors(record["map_odom"], record["odom"], truth.get(robot_id, []), world_map),
        }

    return {
        "status": None,
        "judgment": "not_judged",
        "judgment_reason": "위치 추정 정확도 기준 미확정(수치만 기록)",
        "conditions": {
            "robot_ids": list(records), "scan": arguments.scan, "duration_sim_s": arguments.duration,
            "world_map": {"x_m": arguments.world_map[0], "y_m": arguments.world_map[1],
                          "yaw_rad": arguments.world_map[2], "source": arguments.world_map_source,
                          "note": "검증 전용 변환. 위치 추정 입력으로 쓰지 않음"},
            "tf_future_s": arguments.tf_future,
            "label": arguments.label,
        },
        "tf_publishers": tf_publishers,
        "foreign_odom_parents": sorted(set(foreign)),
        "initial_pose_effects": initial_pose_effects(records),
        "robots": robots,
        "method": "AMCL 추정 = map→odom(유효 시각 ≤ odom 시각 중 마지막) · odom. 실제 = T_world_map⁻¹ · Gazebo 자세(보간)",
    }


# x,y,yaw 문자열을 세 실수로 바꾼다.
def triple(text):
    values = [float(value) for value in text.split(",")]

    if len(values) != 3:
        raise argparse.ArgumentTypeError("x,y,yaw 세 값")

    return values


# 지도 검증 기록(accuracy.json)에서 검증 전용 T_world_map을 읽는다(정렬 보정값 우선).
def world_map_from_accuracy(path):
    transforms = json.loads(Path(path).read_text(encoding="utf-8"))["transforms"]
    key = "optimal_T_world_map" if transforms.get("optimal_T_world_map") else "initial_T_world_map"
    value = transforms[key]

    return [value["x_m"], value["y_m"], value["yaw_rad"]], f"{path}:{key}"


# 인자를 읽고 정해진 시뮬레이션 시간 동안 기록한 뒤 결과를 쓴다.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-ids", default="robot_2")
    parser.add_argument("--scan", choices=["filtered", "raw"], default="filtered")
    parser.add_argument("--duration", type=float, default=30.0, help="기록 시간(시뮬레이션 s)")
    parser.add_argument("--wall-limit", type=float, default=900.0, help="벽시계 상한(s)")
    parser.add_argument("--world-map", type=triple, default=None, help="검증 전용 T_world_map x,y,yaw")
    parser.add_argument("--map-accuracy", type=Path, default=None, help="T_world_map을 읽을 지도 accuracy.json")
    parser.add_argument("--tf-future", type=float, default=1.0, help="AMCL transform_tolerance(s)")
    parser.add_argument("--gz-world", default="ax_test", help="Gazebo 월드 내부 이름(검증 기록용)")
    parser.add_argument("--label", default="")
    parser.add_argument("--output", type=Path, required=True, help="결과 폴더")
    arguments = parser.parse_args()

    if arguments.map_accuracy is not None:
        arguments.world_map, arguments.world_map_source = world_map_from_accuracy(arguments.map_accuracy)
    elif arguments.world_map is not None:
        arguments.world_map_source = "--world-map"
    else:
        parser.error("--world-map 또는 --map-accuracy 필요")

    robot_ids = [value.strip() for value in arguments.robot_ids.split(",") if value.strip()]
    arguments.output.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=[])
    node = make_recorder(robot_ids, arguments.scan, arguments.tf_future)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    truth = GazeboTruth(robot_ids, arguments.gz_world)
    truth.start()

    # 시뮬레이션 시각이 흐르기 시작한 뒤 duration만큼 기록한다.
    wall_deadline = time.monotonic() + arguments.wall_limit

    while node.now_seconds() == 0.0 and time.monotonic() < wall_deadline:
        time.sleep(0.05)

    start = node.now_seconds()
    print(f"[START] 기록 시작 sim {start:.2f} s, {arguments.duration} s 동안", flush=True)

    while node.now_seconds() - start < arguments.duration and time.monotonic() < wall_deadline:
        time.sleep(0.1)

    end = node.now_seconds()
    truth.stop()

    with node.lock:
        records = {robot_id: {key: list(rows) for key, rows in record.items()}
                   for robot_id, record in node.records.items()}
        foreign = list(node.foreign_map_parents)

    result = summarize(records, truth.arrays(), se2(*arguments.world_map), node.tf_chain(),
                       node.tf_publishers(), foreign, arguments)
    result["conditions"]["recorded_sim_s"] = [start, end]
    finalize(result, start, end, arguments.duration, arguments.wall_limit)

    path = arguments.output / "localization_result.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"결과: {path} (status={result['status']}, judgment={result['judgment']})")

    for robot_id, entry in result["robots"].items():
        errors = entry["errors"]
        text = (f"위치 오차 평균 {errors['position_error_m']['mean']:.3f}·최대 {errors['position_error_m']['max']:.3f} m, "
                f"방향 오차 최대 {errors['yaw_error_rad']['max']:.3f} rad, 마지막 {errors['final']['position_error_m']:.3f} m"
                if errors else "오차 계산 표본 없음")
        print(f"[INFO] {robot_id}: 스캔 {entry['scan']['messages']}개, map→odom {entry['map_odom']['messages']}개, {text}")

    for problem in result["problems"]:
        print(f"[INCOMPLETE] {problem}")

    # 콜백 스레드가 끝난 뒤 노드를 없앤다(순서가 바뀌면 Destroyable 예외가 남는다).
    executor.shutdown(timeout_sec=2.0)
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()

    return 0 if result["status"] == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
