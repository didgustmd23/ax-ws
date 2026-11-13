"""실행 중인 시뮬레이션에 B2 위치 추정(공통 map_server 1개 + 로봇별 AMCL)만 연결한다.
Gazebo·제어기·B1을 새로 띄우지 않는다.

  ros2 launch ax_localization localization.launch.py map:=<지도.yaml> \
      [robot_ids:=robot_1,robot_2] [scan:=filtered|raw] \
      [initial_poses:="robot_2=0,0,0;robot_1=-2.5,1.5,0"] [startup_timeout:=60] [startup_report:=<결과.json>]

- 지도: map_server가 /map(frame map, transient_local)으로 발행하고 모든 AMCL이 같은 지도를 쓴다.
- AMCL: /ax/<robot_id>/amcl. 입력 /ax/<robot_id>/sensors/front_lidar/scan_filtered(B1, scan:=raw면 원시 스캔),
  초기 위치 /ax/<robot_id>/initialpose, 추정 /ax/<robot_id>/amcl_pose, TF map → <robot_id>/odom.
- initial_poses는 로봇별 시험용 초기 위치(map 기준 x, y, yaw)다. 지정하지 않은 로봇은 initialpose 토픽을 기다린다.
  감독 노드에는 최초 초기화 확인 후보로만 넘긴다(감독 노드는 initialpose를 보내지 않는다).
- lifecycle: launch가 map_server와 각 AMCL에 configure → activate를 보낸다(launch_ros LifecycleNode).
  nav2 lifecycle_manager는 쓰지 않는다. 시험에서 configure 응답이 DDS 발견 경합으로 유실되자 시간 제한 없이 멈췄고,
  bond가 끊긴 AMCL에 다시 붙으면 launch 초기 위치로 조용히 재초기화되기 때문이다.
  멈추려면 `ros2 lifecycle set /ax/<robot_id>/amcl deactivate`를 쓴다(자동 재활성화 없음, map→odom 발행 중단).
- 시작 감시(StartupMonitor): map_server와 모든 AMCL이 startup_timeout(벽시계 s) 안에 active가 되어야 한다.
  - 시작 중(모두 active 전): configure·activate 거절, 무응답(제한시간 초과), 노드 프로세스 종료는 실패 처리.
    실패 노드·단계·사유를 로그와 startup_report(지정 시 JSON)에 남긴 뒤 예외로 launch를 끝낸다.
    launch는 자기가 실행한 프로세스만 정리하고 종료 코드 1을 돌려준다(시뮬레이션·B1·다른 노드는 건드리지 않음).
  - 활성화 완료 후: 노드 프로세스 종료는 기록만 한다(exit_code·오류 로그, status active 유지, 재시작 없음).
    launch와 다른 노드는 계속 실행되고 그 로봇의 map→odom만 멈춘다.
- 같은 로봇의 map→odom을 발행하는 SLAM·AMCL이나 다른 map_server가 이미 있으면 시작하지 않는다(TF 발행자는 하나여야 한다).
- 위치 신뢰성(health:=true 기본): 로봇마다 /ax/<robot_id>/localization_supervisor가 localization/health를 내고
  localization/reinitialize(같은 세션의 마지막 신뢰 위치 기반 제한 재위치)를 받는다(scripts/localization_supervisor.py).
- 고정 출발 위치(start_poses:=<yaml>): 지도 revision·이미지 해시가 맞을 때만 감독 노드가 최초 1회 initialpose로 보낸다.
  initial_poses(수동 시험용 AMCL 매개변수)와 같은 로봇에 함께 쓰지 않는다.
- 진단 기록(health_diagnostics:=<폴더>, 선택): 감독 노드가 <폴더>/<robot_id>_supervisor.jsonl에 판정 tick·신뢰 기록
  판단·재위치 요청·보낸 initialpose를 덧붙인다. 시험 기록 전용이며 판정 동작은 바꾸지 않는다.
- 전역 재위치·Nav2 연결은 이 launch 범위가 아니다.
"""

import json
import math
import os
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handler import EventHandler
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState, StateTransition
from lifecycle_msgs.msg import Transition

ROBOT_IDS = ("robot_1", "robot_2", "robot_3", "robot_4")

# 같은 로봇에 map→odom을 낼 수 있는 노드 이름
LOCALIZATION_NODES = ("slam_toolbox", "amcl", "localization_supervisor")


# "robot_2=0,0,0;robot_1=-2.5,1.5,0.1" 형식을 {robot_id: (x, y, yaw)}로 바꾼다. 형식이 틀리면 ValueError.
def parse_initial_poses(text, robot_ids):
    poses = {}

    for item in filter(None, (part.strip() for part in text.split(";"))):
        robot_id, separator, values = item.partition("=")
        robot_id = robot_id.strip()

        if not separator or robot_id not in robot_ids:
            raise ValueError(f"initial_poses 항목 '{item}': robot_ids({', '.join(robot_ids)}) 중 하나=x,y,yaw 형식이어야 함")

        if robot_id in poses:
            raise ValueError(f"initial_poses에 {robot_id}가 두 번 있음")

        numbers = [float(value) for value in values.split(",")]

        if len(numbers) != 3 or not all(math.isfinite(value) for value in numbers):
            raise ValueError(f"initial_poses 항목 '{item}': 유한한 x,y,yaw 세 값이 필요함")

        poses[robot_id] = tuple(numbers)

    return poses


# robot_ids 인자를 검사해 목록으로 돌려준다.
def parse_robot_ids(text):
    robot_ids = [value.strip() for value in text.split(",") if value.strip()]

    if not robot_ids or len(robot_ids) != len(set(robot_ids)) or any(value not in ROBOT_IDS for value in robot_ids):
        raise ValueError(f"robot_ids는 {', '.join(ROBOT_IDS)} 중 중복 없는 하나 이상")

    return robot_ids


# 한 로봇의 AMCL 덮어쓰기 값(프레임·입력 토픽·초기 위치)을 만든다.
def amcl_overrides(robot_id, scan, pose=None):
    namespace = f"/ax/{robot_id}"
    suffix = "scan_filtered" if scan == "filtered" else "scan"
    overrides = {
        "use_sim_time": True,
        "global_frame_id": "map",
        "odom_frame_id": f"{robot_id}/odom",
        "base_frame_id": f"{robot_id}/base_link",
        "scan_topic": f"{namespace}/sensors/front_lidar/{suffix}",
        "map_topic": "/map",
        "tf_broadcast": True,
        "set_initial_pose": pose is not None,
    }

    if pose is not None:
        overrides.update({
            "initial_pose.x": pose[0],
            "initial_pose.y": pose[1],
            "initial_pose.z": 0.0,
            "initial_pose.yaw": pose[2],
        })

    return overrides


# 실행 중인 그래프에서 이미 있는 map→odom 발행 후보(SLAM·AMCL)와 map_server를 찾는다.
# 노드 이름 발견만으로는 늦게 보여 놓칠 수 있어(시험에서 확인) 세 신호를 함께 본다.
# - 노드 이름: 대상 namespace의 slam_toolbox·amcl, 어디든 map_server
# - /map 발행자(endpoint)
# - /tf에 실제로 오는 map → <robot>/odom 변환
# 대상 로봇마다 robot_state_publisher가 보일 때까지(그래프가 보인다는 표시) 최대 wait_seconds 기다리고,
# 그 뒤 settle_seconds 동안 세 신호를 모은다. 시뮬레이션 로봇이 보이지 않으면 시작하지 않는다.
def running_conflicts(robot_ids, wait_seconds=10.0, settle_seconds=3.0):
    import rclpy
    from tf2_msgs.msg import TFMessage

    # launch 인자를 ROS 인자로 해석하지 않도록 빈 인자로 초기화한다.
    rclpy.init(args=[])
    node = rclpy.create_node("ax_localization_preflight")
    namespaces = {f"/ax/{robot_id}" for robot_id in robot_ids}
    odom_frames = {f"{robot_id}/odom" for robot_id in robot_ids}
    conflicts = set()

    # /tf에 이미 들어오는 map → <robot>/odom을 기록한다.
    def on_tf(message):
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id in odom_frames:
                conflicts.add(f"/tf map→{transform.child_frame_id} 발행 중")

    node.create_subscription(TFMessage, "/tf", on_tf, 100)

    try:
        deadline = time.monotonic() + wait_seconds
        settle_until = None
        seen = set()

        # 모든 로봇의 robot_state_publisher가 보인 뒤 settle_seconds 더 모은다.
        while time.monotonic() < (settle_until or deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
            graph = [(name, space.rstrip("/") or "/") for name, space in node.get_node_names_and_namespaces()]
            seen = {space for name, space in graph if name == "robot_state_publisher"}

            if settle_until is None and namespaces <= seen:
                settle_until = time.monotonic() + settle_seconds

            for name, space in graph:
                if space in namespaces and any(key in name for key in LOCALIZATION_NODES):
                    conflicts.add(f"{space}/{name}")
                elif name == "map_server":
                    conflicts.add(f"{space.rstrip('/')}/{name}")

            for info in node.get_publishers_info_by_topic("/map"):
                conflicts.add(f"/map 발행자 {info.node_namespace.rstrip('/')}/{info.node_name}")

        if settle_until is None:
            missing = sorted(namespaces - seen)
            raise RuntimeError(f"실행 중인 시뮬레이션 로봇을 찾지 못함({', '.join(missing)}/robot_state_publisher). "
                               "시뮬레이션을 먼저 실행한다")

        return sorted(conflicts)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 지도 파일의 채택 여부를 안내 문구로 만든다. 채택 기록(map_info.yaml)이 없으면 개발용 지도로 표시한다.
def map_notice(map_path):
    info = map_path.parent / "map_info.yaml"

    if map_path.name == "map.yaml" and info.is_file():
        for line in info.read_text(encoding="utf-8").splitlines():
            if line.startswith("map_revision:"):
                return f"[ax_localization] 채택 지도 {map_path} ({line.strip()})"

    return f"[ax_localization] 개발용 지도 {map_path} (채택 기록 map_info.yaml 없음, 운영 지도 아님)"


# lifecycle 시작 실패를 알리는 예외. launch가 이 예외를 받으면 자기 프로세스를 정리하고 종료 코드 1로 끝난다.
class StartupFailure(RuntimeError):
    pass


# 이번 launch가 시작한 lifecycle 노드들이 제한시간 안에 모두 active가 되는지 감시한다.
# 노드별 상태·단계·전이 기록을 들고 있고, 실패 판단 시 기록(로그·JSON)을 남기고 StartupFailure를 던진다.
class StartupMonitor:
    # labels: 감시할 노드 이름 목록, timeout_s: 전체 제한시간(벽시계 s), report: 결과 JSON 경로(없으면 None)
    def __init__(self, labels, timeout_s, report=None, logger=None):
        self.timeout_s = timeout_s
        self.report = Path(report) if report else None
        self.logger = logger
        self.started = time.monotonic()
        self.status = "starting"
        self.failure = None
        self.nodes = {label: {"state": "unconfigured", "stage": "configure", "transitions": [], "exit_code": None}
                      for label in labels}

    # 시작 뒤 경과 시간(s)
    def elapsed(self):
        return round(time.monotonic() - self.started, 3)

    # 현재 기록을 사전으로 만든다.
    def as_dict(self):
        return {"status": self.status, "timeout_s": self.timeout_s, "elapsed_s": self.elapsed(),
                "failure": self.failure, "nodes": self.nodes}

    # 결과 JSON을 쓴다(지정한 경우만). 같은 폴더의 임시 파일에 쓴 뒤 이름을 바꿔(os.replace)
    # 읽는 쪽이 쓰는 도중의 빈 파일·일부 내용을 보지 않게 한다.
    def write_report(self):
        if self.report is None:
            return

        self.report.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.report.with_name(f".{self.report.name}.{os.getpid()}.tmp")
        temporary.write_text(json.dumps(self.as_dict(), ensure_ascii=False, indent=1), encoding="utf-8")
        os.replace(temporary, self.report)

    # 실패를 확정하고 기록한 뒤 예외로 launch를 끝낸다. 이미 끝난 상태면 아무것도 하지 않는다.
    def fail(self, label, stage, reason):
        if self.status != "starting":
            return []

        self.status = "failed"
        self.failure = {"node": label, "stage": stage, "reason": reason, "elapsed_s": self.elapsed()}
        self.write_report()
        summary = ", ".join(f"{name}={entry['state']}" for name, entry in self.nodes.items())

        raise StartupFailure(f"[ax_localization] 시작 실패: {label or '전체'} {stage} 단계, {reason} "
                             f"(노드 상태: {summary}). 이번 launch의 노드를 정리하고 종료한다")

    # 상태 전이 사건을 반영한다. configure 성공이면 activate 이벤트를, 모두 active면 안내를 돌려준다.
    def on_transition(self, label, start, goal, activate):
        entry = self.nodes[label]
        entry["transitions"].append([self.elapsed(), start, goal])
        entry["state"] = goal

        if self.status != "starting":
            return []

        if goal in ("errorprocessing", "finalized"):
            return self.fail(label, entry["stage"], f"전이 오류({start} → {goal})")

        if start == "configuring" and goal != "inactive":
            return self.fail(label, "configure", f"configure 거절({start} → {goal})")

        if start == "activating" and goal != "active":
            return self.fail(label, "activate", f"activate 실패({start} → {goal})")

        if start == "configuring" and goal == "inactive":
            entry["stage"] = "activate"
            return [activate]

        if goal == "active":
            entry["stage"] = "active"
            actions = [LogInfo(msg=f"[ax_localization] {label} 활성화")]

            if all(node["state"] == "active" for node in self.nodes.values()):
                self.status = "active"
                self.write_report()
                actions.append(LogInfo(msg=f"[ax_localization] 모든 노드 활성화({self.elapsed()} s)"))

            return actions

        return []

    # 프로세스 종료를 반영한다. 시작 중이면 실패, 시작이 끝난 뒤면 오류로 기록만 한다.
    # shutting_down이면(Ctrl+C·다른 실패로 인한 정리) 종료 코드만 남긴다.
    def on_exit(self, label, code, shutting_down=False):
        entry = self.nodes[label]
        entry["exit_code"] = code

        if shutting_down:
            return []

        if self.status == "starting":
            return self.fail(label, entry["stage"], f"시작 중 프로세스 종료(종료 코드 {code})")

        if self.status == "active":
            self.write_report()
            return [LogInfo(msg=f"[ax_localization] 오류: 시작 뒤 {label} 프로세스 종료(종료 코드 {code})")]

        return []

    # 제한시간이 지났을 때 부른다. 아직 시작 중이면 무응답 실패다.
    def on_timeout(self):
        if self.status != "starting":
            return []

        waiting = [f"{name}({entry['state']}, {entry['stage']} 대기)"
                   for name, entry in self.nodes.items() if entry["state"] != "active"]

        return self.fail(", ".join(name for name, entry in self.nodes.items() if entry["state"] != "active"),
                         "timeout", f"{self.timeout_s} s 안에 active가 되지 않음: {', '.join(waiting)}")


# 사건 하나를 받아 콜백 결과(동작 목록)를 돌려주는 처리기. 콜백에서 난 예외는 launch로 전달된다.
class CallbackHandler(EventHandler):
    # matcher: 사건 선택 조건, callback(event, context) → 동작 목록
    def __init__(self, matcher, callback):
        super().__init__(matcher=matcher)
        self.callback = callback

    # 사건을 콜백에 넘긴다.
    def handle(self, event, context):
        return self.callback(event, context)


# lifecycle 노드 하나를 감시 대상으로 묶는다: 노드, 상태 전이 처리기, 프로세스 종료 처리기, configure 이벤트.
def lifecycle_actions(node, label, monitor):
    # 이 노드에 보낼 전이 이벤트
    def change(transition):
        return EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node), transition_id=transition))

    activate = change(Transition.TRANSITION_ACTIVATE)

    return [
        node,
        RegisterEventHandler(CallbackHandler(
            matcher=lambda event: isinstance(event, StateTransition) and event.action is node,
            callback=lambda event, context: monitor.on_transition(label, event.start_state, event.goal_state,
                                                                  activate),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=lambda event, context: monitor.on_exit(label, event.returncode, context.is_shutdown),
        )),
        change(Transition.TRANSITION_CONFIGURE),
    ]


# 노드 목록[(LifecycleNode, 이름)]에 감시를 붙이고 전체 제한시간 타이머를 더한 동작 목록을 만든다.
def startup_actions(nodes, timeout_s, report=None):
    monitor = StartupMonitor([label for _, label in nodes], timeout_s, report)
    actions = []

    for node, label in nodes:
        actions.extend(lifecycle_actions(node, label, monitor))

    actions.append(TimerAction(period=float(timeout_s),
                               actions=[OpaqueFunction(function=lambda context: monitor.on_timeout())]))

    return actions, monitor


# 판단 모듈(localization_health.py)이 있는 폴더. 설치 경로(lib/ax_localization)가 없으면 소스 트리를 쓴다.
def scripts_dir():
    installed = Path(get_package_share_directory("ax_localization")).parents[1] / "lib" / "ax_localization"

    if (installed / "localization_health.py").is_file():
        return installed

    return Path(__file__).resolve().parents[1] / "scripts"


# 고정 출발 위치 설정을 읽어 지도와 대조한다. 형식·값·지도 revision이 맞지 않으면 ValueError로 시작을 거절한다.
def start_poses_for(path, map_path, robot_ids):
    import importlib.util

    spec = importlib.util.spec_from_file_location("localization_health", scripts_dir() / "localization_health.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module.load_start_poses(path, map_path, robot_ids)


# 로봇 하나의 위치 신뢰성 감독 노드(LocalizationHealth 발행·재위치 서비스·고정 출발 위치 최초 초기화)
# expected_initial: initial_poses로 AMCL에 넣은 (x, y, yaw). 감독 노드는 보내지 않고 최초 초기화 확인 후보로만 쓴다.
def supervisor_node(robot_id, health_config, start_pose, diagnostics_dir="", expected_initial=None):
    parameters = {"use_sim_time": True, "robot_id": robot_id, "config_file": str(health_config),
                  "start_pose_enabled": start_pose is not None,
                  "expected_initial_enabled": expected_initial is not None}

    if expected_initial is not None:
        parameters.update({f"expected_initial_{key}": float(value)
                           for key, value in zip(("x", "y", "yaw"), expected_initial)})

    # 진단 기록(선택, 시험 기록 전용): 로봇별 파일에 판단 근거를 덧붙인다. 판정 동작은 바꾸지 않는다.
    if diagnostics_dir:
        parameters["diagnostics_file"] = str(Path(diagnostics_dir).expanduser().resolve()
                                             / f"{robot_id}_supervisor.jsonl")

    if start_pose is not None:
        parameters.update({f"start_pose_{key}": value for key, value in start_pose.items()})

    return Node(package="ax_localization", executable="localization_supervisor.py", name="localization_supervisor",
                namespace=f"/ax/{robot_id}", output="screen", parameters=[parameters])


# 인자를 검사하고 공통 map_server와 로봇별 AMCL(lifecycle 활성화 포함)·위치 신뢰성 감독을 만든다.
def setup(context):
    map_path = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    robot_ids = parse_robot_ids(LaunchConfiguration("robot_ids").perform(context))
    scan = LaunchConfiguration("scan").perform(context)
    poses = parse_initial_poses(LaunchConfiguration("initial_poses").perform(context), robot_ids)
    timeout_s = float(LaunchConfiguration("startup_timeout").perform(context))
    report = LaunchConfiguration("startup_report").perform(context) or None
    start_file = LaunchConfiguration("start_poses").perform(context)
    health_enabled = LaunchConfiguration("health").perform(context).lower() == "true"
    diagnostics_dir = LaunchConfiguration("health_diagnostics", default="").perform(context)
    share = Path(get_package_share_directory("ax_localization"))
    health_config = Path(LaunchConfiguration("health_config").perform(context)
                         or share / "config" / "localization_health.yaml")

    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("startup_timeout은 양수(s)")

    if not map_path.is_file():
        raise FileNotFoundError(f"map 인자의 지도 yaml이 없음: {map_path}")

    if scan not in ("filtered", "raw"):
        raise ValueError("scan은 filtered 또는 raw")

    # 고정 출발 위치: 지도 revision·이미지 해시와 로봇별 값을 검사한다(최초 초기화에만 쓰며 감독 노드가 한 번 보낸다).
    start_poses = {}

    if start_file:
        if not health_enabled:
            raise ValueError("start_poses는 health:=true(위치 신뢰성 감독)와 함께 쓴다")

        overlap = sorted(set(poses) & set(robot_ids))

        if overlap:
            raise ValueError(f"initial_poses와 start_poses를 같은 로봇에 함께 쓸 수 없음: {', '.join(overlap)}")

        start_poses = start_poses_for(start_file, map_path, robot_ids)["robots"]

    if health_enabled and not health_config.is_file():
        raise FileNotFoundError(f"health_config 파일이 없음: {health_config}")

    conflicts = running_conflicts(robot_ids)

    if conflicts:
        raise RuntimeError(f"이미 실행 중인 위치 추정·지도 노드: {', '.join(conflicts)}. 하나만 실행한다")

    config = str(Path(get_package_share_directory("ax_localization")) / "config" / "amcl.yaml")

    # 공통 지도 서버 1개(/map)
    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "yaml_filename": str(map_path),
            "topic_name": "map",
            "frame_id": "map",
        }],
    )
    actions = [LogInfo(msg=map_notice(map_path))]
    nodes = [(map_server, "/map_server")]

    # 로봇별 AMCL
    for robot_id in robot_ids:
        namespace = f"/ax/{robot_id}"
        pose = poses.get(robot_id)
        amcl = LifecycleNode(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            namespace=namespace,
            output="screen",
            parameters=[config, amcl_overrides(robot_id, scan, pose)],
        )

        start = start_poses.get(robot_id)
        text = (f"map ({pose[0]}, {pose[1]}, yaw {pose[2]}) [initial_poses]" if pose else
                f"map ({start['x']}, {start['y']}, yaw {start['yaw']}) [start_poses, 최초 1회]" if start else
                "없음(initialpose 대기)")
        actions.append(LogInfo(msg=f"[ax_localization] {namespace}/amcl 초기 위치 {text}"))
        nodes.append((amcl, f"{namespace}/amcl"))

        if health_enabled:
            actions.append(supervisor_node(robot_id, health_config, start, diagnostics_dir, pose))

    # 시작 감시(전체 제한시간·전이 실패·프로세스 종료)
    monitored, _ = startup_actions(nodes, timeout_s, report)

    return actions + monitored


# launch 인자 선언
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="지도 yaml 경로(map_server yaml_filename)"),
        DeclareLaunchArgument("robot_ids", default_value="robot_2", description="쉼표로 구분한 로봇 ID"),
        DeclareLaunchArgument("scan", default_value="filtered", description="filtered(B1) 또는 raw"),
        DeclareLaunchArgument("initial_poses", default_value="",
                              description="시험용 초기 위치 robot_id=x,y,yaw;... (map 기준 m·rad)"),
        DeclareLaunchArgument("startup_timeout", default_value="60.0",
                              description="map_server·모든 AMCL이 active가 되는 전체 제한시간(벽시계 s)"),
        DeclareLaunchArgument("startup_report", default_value="",
                              description="시작 결과 JSON 경로(비우면 쓰지 않음)"),
        DeclareLaunchArgument("start_poses", default_value="",
                              description="고정 출발 위치 yaml(지도 revision·이미지 해시 대조, 최초 초기화에만 사용)"),
        DeclareLaunchArgument("health", default_value="true",
                              description="로봇별 위치 신뢰성 감독(localization/health·reinitialize) 실행"),
        DeclareLaunchArgument("health_config", default_value="",
                              description="위치 신뢰성 판단 값 yaml(비우면 config/localization_health.yaml)"),
        DeclareLaunchArgument("health_diagnostics", default_value="",
                              description="감독 노드 진단 기록 폴더(시험 기록 전용, 비우면 끔. 판정 동작은 같음)"),
        OpaqueFunction(function=setup),
    ])
