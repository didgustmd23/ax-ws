"""실행 중인 시뮬레이션에 B2 지도 작성(slam_toolbox)만 연결한다. Gazebo·제어기·B1을 새로 띄우지 않는다.

  ros2 launch ax_localization mapping.launch.py [robot_id:=robot_2] [scan:=filtered|raw]

- 기본 입력은 B1 `sensors/front_lidar/scan_filtered`(ax_perception sensors.launch.py가 발행)이고 scan:=raw면 원시 스캔이다.
- 같은 로봇의 map→odom을 발행하는 SLAM·AMCL이 이미 있으면 시작하지 않는다(TF 발행자는 하나여야 한다).
"""

import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

ROBOT_IDS = ("robot_1", "robot_2", "robot_3", "robot_4")

# 같은 로봇에 map→odom을 낼 수 있는 노드 이름
LOCALIZATION_NODES = ("slam_toolbox", "amcl")


# 실행 중인 그래프에서 같은 namespace의 SLAM·AMCL 노드를 찾는다.
# 노드 이름 발견만으로는 늦게 보여 놓칠 수 있어 /tf에 실제로 오는 map → <robot>/odom도 함께 본다.
# 이 로봇의 robot_state_publisher가 보일 때까지(그래프가 보인다는 표시) 최대 wait_seconds 기다린 뒤
# settle_seconds 더 모아 판단한다. 로봇이 보이지 않으면 시작하지 않는다.
def running_localization(namespace, wait_seconds=10.0, settle_seconds=3.0):
    import rclpy
    from tf2_msgs.msg import TFMessage

    # launch 인자(robot_id:=...)를 ROS 인자로 해석하지 않도록 빈 인자로 초기화한다.
    rclpy.init(args=[])
    node = rclpy.create_node("ax_mapping_preflight")
    odom_frame = namespace.removeprefix("/ax/") + "/odom"
    found = set()

    # /tf에 이미 들어오는 이 로봇의 map → odom을 기록한다.
    def on_tf(message):
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == odom_frame:
                found.add(f"/tf map→{odom_frame} 발행 중")

    node.create_subscription(TFMessage, "/tf", on_tf, 100)

    try:
        deadline = time.monotonic() + wait_seconds
        settle_until = None

        while time.monotonic() < (settle_until or deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
            graph = [(name, space.rstrip("/")) for name, space in node.get_node_names_and_namespaces()]

            if settle_until is None and ("robot_state_publisher", namespace) in graph:
                settle_until = time.monotonic() + settle_seconds

            found.update(f"{space}/{name}" for name, space in graph
                         if space == namespace and any(key in name for key in LOCALIZATION_NODES))

        if settle_until is None:
            raise RuntimeError(f"실행 중인 시뮬레이션 로봇을 찾지 못함({namespace}/robot_state_publisher). "
                               "시뮬레이션을 먼저 실행한다")

        return sorted(found)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 인자를 검사하고 slam_toolbox 노드를 만든다. Jazzy slam_toolbox는 lifecycle 노드라 configure → activate를 보낸다.
def setup(context):
    robot_id = LaunchConfiguration("robot_id").perform(context)
    scan = LaunchConfiguration("scan").perform(context)

    if robot_id not in ROBOT_IDS:
        raise ValueError(f"robot_id는 {', '.join(ROBOT_IDS)} 중 하나")

    if scan not in ("filtered", "raw"):
        raise ValueError("scan은 filtered 또는 raw")

    namespace = f"/ax/{robot_id}"
    running = running_localization(namespace)

    if running:
        raise RuntimeError(f"이미 map→odom 발행 후보가 실행 중: {', '.join(running)}. 하나만 실행한다")

    config = get_package_share_directory("ax_localization") + "/config/slam_mapping.yaml"
    suffix = "scan_filtered" if scan == "filtered" else "scan"

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        output="screen",
        parameters=[config, {
            "use_sim_time": True,
            "odom_frame": f"{robot_id}/odom",
            "base_frame": f"{robot_id}/base_link",
            "map_frame": "map",
            "scan_topic": f"{namespace}/sensors/front_lidar/{suffix}",
            # save_map의 map_saver는 노드 namespace의 map 토픽을 구독하므로 같은 이름으로 발행한다.
            "map_name": f"{namespace}/map",
            "use_lifecycle_manager": False,
        }],
    )

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    # configure가 끝나 inactive가 되면 activate한다.
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam,
        start_state="configuring",
        goal_state="inactive",
        entities=[
            LogInfo(msg=f"[ax_localization] {namespace}/slam_toolbox 활성화"),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    ))

    return [slam, activate, configure]


# launch 인자 선언
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="robot_2"),
        DeclareLaunchArgument("scan", default_value="filtered", description="filtered(B1) 또는 raw"),
        OpaqueFunction(function=setup),
    ])
