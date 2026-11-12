# A2 시뮬레이션 launch. 공용 시험 월드에 로봇을 띄우고 A2 제어기와 공구 경로를 활성화한다.
#
# 실행: ros2 launch ax_control simulation.launch.py
#       ros2 launch ax_control simulation.launch.py robot_ids:=robot_1 gui:=false
#
# 구성
# - 월드·모델·센서 브리지: ax_description robot_test.launch.py를 포함한다(A1 소유).
#   이 launch는 controller_config_dir로 ax_control/config의 YAML을 넘겨 모델에 gz_ros2_control을 넣게 한다.
# - 제어기(A2): joint_state_broadcaster, diff_drive_controller, 팔 로봇은 arm_controller
# - 공구(A2, epick·suction 팔 로봇만): 흡착 플러그인(모델에 포함), 공구 브리지, suction_tool_node
#
# 띄우지 않는 것
# - A3 Task Executor, A4 게이트·보호 서버, Nav2 (safety:=true일 때 A4/B4 포함)
# - A3·A4가 없으면 tool_command는 전부 거절된다. 정상 동작이다.
#
# 주의
# - A4 게이트가 없어 제어기를 직접 호출할 수 있다. 운영 경로가 아니다.
# - spawner는 controller_manager가 뜰 때까지 기다린다(최대 120 s). 로봇 스폰이 실패하면 제어기도 활성화되지 않는다.

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# robot_test.launch.py의 시험 역할과 같다. 운영 역할·능력 소유는 D2다.
MANIPULATORS = {"robot_1", "robot_3"}

# 흡착 부착 플러그인이 붙는 공구
ATTACH_TOOLS = {"epick", "suction"}

# robot_test.launch.py에 그대로 넘기는 인자와 기본값
PASS_THROUGH_ARGUMENTS = {
    "world": "planned/ax_warehouse.world",
    "test_fixture_world": "",
    "robots": "2",
    "robot_ids": "",
    "gui": "true",
    "sensors": "true",
    "tool": "epick",
    "payload_mass": "0.0",
    # 기본값 false로 기존 A2 전용 실행 방식을 보존한다.
    "safety": "false",
    "safety_enable_simulation_drive": "false",
    "safety_enable_simulation_arm": "false",
    "safety_operational_permits_verified": "false",
    "safety_travel_envelope_verified": "false",
    "safety_arm_operation_verified": "false",
}


# robots·robot_ids 인자를 robot_test.launch.py와 같은 규칙으로 로봇 ID 목록으로 바꾼다.
def selected_robot_ids(robots, robot_ids):
    if robot_ids:
        return [value.strip() for value in robot_ids.split(",")]

    return [f"robot_{index + 1}" for index in range(int(robots))]


# 한 로봇의 제어기 spawner를 만든다. odom TF 접두사는 로봇별 값이라 diff_drive만 따로 띄워 넘긴다.
def controller_spawners(robot_id):
    namespace = f"/ax/{robot_id}"
    common_arguments = [
        "--controller-manager", f"{namespace}/controller_manager",
        "--controller-manager-timeout", "120",
        "--switch-timeout", "60",
        # GUI·소프트웨어 렌더링 중 느린 응답을 10초마다 재전송하지 않는다.
        "--service-call-timeout", "60",
    ]

    # 상태 발행과 팔 제어기
    state_controllers = ["joint_state_broadcaster"]

    if robot_id in MANIPULATORS:
        state_controllers.append("arm_controller")

    state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=state_controllers + common_arguments,
        output="screen",
    )

    # 값이 -p로 시작해 인자 파서가 옵션으로 읽지 않도록 등호로 붙인다.
    drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "diff_drive_controller",
            *common_arguments,
            f"--controller-ros-args=-p tf_frame_prefix:={robot_id}/",
        ],
        output="screen",
    )

    return [state_spawner, drive_spawner]


# 팔 로봇의 공구 노드와 플러그인 사이 브리지를 만든다. ]는 ROS→Gazebo, [는 Gazebo→ROS다.
def tool_nodes(robot_id):
    namespace = f"/ax/{robot_id}"

    tool_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tool_bridge",
        namespace=namespace,
        arguments=[
            f"{namespace}/tool/gz_command@std_msgs/msg/String]gz.msgs.StringMsg",
            f"{namespace}/tool/gz_state@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
        output="screen",
    )

    # tool_command 서버이자 attach_state 발행자. A3·A4가 없으면 모든 명령을 거절한다.
    tool_node = Node(
        package="ax_control",
        executable="suction_tool_node.py",
        name="suction_tool_node",
        namespace=namespace,
        parameters=[{
            "robot_id": robot_id,
            "use_sim_time": True,
        }],
        output="screen",
    )

    return [tool_bridge, tool_node]


# launch 인자를 실제 값으로 풀어 월드 포함·제어기·공구 노드 목록을 만든다.
def launch_setup(context):
    values = {
        name: LaunchConfiguration(name).perform(context)
        for name in PASS_THROUGH_ARGUMENTS
    }
    tool_attach = values["tool"] in ATTACH_TOOLS

    # Gazebo가 ax_control의 흡착 플러그인 라이브러리를 찾도록 경로를 더한다.
    plugin_path = os.pathsep.join(filter(None, [
        str(Path(get_package_prefix("ax_control")) / "lib"),
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    ]))

    # 월드·모델·센서 브리지는 A1 launch가 띄우고, 제어기 설정은 ax_control/config를 쓴다.
    robot_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("ax_description")) / "launch" / "robot_test.launch.py"
        )),
        launch_arguments={
            **values,
            "controller_config_dir": str(Path(get_package_share_directory("ax_control")) / "config"),
            "tool_attach": "true" if tool_attach else "false",
        }.items(),
    )

    actions = [
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        robot_test,
    ]

    # 로봇마다 제어기와 공구 노드
    for robot_id in selected_robot_ids(values["robots"], values["robot_ids"]):
        actions.extend(controller_spawners(robot_id))

        # W1 안전 실행은 명시적으로 요청한 경우에만 로봇별로 붙인다.
        # safety.launch.py의 허용 플래그 기본값(false)은 그대로 유지한다.
        if values["safety"].lower() == "true":
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    Path(get_package_share_directory("ax_safety")) /
                    "launch" / "safety.launch.py"
                )),
                launch_arguments={
                    "robot_id": robot_id,
                    "enable_simulation_drive": values["safety_enable_simulation_drive"],
                    "enable_simulation_arm": values["safety_enable_simulation_arm"],
                    "operational_permits_verified": values["safety_operational_permits_verified"],
                    "travel_envelope_verified": values["safety_travel_envelope_verified"],
                    "arm_operation_verified": values["safety_arm_operation_verified"],
                }.items(),
            ))

        if robot_id in MANIPULATORS and tool_attach:
            actions.extend(tool_nodes(robot_id))

    return actions


# ROS 2 launch가 호출하는 진입점
def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in PASS_THROUGH_ARGUMENTS.items()
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
