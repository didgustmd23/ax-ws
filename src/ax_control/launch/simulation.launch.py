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
# - safety:=false(기본)이면 A4 게이트가 없어 제어기를 직접 호출할 수 있다. 운영 경로가 아니다.
# - spawner는 controller_manager가 뜰 때까지 기다린다(최대 120 s). 로봇 스폰이 실패하면 제어기도 활성화되지 않는다.
#
# A4/B4 설정 파일(safety:=true일 때만, 로봇별로 safety.launch.py에 전달)
# - safety_params_file: 로봇 공통 파일. 비우면 선택된 ax_safety 설치본의 config/safety_limits.yaml
# - safety_stop_evidence_files: 로봇별 정지 판정 역할 파일. 예) robot_1=<share>/config/stop_evidence_ax1.yaml,robot_2=<...ax2.yaml>
# - safety_robot_params_files: 로봇별 주행 자세·팔 한계 파일. 같은 robot_<n>=<경로>,... 형식
#   매핑에 없는 로봇은 빈 값(역할 미정·잠금)으로 넘긴다. robot_id로 파일을 추측하지 않는다.
#   파일 존재만 확인하며 내용이 그 로봇 모델에 맞는지는 호출자 책임이다.
# - 하위 단일 로봇 인자(params_file·stop_evidence_file·robot_params_file·허가 플래그)를 이 진입점에 직접
#   주면 여러 로봇에 같은 값이 섞이므로 거부한다. 단일 로봇은 safety.launch.py를 직접 실행할 때 그 인자를 쓴다.
#   이 이름을 launch configuration으로 남기는 상위 launch(예: 다른 include)에서 포함해도 거부된다.

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# robot_test.launch.py의 시험 역할과 같다. 운영 역할·능력 소유는 D2다.
MANIPULATORS = {"robot_1", "robot_3"}

# 흡착 부착 플러그인이 붙는 공구
ATTACH_TOOLS = {"epick", "suction"}

# robot_test.launch.py(A1)에 그대로 넘기는 월드·모델 인자와 기본값
MODEL_ARGUMENTS = {
    "world": "planned/ax_warehouse.world",
    "test_fixture_world": "",
    "robots": "2",
    "robot_ids": "",
    "gui": "true",
    "sensors": "true",
    "tool": "epick",
    "payload_mass": "0.0",
}

# A4/B4 safety.launch.py에만 쓰는 인자. A1에는 넘기지 않는다.
# 기본값 false로 기존 A2 전용 실행 방식을 보존한다.
SAFETY_ENABLE_ARGUMENT = {"safety": "false"}

# 상위 허가 인자 → safety.launch.py 인자. 기본 false이며 파일 값보다 나중에 적용된다.
SAFETY_FLAG_ARGUMENTS = {
    "safety_enable_simulation_drive": "enable_simulation_drive",
    "safety_enable_simulation_arm": "enable_simulation_arm",
    "safety_operational_permits_verified": "operational_permits_verified",
    "safety_travel_envelope_verified": "travel_envelope_verified",
    "safety_arm_operation_verified": "arm_operation_verified",
}

# 상위 파일 인자. 공통 파일 하나와 로봇별 매핑 두 개
SAFETY_FILE_ARGUMENTS = (
    "safety_params_file",
    "safety_stop_evidence_files",
    "safety_robot_params_files",
)

# 이 진입점에서 받지 않는 하위 단일 로봇 인자(전역 별칭). 주면 모든 로봇에 같은 값이 섞인다.
REJECTED_SAFETY_ALIASES = (
    "params_file",
    "stop_evidence_file",
    "robot_params_file",
    *SAFETY_FLAG_ARGUMENTS.values(),
)

LAUNCH_ARGUMENTS = {
    **MODEL_ARGUMENTS,
    **SAFETY_ENABLE_ARGUMENT,
    **{name: "false" for name in SAFETY_FLAG_ARGUMENTS},
    **{name: "" for name in SAFETY_FILE_ARGUMENTS},
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


# 하위 단일 로봇 인자가 상위 context에 있으면(CLI 등) 거부한다. 조용히 무시하거나 모든 로봇에 섞지 않는다.
def reject_safety_aliases(context):
    given = [name for name in REJECTED_SAFETY_ALIASES if name in context.launch_configurations]

    if given:
        raise ValueError(
            f"simulation.launch.py는 하위 단일 로봇 인자를 받지 않는다: {', '.join(given)}. "
            "safety_params_file, safety_stop_evidence_files:=robot_<n>=<경로>,..., "
            "safety_robot_params_files, safety_<허가 플래그>를 쓴다"
        )


# "robot_1=/a.yaml,robot_2=/b.yaml"을 {robot_id: 절대 경로}로 바꾼다.
# 쉼표로 항목을 나누고 각 항목의 첫 '='만 구분자로 본다. 항목 앞뒤 공백만 지우고 경로 내부 공백은 보존한다.
# 쉼표가 든 경로는 지원하지 않는다(형식 오류로 거부된다).
def robot_file_mapping(argument, text, robot_ids):
    if not text.strip():
        return {}

    mapping = {}

    for item in text.split(","):
        robot_id, separator, path = item.partition("=")
        robot_id = robot_id.strip()
        path = path.strip()

        if not separator or not robot_id or not path:
            raise ValueError(
                f"{argument}: 'robot_<n>=<파일>' 형식이 아니다: {item!r} "
                "(쉼표로 구분, 경로에 쉼표는 쓸 수 없다)"
            )

        if robot_id not in robot_ids:
            raise ValueError(f"{argument}: 선택되지 않은 로봇 {robot_id!r} (선택: {', '.join(robot_ids)})")

        if robot_id in mapping:
            raise ValueError(f"{argument}: 로봇 {robot_id} 중복")

        mapping[robot_id] = path

    # 형식을 모두 확인한 뒤 파일을 본다. 쉼표가 든 경로가 '없는 파일'이 아니라 형식 오류로 드러나게 한다.
    for robot_id, path in mapping.items():
        if not Path(path).is_file():
            raise ValueError(f"{argument}: {robot_id} 파일 없음: {path}")

    return {
        robot_id: str(Path(path).absolute())
        for robot_id, path in mapping.items()
    }


# 공통 파일 경로. 비우면 선택된 ax_safety 설치본의 기본 파일을 쓴다. 빈 값을 하위로 넘기지 않는다.
def common_safety_file(text):
    path = text.strip()

    if not path:
        path = str(Path(get_package_share_directory("ax_safety")) / "config" / "safety_limits.yaml")

    if not Path(path).is_file():
        raise ValueError(f"safety_params_file: 파일 없음: {path}")

    return str(Path(path).absolute())


# 로봇마다 safety.launch.py를 scope가 있는 그룹으로 감싸 넣는다.
# 모든 하위 인자를 빈 값 포함 명시 전달하고 scope를 닫아 앞 로봇의 값이 다음 로봇으로 새지 않게 한다.
# 하위 적용 순서(공통 → 정지 역할 → 로봇별 → 허가 사전)는 safety.launch.py가 정한다.
def safety_actions(values, robot_ids):
    common = common_safety_file(values["safety_params_file"])
    evidence = robot_file_mapping("safety_stop_evidence_files", values["safety_stop_evidence_files"], robot_ids)
    robot_params = robot_file_mapping("safety_robot_params_files", values["safety_robot_params_files"], robot_ids)
    safety_launch = str(Path(get_package_share_directory("ax_safety")) / "launch" / "safety.launch.py")

    actions = []

    for robot_id in robot_ids:
        arguments = {
            "robot_id": robot_id,
            "params_file": common,
            "stop_evidence_file": evidence.get(robot_id, ""),
            "robot_params_file": robot_params.get(robot_id, ""),
            **{child: values[parent] for parent, child in SAFETY_FLAG_ARGUMENTS.items()},
        }

        actions.append(GroupAction(
            [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(safety_launch),
                launch_arguments=arguments.items(),
            )],
            scoped=True,
            forwarding=True,
        ))

    return actions


# launch 인자를 실제 값으로 풀어 월드 포함·제어기·공구 노드 목록을 만든다.
def launch_setup(context):
    reject_safety_aliases(context)

    values = {
        name: LaunchConfiguration(name).perform(context)
        for name in LAUNCH_ARGUMENTS
    }
    tool_attach = values["tool"] in ATTACH_TOOLS
    safety = values["safety"].lower() == "true"
    robot_ids = selected_robot_ids(values["robots"], values["robot_ids"])

    # safety:=false에서 파일 인자를 주면 조용히 무시하지 않는다(기존 허가 플래그는 이전처럼 무시).
    given_files = [name for name in SAFETY_FILE_ARGUMENTS if values[name].strip()]

    if given_files and not safety:
        raise ValueError(", ".join(given_files) + "는 safety:=true일 때만 쓴다")

    # 시작 전에 safety 인자를 모두 검사한다.
    safety_includes = safety_actions(values, robot_ids) if safety else []

    # Gazebo가 ax_control의 흡착 플러그인 라이브러리를 찾도록 경로를 더한다.
    plugin_path = os.pathsep.join(filter(None, [
        str(Path(get_package_prefix("ax_control")) / "lib"),
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    ]))

    # 월드·모델·센서 브리지는 A1 launch가 띄우고, 제어기 설정은 ax_control/config를 쓴다.
    # A1에는 월드·모델 인자만 넘긴다(safety 인자 제외).
    robot_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("ax_description")) / "launch" / "robot_test.launch.py"
        )),
        launch_arguments={
            **{name: values[name] for name in MODEL_ARGUMENTS},
            "controller_config_dir": str(Path(get_package_share_directory("ax_control")) / "config"),
            "tool_attach": "true" if tool_attach else "false",
        }.items(),
    )

    actions = [
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        robot_test,
    ]

    # 로봇마다 제어기와 공구 노드
    for index, robot_id in enumerate(robot_ids):
        actions.extend(controller_spawners(robot_id))

        # W1 안전 실행은 명시적으로 요청한 경우에만 로봇별로 붙인다.
        # safety.launch.py의 허용 플래그 기본값(false)은 그대로 유지한다.
        if safety:
            actions.append(safety_includes[index])

        if robot_id in MANIPULATORS and tool_attach:
            actions.extend(tool_nodes(robot_id))

    return actions


# ROS 2 launch가 호출하는 진입점
def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in LAUNCH_ARGUMENTS.items()
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
