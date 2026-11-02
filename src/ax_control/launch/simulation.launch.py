# A2 개발용 시뮬레이션 launch. Gazebo와 로봇 한 대를 띄우고 제어기를 활성화한다.
#
# 실행: ros2 launch ax_control simulation.launch.py robot_id:=robot_1
# 기동 순서: Gazebo → 로봇 스폰 → 스폰 성공 시 제어기 spawner 실행
#
# 이 launch가 띄우는 것
# - Gazebo(empty 월드), /clock 브리지, robot_state_publisher, 로봇 스폰
# - 제어기 3개: joint_state_broadcaster, arm_controller, diff_drive_controller
# - 공구: 플러그인(ax_suction_attach_system), 공구 브리지, suction_tool_node
#
# 띄우지 않는 것
# - A3 Task Executor, A4 게이트·보호 서버, Nav2, 센서 브리지
# - A3·A4가 없으면 tool_command는 전부 거절된다. 정상 동작이다.
#
# 주의
# - A4 게이트가 없어 제어기를 직접 호출할 수 있다. 운영 경로가 아니다.
# - launch 하나가 Gazebo까지 띄우므로 여러 대를 동시에 실행할 수 없다.
# - 모델은 로컬 임시 모델(temp_robot)이다. 정식 A1 모델이 오면 model_path를 바꾼다.

import os
import xacro
from pathlib import Path
from launch.events import Shutdown
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)


# 흡착 플러그인 설정. 값은 임시 흡착 공구 기준이고 tcp_offset은 gripper_base_link에서 epick_tcp까지다.
# Gazebo가 고정 관절을 합치므로 공구 링크 대신 gripper_base_link를 부모로 쓴다.
# max_attach_distance는 계약에 값이 없어 개발용으로 정한 잠정값이다(m).
SUCTION_PLUGIN = """
<gazebo>
  <plugin filename="ax_suction_attach_system" name="ax_control::SuctionAttachSystem">
    <parent_link>gripper_base_link</parent_link>
    <tcp_offset>0.082 0 0</tcp_offset>
    <max_attach_distance>0.12</max_attach_distance>
    <command_topic>{namespace}/tool/gz_command</command_topic>
    <state_topic>{namespace}/tool/gz_state</state_topic>
  </plugin>
</gazebo>
"""


# 로봇 ID에 맞는 URDF 문자열을 Xacro로 생성한다. 제어기 YAML 두 개의 경로도 함께 넘긴다.
def build_robot_description(robot_id):
    # 패키지 설치 위치 찾기
    description_share = Path(
        get_package_share_directory("ax_description")
    )
    control_share = Path(
        get_package_share_directory("ax_control")
    )

    # 세 파일의 경로 만들기
    model_path = description_share / "temp_robot" / "urdf" / "ax.urdf.xacro"
    common_config = control_share / "config" / "common_controllers.yaml"
    robot_config = control_share / "config" / "ax_controllers.yaml"

    # Xacro에 인자 전달하기
    document = xacro.process_file(
        str(model_path),
        mappings={
            # robot_name은 ros2_control 하드웨어 이름, sensor_robot_id는 센서 토픽에 쓰인다.
            "robot_name": robot_id,
            "sensor_robot_id": robot_id,
            # suction은 비교용 외형이다. 흡착 부착·해제는 구현되어 있지 않다.
            "gripper_model": "suction",
            "ros_namespace": f"/ax/{robot_id}",
            # gz_ros2_control 플러그인이 이 두 YAML을 읽어 제어기를 구성한다.
            "ros2_control_params": str(robot_config),
            "ros2_control_common_params": str(common_config),
            "frame_prefix": f"{robot_id}/",
        },
    )

    # 모델 파일을 고치지 않고 흡착 플러그인을 URDF 끝에 덧붙인다.
    urdf = document.toxml()
    plugin = SUCTION_PLUGIN.format(namespace=f"/ax/{robot_id}")
    insert_at = urdf.rindex("</robot>")

    # 완성된 URDF 반환하기
    return urdf[:insert_at] + plugin + urdf[insert_at:]

# launch 인자를 실제 값으로 풀어 로봇별 노드 목록을 만든다
def launch_setup(context):
    robot_id = LaunchConfiguration("robot_id").perform(context)
    namespace = f"/ax/{robot_id}"
    frame_prefix = f"{robot_id}/"

    # 앞 함수가 생성한 URDF 문자열
    robot_description = build_robot_description(robot_id)

    # robot_description 토픽과 관절·고정 프레임 TF를 발행한다.
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
            "frame_prefix": frame_prefix,
        }],
        remappings=[        # TF 토픽을 프로젝트 계약대로 공용 경로로 연결
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
        output="screen",
    )

    gazebo_share = Path(
        get_package_share_directory("ros_gz_sim")
    )

    # -r은 시작과 동시에 시뮬레이션을 실행한다. 화면 없이 띄우려면 -s --headless-rendering을 더한다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
        }.items(),
    )

    # robot_description 토픽을 읽어 Gazebo에 로봇을 넣는다. 이름은 robot_id와 같다.
    # z 0.40 m는 바퀴가 지면에 닿는 높이(약 0.389 m)보다 살짝 위다.
    spawn_robot = Node(
        package = "ros_gz_sim",
        executable = "create",
        namespace = namespace,
        arguments = [
            "-world", "empty",
            "-name", robot_id,
            "-topic", f"{namespace}/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.40",
        ],
    )

    # 상태 발행과 팔 제어기를 활성화한다. 제어기 매니저는 스폰된 로봇 안에서 시작한다.
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "--controller-manager", 
            f"{namespace}/controller_manager",
            "--controller-manager-timeout", 
            "60",
        ],
        output="screen",
    )

    # odom TF 접두사는 로봇별 값이라 YAML에 고정하지 않고 여기서만 넘긴다.
    drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            f"{namespace}/controller_manager",
            "--controller-manager-timeout",
            "60",
            # 값이 -p로 시작해 인자 파서가 옵션으로 읽지 않도록 등호로 붙인다.
            f"--controller-ros-args=-p tf_frame_prefix:={frame_prefix}",
        ],
        output="screen",
    )

    # 스폰 프로세스 종료 시 호출된다. 성공하면 제어기를 띄우고 실패하면 launch를 끝낸다.
    def on_robot_spawned(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg="Robot spawn failed, stopping launch."),
                EmitEvent(
                    event=Shutdown(reason="Robot spawn failed")
                )
            ]
        
        return [controller_spawner, drive_spawner]

    start_controller_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=on_robot_spawned
        )
    )

    # Gazebo 시계를 ROS /clock으로 넘긴다. use_sim_time을 쓰는 노드는 이 토픽이 있어야 동작한다.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            ("/world/empty/clock", "/clock"),
        ],
        output="screen",
    )

    # 공구 노드와 Gazebo 플러그인 사이 명령·상태를 잇는다. ]는 ROS→Gazebo, [는 Gazebo→ROS다.
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

    # Gazebo가 ax_control의 플러그인 라이브러리를 찾도록 경로를 더한다.
    plugin_path = os.pathsep.join(filter(None, [
        str(Path(get_package_prefix("ax_control")) / "lib"),
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    ]))

    return [
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        gazebo,
        clock_bridge,
        tool_bridge,
        tool_node,
        state_publisher,
        start_controller_after_spawn,
        spawn_robot,
    ]


# ROS 2 launch가 호출하는 진입점
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot_1",
            description="namespace와 TF 접두사에 쓰는 로봇 ID (robot_<n>).",
        ),
        OpaqueFunction(function=launch_setup),
    ])