import xacro
from pathlib import Path
from launch.events import Shutdown
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, EmitEvent, LogInfo, RegisterEventHandler


# 두 YAML 경로를 찾아 Xacro에 전달
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
            "robot_name": robot_id,
            "sensor_robot_id": robot_id,
            "gripper_model": "suction",
            "ros_namespace": f"/ax/{robot_id}",
            "ros2_control_params": str(robot_config),
            "ros2_control_common_params": str(common_config),
            "frame_prefix": f"{robot_id}/",
        },
    )

    # 완성된 URDF 반환하기
    return document.toxml()

# ROS 2 launch가 호출하는 진입점
def generate_launch_description():
    robot_id = "robot_1"
    namespace = f"/ax/{robot_id}"
    frame_prefix = f"{robot_id}/"

    # 앞 함수가 생성한 URDF 문자열
    robot_description = build_robot_description(robot_id)

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
        }.items(),
    )

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

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "diff_drive_controller",
            "arm_controller",
            "--controller-manager", 
            f"{namespace}/controller_manager",
            "--controller-manager-timeout", 
            "60",
        ],
        output="screen",
    )

    def on_robot_spawned(event, context):
        # 로봇이 Gazebo에 스폰되면 컨트롤러 스포너를 실행
        if event.returncode != 0:
            return [
                LogInfo(msg="Robot spawn failed, stopping launch."),
                EmitEvent(
                    event=Shutdown(reason="Robot spawn failed")
                )
            ]
        
        return [controller_spawner]

    start_controller_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=on_robot_spawned
        )
    )

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

    return LaunchDescription([
        gazebo,
        clock_bridge,
        state_publisher,
        start_controller_after_spawn,
        spawn_robot,
    ])