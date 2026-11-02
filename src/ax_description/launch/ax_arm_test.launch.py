from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("ax_description")
    world = PathJoinSubstitution([pkg, "worlds", "ax_arm_test.world"])
    urdf = PathJoinSubstitution([pkg, "urdf", "ax_1.urdf"])
    controller_config = PathJoinSubstitution([
        pkg,
        "config",
        "ax_1_arm_controllers.yaml",
    ])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={"gz_args": world}.items(),
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "AX_1_arm_test", "-file", urdf, "-z", "0.05"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", urdf]),
                value_type=str,
            )
        }],
        output="screen",
    )

    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--param-file", controller_config,
                ],
                output="screen",
            )
        ],
    )

    arm_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "ax_1_arm_controller",
                    "--controller-manager", "/controller_manager",
                    "--param-file", controller_config,
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        arm_controller,
    ])
