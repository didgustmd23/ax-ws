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

    ax1_urdf = PathJoinSubstitution([pkg, "urdf", "ax_1.urdf"])
    ax2_xacro = PathJoinSubstitution([pkg, "urdf", "ax_2.urdf.xacro"])
    ax2_description = ParameterValue(
        Command(["xacro ", ax2_xacro]),
        value_type=str,
    )

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

    ax2_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ax2_state_publisher",
        output="screen",
        parameters=[{"robot_description": ax2_description}],
    )

    spawn_ax1 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ax_1",
            "-file", ax1_urdf,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05",
        ],
        output="screen",
    )

    spawn_ax2 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ax_2",
            "-topic", "/robot_description",
            "-x", "2.5",
            "-y", "-1.5",
            "-z", "0.05",
        ],
        output="screen",
    )

    ax1_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ax1_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", ax1_urdf]),
                value_type=str,
            )
        }],
        output="screen",
    )

    ax1_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    ax1_arm_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "ax_1_arm_controller",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        gz_sim,
        ax1_state_publisher,
        ax2_state_publisher,
        spawn_ax1,
        spawn_ax2,
        ax1_joint_state_broadcaster,
        ax1_arm_controller,
    ])
