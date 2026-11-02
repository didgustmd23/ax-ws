from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("ax_description")
    world = PathJoinSubstitution([pkg, "worlds", "ax_arm_test.world"])
    xacro_file = PathJoinSubstitution([pkg, "urdf", "ax_2.urdf.xacro"])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ax_2_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ax_2",
            "-topic", "/robot_description",
            "-z", "0.05",
        ],
        output="screen",
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
    ])
