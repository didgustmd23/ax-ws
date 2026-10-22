import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_path = get_package_share_directory('ax_description')
    gripper_model = LaunchConfiguration('gripper_model')

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'ax.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' gripper_model:=', gripper_model]),
        value_type=str
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'gripper_model',
            # 기준 공구 EPick 도입 전까지 비교용 가상 4컵 suction을 RViz 기본값으로 둔다.
            # 흡착 부착·해제 구현을 뜻하지 않으며,
            # 공통 Xacro와 Gazebo simulation의 기본 공구(primitive)는 변경하지 않는다.
            default_value='suction',
            choices=['suction', 'robotiq', 'primitive', 'parallel_wide'],
            description=(
                'Tool for RViz shape/kinematic comparison; '
                'suction attach/release and physical grasping are not enabled.'
            )
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description}
            ],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{
                'use_mimic_tags': True,
                'zeros.wide_left_finger_joint': 0.215
            }],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )

    ])
