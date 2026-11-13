"""C3 후보 게시 노드에 로봇 번호와 시뮬레이션 시계를 전달합니다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """모의 시계를 기본으로 사용하는 후보 게시 실행 구성을 만듭니다."""
    return LaunchDescription([
        # 시계와 로봇 번호만 연결하며 실제 IK·충돌 검증기나 C2 입력을 대신 만들지 않습니다.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        Node(package='ax_grasp_planner', executable='candidate_publisher',
             parameters=[{
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
                 'robot_id': ParameterValue(LaunchConfiguration('robot_id'), value_type=str),
             }], output='screen'),
    ])
