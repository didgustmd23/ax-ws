"""D3 통로 노드에 시뮬레이션 시계와 실행 설정을 전달합니다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """시뮬레이션 시계 기본값과 선택 설정을 포함한 실행 구성을 만듭니다."""
    return LaunchDescription([
        # Gazebo 입력 시각과 비교 시계를 맞춥니다. 실제 시계 환경에서는 false를 명시합니다.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('regions_file', default_value=''),
        DeclareLaunchArgument('database', default_value=''),
        Node(package='ax_corridor_manager', executable='corridor_manager',
             parameters=[{
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
                 'regions_file': ParameterValue(
                     LaunchConfiguration('regions_file'), value_type=str),
                 'database': ParameterValue(LaunchConfiguration('database'), value_type=str),
             }], output='screen'),
    ])
