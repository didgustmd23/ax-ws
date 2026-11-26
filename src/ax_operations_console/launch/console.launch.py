"""관제 화면(운영 이름). 작업 목록은 /ax/fleet/tasks, 로봇 입력은 /ax/<robot_id>/... 를 구독만 한다.

인자: port(기본 18780), bind_address(기본 127.0.0.1, 외부 주소는 시작 경고), use_sim_time(기본 true),
robots_file(기본 개발 로봇 목록), map_yaml(기본 ax_localization maps/ax_warehouse/map.yaml).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 관제 노드 하나(운영 profile)
def generate_launch_description():
    share = Path(get_package_share_directory('ax_operations_console'))
    localization = Path(get_package_share_directory('ax_localization'))
    default_map = str(localization / 'maps' / 'ax_warehouse' / 'map.yaml')
    arg = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='18780'),
        DeclareLaunchArgument('bind_address', default_value='127.0.0.1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robots_file',
                              default_value=str(share / 'config' / 'console_robots_dev.yaml')),
        DeclareLaunchArgument('map_yaml', default_value=default_map),
        Node(package='ax_operations_console', executable='operations_console',
             name='ax_operations_console', output='screen',
             parameters=[{'topics_profile': 'operational',
                          'port': ParameterValue(arg('port'), value_type=int),
                          'bind_address': arg('bind_address'),
                          'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool),
                          'robots_file': arg('robots_file'),
                          'map_yaml': arg('map_yaml')}]),
    ])
