"""관제 화면(개발). 작업 목록을 /ax/dev/d2_mock/fleet_tasks에서 받는다. 선택으로 D2 개발 출력 fixture를 같이 띄운다.

인자: with_fixture(기본 false; true면 ax_fleet_manager group_dev_publisher를 fixture 파일로 실행),
fixture_file(기본 ax_fleet_manager config/dev_fixture_transfer.json), port(기본 18780),
bind_address(기본 127.0.0.1), use_sim_time(기본 false: Gazebo 없이 벽시계로 보는 개발 실행).
로봇 입력(/ax/<robot_id>/...)은 운영 이름 그대로다. 발행자가 없으면 화면에 "미수집"으로 나온다.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 관제 노드(dev profile) + 선택 fixture 발행
def generate_launch_description():
    share = Path(get_package_share_directory('ax_operations_console'))
    fleet_share = Path(get_package_share_directory('ax_fleet_manager'))
    localization = Path(get_package_share_directory('ax_localization'))
    default_map = str(localization / 'maps' / 'ax_warehouse' / 'map.yaml')
    default_fixture = str(fleet_share / 'config' / 'dev_fixture_transfer.json')
    arg = LaunchConfiguration
    use_sim_time = ParameterValue(arg('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('with_fixture', default_value='false'),
        DeclareLaunchArgument('fixture_file', default_value=default_fixture),
        DeclareLaunchArgument('port', default_value='18780'),
        DeclareLaunchArgument('bind_address', default_value='127.0.0.1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robots_file',
                              default_value=str(share / 'config' / 'console_robots_dev.yaml')),
        DeclareLaunchArgument('map_yaml', default_value=default_map),
        Node(package='ax_operations_console', executable='operations_console',
             name='ax_operations_console', output='screen',
             parameters=[{'topics_profile': 'dev',
                          'port': ParameterValue(arg('port'), value_type=int),
                          'bind_address': arg('bind_address'),
                          'use_sim_time': use_sim_time,
                          'robots_file': arg('robots_file'),
                          'map_yaml': arg('map_yaml')}]),
        Node(package='ax_fleet_manager', executable='group_dev_publisher',
             name='d2_group_dev_publisher', output='screen',
             condition=IfCondition(arg('with_fixture')),
             parameters=[{'fixture_file': arg('fixture_file'), 'use_sim_time': use_sim_time}]),
    ])
