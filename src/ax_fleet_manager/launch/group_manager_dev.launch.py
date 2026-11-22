"""D2 개발 상태 출력 노드 실행. 출력은 /ax/dev/ 아래만, 운영 Topic·실제 구동 호출 없음.

인자: fixture_file(기본 패키지 config/dev_fixture_transfer.json), store_path(D2 원장, 읽기 전용),
output_topic(기본 /ax/dev/d2_mock/fleet_tasks), mode(DEVELOPMENT_MOCK만 허용, OPERATIONAL은 시작 거부).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 개발 출력 노드 하나만 띄운다(remap 없음)
def generate_launch_description():
    share = Path(get_package_share_directory('ax_fleet_manager'))
    default_fixture = str(share / 'config' / 'dev_fixture_transfer.json')

    return LaunchDescription([
        DeclareLaunchArgument('fixture_file', default_value=default_fixture),
        DeclareLaunchArgument('store_path', default_value=''),
        DeclareLaunchArgument('output_topic', default_value='/ax/dev/d2_mock/fleet_tasks'),
        DeclareLaunchArgument('mode', default_value='DEVELOPMENT_MOCK'),
        Node(package='ax_fleet_manager', executable='group_dev_publisher',
             name='d2_group_dev_publisher', output='screen',
             parameters=[{'fixture_file': LaunchConfiguration('fixture_file'),
                          'store_path': LaunchConfiguration('store_path'),
                          'output_topic': LaunchConfiguration('output_topic'),
                          'mode': LaunchConfiguration('mode')}]),
    ])
