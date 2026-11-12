"""실행 중인 모델에 B1만 연결한다. Gazebo나 제어기를 중복 실행하지 않는다."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 큰 영상·점군 발행 노드에 줄 Fast DDS 프로필(ax_description config/fastdds_large_data.xml 주석 참고).
DDS_PROFILE_ENV = 'FASTRTPS_DEFAULT_PROFILES_FILE'


# B1 노드의 추가 환경을 만든다. 사용자가 프로필을 이미 지정했거나 파일·패키지가 없으면 비워 둔다.
# rmw_fastrtps가 아닌 RMW는 이 변수를 읽지 않으므로 영향이 없다.
def large_data_env():
    if os.environ.get(DDS_PROFILE_ENV):
        return {}

    try:
        profile = Path(get_package_share_directory('ax_description')) / 'config' / 'fastdds_large_data.xml'
    except PackageNotFoundError:
        return {}

    return {DDS_PROFILE_ENV: str(profile)} if profile.is_file() else {}


def setup(context):
    ids = LaunchConfiguration('robot_ids').perform(context).split(',')
    if len(ids) != len(set(ids)) or any(r not in ('robot_1', 'robot_2', 'robot_3', 'robot_4') for r in ids):
        raise ValueError('robot_ids: unique robot_1 through robot_4 required')
    tool = LaunchConfiguration('tool').perform(context)
    if tool not in ('epick', 'suction', 'robotiq', 'primitive', 'parallel_wide'):
        raise ValueError('Unsupported tool')
    # sensor_processing은 처리 영상·깊이·점군(약 1~5 MB)을 발행하므로 큰 메시지 프로필을 준다.
    env = large_data_env()
    return [Node(package='ax_perception', executable='sensor_processing.py',
                 namespace='/ax/' + robot, output='screen',
                 parameters=[{'use_sim_time': True, 'robot_id': robot,
                              'manipulator': robot in ('robot_1', 'robot_3'),
                              'tool_tof': robot in ('robot_1', 'robot_3') and tool in ('epick', 'suction')}],
                 additional_env=env)
            for robot in ids]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot_ids', default_value='robot_1,robot_2'),
                              DeclareLaunchArgument('tool', default_value='epick'),
                              OpaqueFunction(function=setup)])
