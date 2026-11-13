"""AX W1 A4/B4 노드 실행. 기본 설정은 주행/팔 구동 차단."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('ax_safety')) / 'config' / 'safety_limits.yaml')
    robot_id = LaunchConfiguration('robot_id')
    enable_simulation_drive = LaunchConfiguration('enable_simulation_drive')
    enable_simulation_arm = LaunchConfiguration('enable_simulation_arm')
    operational_permits_verified = LaunchConfiguration('operational_permits_verified')
    travel_envelope_verified = LaunchConfiguration('travel_envelope_verified')
    arm_operation_verified = LaunchConfiguration('arm_operation_verified')
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        DeclareLaunchArgument('enable_simulation_drive', default_value='false'),
        DeclareLaunchArgument('enable_simulation_arm', default_value='false'),
        DeclareLaunchArgument('operational_permits_verified', default_value='false'),
        DeclareLaunchArgument('travel_envelope_verified', default_value='false'),
        DeclareLaunchArgument('arm_operation_verified', default_value='false'),
        Node(package='ax_safety', executable='b4_risk_node', output='screen',
             parameters=[config, {'robot_id': robot_id,
                                  'front_scan_topic': ['/ax/', robot_id,
                                                       '/sensors/front_lidar/scan_filtered'],
                                  'rear_scan_topic': ['/ax/', robot_id,
                                                      '/sensors/rear_lidar/scan_filtered']}]),
        Node(package='ax_safety', executable='a4_safety_gate', output='screen',
             # 빈 vector를 YAML에서 읽으면 ROS 2 C++ 파라미터 타입이
             # 불명확해질 수 있으므로 A4의 타입 지정 기본값을 사용한다.
             parameters=[{'robot_id': robot_id,
                                  'use_sim_time': True,
                                  'enable_simulation_drive': ParameterValue(
                                      enable_simulation_drive, value_type=bool),
                                  'operational_permits_verified': ParameterValue(
                                      operational_permits_verified, value_type=bool),
                                  'travel_envelope_verified': ParameterValue(
                                      travel_envelope_verified, value_type=bool),
                                  'arm_operation_verified': ParameterValue(
                                      arm_operation_verified, value_type=bool)}]),
        Node(package='ax_safety', executable='a4_protected_arm_server', output='screen',
             parameters=[{'robot_id': robot_id,
                                  'use_sim_time': True,
                                  'enable_simulation_arm': ParameterValue(
                                      enable_simulation_arm, value_type=bool)}]),
    ])
