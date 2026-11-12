# 공통 시험 월드 진입점: 월드·모델·센서 브리지를 띄운다. 제어기 활성화는 ax_control simulation.launch.py가 맡는다.
"""Shared robot test world; one, two or four role-separated robots.

제어기 설정·활성화는 A2 ax_control이 소유한다. 이 launch는 월드·모델·센서 브리지만 띄우고,
controller_config_dir가 주어지면 그 폴더의 YAML로 gz_ros2_control 플러그인만 모델에 넣는다.
제어기 활성화(spawner)와 공구 노드는 ``ros2 launch ax_control simulation.launch.py``가 실행한다.
"""
import json
import os
import shlex
from pathlib import Path
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Explicit test-fixture roles; operational role/capability ownership belongs to D2.
ROBOT_ROLES = {'robot_1': 'MANIPULATOR', 'robot_2': 'TRANSPORTER',
            'robot_3': 'MANIPULATOR', 'robot_4': 'TRANSPORTER'}
# 모델별 A2 제어기 설정 파일 이름(controller_config_dir 안). 공통 파일 다음에 읽힌다.
CONTROLLER_FILES = {'ax.urdf.xacro': 'ax1_controllers.yaml', 'ax_2.urdf.xacro': 'ax2_controllers.yaml'}
COMMON_CONTROLLER_FILE = 'common_controllers.yaml'


# Gazebo 센서 토픽을 로봇별 ROS 경로로 연결한다. 외부 제어 명령을 대신 생성하지 않는다.
def bridges(robot_id, manipulator, tool='epick'):
    # 로봇 ID별 namespace로 Gazebo 센서를 ROS에 연결해 다른 로봇의 영상과 섞이지 않게 한다.
    sensor_namespace = '/ax/' + robot_id + '/sensors/'
    configuration = []
    # 센서 스트림은 GZ_TO_ROS 방향과 SENSOR_DATA QoS로 등록한다.
    def add_sensor_bridge(suffix, ros_type, gz_type, ros_suffix=None):
        configuration.append({'ros_topic_name': sensor_namespace + (ros_suffix or suffix),
                       'gz_topic_name': sensor_namespace + suffix, 'ros_type_name': ros_type,
                       'gz_type_name': gz_type, 'direction': 'GZ_TO_ROS',
                       'qos_profile': 'SENSOR_DATA'})
    for name in ('front_lidar', 'rear_lidar'):
        add_sensor_bridge(name + '/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')
    for name in (['front_rgb', 'arm_rgb'] if manipulator else ['front_rgb']):
        add_sensor_bridge(name + '/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image')
        add_sensor_bridge(name + '/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')
    for name in (['front_rgbd', 'arm_rgbd'] if manipulator else ['front_rgbd']):
        for suffix, ros_type, gz_type in [('image', 'Image', 'Image'), ('depth_image', 'Image', 'Image'),
                                ('points', 'PointCloud2', 'PointCloudPacked'),
                                ('camera_info', 'CameraInfo', 'CameraInfo')]:
            add_sensor_bridge(name + '/' + suffix, 'sensor_msgs/msg/' + ros_type, 'gz.msgs.' + gz_type)
    add_sensor_bridge('imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')
    if manipulator and tool in ('epick', 'suction'):
        add_sensor_bridge(('epick_tof' if tool == 'epick' else 'suction_tof') + '/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')
    if not manipulator:
        add_sensor_bridge('cargo_contact', 'ros_gz_interfaces/msg/Contacts', 'gz.msgs.Contacts')
    return configuration


# robot_1~robot_4 출발 위치(world x, y, m). yaw는 0(+X)이다.
DEFAULT_SPAWN_POSITIONS = [(0.0, 0.0), (2.5, -1.5), (-3.0, 0.0), (-3.0, -2.5)]
# Per-world spawn overrides keep each robot clear of walls, racks and cargo.
WORLD_SPAWN_POSITIONS = {
    # 두 창고 월드의 출발 창고 내부 위치(m). 순서대로 robot_1~robot_4이며 통로 밖에서 시작한다.
    'planned/ax_warehouse.world': [(-10.0, -2.4), (-6.8, 0.0), (-17.0, -2.4), (-13.0, -2.4)],
    # 단색 보존 월드는 기준(질감) 월드와 재질 외 구조가 같으므로 같은 출발 위치를 쓴다.
    'planned/ax_warehouse_plain.world': [(-10.0, -2.4), (-6.8, 0.0), (-17.0, -2.4), (-13.0, -2.4)],
}
# 큰 영상·점군 발행 노드에 줄 Fast DDS 프로필(config/fastdds_large_data.xml 주석 참고).
DDS_PROFILE_ENV = 'FASTRTPS_DEFAULT_PROFILES_FILE'


# 큰 메시지 발행 노드의 추가 환경을 만든다. 사용자가 프로필을 이미 지정했거나 파일이 없으면 비워 둔다.
# rmw_fastrtps가 아닌 RMW는 이 변수를 읽지 않으므로 영향이 없다.
def large_data_env(package_share):
    profile = Path(package_share) / 'config' / 'fastdds_large_data.xml'

    if os.environ.get(DDS_PROFILE_ENV) or not profile.is_file():
        return {}

    return {DDS_PROFILE_ENV: str(profile)}


# 실행 인자를 평가해 월드·모델·센서 브리지를 구성한다. 제어기 spawner는 만들지 않는다(ax_control 담당).
def setup(context):
    package_share = Path(get_package_share_directory('ax_description'))
    robot_count = int(LaunchConfiguration('robots').perform(context))
    if robot_count not in (1, 2, 4):
        raise ValueError('robots must be 1, 2 or 4')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    controller_config_dir = LaunchConfiguration('controller_config_dir').perform(context)
    control = bool(controller_config_dir)
    if control and not Path(controller_config_dir, COMMON_CONTROLLER_FILE).is_file():
        raise ValueError('controller_config_dir must contain ' + COMMON_CONTROLLER_FILE)
    tool_attach = LaunchConfiguration('tool_attach').perform(context).lower() == 'true'
    sensors_enabled = LaunchConfiguration('sensors').perform(context).lower() == 'true'
    tool = LaunchConfiguration('tool').perform(context)
    test_payload_mass = LaunchConfiguration('payload_mass').perform(context)
    if not 0 <= float(test_payload_mass) <= 2.0:
        raise ValueError('Test payload mass must be between 0 and 2 kg')
    # Fresh per-run files prevent concurrent launch instances overwriting configurations.
    runtime = tempfile.TemporaryDirectory(prefix='ax-w1-')
    runtime_path = Path(runtime.name)
    selected_world = LaunchConfiguration('world').perform(context)
    resources = os.pathsep.join([str(package_share.parent), str(Path(get_package_share_directory('robotiq_description')).parent),
                                 str(Path(get_package_share_directory('epick_description')).parent),
                                 os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    actions = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
               RegisterEventHandler(OnShutdown(on_shutdown=lambda event, ctx: runtime.cleanup()))]
    # 빈 바닥 등은 테스트 도구가 임시 생성한다. 사용자 선택 맵에는 포함하지 않는다.
    test_fixture_world = LaunchConfiguration('test_fixture_world').perform(context)
    world = Path(test_fixture_world) if test_fixture_world else package_share / 'worlds' / selected_world
    if not world.is_file():
        raise ValueError('Selected world is not installed; rebuild ax_description: ' + str(world))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ('-r ' if gui else '-r -s --headless-rendering ') + shlex.quote(str(world)),
                          'on_exit_shutdown': 'true'}.items()))
    all_bridges = [{'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
                    'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock',
                    'direction': 'GZ_TO_ROS'}]
    spawn_positions = DEFAULT_SPAWN_POSITIONS if test_fixture_world else WORLD_SPAWN_POSITIONS.get(selected_world, DEFAULT_SPAWN_POSITIONS)
    robot_id_selection = LaunchConfiguration('robot_ids').perform(context)
    selected_indices = [int(value.removeprefix('robot_')) - 1 for value in robot_id_selection.split(',')] if robot_id_selection else list(range(robot_count))
    if len(selected_indices) != len(set(selected_indices)) or any(robot_index not in range(4) for robot_index in selected_indices):
        raise ValueError('robot_ids must be unique robot_1 through robot_4')
    for robot_index in selected_indices:
        robot_id = 'robot_' + str(robot_index + 1)
        manipulator = ROBOT_ROLES[robot_id] == 'MANIPULATOR'
        robot_namespace = '/ax/' + robot_id
        model_filename = 'ax.urdf.xacro' if manipulator else 'ax_2.urdf.xacro'
        model_mappings = {
            'robot_name': robot_id, 'sensor_robot_id': robot_id,
            'frame_prefix': robot_id + '/',
            'sensors_enabled': 'true' if sensors_enabled else 'false',
            'ros_namespace': robot_namespace,
            'ros2_control_params': str(Path(controller_config_dir, CONTROLLER_FILES[model_filename])) if control else '',
            'ros2_control_common_params': str(Path(controller_config_dir, COMMON_CONTROLLER_FILE)) if control else ''}
        if manipulator:
            model_mappings.update({'gripper_model': tool, 'test_payload_mass': test_payload_mass,
                                   'tool_attach': 'true' if tool_attach else 'false'})
        robot_description = xacro.process_file(str(package_share / 'urdf' / model_filename),
                                               mappings=model_mappings).toxml()
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
            namespace=robot_namespace, parameters=[{'robot_description': robot_description, 'use_sim_time': True,
                                      'frame_prefix': robot_id + '/'}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')], output='screen'))
        spawn_x, spawn_y = spawn_positions[robot_index]
        spawn_robot = Node(package='ros_gz_sim', executable='create',
            arguments=['-name', robot_id, '-topic', robot_namespace + '/robot_description',
                       '-x', str(spawn_x), '-y', str(spawn_y), '-z', '.42' if manipulator else '.30'],
            output='screen')
        actions.append(spawn_robot)
        if sensors_enabled:
            all_bridges.extend(bridges(robot_id, manipulator, tool))
        if manipulator and sensors_enabled and tool in ('epick', 'suction'):
            actions.append(Node(package='ax_description', executable='tof_range.py', namespace=robot_namespace,
                parameters=[{'use_sim_time': True, 'frame_id': robot_id + '/tool_tof_link',
                             'input_topic': 'sensors/' + ('epick_tof' if tool == 'epick' else 'suction_tof') + '/scan'}], output='screen'))
    bridge_file = runtime_path / 'bridge.yaml'
    bridge_file.write_text(yaml.safe_dump(all_bridges))
    # 영상·깊이·점군을 ROS로 내는 발행자이므로 큰 메시지 DDS 프로필을 이 노드에만 준다.
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': str(bridge_file), 'use_sim_time': True}], output='screen',
        additional_env=large_data_env(package_share)))
    (runtime_path / 'manifest.json').write_text(json.dumps({'robots': len(selected_indices), 'control': control,
        'world': str(world), 'bridge_count': len(all_bridges)}, indent=2))
    return actions


# launch 인자와 실행 구성을 반환한다. 이 함수 호출만으로 즉시 로봇을 움직이지는 않는다.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='planned/ax_warehouse.world',
                              choices=['planned/ax_warehouse.world', 'planned/ax_warehouse_plain.world']),
        DeclareLaunchArgument('test_fixture_world', default_value='',
                              description='Test-only generated SDF fixture; not a selectable map'),
        DeclareLaunchArgument('robots', default_value='2', choices=['1', '2', '4']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('controller_config_dir', default_value='',
                              description='A2 ax_control config folder; empty spawns models without ros2_control'),
        DeclareLaunchArgument('tool_attach', default_value='false', choices=['true', 'false'],
                              description='Add the A2 suction attach plugin (epick/suction only)'),
        DeclareLaunchArgument('sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('tool', default_value='epick', choices=['epick', 'suction', 'primitive', 'robotiq', 'parallel_wide']),
        DeclareLaunchArgument('payload_mass', default_value='0.0', description='Fixed test mass at TCP, not suction success'),
        DeclareLaunchArgument('robot_ids', default_value='', description='Optional comma-separated robot IDs'),
        OpaqueFunction(function=setup)])
