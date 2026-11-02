"""Shared robot test world; one, two or four role-separated robots."""
import json
import os
from pathlib import Path
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARM_JOINTS = ['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']
# Explicit test-fixture roles; operational role/capability ownership belongs to D2.
ROBOT_ROLES = {'robot_1': 'MANIPULATOR', 'robot_2': 'TRANSPORTER',
            'robot_3': 'MANIPULATOR', 'robot_4': 'TRANSPORTER'}


def controller_config(robot_id, manipulator, wheel_geometry):
    robot_namespace = '/ax/' + robot_id
    controller_types = {'update_rate': 100,
             'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
             'diff_drive_controller': {'type': 'diff_drive_controller/DiffDriveController'}}
    drive_parameters = {'left_wheel_names': ['front_left_wheel_joint', 'rear_left_wheel_joint'],
             'right_wheel_names': ['front_right_wheel_joint', 'rear_right_wheel_joint'],
             'wheel_separation': wheel_geometry['separation'], 'wheel_radius': wheel_geometry['radius'],
             'base_frame_id': robot_id + '/base_link', 'odom_frame_id': robot_id + '/odom',
             'tf_frame_prefix_enable': False, 'enable_odom_tf': True,
             'publish_rate': 30.0, 'cmd_vel_timeout': 0.2,
             'linear.x.max_velocity': 0.3, 'linear.x.min_velocity': -0.3,
             'linear.x.has_velocity_limits': True,
             'angular.z.max_velocity': 0.5, 'angular.z.min_velocity': -0.5,
             'angular.z.has_velocity_limits': True}
    configuration = {robot_namespace + '/controller_manager': {'ros__parameters': controller_types},
              robot_namespace + '/diff_drive_controller': {'ros__parameters': drive_parameters},
              robot_namespace + '/joint_state_broadcaster': {'ros__parameters': {}}}
    if manipulator:
        controller_types['arm_controller'] = {'type': 'joint_trajectory_controller/JointTrajectoryController'}
        configuration[robot_namespace + '/arm_controller'] = {'ros__parameters': {
            'joints': ARM_JOINTS, 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity']}}
    return configuration


def bridges(robot_id, manipulator, tool='epick'):
    sensor_namespace = '/ax/' + robot_id + '/sensors/'
    configuration = []
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
    else:
        add_sensor_bridge('cargo_contact', 'ros_gz_interfaces/msg/Contacts', 'gz.msgs.Contacts')
    return configuration


def setup(context):
    package_share = Path(get_package_share_directory('ax_description'))
    robot_count = int(LaunchConfiguration('robots').perform(context))
    if robot_count not in (1, 2, 4):
        raise ValueError('robots must be 1, 2 or 4')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    control = LaunchConfiguration('control').perform(context).lower() == 'true'
    sensors_enabled = LaunchConfiguration('sensors').perform(context).lower() == 'true'
    tool = LaunchConfiguration('tool').perform(context)
    test_payload_mass = LaunchConfiguration('payload_mass').perform(context)
    if not 0 <= float(test_payload_mass) <= 2.0:
        raise ValueError('Test payload mass must be between 0 and 2 kg')
    # Fresh per-run files prevent concurrent launch instances overwriting configurations.
    runtime = tempfile.TemporaryDirectory(prefix='ax-w1-')
    runtime_path = Path(runtime.name)
    resources = os.pathsep.join([str(package_share.parent), str(Path(get_package_share_directory('robotiq_description')).parent),
                                 str(Path(get_package_share_directory('epick_description')).parent),
                                 os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    actions = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
               RegisterEventHandler(OnShutdown(on_shutdown=lambda event, ctx: runtime.cleanup()))]
    world = package_share / 'worlds' / LaunchConfiguration('world').perform(context)
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ('-r ' if gui else '-r -s --headless-rendering ') + str(world)}.items()))
    all_bridges = [{'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
                    'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock',
                    'direction': 'GZ_TO_ROS'}]
    # Geometry parameters come from generated CAD measurements, not copied wheel constants.
    import xml.etree.ElementTree as ET
    geometry_root = ET.parse(package_share / 'urdf' / 'ax_2_geometry.xacro').getroot()
    geometry_properties = {property_node.attrib['name']: property_node.attrib['value'] for property_node in geometry_root}
    transporter_wheel_geometry = {'radius': float(geometry_properties['ax2_wheel_radius']),
                    'separation': float(geometry_properties['ax2_wheel_separation'])}
    spawn_positions = [(0.0, 0.0), (2.5, -1.5), (-3.0, 0.0), (-3.0, -2.5)]
    robot_id_selection = LaunchConfiguration('robot_ids').perform(context)
    selected_indices = [int(value.removeprefix('robot_')) - 1 for value in robot_id_selection.split(',')] if robot_id_selection else list(range(robot_count))
    if len(selected_indices) != len(set(selected_indices)) or any(robot_index not in range(4) for robot_index in selected_indices):
        raise ValueError('robot_ids must be unique robot_1 through robot_4')
    for robot_index in selected_indices:
        robot_id = 'robot_' + str(robot_index + 1)
        manipulator = ROBOT_ROLES[robot_id] == 'MANIPULATOR'
        robot_namespace = '/ax/' + robot_id
        controller_file = runtime_path / (robot_id + '_controllers.yaml')
        controller_file.write_text(yaml.safe_dump(controller_config(robot_id, manipulator,
            {'radius': .19631, 'separation': .622} if manipulator else transporter_wheel_geometry)))
        model_filename = 'ax.urdf.xacro' if manipulator else 'ax_2.urdf.xacro'
        robot_description = xacro.process_file(str(package_share / 'urdf' / model_filename), mappings={
            'robot_name': robot_id, 'sensor_robot_id': robot_id,
            'frame_prefix': robot_id + '/', 'gripper_model': tool,
            'sensors_enabled': 'true' if sensors_enabled else 'false', 'test_payload_mass': test_payload_mass,
            'ros_namespace': robot_namespace, 'ros2_control_params': str(controller_file) if control else ''}).toxml()
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
        if control:
            controller_names = ['joint_state_broadcaster', 'diff_drive_controller']
            if manipulator:
                controller_names.append('arm_controller')
            controller_spawner = Node(package='controller_manager', executable='spawner',
                namespace=robot_namespace, arguments=controller_names + ['--controller-manager', robot_namespace + '/controller_manager',
                    '--controller-manager-timeout', '120', '--switch-timeout', '60',
                    '--param-file', str(controller_file)], output='screen')
            actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[controller_spawner])))
        if sensors_enabled:
            all_bridges.extend(bridges(robot_id, manipulator, tool))
        if manipulator and sensors_enabled and tool in ('epick', 'suction'):
            actions.append(Node(package='ax_description', executable='tof_range.py', namespace=robot_namespace,
                parameters=[{'use_sim_time': True, 'frame_id': robot_id + '/tool_tof_link',
                             'input_topic': 'sensors/' + ('epick_tof' if tool == 'epick' else 'suction_tof') + '/scan'}], output='screen'))
    bridge_file = runtime_path / 'bridge.yaml'
    bridge_file.write_text(yaml.safe_dump(all_bridges))
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': str(bridge_file), 'use_sim_time': True}], output='screen'))
    (runtime_path / 'manifest.json').write_text(json.dumps({'robots': len(selected_indices), 'control': control,
        'world': str(world), 'bridge_count': len(all_bridges)}, indent=2))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='drive_test.world', choices=['drive_test.world', 'ax_arm_test.world']),
        DeclareLaunchArgument('robots', default_value='2', choices=['1', '2', '4']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('control', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('tool', default_value='epick', choices=['epick', 'suction', 'primitive', 'robotiq', 'parallel_wide']),
        DeclareLaunchArgument('payload_mass', default_value='0.0', description='Fixed test mass at TCP, not suction success'),
        DeclareLaunchArgument('robot_ids', default_value='', description='Optional comma-separated robot IDs'),
        OpaqueFunction(function=setup)])
