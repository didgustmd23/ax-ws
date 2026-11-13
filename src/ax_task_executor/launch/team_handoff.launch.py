"""Start only Kim's A3/B3 nodes for the team's simulated robot pair."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_nodes(context):
    """Validate an explicit robot/model pairing before creating any nodes."""
    robot_ids = [value.strip() for value in
                 LaunchConfiguration('robot_ids').perform(context).split(',')]
    models = [value.strip() for value in
              LaunchConfiguration('robot_models').perform(context).split(',')]
    supported_robots = ('robot_1', 'robot_2', 'robot_3', 'robot_4')
    if (not robot_ids or len(robot_ids) != len(models)
            or len(set(robot_ids)) != len(robot_ids)
            or any(robot not in supported_robots for robot in robot_ids)
            or any(model not in ('AX_1', 'AX_2') for model in models)):
        raise ValueError('robot_ids must be unique robot_1~4 with one AX_1/AX_2 model per robot')
    regions = LaunchConfiguration('regions_file').perform(context)
    revision = int(LaunchConfiguration('map_revision').perform(context))
    if revision <= 0:
        raise ValueError('map_revision must be positive')
    nodes = []
    for robot, model in zip(robot_ids, models):
        common = {'robot_id': robot, 'use_sim_time': True}
        nodes.extend([
            Node(package='ax_task_executor', executable='local_task_executor',
                 namespace=f'/ax/{robot}', name='local_task_executor', output='screen',
                 parameters=[dict(common, policy='operational')]),
            Node(package='ax_environment', executable='corridor_observer',
                 namespace=f'/ax/{robot}', name='corridor_observer', output='screen',
                 parameters=[dict(common, robot_model=model, regions_file=regions,
                                  map_revision=revision)]),
        ])
    return nodes


def generate_launch_description():
    """Expose the same robot_ids convention as the B2 localization launch."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_ids', default_value='robot_1,robot_2'),
        DeclareLaunchArgument('robot_models', default_value='AX_1,AX_2'),
        DeclareLaunchArgument('regions_file', default_value=''),
        DeclareLaunchArgument('map_revision', default_value='1'),
        OpaqueFunction(function=setup_nodes),
    ])
