"""D1 only: consumes B2 map/TF; publishes TwistStamped exclusively to A4 input."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def parameters(robot, mobility, tree):
    if robot not in ('robot_1', 'robot_2', 'robot_3', 'robot_4'):
        raise ValueError('Unsupported robot_id')
    role = 'robot_1' if robot in ('robot_1', 'robot_3') else 'robot_2'
    footprint = str(mobility[role]['footprint_xy_m'])
    base, odom = robot + '/base_link', robot + '/odom'
    root = '/ax/' + robot
    def params(**values):
        return {'ros__parameters': {'use_sim_time': True, **values}}
    def costmap(global_map):
        cfg = dict(global_frame='map' if global_map else odom, robot_base_frame=base,
                   footprint=footprint, footprint_padding=0.0, resolution=.05,
                   update_frequency=5.0, publish_frequency=2.0,
                   transform_tolerance=.3, always_send_full_costmap=True,
                   plugins=(['static_layer'] if global_map else []) + ['obstacle_layer', 'inflation_layer'],
                   obstacle_layer={'plugin': 'nav2_costmap_2d::ObstacleLayer',
                       'observation_sources': 'front rear',
                       **{name: {'topic': root + '/sensors/' + name + '_lidar/scan_filtered',
                           'data_type': 'LaserScan', 'marking': True, 'clearing': True,
                           'expected_update_rate': .5,
                           'inf_is_valid': True, 'obstacle_max_range': 8.0,
                           'raytrace_max_range': 10.0, 'obstacle_min_range': .12,
                           'raytrace_min_range': .12, 'max_obstacle_height': 2.0}
                          for name in ('front', 'rear')}},
                   inflation_layer={'plugin': 'nav2_costmap_2d::InflationLayer',
                       'inflation_radius': 1.3, 'cost_scaling_factor': 3.0})
        if global_map:
            cfg.update(track_unknown_space=True, static_layer={
                'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/map',
                'map_subscribe_transient_local': True})
        else:
            cfg.update(rolling_window=True, width=6, height=6)
        return params(**cfg)
    return {root: {
        'controller_server': params(controller_frequency=10.0, enable_stamped_cmd_vel=True,
            odom_topic=root + '/diff_drive_controller/odom',
            progress_checker_plugins=['progress_checker'], goal_checker_plugins=['goal_checker'],
            controller_plugins=['FollowPath'],
            progress_checker={'plugin': 'nav2_controller::SimpleProgressChecker',
                'required_movement_radius': .1, 'movement_time_allowance': 10.0},
            goal_checker={'plugin': 'nav2_controller::SimpleGoalChecker',
                'xy_goal_tolerance': .1, 'yaw_goal_tolerance': .1, 'stateful': True},
            FollowPath={'plugin': 'dwb_core::DWBLocalPlanner', 'min_vel_x': 0.0,
                'max_vel_x': .15, 'min_vel_y': 0.0, 'max_vel_y': 0.0,
                'max_vel_theta': .2, 'min_speed_xy': 0.0, 'max_speed_xy': .15,
                'min_speed_theta': 0.0, 'acc_lim_x': .15, 'acc_lim_y': 0.0,
                'acc_lim_theta': .2, 'decel_lim_x': -.15, 'decel_lim_y': 0.0,
                'decel_lim_theta': -.2, 'vx_samples': 10, 'vy_samples': 1,
                'vtheta_samples': 20, 'sim_time': 2.0, 'transform_tolerance': .3,
                'critics': ['RotateToGoal', 'Oscillation', 'ObstacleFootprint',
                            'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist']}),
        'planner_server': params(planner_plugins=['GridBased'],
            GridBased={'plugin': 'nav2_navfn_planner::NavfnPlanner', 'tolerance': .1,
                'use_astar': True, 'allow_unknown': False}),
        'bt_navigator': params(global_frame='map', robot_base_frame=base,
            odom_topic=root + '/diff_drive_controller/odom', navigators=['navigate_to_pose'],
            navigate_to_pose={'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'},
            default_nav_to_pose_bt_xml=tree),
        'local_costmap': {'local_costmap': costmap(False)},
        'global_costmap': {'global_costmap': costmap(True)}}}


def setup(context):
    ids = LaunchConfiguration('robot_ids').perform(context).split(',')
    if not ids or len(ids) != len(set(ids)):
        raise ValueError('Unique robot_ids required')
    share = Path(get_package_share_directory('ax_navigation'))
    mobility = yaml.safe_load((Path(get_package_share_directory('ax_description')) /
                               'config/a1_validation.yaml').read_text())['mobility']
    runtime = tempfile.TemporaryDirectory(prefix='ax-nav2-')
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=lambda event, ctx: runtime.cleanup()))]
    for robot in ids:
        data = parameters(robot, mobility, str(share / 'config/navigate.xml'))
        path = Path(runtime.name) / (robot + '.yaml')
        path.write_text(yaml.safe_dump(data))
        namespace = '/ax/' + robot
        for package, executable in [('nav2_controller', 'controller_server'),
                                    ('nav2_planner', 'planner_server'),
                                    ('nav2_bt_navigator', 'bt_navigator')]:
            actions.append(Node(package=package, executable=executable, namespace=namespace,
                output='screen', parameters=[str(path)],
                remappings=[('tf', '/tf'), ('tf_static', '/tf_static'),
                            ('cmd_vel', 'nav/cmd_vel')]))
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', namespace=namespace, output='screen',
            parameters=[{'use_sim_time': True, 'autostart': False,
                'node_names': ['controller_server', 'planner_server', 'bt_navigator']}]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot_ids', default_value='robot_1,robot_2'),
                              OpaqueFunction(function=setup)])
