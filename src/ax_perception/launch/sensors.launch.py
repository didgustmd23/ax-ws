"""실행 중인 모델에 B1만 연결한다. Gazebo나 제어기를 중복 실행하지 않는다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    ids = LaunchConfiguration('robot_ids').perform(context).split(',')
    if len(ids) != len(set(ids)) or any(r not in ('robot_1', 'robot_2', 'robot_3', 'robot_4') for r in ids):
        raise ValueError('robot_ids: unique robot_1 through robot_4 required')
    tool = LaunchConfiguration('tool').perform(context)
    if tool not in ('epick', 'suction', 'robotiq', 'primitive', 'parallel_wide'):
        raise ValueError('Unsupported tool')
    return [Node(package='ax_perception', executable='sensor_processing.py',
                 namespace='/ax/' + robot, output='screen',
                 parameters=[{'use_sim_time': True, 'robot_id': robot,
                              'manipulator': robot in ('robot_1', 'robot_3'),
                              'tool_tof': robot in ('robot_1', 'robot_3') and tool in ('epick', 'suction')}])
            for robot in ids]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot_ids', default_value='robot_1,robot_2'),
                              DeclareLaunchArgument('tool', default_value='epick'),
                              OpaqueFunction(function=setup)])
