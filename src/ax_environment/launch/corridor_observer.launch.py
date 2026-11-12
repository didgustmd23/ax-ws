"""B3 통로 관측 노드를 로봇 1대에 띄운다(/ax/<robot_id> namespace).

사용: ros2 launch ax_environment corridor_observer.launch.py robot_id:=robot_2 robot_model:=AX_2 [regions_file:=...]
robot_model은 robot_id에서 추론하지 않으므로 반드시 지정한다(AX_1 조작·AX_2 운송).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# launch 인자를 선언하고 노드를 만든다.
def generate_launch_description():
    names = ('robot_id', 'robot_model', 'regions_file', 'use_sim_time', 'outline_margin_m')
    defaults = {'robot_id': 'robot_2', 'robot_model': '', 'regions_file': '', 'use_sim_time': 'true',
                'outline_margin_m': '0.0'}
    arguments = [DeclareLaunchArgument(name, default_value=defaults[name]) for name in names]

    robot_id = LaunchConfiguration('robot_id')
    node = Node(
        package='ax_environment', executable='corridor_observer', name='corridor_observer',
        namespace=['/ax/', robot_id], output='screen',
        parameters=[{
            'robot_id': robot_id,
            'robot_model': LaunchConfiguration('robot_model'),
            'regions_file': LaunchConfiguration('regions_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'outline_margin_m': LaunchConfiguration('outline_margin_m'),
        }])
    return LaunchDescription([*arguments, node])
