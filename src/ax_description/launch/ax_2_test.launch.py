from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ax_description'))
                                          / 'launch' / 'robot_test.launch.py')),
        launch_arguments={'robot_ids': 'robot_2'}.items())])
