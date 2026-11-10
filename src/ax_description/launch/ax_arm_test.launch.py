# 모델 실행 진입점. 공통 robot_test.launch.py를 사용하고 로봇 선택·월드 인자만 구분한다.
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# launch 인자와 실행 구성을 반환한다. 이 함수 호출만으로 즉시 로봇을 움직이지는 않는다.
def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ax_description'))
                                          / 'launch' / 'robot_test.launch.py')),
        launch_arguments={'robot_ids': 'robot_1', 'world': 'planned/ax_warehouse.world'}.items())])
