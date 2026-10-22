import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Gazebo 모델 생성과 팔·집게·LiDAR·카메라·주행 명령 브리지를 함께 실행한다.
# 모델 이름 ax는 Xacro의 제어 토픽과 각 브리지의 /model/ax/... 경로에 맞춘다.

def generate_launch_description():
    # ros_gz_sim의 기본 launch 재사용. '-r'은 empty.sdf를 시작부터 재생한다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 변환된 URDF를 empty 월드에 ax라는 이름으로 생성한다. z=0.42 m는 몸통 원점과 바퀴 외곽 기준 지면 위 초기 생성 높이다.
    # 이 액션은 아래 OnProcessExit 콜백에서만 실행된다.
    robot_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-file', '/tmp/ax.urdf',
            '-name', 'ax',
            '-x', '0',
            '-y', '0',
            '-z', '0.42'
        ],
        output='screen'
    )

    # 현재 터미널 위치나 사용자 홈 경로 대신 설치된 ROS 패키지에서 모델 파일을 찾는다.
    xacro_file = os.path.join(
        get_package_share_directory('ax_description'),
        'urdf',
        'ax.urdf.xacro'
    )

    # Gazebo 생성용 URDF를 먼저 저장한다. /tmp/ax.urdf는 동일 PC의 동시 실행에서 공유된다.
    xacro_convert = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', '/tmp/ax.urdf'],
        output='screen'
    )

    # Xacro가 성공한 경우만 생성 요청을 보낸다. 변환 실패 시 이전 임시 파일을 생성에 사용하지 않는다.
    # 이 이벤트는 Xacro 종료를 기준으로 하며 Gazebo 준비 완료를 직접 검사하지는 않는다.
    spawn_after_xacro = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro_convert,
            on_exit=lambda event, context: (
                [robot_spawn]
                if event.returncode == 0
                else [LogInfo(msg='AX Xacro conversion failed. Skipping robot spawn.')]
            )
        )
    )

    # 집게 명령 분배 노드와 원통·팔·손가락 7개 위치 명령 브리지를 포함한다.
    arm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ax_control'),
                'launch',
                'arm_control.launch.py'
            )
        )
    )

    # '['는 Gazebo → ROS 단방향: /scan을 sensor_msgs/LaserScan으로 전달한다.
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    # ROS용 robot_description은 같은 Xacro를 별도로 변환해 문자열 파라미터로 전달한다.
    # 고정 관절 TF를 발행하며, 이동 관절 TF에는 별도의 /joint_states 입력이 필요하다.
    # 현재 launch는 /joint_states 피드백과 /clock 브리지, use_sim_time을 구성하지 않는다.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            )
        }],
        output='screen'
    )

    # 카메라 원본 영상을 Gazebo에서 ROS Image로 전달한다. 광학 프레임은 모델에서 지정한다.
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    # 카메라 내부 파라미터를 CameraInfo로 전달한다. Gazebo 출력 토픽과 이 이름이 일치해야 한다.
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # ROS /cmd_vel을 Gazebo DiffDrive의 /model/ax/cmd_vel로 전달한다.
    # Twist의 linear.x는 전후 속도(m/s), angular.z는 회전 속도(rad/s)이다.
    # 이 브리지는 명령 방향만 제공하며 odometry/TF의 ROS 수신 브리지는 별도로 필요하다.
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/model/ax/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            ('/model/ax/cmd_vel', '/cmd_vel')
        ],
        output='screen'
    )

    # 변환 시작 전에 종료 이벤트를 등록한다. 나머지 노드의 나열 순서는 준비 완료를 보장하지 않는다.
    return LaunchDescription([
        gazebo,
        spawn_after_xacro,
        xacro_convert,
        arm_control,
        lidar_bridge,
        robot_state_publisher,
        camera_bridge,
        camera_info_bridge,
        cmd_vel_bridge,
    ])
