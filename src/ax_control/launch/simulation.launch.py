"""AX_1, AX_2 또는 둘 다를 로봇별 ros2_control 관리자와 센서 브리지로 실행한다."""
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from ax_control.model_contracts import controllers, sensor_bridges
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


# Gazebo → ROS 브리지 노드를 만든다. 센서 토픽은 로봇별 경로다.
def bridge(name, specs, remappings=()):
    return Node(package='ros_gz_bridge', executable='parameter_bridge', name=name,
                arguments=[f'{topic}@{ros_type}[{gz_type}'
                           for topic, ros_type, gz_type in specs],
                remappings=list(remappings), parameters=[{'use_sim_time': True}],
                output='screen')


# 로봇 하나의 namespace 전체 이름만 키로 쓰는 제어기 설정 YAML 사본을 쓰고 경로를 돌려준다.
def controller_params(kind, namespace, frames, folder, robot_id):
    config = Path(get_package_share_directory('ax_control')) / 'config'
    source = config / f'{kind}_controllers.yaml'
    # 한 Gazebo 프로세스의 첫 gz_ros2_control 인자는 전역 ROS 인자가 되어 다른 로봇 노드에도
    # 적용될 수 있다. 그래서 '/**/' 대신 이 로봇의 namespace 전체 이름만 키로 쓴다.
    params = {key.replace('/**', namespace, 1): value
              for key, value in yaml.safe_load(source.read_text()).items()}

    # diff_drive_controller는 namespace 대신 이 프레임 이름을 odom·TF에 그대로 쓴다.
    drive = params[f'{namespace}/diff_drive_controller']['ros__parameters']
    drive['odom_frame_id'] = f'{frames}odom'
    drive['base_frame_id'] = f'{frames}base_link'

    path = folder / f'{robot_id}_controllers.yaml'
    path.write_text(yaml.safe_dump(params, sort_keys=False))
    return path


# 선택한 모델마다 Xacro를 한 번 펼쳐 Gazebo 생성·TF·제어기·브리지에 같은 description을 쓰는
# launch 동작 목록을 만든다.
def launch_models(context):
    selection = LaunchConfiguration('robot_model').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    share = Path(get_package_share_directory('ax_description'))
    world = share / 'worlds/ax_empty.sdf'

    # 렌더링·IMU·접촉 시스템은 두 모델이 공유하는 월드에서 한 번 로드한다.
    args = f'-r "{world}"' if gui else f'-s -r --headless-rendering "{world}"'
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': args}.items()),
        bridge('clock_bridge', [('/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock')])]

    # 실행마다 다른 폴더를 사용하여 이전 실행의 URDF를 재사용하지 않는다.
    folder = Path(tempfile.mkdtemp(prefix='ax_sim_'))
    robots = ('ax1', 'ax2') if selection == 'both' else (selection,)

    for kind in robots:
        is_arm = kind == 'ax1'
        robot_id = 'robot_1' if is_arm else 'robot_2'

        # 기존 단일 AX_1 명령 인터페이스를 유지한다. 두 모델 실행에서는 로봇별 명령,
        # 관절 상태, odometry, TF 프레임 ID를 쓴다.
        legacy = is_arm and selection == 'ax1'
        model = 'ax' if legacy else robot_id
        namespace = '' if legacy else f'/ax/{robot_id}'
        frames = '' if legacy else f'{robot_id}/'
        filename = 'ax.urdf.xacro' if is_arm else 'ax_2.urdf.xacro'

        params = controller_params(kind, namespace, frames, folder, robot_id)
        document = xacro.process_file(str(share / 'urdf' / filename), mappings={
            'robot_name': model, 'frame_prefix': frames, 'sensor_robot_id': robot_id,
            'ros_namespace': namespace, 'ros2_control_params': str(params)})
        document.documentElement.setAttribute('name', model)
        description = document.toxml()
        urdf = folder / f'{robot_id}.urdf'
        urdf.write_text(description)

        # 3 m 간격의 초기 배치는 충돌 방지용 개발 설정이며 인계 위치가 아니다.
        actions.append(Node(
            package='ros_gz_sim', executable='create', name=f'{robot_id}_spawn',
            arguments=['-world', 'empty', '-file', str(urdf), '-name', model,
                       '-x', '3' if selection == 'both' and not is_arm else '0',
                       '-y', '0', '-z', '0.42' if is_arm else '0.30'], output='screen'))

        # Gazebo와 TF에 같은 description을 주고, GUI JointState는 사용하지 않는다.
        # controller_manager도 같은 namespace의 robot_description 토픽에서 모델을 읽는다.
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=namespace, parameters=[{'robot_description': description,
                                             'frame_prefix': frames, 'use_sim_time': True}],
            output='screen'))
        actions.append(bridge(f'{robot_id}_sensor_bridge', sensor_bridges(robot_id)))

        # 구동·관절 상태·odom·이동 TF는 gz_ros2_control이 ROS로 직접 낸다(Gazebo 브리지 없음).
        # joint_states는 {namespace}/joint_states, odom은 {namespace}/odom, TF는 /tf.
        # 다른 로봇을 생성·센서를 적재하는 동안 Gazebo 진행이 멈추면 제어기 전환이 기본 5 s를
        # 넘길 수 있다(두 로봇 실행에서 실패 확인). 전환 대기 시간을 늘린다.
        manager = ['--controller-manager', f'{namespace}/controller_manager',
                   '--controller-manager-timeout', '120', '--switch-timeout', '60']

        names = controllers(robot_id)
        names.remove('diff_drive_controller')

        # arm_controller의 공개 이름은 arm_command_guard가 받고, 실제 제어기는 내부 이름을 쓴다.
        # 정상 팔 명령 경로를 보호 노드 → 내부 action 하나로 만들기 위함이며 접근 차단은 아니다.
        arguments = []
        for name in names:
            arguments += ['--controller', name]
            if name == 'arm_controller':
                arguments += ['--controller-ros-args', ' '.join(
                    f'-r ~/{entry}:=arm_controller/internal/{entry}'
                    for entry in ('joint_trajectory', 'follow_joint_trajectory'))]

        actions.append(Node(
            package='controller_manager', executable='spawner', namespace=namespace,
            name=f'{robot_id}_controller_spawner', arguments=[*arguments, *manager],
            output='screen'))

        # diff_drive_controller의 ~/odom을 기존 계약 {namespace}/odom으로 옮긴다.
        actions.append(Node(
            package='controller_manager', executable='spawner', namespace=namespace,
            name=f'{robot_id}_drive_spawner',
            arguments=['diff_drive_controller', *manager,
                       '--controller-ros-args', '-r ~/odom:=odom'],
            output='screen'))

        # 기존 Twist 입력 {namespace}/cmd_vel을 diff_drive_controller의 TwistStamped로 전달한다.
        actions.append(Node(
            package='ax_control', executable='cmd_vel_relay', namespace=namespace,
            parameters=[{'use_sim_time': True, 'frame_id': f'{frames}base_link'}],
            output='screen'))

        if is_arm:
            # 팔 명령은 기존 이름 {namespace}/arm_controller/joint_trajectory 토픽이나
            # follow_joint_trajectory action으로 보낸다. arm_command_guard가 명령 가능 범위·보간
            # 경로를 검사해 내부 arm_controller action으로만 전달하고 관절 상태로 보호 정지한다.
            share = Path(get_package_share_directory('ax_control'))
            guard = share / 'config' / 'arm_command_guard.yaml'
            actions.append(Node(
                package='ax_control', executable='arm_command_guard', namespace=namespace,
                parameters=[str(guard), {'use_sim_time': True}], output='screen'))

            actions.append(Node(
                package='ax_control', executable='gripper_controller', namespace=namespace,
                parameters=[{'use_sim_time': True}], remappings=[
                    (f'/{topic}', f'{namespace}/{topic}')
                    for topic in ('gripper/command', 'finger_controller/commands')],
                output='screen'))

    return actions


# 두 번째 월드를 띄우지 않고 로봇 모델·GUI 선택 인자를 제공하는 launch 설명을 만든다.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='ax1', choices=['ax1', 'ax2', 'both'],
                              description='AX_1 arm robot, AX_2 shelf robot, or both.'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_models),
    ])
