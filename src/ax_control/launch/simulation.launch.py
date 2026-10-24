"""Run AX_1, AX_2, or both with isolated model topics and real state feedback."""
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from ax_control.model_contracts import ARM_CHANNELS, sensor_bridges
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def bridge(name, specs, remappings=()):
    """Create a Gazebo-to-ROS bridge; sensor topics are robot-specific."""
    return Node(package='ros_gz_bridge', executable='parameter_bridge', name=name,
                arguments=[f'{topic}@{ros_type}[{gz_type}'
                           for topic, ros_type, gz_type in specs],
                remappings=list(remappings), parameters=[{'use_sim_time': True}],
                output='screen')


def launch_models(context):
    """Expand each model once; use the same description for Gazebo and TF."""
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
        # Preserve the existing single-AX_1 command interface. Two-model mode
        # uses robot-specific commands, joint states, odometry and TF frame IDs.
        legacy = is_arm and selection == 'ax1'
        model = 'ax' if legacy else robot_id
        namespace = '' if legacy else f'/ax/{robot_id}'
        frames = '' if legacy else f'{robot_id}/'
        filename = 'ax.urdf.xacro' if is_arm else 'ax_2.urdf.xacro'
        document = xacro.process_file(str(share / 'urdf' / filename), mappings={
            'robot_name': model, 'frame_prefix': frames, 'sensor_robot_id': robot_id})
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
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=namespace, parameters=[{'robot_description': description,
                                             'frame_prefix': frames, 'use_sim_time': True}],
            output='screen'))
        actions.append(bridge(f'{robot_id}_sensor_bridge', sensor_bridges(robot_id)))

        # 관절 상태는 각 RSP로 보내고, TF는 고유 프레임 이름으로 공통 토픽에 합친다.
        actions.append(bridge(f'{robot_id}_state_bridge', [
            (f'/model/{model}/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model'),
            (f'/model/{model}/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
            (f'/model/{model}/tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V')], [
                (f'/model/{model}/joint_states', f'{namespace}/joint_states'),
                (f'/model/{model}/odometry', f'{namespace}/odom'),
                (f'/model/{model}/tf', '/tf')]))

        # 명령은 ROS→Gazebo 단방향이며 robot_2에는 팔 채널을 만들지 않는다.
        arguments = [f'/model/{model}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist']
        remappings = [(f'/model/{model}/cmd_vel', f'{namespace}/cmd_vel')]
        if is_arm:
            for joint, channel in ARM_CHANNELS:
                topic = f'/model/{model}/{joint}/position'
                arguments.append(f'{topic}@std_msgs/msg/Float64]gz.msgs.Double')
                remappings.append((topic, f'{namespace}/{channel}/command'))
            actions.append(Node(
                package='ax_control', executable='gripper_controller', namespace=namespace,
                parameters=[{'use_sim_time': True}], remappings=[
                    (f'/{channel}/command', f'{namespace}/{channel}/command')
                    for channel in ('gripper', 'left_finger', 'right_finger')],
                output='screen'))
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name=f'{robot_id}_command_bridge', arguments=arguments,
                            remappings=remappings, parameters=[{'use_sim_time': True}],
                            output='screen'))
    return actions


def generate_launch_description():
    """Expose explicit model selection without starting a second world."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='ax1', choices=['ax1', 'ax2', 'both'],
                              description='AX_1 arm robot, AX_2 shelf robot, or both.'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_models),
    ])
