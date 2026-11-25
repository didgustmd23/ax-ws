"""AX A4/B4 노드 실행. 기본 설정은 주행/팔 구동 차단.

매개변수 파일:
- params_file: 로봇 공통값(기본 패키지 config/safety_limits.yaml). 키는 /**/<노드> 형식.
- stop_evidence_file: 모델별 정지 판정 역할(팔 유무·필수 팔 관절, config/stop_evidence_ax1|ax2.yaml).
  기본은 없음(= 역할 미정, STOPPED 미확인). robot_id로 자동 선택하지 않는다. 허가와 무관하다.
- robot_params_file: 로봇별 값(주행 자세 travel_joint_*·팔 한계 arm_*). 기본은 없음(= 미검증·잠금).
  이 launch는 robot_id 하나만 띄우므로 로봇별 파일은 그 로봇에만 적용된다.
적용 순서: params_file → stop_evidence_file → robot_params_file → launch 인자 사전(허가 플래그·robot_id·use_sim_time).
허가 플래그는 파일 값과 무관하게 launch 인자(기본 false)가 마지막에 덮는다.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 노드별 매개변수 목록: 공통 파일, (있으면) 정지 역할 파일·로봇별 파일, 마지막에 launch 인자 사전
def parameter_files(context):
    files = [LaunchConfiguration('params_file').perform(context)]

    for arg in ('stop_evidence_file', 'robot_params_file'):
        path = LaunchConfiguration(arg).perform(context)
        if path:
            if not Path(path).is_file():
                raise RuntimeError(f'{arg} 없음: {path}')
            files.append(path)

    return files


def launch_nodes(context):
    files = parameter_files(context)
    robot_id = LaunchConfiguration('robot_id')
    flag = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)  # noqa: E731

    return [
        # B4는 B1 센서와 B2/B3 관측을 모아 A4가 읽는 SafetyState를 발행한다.
        # 로봇별 namespace로 감싸 여러 로봇의 B4 노드 이름이 겹치지 않게 한다.
        Node(package='ax_safety', executable='b4_risk_node', namespace=['/ax/', robot_id], output='screen',
             parameters=files + [{'robot_id': robot_id,
                                  # 시뮬레이션에서는 StateMeta 시각도 ROS 시뮬레이션
                                  # 시각을 사용하므로 launch에서 명시적으로 전달한다.
                                  'use_sim_time': True,
                                  'front_scan_topic': ['/ax/', robot_id,
                                                       '/sensors/front_lidar/scan_filtered'],
                                  'rear_scan_topic': ['/ax/', robot_id,
                                                      '/sensors/rear_lidar/scan_filtered']}]),

        # A4 주행 게이트는 SafetyState·odom·joint state·D2/D3 허가를 모두 확인한다.
        # 공통 파일(정지 판정 값 등)과 로봇별 파일(주행 자세)을 받고, 허가 플래그는 launch 인자가 마지막에 정한다.
        Node(package='ax_safety', executable='a4_safety_gate', namespace=['/ax/', robot_id], output='screen',
             parameters=files + [{'robot_id': robot_id,
                                  'use_sim_time': True,
                                  'enable_simulation_drive': flag('enable_simulation_drive'),
                                  'operational_permits_verified': flag('operational_permits_verified'),
                                  'travel_envelope_verified': flag('travel_envelope_verified'),
                                  'arm_operation_verified': flag('arm_operation_verified')}]),

        # 팔 action은 A3 lease를 확인하는 별도 서버를 거쳐 A2 controller로 전달된다.
        # 로봇별 namespace로 감싸 보호 팔 서버의 이름 충돌을 막는다.
        Node(package='ax_safety', executable='a4_protected_arm_server', namespace=['/ax/', robot_id], output='screen',
             parameters=files + [{'robot_id': robot_id,
                                  'use_sim_time': True,
                                  'enable_simulation_arm': flag('enable_simulation_arm')}]),

        # W3 A4 감독기. 기존 safety_gate의 ActuationState와 B4 SafetyState를
        # 소비하여 시계·상태 스트림·실제 STOPPED를 검증한다.
        # 감독기는 Schema 2를 다시 발행하지 않으며, 주행·팔 허가는 기존
        # safety_gate가 계속 최종 결정한다.
        Node(package='ax_safety', executable='a4_supervisor_node', namespace=['/ax/', robot_id], output='screen',
             parameters=[{'robot_id': robot_id,
                          'use_sim_time': True,
                          # D4가 전역 SAFE_STOP을 소유하므로 기본값은 false다.
                          # 팀 합의 후에만 D4 서비스 자동 요청을 활성화한다.
                          'request_system_safe_stop': False}]),
    ]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('ax_safety')) / 'config' / 'safety_limits.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        DeclareLaunchArgument('params_file', default_value=default_config,
                              description='로봇 공통 A4/B4 매개변수 파일(/**/<노드> 키)'),
        DeclareLaunchArgument('stop_evidence_file', default_value='',
                              description='모델별 정지 판정 역할 파일(config/stop_evidence_ax1|ax2.yaml). 비우면 STOPPED 미확인'),
        DeclareLaunchArgument('robot_params_file', default_value='',
                              description='로봇별 주행 자세·팔 한계 파일. 비우면 미검증·잠금'),
        # 각 인자는 명시적으로 true로 전달된 시험에서만 해당 안전 경로를 열도록 기본 false를 사용한다.
        DeclareLaunchArgument('enable_simulation_drive', default_value='false'),
        DeclareLaunchArgument('enable_simulation_arm', default_value='false'),
        DeclareLaunchArgument('operational_permits_verified', default_value='false'),
        DeclareLaunchArgument('travel_envelope_verified', default_value='false'),
        DeclareLaunchArgument('arm_operation_verified', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])
