"""김성민 담당 노드를 상대 파트의 공통 경로에 연결해 실행합니다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_nodes(context):
    """로봇·모델 조합을 검증한 뒤 내 담당 노드만 구성합니다."""
    # B2와 같은 robot_ids 형식을 사용합니다. ID와 모델의 개수·순서가 맞아야 합니다.
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
    with_w3 = LaunchConfiguration('with_w3', default='false').perform(context).lower()
    if with_w3 not in ('true', 'false'):
        raise ValueError('with_w3 must be true or false')
    nodes = []
    for robot, model in zip(robot_ids, models):
        # 기본은 로봇별 A3·B3다. 상대 파트 노드나 안전 허용 설정은 변경하지 않는다.
        common = {'robot_id': robot, 'use_sim_time': True}
        # 상대가 Gazebo 시각으로 보낸 stamp를 벽시계와 비교하지 않도록 내 노드의 시계를 맞춘다.
        nodes.extend([
            Node(package='ax_task_executor', executable='local_task_executor',
                 namespace=f'/ax/{robot}', name='local_task_executor', output='screen',
                 parameters=[dict(common, policy='operational')]),
            Node(package='ax_environment', executable='corridor_observer',
                 namespace=f'/ax/{robot}', name='corridor_observer', output='screen',
                 parameters=[dict(common, robot_model=model, regions_file=regions,
                                  map_revision=revision)]),
        ])
        if with_w3 == 'true' and model == 'AX_1':
            # C3는 조작 역할 모델에만 시작한다. 실제 C2/C4 호출·검증기가 없으면 후보는 비어 있다.
            nodes.append(Node(
                package='ax_grasp_planner', executable='candidate_publisher',
                namespace=f'/ax/{robot}', name='candidate_publisher', output='screen',
                parameters=[common]))
    if with_w3 == 'true':
        # D3는 전체 로봇을 받는 단일 원장이다. 두 번 시작하거나 미합의 정책을 켜지 않는다.
        database = LaunchConfiguration('corridor_database', default='').perform(context)
        parameters = {'use_sim_time': True, 'robot_ids': robot_ids, 'regions_file': regions}
        # B3와 D3는 같은 영역 파일을 읽는다. 관측 경계와 허가 대상이 서로 달라지지 않게 한다.
        if database:
            parameters['database'] = database
        nodes.append(Node(
            package='ax_corridor_manager', executable='corridor_manager',
            name='corridor_manager', output='screen', parameters=[parameters]))
    return nodes


def generate_launch_description():
    """B2와 같은 로봇 목록 규칙에 선택적 C3/D3 실행 설정을 제공합니다."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_ids', default_value='robot_1,robot_2'),
        DeclareLaunchArgument('robot_models', default_value='AX_1,AX_2'),
        DeclareLaunchArgument('regions_file', default_value=''),
        DeclareLaunchArgument('with_w3', default_value='false'),
        DeclareLaunchArgument('corridor_database', default_value=''),
        # B2 채택 지도와 B3 영역의 번호를 맞춘다. 운영 영역 승인이나 배정을 대신 허용하지 않는다.
        DeclareLaunchArgument('map_revision', default_value='2'),
        OpaqueFunction(function=setup_nodes),
    ])
