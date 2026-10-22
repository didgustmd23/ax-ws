from launch import LaunchDescription
from launch_ros.actions import Node


# 팔·집게 위치 명령을 ROS에서 Gazebo로 전달한다. 모델 생성은 simulation.launch.py가 담당한다.
# 브리지 표기: 토픽@ROS타입]Gazebo타입에서 ']'는 ROS → Gazebo 단방향을 뜻한다.
# remappings는 ROS 쪽 이름만 바꾸며, Gazebo 목적 토픽은 /model/ax/...로 유지된다.
# Float64 값은 회전 관절에서 rad, 직선 이동 손가락에서 m 단위의 목표 위치다.
def generate_launch_description():
    return LaunchDescription([
        # /gripper/command 값을 양쪽 /left_finger/command, /right_finger/command로 분배한다.
        # 공통 집게 명령과 개별 손가락 명령을 동시에 보내면 목표값이 경쟁할 수 있다.
        Node(
            package='ax_control',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen'
        ),

        # 왼손 손가락: 양의 목표 변위는 +Y 방향으로 벌림. 허용 범위 -0.01~0.02 m.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='left_finger_bridge',
            arguments=[
                '/model/ax/left_finger_joint/position'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            remappings=[
                ('/model/ax/left_finger_joint/position',
                 '/left_finger/command')
            ],
            output='screen'
        ),

        # 오른손 손가락: 양의 목표 변위는 -Y 방향으로 벌림. 허용 범위 -0.01~0.02 m.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='right_finger_bridge',
            arguments=[
                '/model/ax/right_finger_joint/position'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            remappings=[
                ('/model/ax/right_finger_joint/position',
                 '/right_finger/command')
            ],
            output='screen'
        ),

        # 중앙 원통: 하단 받침은 고정하고 원통·팔 전체를 Z축 회전한다.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='column_yaw_bridge',
            arguments=[
                '/model/ax/column_yaw_joint/position'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            remappings=[
                ('/model/ax/column_yaw_joint/position', '/column_yaw/command')
            ],
            output='screen'
        ),

        # 어깨: 형상 기준 X축 굽힘 목표. 허용 범위 -1.5708~1.5708 rad.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='shoulder_bridge',
            arguments=[
                '/model/ax/shoulder_joint/position'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            remappings=[
                ('/model/ax/shoulder_joint/position',
                 '/shoulder/command')
            ],
            output='screen'
        ),

        # 팔꿈치: 형상 기준 X축 굽힘 목표. 허용 범위 -2.0~2.0 rad.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='elbow_bridge',
            arguments=[
                '/model/ax/elbow_joint/position'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            remappings=[
                ('/model/ax/elbow_joint/position',
                 '/elbow/command')
            ],
            output='screen'
        ),

        # 손목: 형상 기준 X축 굽힘 목표. 허용 범위 -1.57~1.57 rad.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='wrist_bridge',
            arguments=[
                '/model/ax/wrist_joint/position'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            remappings=[
                ('/model/ax/wrist_joint/position',
                 '/wrist/command')
            ],
            output='screen'
        ),

        # 집게 회전: X축 롤 목표. 허용 범위 약 -π~π rad.
        # 이 launch는 명령만 전달하며 관절 상태 피드백이나 입력 범위 검사를 제공하지 않는다.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gripper_rotation_bridge',
            arguments=[
                '/model/ax/gripper_rotation_joint/position'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            remappings=[
                ('/model/ax/gripper_rotation_joint/position',
                 '/gripper_rotation/command')
            ],
            output='screen'
        ),
    ])
