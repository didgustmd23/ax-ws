"""개발 모델의 ROS/Gazebo 센서·명령 계약."""


# ros2_control 명령 계약. 팔은 arm_controller(JointTrajectoryController) 하나가,
# primitive 손가락은 finger_controller 하나가 명령한다. 관절 순서는 제어기 설정과 같다.
ARM_JOINTS = ('column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint',
              'gripper_rotation_joint')
FINGER_JOINTS = ('left_finger_joint', 'right_finger_joint')


# 로봇별로 실행할 제어기 이름을 활성화 순서대로 돌려준다.
def controllers(robot_id, tool='primitive'):
    # 상태 방송을 먼저 켜서 다른 제어기 활성화 전후의 관절 상태를 남긴다.
    names = ['joint_state_broadcaster', 'diff_drive_controller']
    if robot_id == 'robot_1':
        names.append('arm_controller')
        if tool == 'primitive':
            names.append('finger_controller')
    elif robot_id != 'robot_2':
        raise ValueError(f'Unknown robot: {robot_id}')
    return names


# 로봇별 센서 브리지 목록 (Gazebo 토픽, ROS 형식, Gazebo 형식)을 전역 별칭 없이 돌려준다.
def sensor_bridges(robot_id, tool='primitive'):
    # tool은 AX_1 gripper_model이며 'suction'에만 흡착판 중앙 ToF가 있다.
    if robot_id not in ('robot_1', 'robot_2'):
        raise ValueError(f'Unknown sensor robot: {robot_id}')

    # Gazebo 센서와 ROS 구독자가 같은 로봇별 경로를 사용한다.
    prefix = f'/ax/{robot_id}/sensors'
    result = [(f'{prefix}/{side}_lidar/scan', 'sensor_msgs/msg/LaserScan',
               'gz.msgs.LaserScan') for side in ('front', 'rear')]
    result.append((f'{prefix}/imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU'))

    # 카메라 수에 RGB-D도 포함한다. 팔 카메라는 robot_1에만 있다.
    cameras = [('front_rgbd', True), ('front_rgb', False)]
    if robot_id == 'robot_1':
        cameras += [('arm_rgbd', True), ('arm_rgb', False)]

    for name, depth in cameras:
        topic = f'{prefix}/{name}'
        image = 'image' if depth else 'image_raw'
        result.extend([(f'{topic}/{image}', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                       (f'{topic}/camera_info', 'sensor_msgs/msg/CameraInfo',
                        'gz.msgs.CameraInfo')])
        if depth:
            # RGB-D의 영상·깊이·점군을 각각 브리지한다.
            result.extend([(f'{topic}/depth_image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                           (f'{topic}/points', 'sensor_msgs/msg/PointCloud2',
                            'gz.msgs.PointCloudPacked')])

    if robot_id == 'robot_1' and tool == 'suction':
        # 1빔 LaserScan(ranges[0], m, 센서 원점 기준)으로 전달해 원시 무효값 inf를 보존한다.
        # 설치된 브리지의 LaserScan->Range 변환은 +inf를 max_range + 1(범위 밖 유한값)로
        # 바꾸므로 쓰지 않는다. 구독 계약은 docs/guides/INTERFACES.md를 따른다.
        result.append((f'{prefix}/suction_tof/scan', 'sensor_msgs/msg/LaserScan',
                       'gz.msgs.LaserScan'))

    if robot_id == 'robot_2':
        # 접촉 원시값이며 적재 완료/상자 보유 판정은 별도 구현 대상이다.
        result.append((f'{prefix}/cargo_contact', 'ros_gz_interfaces/msg/Contacts',
                       'gz.msgs.Contacts'))

    return result
