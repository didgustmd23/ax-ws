"""ROS/Gazebo sensor and command contracts for the development models."""


ARM_CHANNELS = (
    ('column_yaw_joint', 'column_yaw'), ('shoulder_joint', 'shoulder'),
    ('elbow_joint', 'elbow'), ('wrist_joint', 'wrist'),
    ('gripper_rotation_joint', 'gripper_rotation'),
    ('left_finger_joint', 'left_finger'), ('right_finger_joint', 'right_finger'),
)


def sensor_bridges(robot_id):
    """Return (Gazebo topic, ROS type, Gazebo type) without global aliases."""
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
    if robot_id == 'robot_2':
        # 접촉 원시값이며 적재 완료/상자 보유 판정은 별도 구현 대상이다.
        result.append((f'{prefix}/cargo_contact', 'ros_gz_interfaces/msg/Contacts',
                       'gz.msgs.Contacts'))
    return result
