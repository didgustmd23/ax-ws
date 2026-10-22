# 팀 연결 인터페이스

기준 소스: `ax.urdf.xacro`, `simulation.launch.py`, `arm_control.launch.py`, `gripper_controller.py`. 이 문서는 **현재 코드의 단일 `ax` 모델 인터페이스**만 다룬다. 팔 로봇 `robot_1`·적재함 로봇 `robot_2`의 협업 그룹, ros2_control 전환, 그룹·하위 Action과 보완된 `SafetyState`는 [통합 상세설계](../design/DETAILED_DESIGN.md)의 목표 계약이며 아직 구현되지 않았다. 길이 m, 각도 rad. 축은 joint 좌표계, xyz/rpy는 부모 링크 기준이다. 아래는 개발용 설정이며 실측 사양이 아니다. [AX 5축 사양안](../design/AX_5DOF_ARM_SPEC.md)의 v0.1 질량·URDF 속도 설정을 반영했다. 팔 조립체 질량은 받침·기둥·공통 어댑터 포함 16.2 kg이다. 충돌 상자와 가상 모터·하우징 질량 분포를 보완했고, [정적 지지 여유 계산](../validation/PHYSICAL_PROPERTIES_REVIEW.md)에서 일부 미필터링 관절 샘플의 지지 영역 이탈을 확인했다. 전 가동 범위의 하중 운반은 검증되지 않았다. 가속도는 `arm_joint_limits.yaml`의 계획값으로, 현재 Float64 브리지에서 강제하지 않는다.

## 링크와 관절

루트는 `base_link`. 현재 원통 회전·카메라·광학 프레임을 포함하여 `primitive` 집게 선택(Gazebo·공통 Xacro 기본값)은 관절 16개·링크 17개, `robotiq` 선택은 관절 23개·링크 24개다. `parallel_wide`는 관절 18개·링크 19개, RViz 기본값인 기준 공구 `suction`은 관절 20개·링크 21개다. 신규 집게의 관절·선택 방법은 [집게 비교 안내](GRIPPER_COMPARISON.md)를 참고한다. 몸통·바퀴·팔은 분리 OBJ를 연결했으며, 팔 관절 위치·축은 형상에서 추정했다. 관절각 0은 원본 OBJ 조립 자세다. 아래 표의 임시 손가락은 `primitive`에만 해당한다.

| 관절 | 부모 → 자식 | 종류 | xyz (m) | rpy (rad) | 축 | 위치 범위 |
|---|---|---|---|---|---|---|
| front_left_wheel_joint | base_link → front_left_wheel | continuous | 0.437 0.311 -0.19277325 | -π/2 0 0 | 0 0 1 | 연속 |
| front_right_wheel_joint | base_link → front_right_wheel | continuous | 0.437 -0.311 -0.19277325 | -π/2 0 0 | 0 0 1 | 연속 |
| rear_left_wheel_joint | base_link → rear_left_wheel | continuous | -0.443 0.311 -0.19277325 | -π/2 0 0 | 0 0 1 | 연속 |
| rear_right_wheel_joint | base_link → rear_right_wheel | continuous | -0.443 -0.311 -0.19277325 | -π/2 0 0 | 0 0 1 | 연속 |
| arm_mount_joint | base_link → arm_mount_link | fixed | 0.177 0.185 0.22922675 | 0 0 0 | 해당 없음 | 고정 |
| column_yaw_joint | arm_mount_link → column_link | revolute | 0 0 0.168 | 0 0 0 | 0 0 1 | -3.14159 ~ 3.14159 rad |
| arm_upper_mount_joint | column_link → arm_rotating_base_link | fixed | 0 0 0.72 | 0 0 0 | 해당 없음 | 고정 |
| shoulder_joint | arm_rotating_base_link → upper_arm_link | revolute | 0.06 0 0.1 | 0 0 0 | 1 0 0 | -1.5708 ~ 1.5708 rad |
| elbow_joint | upper_arm_link → forearm_link | revolute | 0.132 0 0.56 | 0 0 0 | 1 0 0 | -2.0 ~ 2.0 rad |
| wrist_joint | forearm_link → wrist_link | revolute | 0.09 0.455 -0.44 | 0 0 0 | 1 0 0 | -1.57 ~ 1.57 rad |
| gripper_rotation_joint | wrist_link → gripper_base_link | revolute | 0 0 -0.274 | 0 π/2 0 | 1 0 0 | -3.14159 ~ 3.14159 rad |
| left_finger_joint | gripper_base_link → left_finger_link | prismatic | 0.02 0.025 0 | 0 0 0 | 0 1 0 | -0.01 ~ 0.02 m |
| right_finger_joint | gripper_base_link → right_finger_link | prismatic | 0.02 -0.025 0 | 0 0 0 | 0 -1 0 | -0.01 ~ 0.02 m |
| lidar_joint | base_link → lidar_link | fixed | 0.20 0 0.16 | 0 0 0 | 해당 없음 | 고정 |
| camera_joint | base_link → camera_link | fixed | 0.20 0 0.20 | 0 0 0 | 해당 없음 | 고정 |
| camera_optical_joint | camera_link → camera_optical_frame | fixed | 0 0 0 | -1.570796 0 -1.570796 | 해당 없음 | 고정 |

손가락은 양의 축 방향이 반대이므로 같은 양의 변위로 벌어진다. `/gripper/command` 값은 전체 집게 폭이 아니라 손가락 각각의 목표 변위다. 바퀴 로컬 Z축은 rpy 회전으로 차체 Y축 방향에 정렬된다.

팔 메시 연결 후 어깨·팔꿈치·손목 축은 기존 임시 Y축에서 형상 기준 X축으로 변경했다. 상위 IK·목표 pose는 새 URDF 기준으로 계산해야 하며 과거 관절각·pose·검증 기록을 그대로 적용하지 않는다. 집게 본체·손가락은 기존 임시 형상을 손목 하단으로 옮겨 유지했고 실제 집게 연결기구는 아직 연결하지 않았다. 팔 collision은 연결 부품별 방향 상자로, 관성은 가상 질량 분포로 보완했으며 동적 제어·파지 성능은 아직 미검증이다.

`arm_mount_link`는 하단 받침·외측 지지 구조이며 고정된다. `column_link`는 중앙 원통과 3개 링으로, `column_yaw_joint` 회전 시 위쪽 팔 전체를 함께 회전시킨다. 중복된 상단 `arm_yaw_joint`는 제거하고 `arm_upper_mount_joint`로 고정했다. 팔의 수평 회전 명령은 `/column_yaw/command` 하나만 사용한다. 링의 가동부 귀속과 실제 기구 사양은 모델링 담당자 확인이 필요하다.

## ROS 입력 → Gazebo 명령

이 절의 집게 명령은 기존 `primitive` Gazebo 구성용이다. RViz 비교용 Robotiq은 `robotiq_85_left_knuckle_joint`의 0~0.8 rad 입력과 mimic 5개로 개폐하며, `/gripper/command`의 손가락 변위(m)를 그대로 적용하지 않는다. [집게 비교 안내](GRIPPER_COMPARISON.md)를 참고한다.

모든 입력 타입은 `std_msgs/msg/Float64`, Gazebo 타입은 `gz.msgs.Double`. 속도가 아닌 위치 목표이며 ROS → Gazebo 단방향이다.

| ROS 입력 | Gazebo 목적 토픽 | 범위 / 단위 |
|---|---|---|
| /column_yaw/command | /model/ax/column_yaw_joint/position | -3.14159 ~ 3.14159 rad (임시), URDF 최대 속도 0.35 rad/s 설정 |
| /shoulder/command | /model/ax/shoulder_joint/position | -1.5708 ~ 1.5708 rad |
| /elbow/command | /model/ax/elbow_joint/position | -2.0 ~ 2.0 rad |
| /wrist/command | /model/ax/wrist_joint/position | -1.57 ~ 1.57 rad |
| /gripper_rotation/command | /model/ax/gripper_rotation_joint/position | -3.14159 ~ 3.14159 rad |
| /left_finger/command | /model/ax/left_finger_joint/position | -0.01 ~ 0.02 m |
| /right_finger/command | /model/ax/right_finger_joint/position | -0.01 ~ 0.02 m |
| /gripper/command | 노드가 위 양쪽 손가락 ROS 토픽으로 같은 값을 분배 | -0.01 ~ 0.02 m |

공통 집게 명령과 개별 손가락 명령을 동시에 발행하면 경쟁하므로 발행 주체를 하나로 정한다. 범위 검사·상태 피드백·집기 완료 응답은 없다. 모델 이름 `ax` 변경 시 목적 토픽과 브리지도 수정한다.

## 센서·TF·주행

| 인터페이스 | 타입 / 프레임 | 제공 상태 |
|---|---|---|
| /scan | sensor_msgs/msg/LaserScan / lidar_link | Gazebo → ROS. 181 샘플, ±90도, 10 Hz 설정, 거리 0.12~10 m. 현재 물리 모델 변경 후 실제 수신은 재검증하지 않음 |
| /tf_static | tf2_msgs/msg/TFMessage | robot_state_publisher가 고정 관절 TF 발행 |
| /tf | tf2_msgs/msg/TFMessage | 이동 관절 TF에는 joint_states 입력 필요. 현재 시뮬레이션 상태 공급 없음 |
| /joint_states | sensor_msgs/msg/JointState | 현재 Gazebo 피드백 발행·브리지 없음. display GUI 값은 실제 상태가 아님 |
| /clock | rosgraph_msgs/msg/Clock | ROS 브리지 없음, use_sim_time 설정도 없음 |
| /cmd_vel | geometry_msgs/msg/Twist | ROS → Gazebo `/model/ax/cmd_vel` 단방향 브리지. linear.x m/s, angular.z rad/s. 구동 차단 게이트·watchdog 없음 |
| /odom | 미확정 | ROS 수신 브리지 없음 |
| /camera/image_raw, /camera/camera_info | sensor_msgs/msg/Image, CameraInfo / camera_optical_frame | Gazebo → ROS. RGB 640×480, 30 Hz 설정, 수평 시야각 약 60도. 깊이 출력 없음. 실제 수신은 재검증하지 않음 |
| odom → base_link | TF | ROS 전달 구성 없음 |

DiffDrive는 네 바퀴를 좌우 쌍으로 사용한다. 분리 OBJ 장착 후 반지름 0.19631 m, 폭 0.144 m, 좌우 간격 0.622 m로 변경했다. 이는 mm 단위 가정하에 메시 외곽으로 추정한 값으로 실제 접지 반지름·주행 보정값은 아니다. 이전 실제 실행에서 Gazebo `/model/ax/cmd_vel`, `/model/ax/odometry`, `/model/ax/tf`가 발견되었다. 바퀴 교체 후 물리 주행은 재검증하지 않았다. Gazebo 토픽 존재와 ROS 브리지 제공은 별개이며 Nav2 연결 완료를 의미하지 않는다.

현재 센서는 전방 라이다 1개와 RGB 카메라 1개다. 설계에서 확정한 전·후방 라이다, RGB-D 카메라, IMU, robot_2 적재함 접촉 센서는 아직 모델에 없다. 인식 토픽·팔 자세 목표 API·흡착·집기 결과 API는 아직 없다. 지도 YAML과 PGM은 함께 전달한다.
