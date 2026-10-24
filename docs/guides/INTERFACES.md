# 팀 연결 인터페이스

기준 소스: `ax.urdf.xacro`, `simulation.launch.py`, `arm_control.launch.py`, `gripper_controller.py`. 이 문서는 **현재 AX_1·AX_2 개발 모델과 센서·Gazebo 플러그인 인터페이스**를 다룬다. `simulation.launch.py robot_model:=ax1`은 기존 모델 `ax`와 루트 명령 토픽을 유지하고, `ax2`는 `robot_2`, `both`는 `robot_1`·`robot_2`를 생성한다. 팔 로봇 `robot_1`·적재함 로봇 `robot_2`의 협업 그룹, ros2_control 전환, 그룹·하위 Action과 보완된 `SafetyState`는 [통합 상세설계](../design/DETAILED_DESIGN.md)의 목표 계약이며 아직 구현되지 않았다. 길이 m, 각도 rad. 축은 joint 좌표계, xyz/rpy는 부모 링크 기준이다. 아래는 개발용 설정이며 실측 사양이 아니다. [AX 5축 사양안](../design/AX_5DOF_ARM_SPEC.md)의 v0.1 질량·URDF 속도 설정을 반영했다. 팔 조립체 질량은 센서 제외, 받침·기둥·공통 어댑터 포함 16.2 kg이며 손목 카메라 2개의 개발용 질량 0.3 kg은 별도다. 충돌 상자와 가상 모터·하우징 질량 분포를 보완했고, [정적 지지 여유 계산](../validation/PHYSICAL_PROPERTIES_REVIEW.md)에서 일부 미필터링 관절 샘플의 지지 영역 이탈을 확인했다. 전 가동 범위의 하중 운반은 검증되지 않았다. 가속도는 `arm_joint_limits.yaml`의 계획값으로, 현재 Float64 브리지에서 강제하지 않는다.

## 링크와 관절

루트는 `base_link`. 현재 원통 회전·카메라·광학 프레임을 포함하여 `primitive` 집게 선택(Gazebo·공통 Xacro 기본값)은 관절 24개·링크 25개, `robotiq` 선택은 관절 31개·링크 32개다. `parallel_wide`는 관절 26개·링크 27개, RViz 기본값인 기준 공구 `suction`은 관절 28개·링크 29개다. 신규 집게의 관절·선택 방법은 [집게 비교 안내](GRIPPER_COMPARISON.md)를 참고한다. 몸통·바퀴·팔은 분리 OBJ를 연결했으며, 팔 관절 위치·축은 형상에서 추정했다. 관절각 0은 팔 내부의 원본 OBJ 조립 자세를 유지하며, 팔 조립체 전체는 회전 기둥 축이 차체 X/Y 중심에 오도록 원본 대비 X=-177 mm, Y=-185 mm 이동했다. 아래 표의 임시 손가락은 `primitive`에만 해당한다.

| 관절 | 부모 → 자식 | 종류 | xyz (m) | rpy (rad) | 축 | 위치 범위 |
|---|---|---|---|---|---|---|
| front_left_wheel_joint | base_link → front_left_wheel | continuous | 0.437 0.311 -0.19277325 | -π/2 0 0 | 0 0 1 | 연속 |
| front_right_wheel_joint | base_link → front_right_wheel | continuous | 0.437 -0.311 -0.19277325 | -π/2 0 0 | 0 0 1 | 연속 |
| rear_left_wheel_joint | base_link → rear_left_wheel | continuous | -0.443 0.311 -0.19277325 | -π/2 0 0 | 0 0 1 | 연속 |
| rear_right_wheel_joint | base_link → rear_right_wheel | continuous | -0.443 -0.311 -0.19277325 | -π/2 0 0 | 0 0 1 | 연속 |
| arm_mount_joint | base_link → arm_mount_link | fixed | 0 0 0.22922675 | 0 0 0 | 해당 없음 | 고정 |
| column_yaw_joint | arm_mount_link → column_link | revolute | 0 0 0.168 | 0 0 0 | 0 0 1 | -3.14159 ~ 3.14159 rad |
| arm_upper_mount_joint | column_link → arm_rotating_base_link | fixed | 0 0 0.5753438 | 0 0 0 | 해당 없음 | 고정 |
| shoulder_joint | arm_rotating_base_link → upper_arm_link | revolute | 0.1144427 0 0.1 | 0 0 0 | 1 0 0 | -1.5708 ~ 1.5708 rad |
| elbow_joint | upper_arm_link → forearm_link | revolute | 0.132 0 0.56 | 0 0 0 | 1 0 0 | -2.0 ~ 2.0 rad |
| wrist_joint | forearm_link → wrist_link | revolute | 0.09 0.455 -0.44 | 0 0 0 | 1 0 0 | -1.57 ~ 1.57 rad |
| gripper_rotation_joint | wrist_link → gripper_base_link | revolute | 0 0 -0.274 | 0 π/2 0 | 1 0 0 | -3.14159 ~ 3.14159 rad |
| left_finger_joint | gripper_base_link → left_finger_link | prismatic | 0.02 0.025 0 | 0 0 0 | 0 1 0 | -0.01 ~ 0.02 m |
| right_finger_joint | gripper_base_link → right_finger_link | prismatic | 0.02 -0.025 0 | 0 0 0 | 0 -1 0 | -0.01 ~ 0.02 m |

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

센서 토픽은 실행 모드와 무관하게 `/ax/robot_1/sensors/` 또는 `/ax/robot_2/sensors/` 아래에 있다. 이전 `/scan`, `/camera/image_raw`, `/camera/camera_info` 전역 별칭은 제거했다. 센서 장착 좌표·방향의 기준은 `src/ax_description/config/sensor_mounts.yaml`이다. 아래 해상도·주기·질량·장착점은 제품 사양이나 보정 결과가 아닌 개발 가정이다.

| 센서 | AX_1 | AX_2 | 출력·설정 |
|---|---|---|---|
| `front_lidar`, `rear_lidar` | 각 1개 | 각 1개 | `.../{name}/scan`: LaserScan, 181점·±90°·10 Hz·0.12~10 m. 후방 링크 yaw=π |
| `front_rgbd` | 1개 | 1개 | `.../front_rgbd/{image,depth_image,points,camera_info}`: Image·PointCloud2·CameraInfo |
| `front_rgb` | 1개 | 1개 | `.../front_rgb/{image_raw,camera_info}` |
| `arm_rgbd`, `arm_rgb` | 손목 좌우에 각 1개 | 없음 | 위 카메라와 같은 접미사. q=0에서 아래쪽을 봄 |
| `imu` | 몸통 1개 | 몸통 1개 | `.../imu`: Imu, 100 Hz, 개발용 이상적 센서 |
| `cargo_contact` | 없음 | 중간 선반 1개 | `.../cargo_contact`: ros_gz_interfaces/Contacts, 30 Hz. 적재 판정이 아닌 접촉 원시값 |

카메라 설정은 RGB 640×480·15 Hz·수평 FOV 60°이며 RGB-D 깊이 clip은 0.05~5 m다. 카메라 `{name}_link`는 +X 전방이고 `{name}_optical_frame`은 +Z 전방·+X 오른쪽·+Y 아래다. 전방 카메라는 차체 앞을 보며, 팔 카메라는 `wrist_link`를 따라 움직인다. 작업 자세별 가림·관측 범위·내참수 보정·실측 주기는 별도 검증 대상이다. AX_2의 `ax_top_lidar.obj`는 보존한 외형이며 세 번째 측정 센서는 아니다.

기본 단일 AX_1 실행의 명령 `/cmd_vel`, 팔 명령, `/joint_states`, `/odom`은 기존 루트 이름을 사용한다. AX_2 단독 실행과 두 로봇 실행에서는 `/ax/robot_i/{cmd_vel,joint_states,odom}` 및 AX_1의 `/ax/robot_1/{column_yaw,shoulder,elbow,wrist,gripper_rotation,left_finger,right_finger,gripper}/command`를 사용한다. AX_2에는 팔 명령이 없다. `arm_control.launch.py`는 단일 `ax`용 기존 보조 launch이므로 simulation과 중복 실행하지 않는다.

| 상태·시간 | 제공 구성 |
|---|---|
| `joint_states` | Gazebo JointStatePublisher → gz.msgs.Model → sensor_msgs/JointState |
| `odom` | DiffDrive → nav_msgs/Odometry |
| `/tf` | DiffDrive의 odom→base_link 및 robot_state_publisher의 이동 관절 TF |
| `/tf_static` | robot_state_publisher의 고정 관절 TF |
| `/clock` | 월드 clock → rosgraph_msgs/Clock, 시뮬레이션 노드 use_sim_time=true |

단일 AX_1의 프레임 이름은 `base_link`, `front_lidar_link` 등이다. AX_2 단독·두 로봇 실행에서는 `robot_i/base_link`, `robot_i/odom`, `robot_i/front_rgbd_optical_frame`처럼 접두사를 붙인다. 같은 `/tf`, `/tf_static`에서 로봇별 트리가 구분되지만 공통 `map` 연결·위치 추정·Nav2는 미구현이다. RViz display는 단일 모델용 GUI JointState를 사용하며 물리 피드백이 아니다.

AX_1 바퀴 반경·좌우 간격은 0.19631 m·0.622 m다. AX_2는 현재 OBJ에서 생성한 약 0.149901 m·0.836695 m이며 차체 bbox는 약 1.043×0.996×0.406 m다. AX_2는 원본 조립 좌표에서 차체 bbox 중심을 공통으로 빼서 8개 메시의 상대 위치를 유지한다. 선반 외형은 열린 부품을 포함해 Y 방향 약 1.993 m다. 충돌은 생성기의 보수적 부품/면 상자 근사이며 최종 적재 공간이나 허용 하중을 보증하지 않는다.

AX_2의 개발용 질량 예산은 몸통 12 kg·선반 5 kg·바퀴 각 0.4 kg·전방 장식 0.05 kg·상단 하우징 0.4 kg이다. 센서와 접촉 패드 질량은 별도다. `ax_2_physical_properties.json`과 `ax_2_geometry.xacro`는 `generate_physical_properties.py`가 생성한다. AX_1과 질량·COM·관성을 공유하지 않는다.

월드는 Sensors·Imu·Contact 시스템을 한 번만 로드한다. URDF는 실행마다 `/tmp/ax_sim_*/robot_i.urdf`로 따로 생성한다. 독립 시뮬레이션을 동시에 실행하려면 ROS domain과 Gazebo partition을 모두 격리해야 한다. 두 모델 생성과 센서·상태 브리지 구성은 협업 Task Executor·흡착·접촉+영역 기반 적재 판정·안전 차단의 구현 완료를 뜻하지 않는다.
