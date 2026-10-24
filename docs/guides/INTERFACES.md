# 팀 연결 인터페이스

기준 소스: `ax.urdf.xacro`, `simulation.launch.py`, `arm_control.launch.py`, `gripper_controller.py`. 이 문서는 **현재 AX_1·AX_2 개발 모델과 센서·Gazebo 플러그인 인터페이스**를 다룬다. `simulation.launch.py robot_model:=ax1`은 기존 모델 `ax`와 루트 명령 토픽을 유지하고, `ax2`는 `robot_2`, `both`는 `robot_1`·`robot_2`를 생성한다. 팔 로봇 `robot_1`·적재함 로봇 `robot_2`의 협업 그룹, ros2_control 전환, 그룹·하위 Action과 보완된 `SafetyState`는 [통합 상세설계](../design/DETAILED_DESIGN.md)의 목표 계약이며 아직 구현되지 않았다. 길이 m, 각도 rad. 축은 joint 좌표계, xyz/rpy는 부모 링크 기준이다. 아래는 개발용 설정이며 실측 사양이 아니다. [AX 5축 사양안](../design/AX_5DOF_ARM_SPEC.md)의 v0.1 질량·URDF 속도 설정을 반영했다. 팔 조립체 질량은 센서 제외, 받침·기둥·공통 어댑터 포함 16.2 kg이며 팔 카메라 2개(손목 RGB-D·상완 RGB)의 개발용 질량 0.3 kg과 `suction` 공구 ToF 0.01 kg은 별도다. 충돌 상자와 가상 모터·하우징 질량 분포를 보완했고, [정적 지지 여유 계산](../validation/PHYSICAL_PROPERTIES_REVIEW.md)에서 일부 미필터링 관절 샘플의 지지 영역 이탈을 확인했다. 전 가동 범위의 하중 운반은 검증되지 않았다. 가속도는 `arm_joint_limits.yaml`의 계획값으로, 현재 Float64 브리지에서 강제하지 않는다.

## 링크와 관절

루트는 `base_link`. 현재 원통 회전·카메라·광학 프레임을 포함하여 `primitive` 집게 선택(Gazebo·공통 Xacro 기본값)은 관절 24개·링크 25개, `robotiq` 선택은 관절 31개·링크 32개다. `parallel_wide`는 관절 26개·링크 27개, RViz 기본값인 기준 공구 `suction`은 ToF 포함 관절 29개·링크 30개다. 신규 집게의 관절·선택 방법은 [집게 비교 안내](GRIPPER_COMPARISON.md)를 참고한다. 몸통·바퀴·팔은 분리 OBJ를 연결했으며, 팔 관절 위치·축은 형상에서 추정했다. 관절각 0은 팔 내부의 원본 OBJ 조립 자세를 유지하며, 팔 조립체 전체는 회전 기둥 축이 차체 X/Y 중심에 오도록 원본 대비 X=-177 mm, Y=-185 mm 이동했다. 아래 표의 임시 손가락은 `primitive`에만 해당한다.

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

공통 집게 명령과 개별 손가락 명령을 동시에 발행하면 경쟁하므로 발행 주체를 하나로 정한다. 범위 검사·상태 피드백·집기 완료 응답은 없다. 모델 이름 `ax` 변경 시 목적 토픽과 브리지도 수정한다. 현재 Gazebo에서는 어깨 명령 −1.0 rad에도 `shoulder_joint`가 하한 −1.5708 rad 부근에 머무는 현상이 관찰됐다(명령 없는 상태 포함, 센서 배치 변경 전 모델에서도 동일). URDF 질량으로 계산한 무부하(상자 없음) 어깨축 정적 중력 토크 크기는 영 자세에서 `primitive` 11.6 N·m·`suction` 13.6 N·m이고, 선택한 작업 자세 3개(q_shoulder, q_elbow, q_wrist = −1.0·1.0·0, −0.9·0.61·0.29, −1.5708·0.95·0.4 rad)에서 `primitive` 33.3~38.8 N·m·`suction` 36.7~42.9 N·m다. URDF effort 제한은 20 N·m, 위치 제어기는 `p_gain`만 설정돼 있다. 명령 수신·제어 출력 제한·이득·충돌 여부는 확인하지 않았으므로 원인은 미확정이다.

## 센서·TF·주행

센서 토픽은 실행 모드와 무관하게 `/ax/robot_1/sensors/` 또는 `/ax/robot_2/sensors/` 아래에 있다. 이전 `/scan`, `/camera/image_raw`, `/camera/camera_info` 전역 별칭은 제거했다. 센서 장착 좌표·방향의 기준은 `src/ax_description/config/sensor_mounts.yaml`이다. 아래 해상도·주기·질량·장착점은 제품 사양이나 보정 결과가 아닌 개발 가정이다.

| 센서 | AX_1 | AX_2 | 출력·설정 |
|---|---|---|---|
| `front_lidar`, `rear_lidar` | 각 1개 | 각 1개 | `.../{name}/scan`: LaserScan, 181점·±90°·10 Hz·0.12~10 m. 후방 링크 yaw=π |
| `front_rgbd` | 1개 | 1개 | `.../front_rgbd/{image,depth_image,points,camera_info}`: Image·PointCloud2·CameraInfo |
| `front_rgb` | 1개 | 1개 | `.../front_rgb/{image_raw,camera_info}` |
| `arm_rgbd` | `wrist_link` 바깥쪽(+Y) 1개 | 없음 | 위 RGB-D와 같은 접미사. `suction`은 흡착판 바깥 상자 윗면을 비스듬히 보는 좌표, 그 외 공구는 기존 손목 좌표 |
| `suction_tof` | `suction` 공구에서만 `suction_base_link` 4컵 중앙 1개 | 없음 | `.../suction_tof/scan`: 1빔 LaserScan, `ranges[0]`(m, 센서 원점 기준), 0.02~0.50 m, 30 Hz. 범위 밖·무반사는 `inf` |
| `arm_rgb` | `upper_arm_link` 1개 | 없음 | 위 RGB와 같은 접미사. 상완 +Y 면 하단에서 공구 말단·상자 윗면 쪽을 봄 |
| `imu` | 몸통 1개 | 몸통 1개 | `.../imu`: Imu, 100 Hz, 개발용 이상적 센서 |
| `cargo_contact` | 없음 | 중간 선반 1개 | `.../cargo_contact`: ros_gz_interfaces/Contacts, 30 Hz. 적재 판정이 아닌 접촉 원시값 |

카메라 설정은 RGB 640×480·15 Hz·수평 FOV 60°이며 RGB-D 깊이 clip은 0.05~5 m다. 카메라 `{name}_link`는 +X 전방이고 `{name}_optical_frame`은 +Z 전방·+X 오른쪽·+Y 아래다. 몸체 전방은 두 로봇 모두 `front_rgb`(Y=+0.10)–`front_lidar`(Y=0)–`front_rgbd`(Y=−0.10)를 같은 높이(AX_1 Z=0.20, AX_2 Z=0.14, `base_link` 기준)에 두고 모두 +X를 향한다. LiDAR는 ±90° 스캔면이 카메라 몸체 앞을 지나도록 카메라(AX_1 X=0.73, AX_2 X=0.57)보다 0.04 m 앞에 있다. 이 높이에서는 그보다 낮은 장애물이 전방 LiDAR 스캔면에 잡히지 않는다. 팔 센서 역할은 `arm_rgbd`가 접근 전 물체 표면·높이·주변 형상 관찰, `suction_tof`가 최종 접근 간격 측정, `arm_rgb`가 컵 말단과 물체 접촉 장면 관찰이다. `arm_rgbd`는 모든 공구에서 `wrist_link`에 있다. `gripper_model:=suction`에서는 xyz (0, 0.18, −0.20) m, rpy (π, 1.2606, −π/2)로 광축이 손목 기준 (0, 0.13, −0.356) m, 즉 컵 바깥의 상자 윗면을 향하며 말단 회전을 따라 돌지 않는다. 손목 하우징까지의 브래킷은 모델링하지 않았다. 흡착판이 없는 `primitive`·`robotiq`·`parallel_wide`(Gazebo 기본 `primitive` 포함)는 이전 손목 좌표(0, 0.13, −0.18 m, rpy 0, 1.1570, −π/2)를 유지한다. `suction_tof`는 `suction` 공구에서만 생성되며 `suction_base_link`(흡착판 윗면 원점, +Z가 물체 방향) 기준 xyz (0, 0, 0.031) m, rpy (0, −π/2, 0)으로 측정 방향(링크 +X)이 컵 방향이고 `gripper_rotation_joint`와 함께 돈다. 0.012×0.02×0.02 m 몸체는 흡착판 아래 z 0.025~0.037 m에 있어 흡착판과 겹치지 않고 컵 끝면(z 0.062 m)보다 돌출되지 않는다. Gazebo `gpu_lidar` 1샘플로 근사한 개발용 센서이며 특정 실물 ToF의 정확도·재질 응답을 재현하지 않는다. 측정값은 센서 원점 기준 거리이고 컵 끝면까지의 접근 간격은 `ranges[0] − 0.031` m다(접촉 시 원시값은 약 0.031 m이며 0이 아니다). 측정 범위 0.02~0.50 m는 접근 전(간격 0.15 m)부터 접촉까지 포함하도록 정한 개발 가정이다. 설치된 `ros_gz_bridge`의 `gz.msgs.LaserScan`→`sensor_msgs/msg/Range` 변환은 원시 `+inf`를 `max_range + 1`(현재 1.5 m)이라는 범위 밖 유한값으로 바꾸므로(`max_range` 2.0에서 3.0으로 확인), 원시 무효값 `inf`를 보존하는 현재 1빔 `sensor_msgs/msg/LaserScan`을 유지한다. 구독 계약: 구독자는 `ranges` 길이가 1인지, `ranges[0]`이 유한값이고 `range_min` 이상 `range_max` 이하인지, 메시지 시각이 허용 지연 안인지 확인한 뒤에만 `ranges[0] − 0.031` m로 환산한다. 여기서 `lip_offset` 0.031 m는 현재 형상의 컵 끝면 z 0.062 m와 센서 원점 z 0.031 m의 차이이며, 환산값은 변형되지 않은 컵 끝면까지의 중앙 광선 기준 접근 간격일 뿐 네 컵 모두의 접촉이나 흡착 성공을 뜻하지 않는다. `inf`·`-inf`·NaN·범위 밖·오래된 값과 음수 간격은 무효로 다루며 0 또는 접촉 성공으로 바꾸지 않는다. 허용 지연과 이 값을 쓰는 구독 노드는 아직 정하지 않았다. 브리지는 `model_contracts.sensor_bridges(robot_id, tool="suction")`에서만 추가된다. 기본 `simulation.launch.py`는 `primitive` 구성이라 ToF 토픽이 없고, 흡착 공구·ToF는 별도로 생성한 흡착 공구 사본과 전용 브리지로만 Gazebo 검증했다. `arm_rgb`는 `upper_arm_link` 기준 xyz (0.05, 0.142, 0.15) m에서 같은 위치의 방향 3안 중 rpy (2.1677, −0.2604, 1.2929)를 택했다. 관찰용 영상이며 실제 접촉·흡착 성공 판정이 아니다. 정적 기하 계산(개발용 가정: 작업대 높이 0.45·0.60·0.75·0.90 m, 0.30×0.34×0.30 m 상자를 작업대에 고정, 흡착 공구·공구축 수직, 컵 끝면 위 0.15·0.10·0.05·0.03·0.02·0.015·0.01·0.005 m에서 접촉까지 하강, 상자를 붙여 0.02·0.05·0.10 m 들어올림, 말단 회전 0·π/2, 가림 형상에 차체·바퀴·팔·공구·다른 팔 센서·작업대·상자 포함, 관측 센서 자신만 제외)에서 480자세를 비교했다. 가림 판정은 목표 자신의 표면 끝점 교차만 제외하고 다른 형상은 목표 1 mm 앞이라도 가림으로 센다. 거리 표본은 기둥 좌표계에서 흡착 기준점의 Y 성분 0.60·0.75·0.90(측면)·0.95·1.05 m(전방)이며 FK로 계산한 기둥축까지 실제 수평거리는 각각 0.688·0.822·0.961·1.008·1.103 m다. 결과: `arm_rgbd` 광축은 480자세 모두 상자에 닿고 깊이가 유효하며, 흡착판 바깥 상자 윗면 표본(16개 중 6개 이상)은 접근 전·하강 0.10 m·접근 단계 120/120에서 보이고 간격 0.03 m 이하에서는 평균 3개로 줄어든다(이전 4컵 중앙 RGB-D는 0/480, 접촉·들어올림 깊이 무효). `suction_tof` 광선은 모든 단계에서 상자에 닿고(컵·흡착판 미측정) 접촉·들어올림까지 범위 안이다. `arm_rgb`의 컵·상자 윗면 동시 가시성(컵 고무 측면 표본 32개 중 4개 이상과 상자 윗면 표본 12개 중 3개 이상)은 413/480이며, 가까운 측면(0.60 m) 35/96, 낮은 전방(높이 0.45 m·1.05 m) 18/24, 접촉 34/40, 초기 들어올림(0.02 m) 36/40이다. 남은 접촉·초기 들어올림 실패 10건(0.60 m 표본의 높이 0.75·0.90 m 각 4건, 1.05 m 표본의 높이 0.45 m 2건)은 공구가 시야 밖으로 나가는 경우이며, 접촉 단계 자체의 실패 수는 이전 방향과 같다(34/40). 이 10건은 작업대·정차 위치·작업 자세를 정할 때 확인할 제한으로 남긴다. 동시 가시성은 접촉·흡착 성공 판정이나 영상 품질 보장이 아니다. 같은 계산에서 센서·로봇과 작업대·상자의 겹침은 없었다. 관절 격자 검사에서 `suction` 손목 RGB-D는 어깨 ±1.57 rad 끝의 비작업 자세 7개에서 공구와 무관하게 차체 충돌 상자와 겹친다. 관절을 고정한 사본의 Gazebo 영상은 정적 시야 검증이다. 검사한 표본 사이의 연속 범위는 보장하지 않으며 확정 작업대·인계 위치, 동적 경로, 내참수 보정·실측 주기는 별도 검증 대상이다. 격리 Gazebo에서 검사한 robot_1 전·후방과 robot_2 전방 LiDAR의 각 20프레임에서는 +45° 한 샘플이 프레임과 무관하게 고정값(0.185 m, `-inf`, 0.165 m)으로 관찰됐다. 같은 설정의 단독 LiDAR(외형 상자 유무 포함)에서는 재현되지 않았다. 원인은 확인하지 않았으므로 특정 부품이나 렌더링 방식으로 단정하지 않는다. AX_2의 `ax_top_lidar.obj`는 보존한 외형이며 세 번째 측정 센서는 아니다.

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
