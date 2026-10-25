# 팀 연결 인터페이스

[링크·관절](#링크와-관절) · [명령 계약](#ros-명령--ros2_control) · [관절 보호](#팔-명령-보호arm_command_guard-개발용) · [미해결 항목](#알려진-제한과-미해결-항목) · [센서·TF](#센서tf주행)

기준 소스: `ax.urdf.xacro`, `ax_2.urdf.xacro`, `ros2_control.xacro`, `simulation.launch.py`, `ax_control/config/*_controllers.yaml`, `gripper_controller.py`, `cmd_vel_relay.py`, `arm_command_guard.py`, `arm_guard_logic.py`, `arm_command_guard.yaml`.

이 문서는 **현재 AX_1·AX_2 개발 모델과 센서·ros2_control 인터페이스**를 다룬다. `simulation.launch.py robot_model:=ax1`은 기존 모델 `ax`와 루트 명령 토픽을 유지하고, `ax2`는 `robot_2`, `both`는 `robot_1`·`robot_2`를 생성한다.

팔 로봇 `robot_1`·적재함 로봇 `robot_2`의 협업 그룹, 그룹·하위 Action과 보완된 `SafetyState`는 [통합 상세설계](../design/DETAILED_DESIGN.md)의 목표 계약이며 아직 구현되지 않았다. 길이 m, 각도 rad. 축은 joint 좌표계, xyz/rpy는 부모 링크 기준이다. 아래는 개발용 설정이며 실측 사양이 아니다. 상세한 구현·설계·검증 상태를 구분해 읽는다.

[AX 5축 사양안](../design/AX_5DOF_ARM_SPEC.md)의 v0.1 질량·URDF 속도 설정을 반영했다. 팔 조립체 질량은 센서 제외, 받침·기둥·공통 어댑터 포함 16.2 kg이며 팔 카메라 2개(손목 RGB-D·상완 RGB)의 개발용 질량 0.3 kg과 `suction` 공구 ToF 0.01 kg은 별도다.

충돌 상자와 가상 모터·하우징 질량 분포를 보완했고, [정적 지지 여유 계산](../validation/PHYSICAL_PROPERTIES_REVIEW.md)에서 일부 미필터링 관절 샘플의 지지 영역 이탈을 확인했다. 전 가동 범위의 하중 운반은 검증되지 않았다. 가속도 계획값은 `arm_command_guard`가 명령 보간 경로 검사와 정지 계획에 사용한다.

실제 관절의 순간 가속도 제한 준수는 확인되지 않았고 초과가 관찰됐다.

## 링크와 관절

루트는 `base_link`. 현재 원통 회전·카메라·광학 프레임을 포함하여 `primitive` 집게 선택(Gazebo·공통 Xacro 기본값)은 관절 24개·링크 25개, `robotiq` 선택은 관절 31개·링크 32개다. `parallel_wide`는 관절 26개·링크 27개, RViz 기본값인 비교용 공구 `suction`은 ToF 포함 관절 29개·링크 30개다.

신규 집게의 관절·선택 방법은 [집게 비교 안내](GRIPPER_COMPARISON.md)를 참고한다. 몸통·바퀴·팔은 분리 OBJ를 연결했으며, 팔 관절 위치·축은 형상에서 추정했다. 관절각 0은 팔 내부의 원본 OBJ 조립 자세를 유지하며, 팔 조립체 전체는 회전 기둥 축이 차체 X/Y 중심에 오도록 원본 대비 X=-177 mm, Y=-185 mm 이동했다. 아래 표의 임시 손가락은 `primitive`에만 해당한다.

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

팔 메시 연결 후 어깨·팔꿈치·손목 축은 기존 임시 Y축에서 형상 기준 X축으로 변경했다. 상위 IK·목표 pose는 새 URDF 기준으로 계산해야 하며 과거 관절각·pose·검증 기록을 그대로 적용하지 않는다. 집게 본체·손가락은 기존 임시 형상을 손목 하단으로 옮겨 유지했고 실제 집게 연결기구는 아직 연결하지 않았다. 팔 collision은 연결 부품별 방향 상자로, 관성은 가상 질량 분포로 보완했으며 공구별 실제 파지·하중·동적 안정성은 미검증이다. 무부하 대표 자세 제어 결과는 아래 절에서 별도로 다룬다.

`arm_mount_link`는 하단 받침·외측 지지 구조이며 고정된다. `column_link`는 중앙 원통과 3개 링으로, `column_yaw_joint` 회전 시 위쪽 팔 전체를 함께 회전시킨다. 중복된 상단 `arm_yaw_joint`는 제거하고 `arm_upper_mount_joint`로 고정했다. 팔의 수평 회전은 `column_yaw_joint` 하나로만 명령한다.

`{ns}/arm_controller/joint_trajectory`(trajectory_msgs/JointTrajectory) 토픽이나 `{ns}/arm_controller/follow_joint_trajectory`(control_msgs/FollowJointTrajectory) action의 `joint_names`에 이 관절을 넣는다. 링의 가동부 귀속과 실제 기구 사양은 모델링 담당자 확인이 필요하다.

## ROS 명령 → ros2_control

이 절의 집게 명령은 기존 `primitive` Gazebo 구성용이다. RViz 비교용 Robotiq은 `robotiq_85_left_knuckle_joint`의 0~0.8 rad 입력과 mimic 5개로 개폐하며, `/gripper/command`의 손가락 변위(m)를 그대로 적용하지 않는다. [집게 비교 안내](GRIPPER_COMPARISON.md)를 참고한다.

`simulation.launch.py`는 로봇마다 gz_ros2_control이 호스팅하는 `controller_manager`를 하나 만들고 spawner로 아래 제어기를 활성화한다. 바퀴·팔·손가락의 Gazebo 구동 주체는 이 플러그인 하나이며, 이전 Gazebo DiffDrive·JointPositionController·JointStatePublisher와 Float64 명령·상태 브리지는 제거했다.

각 관절의 명령 인터페이스는 한 제어기만 점유한다. 아래 `{ns}`는 단일 AX_1 실행에서 빈 문자열(루트), AX_2 단독·두 로봇 실행에서 `/ax/robot_1` 또는 `/ax/robot_2`다.

| 제어기 (형식) | 관절·명령 인터페이스 | ROS 입력 | 단위 |
|---|---|---|---|
| `arm_controller` (JointTrajectoryController) | `column_yaw`·`shoulder`·`elbow`·`wrist`·`gripper_rotation`의 position | 내부 action `{ns}/arm_controller/internal/follow_joint_trajectory`, `arm_command_guard`만 사용. 공개 입력은 아래 표 | rad |
| `finger_controller` (position_controllers/JointGroupPositionController, `primitive`만) | `left_finger`·`right_finger`의 position | `{ns}/finger_controller/commands` (std_msgs/Float64MultiArray `[left, right]`), `gripper_controller`만 발행 | m |
| `diff_drive_controller` | 바퀴 4개의 velocity | `{ns}/diff_drive_controller/cmd_vel` (geometry_msgs/TwistStamped), `cmd_vel_relay`만 발행 | m/s, rad/s |
| `joint_state_broadcaster` | 위 관절의 position·velocity·effort 상태 | 없음(상태 발행) | |

| 공개 입력 | 타입 | 처리 |
|---|---|---|
| `{ns}/cmd_vel` | geometry_msgs/Twist | `cmd_vel_relay`가 수신 시각과 `base_link`(다중 실행은 `robot_i/base_link`)를 붙여 TwistStamped로 전달한다. Jazzy `diff_drive_controller`가 TwistStamped만 받기 때문이며 속도 제한·정지 판단은 하지 않는다. |
| `{ns}/gripper/command` | std_msgs/Float64 | `gripper_controller`가 같은 값을 `[left, right]`로 `finger_controller`에 보낸다. 손가락 각각의 목표 변위이며 범위는 −0.01~0.02 m다. |
| `{ns}/arm_controller/joint_trajectory`, action `{ns}/arm_controller/follow_joint_trajectory` | trajectory_msgs/JointTrajectory, control_msgs/FollowJointTrajectory | `arm_command_guard`가 검사해 통과한 명령만 내부 action으로 전달한다(아래 "팔 명령 보호"). 물리 위치 한계: `column_yaw` ±3.14159(임시, URDF 최대 속도 0.35 rad/s), `shoulder` ±1.5708, `elbow` ±2.0, `wrist` ±1.57, `gripper_rotation` ±3.14159 rad. 명령 가능 범위는 양쪽 각각 0.02 rad 안쪽이다. |

이전 `/{column_yaw,shoulder,elbow,wrist,gripper_rotation,left_finger,right_finger}/command` Float64 토픽과 단일 `ax`용 `arm_control.launch.py`는 제거했다. 팔 목표는 일부 관절만 담을 수 있고(`allow_partial_joints_goal`), 빠진 관절은 보호 노드가 검사 시점의 측정 위치로 채운다.

움직이는 중 부분 목표를 보낼 때 기존 목표 유지와 같다고 가정하지 않는다. 궤적 토픽과 action을 함께 쓰면 나중 목표가 앞 목표를 대체하므로 발행 주체를 하나로 정한다. 궤적 시간은 호출자가 정하며, `arm_command_guard`가 명령 가능 범위와 관절별 속도·가속도를 넘는 명령을 거부한다(시간을 대신 정하지 않음). TCP 속도는 강제하지 않는다. 설정된 제한과 실제 강제를 구분한다.

`controller_manager`의 `enforce_command_limits`는 설정하지 않아 Jazzy 기본값(false)이므로 ros2_control 자체는 명령을 제한하지 않는다. 물리 시뮬레이션에서 걸리는 것은 Gazebo 관절 위치 한계, 아래 effort 상한, gz-physics가 DART 속도 한계로 넘기는 URDF velocity다(위치 한계에 닿지 않았을 때).

action 목표 허용값(관절별 0.02 rad, 정지 속도 0.01 rad/s, `goal_time` 2 s)은 개발용 임시값이다. 집기 완료 응답은 없다.

### 팔 명령 보호(`arm_command_guard`, 개발용)

공개 이름은 그대로이고, 실제 `arm_controller`의 입력은 launch가 내부 이름으로 옮겼다. 정상 팔 명령 경로는 공개 토픽·action → `arm_command_guard` → `{ns}/arm_controller/internal/follow_joint_trajectory`(action) 하나다.

내부 토픽 `{ns}/arm_controller/internal/joint_trajectory`도 남아 있으며, 이 경로 정리는 접근 차단(보안)을 보장하지 않는다.

#### 입력과 보간 경로

**명령 가능 범위**

URDF 물리 위치 한계는 그대로 두고 양쪽 각각 0.02 rad 안쪽(`ax_control/config/arm_command_guard.yaml`, 개발용 잠정값). `column_yaw`·`gripper_rotation` ±3.12159, `shoulder` ±1.5508, `elbow` ±1.98, `wrist` ±1.55 rad

**수락 전 검사**

토픽·action 모두 적용. 알 수 없는·중복 관절명, 배열 길이, 비정상 수치, `time_from_start`가 0 이하이거나 증가하지 않음, effort 필드, 속도 없는 가속도, 점마다 다른 필드, 0이 아닌 `header.stamp`(지연 시작), multi-DOF를 거부한다. 명령한 관절의 끝점이 범위 밖이면 거부하며 잘라서 실행하지 않는다

**보간 경로 검사**

현재 상태에서 시작해 JTC와 같은 3차(속도 포함)·5차(가속도 포함) 보간의 구간 다항식마다 양 끝과 내부 극값(도함수의 근)을 구해, 경로의 위치가 범위를, 속도가 `min(URDF velocity, max_velocity)`를, 가속도가 `max_acceleration`(`arm_joint_limits.yaml`)을 넘으면 거부한다.

표본 간격보다 짧은 구간 안의 초과도 거부한다(예: 0.005 s 구간 내부 가속도 약 394 rad/s²). 검사는 관절 상태를 복사해 잠금 밖에서 하고, 결과를 쓰기 전에 상태 변경 여부와 상태 최신성을 다시 확인한다. 위치만 있는 궤적은 각 점에서 속도 0인 3차식으로 바꿔 실행한다(JTC 선형 보간의 무한 가속도를 피하기 위한 계약 변경, 이벤트에 기록). 시간 재설정·자동 감속은 하지 않는다

**범위 밖에서 시작**

보호 정지 등으로 여유 구역에 있는 관절은 시작 위치보다 바깥으로 가지 않는 경로만 허용한다(안쪽 복귀)

#### 보호 정지와 결과 확인

**빈 궤적(토픽)**

기존 JTC 계약대로 현재 위치 유지(진행 중 목표 취소). 감속 정지가 진행 중이면(정지 판정 전, 감속 중 상태 이상으로 모드가 `state_invalid`인 경우 포함) 무시하고 거부 이벤트를 남긴다. 빈 궤적·공개 취소·상태 이상 처리는 보호 정지 목표를 취소하지 않으며, 물리 한계 도달 고장 잠금만 취소한다

**감속 시작 판단**

`joint_states` 위치·속도로, 한계 쪽 속도 v가 0.01 rad/s를 넘고 남은 거리 − (v×0.05 s + v²/(2×max_acceleration) + 0.01 rad) < 0.005 rad이면 보호 정지한다. 반응 시간 0.05 s는 측정한 감속 시작 지연(최대 0.038 s)의 여유값이다

**보호 정지**

진행 중 목표를 중단(결과 `ABORTED`, `error_string`에 사유)하고, 관절마다 자기 max_acceleration으로 등감속해 멈추는 궤적을 내부 action으로 보낸다. 관절 i는 T_i = \|v_i\|/a_i에 v_i²/(2a_i)만큼 이동해 멈추고(감속 시작 판단의 등감속 항과 같은 식), 먼저 멈춘 관절은 그 위치를 유지한다.

관절이 멈추는 시각마다 궤적 점(위치·속도)을 두어 JTC 3차 보간이 이 프로파일을 그대로 재현한다. 다른 관절의 정지 시간을 늘리지 않으며, `stop_minimum_time_s`(0.01 s)보다 빨리 멈출 느린 관절만 그 시간에 멈춘다(감속도는 더 작아짐). 최대 감속으로도 물리 한계 안에서 멈출 수 없으면 목표를 자르지 않고 `stop_reaches_limit` 이벤트를 남긴다(실제 도달은 고장 잠금).

감속 중에는 새 목표를 거부한다. 명령 중단·제어기 비활성화는 쓰지 않는다

**정지 확인**

내부 정지 목표가 성공(`SUCCEEDED`)으로 끝나고, 그 뒤 받은 `joint_states`의 팔 5축 속도가 모두 `stop_confirm_velocity_rad_s`(0.01 rad/s) 이하일 때만 `protected_hold`로 바꾸고 안쪽 복귀 명령을 받는다.

내부 정지 목표의 수락 거부·중단·취소·요청/결과 통신 예외는 즉시, 계획 정지 시간 + `stop_confirm_timeout_s`(1.0 s) 안에 결과나 속도 확인이 없으면 그 시점에 정지 실패(`stop_failed` 이벤트, `fault` 잠금)로 처리한다. 정지 목표가 거부되면 대체하려던 이전 내부 목표는 취소해 JTC가 현재 위치를 유지하게 한다

#### 상태 이상·취소·고장

**상태 이상**

`joint_states`가 0.5 s 넘게 끊기거나(수신 시각, 단조 시계) 비정상 값(비유한 값, 관절 속도가 `velocity_plausibility_factor` 1.25 × max_velocity 초과)이면 새 목표를 거부하고, 진행 중 일반 목표는 취소해 JTC가 현재 위치를 유지하게 한다.

진행 중 보호 감속은 끝까지 수행하고, 상태가 돌아오면 정지 확인을 이어 간다(상태 이상 중에 정지 목표가 성공해도, 돌아온 상태의 속도를 확인한 뒤에만 `protected_hold`. 돌아오지 않으면 확인 제한시간에 `fault`). 속도 한계를 크게 넘는 표본은 타당성 검사에서 제외해 보호 판단·정지 궤적에 쓰지 않는다.

이는 개발용 이상 표본 처리이며 실제 과속이 불가능하다는 보장은 아니다(Gazebo 시험에서 제어기 활성 직후 손목 2.10 rad/s 한 표본을 관측했고, 이 값으로 만든 정지 궤적이 손목을 물리 상한까지 움직였다). 감시는 실행기 타이머가 아닌 전용 스레드가 0.05 s마다 한다.

`header.stamp`가 직전보다 0.5 s 이내로 과거인 메시지는 순서가 뒤바뀐 것으로 보고 버리고, 더 크게 과거면 발행 쪽 시계 재설정으로 보고 받아들인다(`stamp_jump_back` 이벤트)

**한계 도달**

실제 위치가 물리 한계에 닿거나 넘으면 `fault`로 잠그고 모든 새 목표를 거부한다. 정지 실패도 같은 `fault`로 잠근다.

`~/clear_fault`(std_srvs/Trigger)는 관절 상태가 있고, 팔 5축 위치·속도가 모두 있으며 유한하고(NaN·±inf·속도 누락이면 거부), 최신이며, 모든 관절이 물리 한계에서 0.005 rad 이상 떨어져 있고, 속도가 모두 0.01 rad/s 이하일 때만 `protected_hold`로 해제한다. 거부 사유는 응답과 `clear_fault_rejected` 이벤트에 남긴다.

한계에 닿은 관절은 DART 동작 때문에 일반 명령으로 복귀하지 못하므로(아래 한계 고착 설명 참고) 자동 복귀 성공으로 처리하지 않는다

**선점·동시 구동**

내부 목표는 항상 하나다. 새 공개 목표는 이전 공개 목표를 `ABORTED`(preempted)로 끝내고, 보호 동작이 일반 목표보다 우선한다. 취소는 내부 목표를 취소해 현재 위치를 유지한다. 모드가 바뀔 때마다 늘어나는 상태 변경 번호를 수락 검사 때 기록하고, 내부로 보내기 직전에 번호 비교와 현재 상태로의 재검사를 한다. 수락 대기 중 보호 정지·고장·상태 만료가 생긴 목표는 보내지 않고 `ABORTED`로 끝낸다.

토픽 궤적은 도착 순서대로 하나씩 처리한다

**기록**

`~/events`(std_msgs/String JSON: 수락·거부 사유·보호·정지 실패·고장·결과), `~/status`(현재 모드, transient local). 이벤트·상태 발행과 로그는 전용 송출 스레드가 순서대로 한다(잠금을 쥔 채 로그를 쓰지 않음). 공개 action 거부는 표준 REJECT라 사유는 이벤트·로그로만 확인한다

**한계**

개발용 관절 보호이며 안전 게이트·최종 구동 차단이 아니다. 토크가 부족해 처지는 관절은 명령 제한만으로 막을 수 없다. TCP 속도·자기 충돌은 다루지 않는다

#### 검증 범위

| 구분 | 확인 내용 | 해석 범위 |
|---|---|---|
| 초기 단축 시험 | 단일 AX_1·primitive·무부하에서 한계 접근과 복귀 확인 | 당시 구현의 기록이며 후속 다축·경합 결함이 없다는 근거가 아님 |
| 다축 수정 후 Gazebo | 3개 시나리오 각 3회, 물리 한계 접촉 0회, 정지 위치·복귀 확인 | 시험한 자세·속도·공구에 한정. 실제 순간 감속은 계획 가속도를 초과 |
| 후속 모의 노드 시험 | 상태 만료 중 보호 유지, 비정상 상태의 고장 해제 거부를 회귀 시험으로 확인 | 이 변경 뒤 같은 장애를 Gazebo에서 다시 주입하지는 않음 |

실행 원본은 실행별 임시 경로에 있으며 Git에 선별 보관하지 않았다. 과거 시험과 후속 코드의 검증 범위를 합쳐 전 조건 통과로 표현하지 않는다.

gz_ros2_control 1.2.20은 위치 명령을 (목표−현재)×10 s⁻¹의 Gazebo 관절 속도 명령으로 적용하고(`position_proportional_gain` 0.1 × 관리자 주기 100 Hz), URDF limit의 effort가 그 출력 상한이다. 실제 모터·감속기 모델이 아니다.

`joint_states`의 effort는 Gazebo 관절 전달 렌치의 축 성분이며, 관절 한계에 닿은 상태에서는 한계 반력이 포함되므로 모터 토크로 읽지 않는다.

정지와 자세 유지(외부 하중 없이 자중에서 자세 유지, 단일 시험. 개발용 판정: 속도 절댓값 0.01 미만 1 s 유지, 시뮬레이션 시간, 단일 AX_1 격리 실행):

- 주행: `{ns}/cmd_vel`이 0.5 s(`cmd_vel_timeout`) 동안 끊기면 `diff_drive_controller`가 바퀴 목표 속도를 0으로 만든다. 0.3 m/s 주행 후 명령을 끊었을 때 마지막 명령부터 바퀴 정지 0.42 s, odom 정지 0.52 s, 추가 이동 0.12 m였다. 속도 0 명령을 한 번 보내면 바퀴 정지 0.02 s, 추가 이동 0.003 m였고, 정지 후 다시 명령하면 같은 방식으로 움직이고 멈췄다. 이 시간 초과는 안전 기능이 아니다.
- 팔: 새 명령이 없어도 `arm_controller`는 마지막 목표를 계속 유지한다(`cmd_timeout` 0, watchdog 아님). 이전 effort(어깨 20·팔꿈치 15 N·m) 기준 측정이다. 정적 중력이 그 effort 상한보다 작은 자세(어깨 11.0~14.2, 팔꿈치 14.0 N·m)에서 명령 없이 10 s 동안 관절 위치 변화·속도는 0으로 기록됐고, 새 목표 재입력 후 오차 0으로 다시 유지됐다. action 목표를 진행 중 취소하면 관절이 0.1 s 안에 멈추고 이후 5 s 동안 위치 변화가 없었다. 같은 자세를 유지한 채 0.3 m/s·0.3 rad/s로 주행하면 팔꿈치가 최대 0.038 rad 벗어났고 주행 정지 3 s 뒤에도 0.020 rad 낮았다(팔꿈치 정적 중력 14.0 N·m, 당시 상한 15 N·m). 현재 effort(어깨 100·팔꿈치 65·손목 25 N·m)에서는 같은 주행 조건의 팔꿈치 이탈이 0.0006 rad 이하였다([팔 사양](../design/AX_5DOF_ARM_SPEC.md) 6.2절).
- 정지 요청이나 목표 전송만으로 정지 완료로 보지 않는다. 실제 정지는 `joint_states`·`odom`의 속도와 위치로 확인한다. 팔 제어기 비활성화는 정지 방법으로 쓰지 않는다. 비활성 관절에는 gz_ros2_control `hold_joints`가 effort 상한 안에서 속도 0 명령을 주지만 하중 유지는 검증하지 않았다. 관절 단위 명령 검사와 한계 접근 보호 정지는 `arm_command_guard`로 개발용 구현했지만, 시스템 전체 안전 게이트·watchdog·최종 구동 차단은 미구현이다. 관절 보호의 개발용 정지 확인 기준과 시스템 안전 정지 기준을 구분한다.

### 이전 어깨 처짐과 현재 한계 고착

**이전 제어·effort 조건의 진단 기록.** 이전 Gazebo JointPositionController 구성(`p_gain` 10, I·D항 없음)에서는 중력을 0으로 한 진단 사본에서만 모든 관절이 목표를 따랐다. effort만 200 N·m로 올려도 처짐이 같았고, 이득만 올리면 영 자세만 유지됐다. 명령 수신과 충돌(`self_collide` 미설정)은 원인이 아니었다.

원인은 낮은 비례 이득과, 작업 자세의 정적 중력 토크보다 작은 미검증 effort 제한(어깨 20·팔꿈치 15 N·m)이 겹친 것이다. URDF 질량으로 계산한 무부하 어깨 정적 중력 토크는 영 자세에서 11.6 N·m(`primitive`)·13.6 N·m(`suction`)이다.

대표 자세 P1(−1.0·1.0·0)·P2(−0.6·0.8·−0.2)·P3(−0.3·0.6·−0.3 rad, 어깨·팔꿈치·손목)에서는 38.7·31.7·23.8 N·m(`primitive`)이다. 전환 직후의 비교 시험은 기존 effort를 유지했으므로 P1~P3에서 어깨가 하한까지 떨어졌다. 현재는 잠정 effort(100·65·25 N·m)를 적용해 무부하 대표 자세가 정착·유지된다.

다만 이 복귀 실패는 effort 부족만으로 설명되지 않는다. 검증용 사본(어깨 100·팔꿈치 65·손목 25 N·m, 2026-10-25)에서도 어깨가 하한 −1.5708 rad에 정확히 닿으면 제어기가 활성인데도 이후 상승 목표(영 자세, −1.5 rad)에 전혀 움직이지 않았다(속도 0, action `goal_time_tolerance` 초과). 같은 사본에서 −1.55 rad까지 내렸을 때는 정상 복귀했다.

하한 자세의 정적 중력은 19.3 N·m라 100 N·m로 충분하므로 effort 부족과 별개인 한계 고착 현상으로 구분한다.

**한계 고착의 유력 원인(소스 근거와 재현 관측의 일치).** gz_ros2_control 1.2.20과 gz-sim 8.15.0은 위치 한계를 처리하지 않고 속도 명령을 그대로 넘기며, gz-physics 7.8.0 dartsim은 이를 DART 6.13.2 `SERVO` 명령으로 설정한다.

DART `JointConstraint::update()`는 위치 한계 → 속도 한계 → 서보 순으로 하나만 적용하므로, 관절 위치가 하한을 아주 조금이라도 넘으면 그 시간 단계에서 서보(속도 명령)를 적용하지 않는다. 기본 오차 허용값이 0이라 한계 복원 속도도 초과량/Δt(관측 초과량 2×10⁻¹⁴ rad에서 약 10⁻¹¹ rad/s)뿐이다.

중력이나 하향 명령으로 하한에 눌린 관절은 이 상태에 머물며, 이후 상향 명령이 action·제어기 목표·명령 인터페이스까지 정상 전달되어도 움직이지 않는다.

재현: 후보 effort에서 어깨를 −1.5708 rad(정확한 하한)나 −1.5709 rad로 내리면 영 자세·−1.5 rad 복귀가 실패하고, −1.55·−1.5707 rad로 내리거나 진단용 사본에서 하한을 −1.60 rad로 넓히면 같은 −1.5708 명령 뒤에도 복귀한다. DART 내부 값(구속 활성 여부)은 직접 관측하지 않았고, 소스와 6개 경우의 관측이 일치한다는 근거다.

임시 운용 제한(해결 아님): 팔 목표 위치는 각 관절 위치 한계에서 0.02 rad 이상 안쪽으로 둔다. 2026-10-25부터 `arm_command_guard`가 명령 가능 범위와 한계 접근 보호 정지로 이 제한을 적용한다. 한계에 이미 닿은 관절은 현재 일반 명령으로 복귀하지 않으며, 상태 강제 재설정·한계 확대·제어기 비활성화는 복귀 해법으로 쓰지 않는다.

**기존 effort에서의 전환 직후 측정.** P1~P3 측정(자세마다 새로 실행, 궤적 이동 4 s 포함, 개발용 기준 0.02 rad·0.01 rad/s 1 s 유지)의 정착 시간 6.3·6.7·8.3 s는 팔꿈치 결과다. 같은 시험에서 어깨는 하한에 머물러 목표에 도달하지 못했으므로 팔 전체 정착은 실패다. 영 자세와 effort 안 자세(L1·L2, 정착 5.7 s)는 0.02 rad 안에서 유지된다.

이 결과는 외부 하중 없는 자중 조건의 단일 시험이다. 토크 정격 확정은 [팔 사양](../design/AX_5DOF_ARM_SPEC.md) 6절의 미해결 항목이다. 자중·2 kg 가정 정적 토크와 현재 잠정 적용값의 산정 근거는 같은 문서 6.1절에 있다.

### 알려진 제한과 미해결 항목

| 항목 | 현재 상태 | 다음 확인 |
|---|---|---|
| 실제 순간 감속 | 정지 계획은 설정 가속도 이내지만 실제 속도 차분에서 약 2.3~9.4 rad/s² 감속을 관찰 | JTC 목표 교체·구동 변환 경로를 조사. `open_loop_control`은 검토 후보이며 채택·해결되지 않음 |
| 보호 중 한계 도달 후 고장 해제 | 보호 목표 취소 후 `closed`가 남아, 정상 상태에서 해제 성공을 응답해도 새 목표가 `protective stop in progress`로 거부되는 경로를 모의 재현 | 내부 목표 종료 확인·보호 기록 종료·해제 후 명령 재개까지 회귀 시험 필요 |
| 물리 한계의 DART 고착 | 이미 한계에 닿은 관절은 일반 상승 명령으로 복귀하지 않는 현상이 남음 | 한계 접근 보호는 예방 기능이며 엔진 고착 해결이 아님 |
| 안전·하중 | 안전 게이트·최종 구동 차단, 2 kg 실제 하중 유지·흡착 부착/해제는 미완료 | G1과 집기·운반 완료를 별도로 판정 |
| 적용 범위 | TCP 속도·자기 충돌·토크 부족 처짐·두 로봇 동시 보호 검증 미완료 | 시험한 무부하 단일 AX_1 범위를 넘어 보장하지 않음 |

고장 해제 응답 성공만으로 재개 가능하다고 판단하지 않는다. 위치·속도뿐 아니라 내부 목표 종료와 후속 명령 수락까지 확인해야 한다. 이 문서 갱신으로 운영 코드를 수정하거나 해당 결함을 해결한 것은 아니다.

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

### 카메라·전방 센서 장착

카메라 설정은 RGB 640×480·15 Hz·수평 FOV 60°이며 RGB-D 깊이 clip은 0.05~5 m다. 카메라 `{name}_link`는 +X 전방이고 `{name}_optical_frame`은 +Z 전방·+X 오른쪽·+Y 아래다.

몸체 전방은 두 로봇 모두 `front_rgb`(Y=+0.10)–`front_lidar`(Y=0)–`front_rgbd`(Y=−0.10)를 같은 높이(AX_1 Z=0.20, AX_2 Z=0.14, `base_link` 기준)에 두고 모두 +X를 향한다. LiDAR는 ±90° 스캔면이 카메라 몸체 앞을 지나도록 카메라(AX_1 X=0.73, AX_2 X=0.57)보다 0.04 m 앞에 있다.

이 높이에서는 그보다 낮은 장애물이 전방 LiDAR 스캔면에 잡히지 않는다. 팔 센서 역할은 `arm_rgbd`가 접근 전 물체 표면·높이·주변 형상 관찰, `suction_tof`가 최종 접근 간격 측정, `arm_rgb`가 컵 말단과 물체 접촉 장면 관찰이다. `arm_rgbd`는 모든 공구에서 `wrist_link`에 있다.

`gripper_model:=suction`에서는 xyz (0, 0.18, −0.20) m, rpy (π, 1.2606, −π/2)로 광축이 손목 기준 (0, 0.13, −0.356) m, 즉 컵 바깥의 상자 윗면을 향하며 말단 회전을 따라 돌지 않는다. 손목 하우징까지의 브래킷은 모델링하지 않았다.

흡착판이 없는 `primitive`·`robotiq`·`parallel_wide`(Gazebo 기본 `primitive` 포함)는 이전 손목 좌표(0, 0.13, −0.18 m, rpy 0, 1.1570, −π/2)를 유지한다.

### 중앙 ToF와 거리 구독 계약

`suction_tof`는 `suction` 공구에서만 생성되며 `suction_base_link`(흡착판 윗면 원점, +Z가 물체 방향) 기준 xyz (0, 0, 0.031) m, rpy (0, −π/2, 0)으로 측정 방향(링크 +X)이 컵 방향이고 `gripper_rotation_joint`와 함께 돈다. 0.012×0.02×0.02 m 몸체는 흡착판 아래 z 0.025~0.037 m에 있어 흡착판과 겹치지 않고 컵 끝면(z 0.062 m)보다 돌출되지 않는다.

Gazebo `gpu_lidar` 1샘플로 근사한 개발용 센서이며 특정 실물 ToF의 정확도·재질 응답을 재현하지 않는다. 측정값은 센서 원점 기준 거리이고 컵 끝면까지의 접근 간격은 `ranges[0] − 0.031` m다(접촉 시 원시값은 약 0.031 m이며 0이 아니다). 측정 범위 0.02~0.50 m는 접근 전(간격 0.15 m)부터 접촉까지 포함하도록 정한 개발 가정이다.

설치된 `ros_gz_bridge`의 `gz.msgs.LaserScan`→`sensor_msgs/msg/Range` 변환은 원시 `+inf`를 `max_range + 1`(현재 1.5 m)이라는 범위 밖 유한값으로 바꾸므로(`max_range` 2.0에서 3.0으로 확인), 원시 무효값 `inf`를 보존하는 현재 1빔 `sensor_msgs/msg/LaserScan`을 유지한다.

**구독 계약:** 구독자는 `ranges` 길이가 1인지, `ranges[0]`이 유한값이고 `range_min` 이상 `range_max` 이하인지, 메시지 시각이 허용 지연 안인지 확인한 뒤에만 `ranges[0] − 0.031` m로 환산한다.

여기서 `lip_offset` 0.031 m는 현재 형상의 컵 끝면 z 0.062 m와 센서 원점 z 0.031 m의 차이이며, 환산값은 변형되지 않은 컵 끝면까지의 중앙 광선 기준 접근 간격일 뿐 네 컵 모두의 접촉이나 흡착 성공을 뜻하지 않는다. `inf`·`-inf`·NaN·범위 밖·오래된 값과 음수 간격은 무효로 다루며 0 또는 접촉 성공으로 바꾸지 않는다. 허용 지연과 이 값을 쓰는 구독 노드는 아직 정하지 않았다.

브리지는 `model_contracts.sensor_bridges(robot_id, tool="suction")`에서만 추가된다. 기본 `simulation.launch.py`는 `primitive` 구성이라 ToF 토픽이 없고, 흡착 공구·ToF는 별도로 생성한 흡착 공구 사본과 전용 브리지로만 Gazebo 검증했다.

### 상완 RGB와 시야 검사 조건

`arm_rgb`는 `upper_arm_link` 기준 xyz (0.05, 0.142, 0.15) m에서 같은 위치의 방향 3안 중 rpy (2.1677, −0.2604, 1.2929)를 택했다. 관찰용 영상이며 실제 접촉·흡착 성공 판정이 아니다.

정적 기하 계산(개발용 가정: 작업대 높이 0.45·0.60·0.75·0.90 m, 0.30×0.34×0.30 m 상자를 작업대에 고정, 흡착 공구·공구축 수직, 컵 끝면 위 0.15·0.10·0.05·0.03·0.02·0.015·0.01·0.005 m에서 접촉까지 하강, 상자를 붙여 0.02·0.05·0.10 m 들어올림, 말단 회전 0·π/2, 가림 형상에 차체·바퀴·팔·공구·다른 팔 센서·작업대·상자 포함, 관측 센서 자신만 제외)에서 480자세를 비교했다.

가림 판정은 목표 자신의 표면 끝점 교차만 제외하고 다른 형상은 목표 1 mm 앞이라도 가림으로 센다. 거리 표본은 기둥 좌표계에서 흡착 기준점의 Y 성분 0.60·0.75·0.90(측면)·0.95·1.05 m(전방)이며 FK로 계산한 기둥축까지 실제 수평거리는 각각 0.688·0.822·0.961·1.008·1.103 m다.

**검사 결과:** `arm_rgbd` 광축은 480자세 모두 상자에 닿고 깊이가 유효하며, 흡착판 바깥 상자 윗면 표본(16개 중 6개 이상)은 접근 전·하강 0.10 m·접근 단계 120/120에서 보이고 간격 0.03 m 이하에서는 평균 3개로 줄어든다(이전 4컵 중앙 RGB-D는 0/480, 접촉·들어올림 깊이 무효).

`suction_tof` 광선은 모든 단계에서 상자에 닿고(컵·흡착판 미측정) 접촉·들어올림까지 범위 안이다. `arm_rgb`의 컵·상자 윗면 동시 가시성(컵 고무 측면 표본 32개 중 4개 이상과 상자 윗면 표본 12개 중 3개 이상)은 413/480이며, 가까운 측면(0.60 m) 35/96, 낮은 전방(높이 0.45 m·1.05 m) 18/24, 접촉 34/40, 초기 들어올림(0.02 m) 36/40이다.

남은 접촉·초기 들어올림 실패 10건(0.60 m 표본의 높이 0.75·0.90 m 각 4건, 1.05 m 표본의 높이 0.45 m 2건)은 공구가 시야 밖으로 나가는 경우이며, 접촉 단계 자체의 실패 수는 이전 방향과 같다(34/40). 이 10건은 작업대·정차 위치·작업 자세를 정할 때 확인할 제한으로 남긴다. 동시 가시성은 접촉·흡착 성공 판정이나 영상 품질 보장이 아니다.

같은 계산에서 센서·로봇과 작업대·상자의 겹침은 없었다. 관절 격자 검사에서 `suction` 손목 RGB-D는 어깨 ±1.57 rad 끝의 비작업 자세 7개에서 공구와 무관하게 차체 충돌 상자와 겹친다. 관절을 고정한 사본의 Gazebo 영상은 정적 시야 검증이다. 검사한 표본 사이의 연속 범위는 보장하지 않으며 확정 작업대·인계 위치, 동적 경로, 내참수 보정·실측 주기는 별도 검증 대상이다.

**LiDAR 잔여 현상:** 격리 Gazebo에서 검사한 robot_1 전·후방과 robot_2 전방 LiDAR의 각 20프레임에서는 +45° 한 샘플이 프레임과 무관하게 고정값(0.185 m, `-inf`, 0.165 m)으로 관찰됐다. 같은 설정의 단독 LiDAR(외형 상자 유무 포함)에서는 재현되지 않았다. 원인은 확인하지 않았으므로 특정 부품이나 렌더링 방식으로 단정하지 않는다.

AX_2의 `ax_top_lidar.obj`는 보존한 외형이며 세 번째 측정 센서는 아니다.

### namespace·상태·TF

기본 단일 AX_1 실행의 `/cmd_vel`, `/gripper/command`, `/arm_controller/...`, `/joint_states`, `/odom`, `/controller_manager`는 루트 이름을 사용한다.

AX_2 단독 실행과 두 로봇 실행에서는 `/ax/robot_i/{cmd_vel,joint_states,odom,controller_manager}`와 AX_1의 `/ax/robot_1/{gripper/command,arm_controller/...}`를 사용한다. 두 로봇의 제어기 설정은 launch가 로봇 namespace 전체 이름을 키로 쓴 사본으로 만들어 서로 섞이지 않는다. AX_2에는 팔·손가락 제어기가 없다.

| 상태·시간 | 제공 구성 |
|---|---|
| `joint_states` | `joint_state_broadcaster` → sensor_msgs/JointState (position·velocity, 팔·손가락은 effort 포함) |
| `odom` | `diff_drive_controller`가 바퀴 위치 피드백으로 계산한 nav_msgs/Odometry (`~/odom`을 `{ns}/odom`으로 이름 변경) |
| `/tf` | `diff_drive_controller`의 odom→base_link 및 robot_state_publisher의 이동 관절 TF |
| `/tf_static` | robot_state_publisher의 고정 관절 TF |
| `/clock` | 월드 clock → rosgraph_msgs/Clock, 시뮬레이션 노드 use_sim_time=true (`controller_manager`는 gz_ros2_control이 강제 설정) |

단일 AX_1의 프레임 이름은 `base_link`, `front_lidar_link` 등이다. AX_2 단독·두 로봇 실행에서는 `robot_i/base_link`, `robot_i/odom`, `robot_i/front_rgbd_optical_frame`처럼 접두사를 붙인다. 같은 `/tf`, `/tf_static`에서 로봇별 트리가 구분되지만 공통 `map` 연결·위치 추정·Nav2는 미구현이다.

RViz display는 단일 모델용 GUI JointState를 사용하며 물리 피드백이 아니다.

### 차체·바퀴·물리 특성

AX_1 바퀴 반경·좌우 간격은 0.19631 m·0.622 m다. AX_2는 현재 OBJ에서 생성한 약 0.149901 m·0.836695 m이며 차체 bbox는 약 1.043×0.996×0.406 m다. AX_2는 원본 조립 좌표에서 차체 bbox 중심을 공통으로 빼서 8개 메시의 상대 위치를 유지한다. 선반 외형은 열린 부품을 포함해 Y 방향 약 1.993 m다.

충돌은 생성기의 보수적 부품/면 상자 근사이며 최종 적재 공간이나 허용 하중을 보증하지 않는다.

AX_2의 개발용 질량 예산은 몸통 12 kg·선반 5 kg·바퀴 각 0.4 kg·전방 장식 0.05 kg·상단 하우징 0.4 kg이다. 센서와 접촉 패드 질량은 별도다. `ax_2_physical_properties.json`과 `ax_2_geometry.xacro`는 `generate_physical_properties.py`가 생성한다. AX_1과 질량·COM·관성을 공유하지 않는다.

월드는 Sensors·Imu·Contact 시스템을 한 번만 로드한다. URDF는 실행마다 `/tmp/ax_sim_*/robot_i.urdf`로 따로 생성한다. 독립 시뮬레이션을 동시에 실행하려면 ROS domain과 Gazebo partition을 모두 격리해야 한다. 두 모델 생성과 센서·상태 브리지 구성은 협업 Task Executor·흡착·접촉+영역 기반 적재 판정·안전 차단의 구현 완료를 뜻하지 않는다.
