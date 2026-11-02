# ax_control

A2 기본 제어 패키지다. 제어기 설정과 개발용 시뮬레이션 launch를 둔다.
명령 경로 계약은 [공통 인터페이스 계약](../../docs/design/INTERFACE_CONTRACT.md) 4.3절이 기준이다.

## 실행

```bash
ros2 launch ax_control simulation.launch.py robot_id:=robot_1
```

`robot_id`는 namespace `/ax/<robot_id>`와 TF 접두사 `<robot_id>/`를 정한다.
launch 하나가 Gazebo와 로봇 한 대를 함께 띄우므로 여러 대 동시 실행은 지원하지 않는다.
모델은 정식 A1 모델 전까지 로컬 임시 모델(`ax_description/temp_robot`)을 쓴다.

## A4 연결 지점

| 구분 | 경로 | 호출 주체 |
|---|---|---|
| 바퀴 입력 | `/ax/<robot_id>/diff_drive_controller/cmd_vel` (`TwistStamped`) | A4 바퀴 게이트 |
| 팔 입력 | `/ax/<robot_id>/arm_controller/follow_joint_trajectory` (`FollowJointTrajectory`) | A4 팔 보호 서버 |
| 상태 출력 | `/ax/<robot_id>/joint_states`, `diff_drive_controller/odom`, TF `odom→base_link` | 소비자 전체 |

A4 게이트는 아직 없다. 지금은 제어기를 누구나 직접 호출할 수 있으므로 이 패키지만으로는 구동 차단이 되지 않는다.
팔 제어기의 `joint_trajectory` 토픽은 개발 시험에만 쓰고 운영 경로로 쓰지 않는다.

## 명령 끊김

`cmd_vel_timeout`은 0.5 s다. A4 게이트의 0.2 s 만료를 대신하지 않으며 게이트가 멈췄을 때 뒤에서 받치는 값이다.

## 정지 피드백

실제 정지 판정(D-06)과 `actuation_state` 발행은 A4 담당이다. A2는 아래 실측값을 제공한다.

| 토픽 | 주기 | 쓸 수 있는 값 | 한계 |
|---|---|---|---|
| `joint_states` | 100 Hz | 관절 위치·속도 | 바퀴 `effort`는 NaN이므로 쓰지 않는다 |
| `diff_drive_controller/odom` | 50 Hz | 선속도·각속도 | 바퀴 회전으로 계산하므로 차체 미끄러짐을 보지 못한다 |

임시 모델 무부하, 시뮬레이션 시간 기준 1회 측정값이다. 명령 종료부터 D-06 값 안으로 들어오기 시작한 시점까지이며, A4의 0.5 s 유지 시간은 포함하지 않는다.

| 시나리오 | odom 기준 | Gazebo 실제 | odom 정지 후 실제 이동 |
|---|---|---|---|
| 직진 0.3 m/s | 0.723 s | 0.757 s | 0.7 mm |
| 직진 0.8 m/s | 1.647 s | 1.647 s | 0.3 mm |
| 제자리 회전 0.5 rad/s | 0.768 s | 0.768 s | 0.0 mm |
| 호 주행 0.3 m/s, 0.3 rad/s | 0.705 s | 1.589 s | 40.4 mm |
| 팔 이동·복귀 | 0.36 s | 0.36 s | 0.0 mm |

호 주행에서는 odom이 정지를 먼저 보고한다. odom만으로 실제 정지를 판정하면 차체가 아직 움직일 수 있다.
Gazebo 실제 값은 시험 비교용이며 운영 판정에 쓰지 않는다.

## 개발 시험

```bash
python3 src/ax_control/scripts/check_command_paths.py --robot-id robot_1
python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_1
```

`check_command_paths.py`는 시뮬레이션 실행 중 제어기를 A4 없이 직접 구동해 관절 상태, 바퀴 구동·정지, 팔 궤적, 잘못된 관절 거부, TF 접두사를 점검한다.
`check_stop_feedback.py`는 피드백 주기와 시나리오별 정지 시간을 재고 제어기 측정값을 Gazebo 실제 값과 비교한다.
둘 다 운영 경로가 아니므로 격리된 개발 시뮬레이션에서만 실행한다.
