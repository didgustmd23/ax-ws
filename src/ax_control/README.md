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

## 개발 시험

```bash
python3 src/ax_control/scripts/check_command_paths.py --robot-id robot_1
```

시뮬레이션 실행 중 제어기를 A4 없이 직접 구동해 관절 상태, 바퀴 구동·정지, 팔 궤적, 잘못된 관절 거부, TF 접두사를 점검한다.
운영 경로가 아니므로 격리된 개발 시뮬레이션에서만 실행한다.
