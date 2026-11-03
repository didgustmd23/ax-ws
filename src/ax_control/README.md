# ax_control

A2 기본 제어 패키지다. 모든 로봇의 제어기 설정(단일 기준)과 제어 포함 시뮬레이션 launch, 흡착 부착 플러그인을 둔다.
명령 경로 계약은 [공통 인터페이스 계약](../../docs/design/INTERFACE_CONTRACT.md) 4.3절이 기준이다.

## 실행

```bash
ros2 launch ax_control simulation.launch.py                         # robot_1(팔)·robot_2(적재함)
ros2 launch ax_control simulation.launch.py robot_ids:=robot_1 gui:=false
```

인자는 `ax_description robot_test.launch.py`와 같다: `world`(`ax_test.world`), `robots`(1·2·4), `robot_ids`,
`gui`, `sensors`, `tool`(기본 `epick`), `payload_mass`. 로봇 ID `robot_<n>`이 namespace `/ax/robot_<n>`과 TF 접두사 `robot_<n>/`를 정한다.

| 구성 | 소유 | 내용 |
|---|---|---|
| 월드·모델·센서 브리지 | A1 `robot_test.launch.py` | 이 launch가 포함한다. `controller_config_dir`로 아래 YAML을 넘기면 모델에 gz_ros2_control이 들어간다 |
| 제어기 설정 | A2 `config/` | `common_controllers.yaml`(공통) → 모델별 `ax1_controllers.yaml`(AX_1, 팔 포함)·`ax2_controllers.yaml`(AX_2) 순서로 읽는다 |
| 제어기 활성화 | A2 이 launch | 로봇별 spawner. `diff_drive_controller`는 `tf_frame_prefix`를 로봇별로 넘긴다 |
| 공구 | A2 | `tool:=epick`·`suction`일 때 팔 로봇에 흡착 플러그인(모델 `ax_tool_attach` 매크로)·공구 브리지·`suction_tool_node` |

`robot_test.launch.py`를 직접 실행하면 제어기 없이 모델·센서만 뜬다.

### 바퀴 보정값

| 모델 | `wheel_separation_multiplier` | 근거 |
|---|---|---|
| AX_1 | 1.2315 | 임시 모델 실측. A1 모델 drive_repair 증거(제자리 회전 좌·우 각 1회)에서 계산한 1.232와 일치 |
| AX_2 | 1.1318 | A1 drive_repair 증거에서 계산한 잠정값. A2 직접 재측정 필요 |

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

## 흡착 부착·해제

기본 `DetachableJoint`는 연결 대상을 불러올 때 고정해서 실행 중에 정해지는 물체를 붙일 수 없다. 설계의 조건부 기준에 따라 플러그인 `ax_suction_attach_system`을 쓴다.

| 구분 | 경로 | 비고 |
|---|---|---|
| 요청 | `/ax/<robot_id>/manipulation/tool_command` (`ToolCommand`) | C4 → 공구 노드. 수락은 완료가 아니다 |
| 결과 | `/ax/<robot_id>/manipulation/attach_state` (`AttachState`, 20 Hz) | 플러그인의 실제 연결 상태. 보유 판정은 C4 담당이다 |
| 내부 | `tool/gz_command`, `tool/gz_state` | 공구 노드와 플러그인 사이 전용. 다른 노드는 쓰지 않는다 |

수락에는 `arm_allowed`가 유효·최신이고 A3 `CheckCommandLease`가 허용해야 한다. A3·A4가 없으면 모든 요청을 거절한다.
안전 정지나 A3 무응답으로 물체를 자동 해제하지 않으며, 해제는 명시적 `RELEASE`로만 한다.

- 부착 조건은 `epick_tcp`와 물체 링크 중심의 거리 0.12 m 이하다. 계약에 값이 없어 정한 개발용 잠정값이고 `ax_description/urdf/ros2_control.xacro`의 `ax_tool_attach` 매크로에 있다.
- 플러그인 부모는 `gripper_base_link`이고 `epick_tcp`까지 거리는 EPick 0.1373 m, 가상 4컵 0.082 m다(`ax.urdf.xacro`). 공구 기하와 어긋나면 `tools/tests/test_w1_models.py`가 실패한다.
- 물체 이름은 Gazebo 모델 이름과 같아야 하며 공백을 쓸 수 없다.
- 진공·접촉 물리는 없다. 조건을 만족하면 고정 관절로 잇는다.
- 받침면에 닿은 채 수평으로 끌면 팔이 움직이지 않는 것을 확인했다. 부착 뒤 먼저 들어 올려야 한다.

## 개발 시험

```bash
python3 src/ax_control/scripts/check_command_paths.py --robot-id robot_1
python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_1
python3 src/ax_control/scripts/check_tool_attach.py --robot-id robot_1
```

`check_command_paths.py`는 시뮬레이션 실행 중 제어기를 A4 없이 직접 구동해 관절 상태, 바퀴 구동·정지, 팔 궤적, 잘못된 관절 거부, TF 접두사를 점검한다.
`check_stop_feedback.py`는 피드백 주기와 시나리오별 정지 시간을 재고 제어기 측정값을 Gazebo 실제 값과 비교한다.
`check_tool_attach.py`는 A3·A4 모의 상대를 띄우고 상자를 만들어 부착·거절·재전송·안전 정지 중 유지·해제를 시험한다. 시뮬레이션을 새로 띄운 뒤 한 번만 실행한다.
모두 운영 경로가 아니므로 격리된 개발 시뮬레이션에서만 실행한다.
