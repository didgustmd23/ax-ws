# ax_control

A2 기본 제어 패키지다. 모든 로봇의 제어기 설정(단일 기준)과 제어 포함 시뮬레이션 launch, 흡착 부착 플러그인을 둔다.
명령 경로 계약은 [공통 인터페이스 계약](../../docs/design/INTERFACE_CONTRACT.md) 4.3절이 기준이다.

주행 기본값은 0.67 m/s, 상한은 ±0.67 m/s이며 가속·감속은 양방향 0.67 m/s²다.
정지 출발 후 1초 가속·1초 정속으로 이론상 1.005m 이동하는 AX 시뮬레이션 개발용 설정이며 제조사 공식값이나 AX 실물 정격은 아니다.
회전은 AX 개발용 기본 0.20 rad/s, 상한 ±0.50 rad/s, 가감속 0.20 rad/s²다.
`config/common_controllers.yaml`을 제어기와 조종 UI가 함께 읽는다. 제한 후 명령은 `diff_drive_controller/cmd_vel_out`, 실제 주행 추정은 `odom`에서 확인한다.
정지 명령도 감속 제한을 거치므로 최대 속도에서 약 1초·0.335m의 정지 시간·거리가 필요하다.
아래 기존 시험의 속도·정지 측정값은 당시 시험 기록이며 현재 권장 속도와 구분한다.

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
| AX_1 | 1.2315 | 임시 모델 실측. A1 모델 drive_repair 증거의 1.232와 일치. `check_wheel_calibration.py` 측정(2026-11-04, 좌·우 각 2회) 권장 1.2321 |
| AX_2 | 1.1318 | A1 drive_repair 증거에서 계산한 잠정값. `check_wheel_calibration.py` 측정(2026-11-04, 좌·우 각 2회) 권장 1.1291 |

두 측정 모두 현재값과 0.3 % 안쪽으로 맞아 설정은 바꾸지 않았다. 물리 설정(질량·마찰·robot_2 차체)이 확정되면 같은 도구로 다시 재고 반영 여부를 정한다.

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

현재 주행 제한(직진 0.67 m/s·0.67 m/s², 회전 0.50 rad/s·0.20 rad/s²)과 12컵 헤드, 무부하에서 잰 2회 평균이다(2026-11-06, `check_stop_feedback.py --repeats 2`).
명령 종료부터 D-06 값 안으로 들어오기 시작한 시점까지이며, A4의 0.5 s 유지 시간은 포함하지 않는다. 이 표와 `--stop-mode` 추가 전 결과(11-09 S005 재검증·robot_2 정지 시간 포함)는 모두 `command_loss` 방식(주행 명령 발행만 끊고 0을 보내지 않음)이라 제어기 `cmd_vel_timeout`(0.5 s) 대기와 감속이 함께 들어 있다.
"보고값"은 odom·`joint_states`, "Gazebo 실제"는 차체 모델 자세와 팔 링크 자세로 다시 계산한 관절각 속도다.
물리 설정(effort·질량·마찰)이 잠정값이므로 성능값이 아니라 측정 경로 확인용이다.

| 시나리오 | 보고값 기준 | Gazebo 실제 | 보고값 정지 후 실제 이동 |
|---|---|---|---|
| 직진 0.3 m/s | 0.955 s | 0.955 s | 0.0 mm |
| 직진 상한 0.67 m/s | 1.525 s | 1.508 s | 0.0 mm |
| 제자리 회전 상한 0.5 rad/s | 2.928 s | 2.910 s | 0.0 mm |
| 호 주행 0.3 m/s, 0.3 rad/s | 1.924 s | 1.924 s | 0.0 mm |
| 팔 이동 | 0.347 s | 0.381 s | 0.0 mm |
| 팔 복귀 | 0.361 s | 0.395 s | 0.0 mm |

감속 제한 때문에 정지가 길어졌다. 특히 회전은 감속 0.20 rad/s²라 상한에서 약 2.9 s가 걸린다.
그래서 주행 시나리오의 기록 구간은 고정 3 s가 아니라 "명령 속도 ÷ 감속 제한 + D-06 유지 0.5 s + 여유 1 s"(최소 3 s)로 계산한다.
제한 전 측정(2026-11-04)에서는 호 주행 때 odom이 실제보다 0.17 s 먼저 정지를 보고했고, 이전 임시 모델에서는 odom 정지 뒤 차체가 40.4 mm 더 움직였다.
odom만으로 실제 정지를 판정하면 차체가 아직 움직일 수 있다.
Gazebo 실제 값은 시험 비교용이며 운영 판정에 쓰지 않는다.

주행 정지·바퀴 odom 판정 기준은 [`config/stop_odom_criteria_dev.yaml`](config/stop_odom_criteria_dev.yaml)이다(개발용, 2026-11-14 담당(2번) 승인, 소비자 합의 전, 운영·실물 기준 아님). 정지 정의는 계약 D-06을 따르고 이론 감속 시간은 `common_controllers.yaml` 감속 제한으로 계산한다. 두 측정 도구는 `--criteria`로 이 파일을 읽어 결과에 항목별 판정을 더한다(아래 사용법).

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
- 플러그인 부모는 `gripper_base_link`이고 `epick_tcp`까지 거리는 다중 흡착판(`epick`) 0.15034361 m, 가상 4컵 0.082 m다(`ax.urdf.xacro`의 `tool_attach_tcp_offsets`). 다중 흡착판 TCP는 집게 TCP와 같은 위치에 맞췄다([교체 공구 TCP](../ax_description/config/README.md#교체-공구-tcp)). 공구 기하를 바꾸면 TCP 거리 설정도 함께 확인한다.
- 물체 이름은 Gazebo 모델 이름과 같아야 하며 공백을 쓸 수 없다.
- 진공·접촉 물리는 없다. 조건을 만족하면 고정 관절로 잇는다.
- `epick` 공구는 참고 사진으로 만든 다중 흡착판 비교 모델(4×3 컵, 지름 28 mm, 간격 36 mm, `ax_description/config/epick.yaml`)이며 실물 제품 사양이 아니다.
  헤드 치수·장착 위치가 바뀌면 TCP 거리(`tool_attach_tcp_offsets`), 부착 거리, ToF `lip_offset_m`, 시험 상자 위치(`BOX_BELOW_TCP_M`), `check_payload_tracking.py`의 뻗은 자세 충돌 선별을 함께 다시 확인한다.
- 받침면에 닿은 채 수평으로 끌면 팔이 움직이지 않는 것을 확인했다. 부착 뒤 먼저 들어 올려야 한다.

## 개발 시험

```bash
python3 src/ax_control/scripts/check_command_paths.py --robot-id robot_1
python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_1 [--repeats 5]
python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_2 [--repeats 3] [--payload-kg 0.5|1|2]
python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_2 --stop-mode zero_command [--repeats 3]
python3 src/ax_control/scripts/check_wheel_calibration.py --robot-id robot_2 [--repeats 3] [--settle 4.0]
python3 src/ax_control/scripts/check_wheel_calibration.py --robot-id robot_1 --arm-pose=-1,0,0,0,0 --settle 4.0
python3 src/ax_control/scripts/check_tool_attach.py --robot-id robot_1 [--box-mass 2.0]
python3 src/ax_control/scripts/check_payload_tracking.py --robot-id robot_1 [--masses 0 0.5 1 2] [--repeats 3]
```

`check_stop_feedback.py`·`check_wheel_calibration.py`의 개발용 판정 기준은 [`config/stop_odom_criteria_dev.yaml`](config/stop_odom_criteria_dev.yaml)이다(개발용, 2026-11-14 담당(2번) 승인, 소비자 합의 전, 운영·실물 기준 아님). `--criteria <기준 파일>`을 주면 결과에 `criteria_items`·`judgment`가 더해진다(판정 값 PASS·FAIL·not_judged·not_applicable·recorded). 과거 결과 재판정과 여러 회차 묶음 판정은 ROS 없이 `python3 src/ax_control/scripts/a2_criteria.py --rejudge <result.json> --criteria <기준 파일>`, `... --batch <result.json>... --criteria <기준 파일> --output <묶음.json>`로 한다.

묶음 판정은 같은 회차가 두 번 이상 들어오면 반복 수·부하·coverage를 계산하기 전에 묶음 전체를 거부한다. 중복 기준은 셋이다. 정규화한 경로(`..`·symlink 별칭 포함), 원본 바이트 SHA-256, 측정 내용 SHA-256이며 하나라도 같으면 중복이다. 측정 내용 SHA-256은 판정 부가 필드 `criteria`·`criteria_items`·`judgment`·`judgment_reason`·`judged_subset`·`rejudged_from`·`sample_gap`을 빼고 계산한다. `rtf`는 측정값만 남긴다. 재판정을 거듭해 판정기 기록 사전(`rtf_min`·`value`·`use`)이 겹친 경우에도 그 겹만 벗기고, 값이 없으면 `rtf`가 없는 결과와 같게 본다. 따라서 원본과 `.judged.json`, 기준을 바꿔 다시 만든 재판정본, 서식만 다른 사본도 같은 회차로 본다. 거부하면 stderr에 `중복 회차 입력`과 두 입력 경로·일치 근거를 내고 종료 코드 2로 끝난다. 묶음 파일은 쓰지 않고 입력 원본과 기존 출력은 그대로 둔다. 한 파일 안의 반복과 서로 다른 회차의 반복은 기존대로 더한다. 결과에 회차 식별자가 없으므로, 따로 측정했더라도 저장 내용이 완전히 같으면 구별할 수 없어 중복으로 거부한다. 파일 이름이 달라도 독립 회차로 보지 않는다.

`--world-file <월드 파일>`을 주면 결과 `conditions`에 `world_file`·`world_sha256`이 남는다. 같은 `GZ_PARTITION`의 `gz sim` 서버 명령줄 월드와 sha256이 같은지도 `world_check`에 남긴다. 서버 명령줄을 찾지 못하면 "인자 기준"으로 남는다. 측정 시작 시 같은 ROS domain에서 `/ax/<id>/joint_states` 발행자가 있는 로봇이 `active_robot_ids`·`active_robot_count`로, 도구 프로세스의 렌더러 환경값이 `renderer`로 기록된다. 묶음 판정의 반복 수(A2-S7)와 부하 불변성(LD-1)은 이 필드를 쓰며, 필드가 없는 옛 결과에서는 `not_judged`다.

정지 피드백·바퀴 보정·부착·하중 시험 도구는 시뮬레이션과 같은 `ROS_DOMAIN_ID`·`GZ_PARTITION`·`ROS_AUTOMATIC_DISCOVERY_RANGE`에서 실행해야 한다. 대상(robot_description·제어기 서비스·Gazebo 월드)을 찾지 못하면 현재 환경 변수, 다른 노드가 발행 중인 로봇 ID, 발행자가 있는 Gazebo 월드와 원인 추정을 함께 출력한다. 정지 피드백·바퀴 보정·부착 결과의 `status`는 `completed`·`failed`·`interrupted`(Ctrl+C, 종료 코드 130)이며, 중단돼도 `result.json`을 남긴다. 정지 피드백은 시나리오마다, 부착 시험은 판정마다 중간 저장하므로 `last_scenario`·`last_check`까지의 결과가 보존된다.

`check_command_paths.py`는 시뮬레이션 실행 중 제어기를 A4 없이 직접 구동해 관절 상태, 바퀴 구동·정지, 팔 궤적, 잘못된 관절 거부, TF 접두사를 점검한다.
`check_stop_feedback.py`는 피드백 주기와 시나리오별 정지 시간을 재고 제어기 측정값을 Gazebo 실제 값과 비교한다. 팔의 실제 값은 Gazebo 링크 자세로 다시 계산한 관절각 속도다. `--repeats`로 반복하면 평균·표준편차를 내고, 주행 시나리오는 측정 뒤 반대 명령으로 되돌아간다. 결과는 `log/a2_stop_feedback/<KST 시각>/result.json`이다.
대상 범위는 실행 중 모델의 `robot_description`(URDF)으로 정한다.
- 공통 주행 시나리오(직진 0.3 m/s·상한 직진·제자리 회전 상한·호 주행): 바퀴 `joint_states`·odom·Gazebo 차체 자세만 쓰며 모든 로봇에 적용한다.
- 팔 시나리오와 주행 중 팔 정지 조건: 팔 관절이 모두 있는 로봇(robot_1 계열)만. 팔이 없으면(robot_2) `arm_scenarios`·`observations.arm`을 `not_applicable`로, 주행 결과에 `arm_stop: not_applicable`을 남기며 팔 정지 성공으로 기록하지 않는다. 팔 관절이 일부만 있거나 바퀴 관절이 없으면 대상 오류로 실패한다.
- 주행 정지 방식 `--stop-mode`(주행 시나리오만, 팔 Action 시나리오는 기존 기준 그대로):
  - `command_loss`(기본, 기존 결과와 같은 방식): 주행 명령 발행을 끊고 정상 측정 구간에는 0을 보내지 않는다. 기준 시각은 발행 중단 시각이고, 제어기가 `cmd_vel_timeout` 경과 뒤 스스로 감속하므로 결과에 timeout 대기가 들어간다. 기록 구간은 실행 중 `cmd_vel_timeout`만큼 늘린다(회전 4.0→4.5 s, 호 3.0→3.5 s).
  - `zero_command`: 주행 직후 선속도·각속도 0의 `TwistStamped`를 바로 보내고 기록 구간 끝까지 0.05 s(시뮬레이션 시간)마다 유효한 stamp로 다시 보낸다. 기준 시각은 최초 0 발행 시각이며 반복 발행으로 바뀌지 않는다. 제어기 수신 시각은 관측하지 않으므로 발행 시각을 수신 시각으로 쓰지 않는다.
  - 시험 보호·예외·종료 정리의 정지 명령은 두 방식 모두 기존과 같다. 되돌아가는 주행(측정 안 함)도 같은 방식으로 멈춘다.
  - 결과 파일마다 `stop_mode`가 하나다(최상위 `stop_mode`, `conditions.stop_mode`·`stop_reference`·`cmd_vel_timeout_s`). 한 시나리오에 다른 방식 결과가 섞이면 집계가 실패한다. 회차마다 `command`(명령 시작·발행 수, 마지막 비영 명령 stamp·발행 시각, 발행 중단 시각, 최초 0 발행 시각·stamp, 0 발행 수·최대 간격, 기준 시각)와 `deceleration_onset_s`(기준 시각 직전 속도의 95 % 아래로 처음 내려간 시점, 기록용)를 남긴다.
  - 명령 구간을 다 채우지 못하면(시뮬레이션 정지·벽시계 상한) `failed`, 명령 구간 표본이 없으면 정지 미확인으로 남긴다.
- 정지 시간(`odom_stop_s`·`truth_stop_s`)은 기존 의미 그대로 기준 시각부터 D-06 값 안으로 들어오기 시작한 시점이다. 0.5 s 유지까지 확인된 시각은 `*_hold_confirmed_s`, 두 정지 시각 차이(실제−odom)는 `stop_time_difference_s`로 따로 남긴다.
- 필요한 입력이 0.25 s 넘게 끊기면 표본을 만들지 않고, 명령 종료 뒤 표본 부족·입력 단절·기록 구간 부족·기준 미확인은 정지 미확인으로 남긴다. `runs`의 실패 항목은 기존처럼 `null`이고 사유는 `failure_details`에 있다. 0.25 s는 자료 연속성 확인값이며 성능 기준이 아니다.
- 시험 조건(`conditions`): 로봇·월드·`robot_description` 해시·바퀴 마찰·제어기 설정 파일 해시·실행 중 diff_drive 파라미터·하중.
- `--payload-kg`(팔 없는 적재 로봇 전용, A1 `test_payloads_kg` 0.5·1·2 kg만): A1 이동 시험(`tools/verify_a1_envelope.py --mobility`)과 같이 20 cm 상자를 `cargo_link` 중심 0.12 m 위에 만들어 적재판에 올린다. 고정 관절·마찰 변경은 없고(수동 적재판), 시험 뒤 상자를 지운다. 적재 결과를 다른 상자·배치로 일반화하지 않는다.
- 시험 보호용 중단(성능 기준 아님): 차체 기울기 `--tilt-abort`(기본 0.15 rad) 초과, 차체 높이 변화 0.02 m 초과, 적재물이 `cargo_link` 중심에서 수평 0.02 m·높이 0.01 m(A1 이동 시험 값) 넘게 벗어나면 정지 명령을 보내고 `failed`로 남긴다.
`check_wheel_calibration.py`는 제자리 회전(좌·우)과 직진(전·후진)을 반복하며 odom 값과 Gazebo 실제 값의 비율로 `wheel_separation_multiplier` 권장값과 `left/right_wheel_radius_multiplier` 참고값을 계산한다. 실행 중인 제어기의 파라미터를 읽으며 설정 파일은 바꾸지 않는다. 결과는 `log/a2_wheel_calibration/<KST 시각>/result.json`이다.
회차마다 다음을 함께 기록한다.
- 차체 최대 기울기(Gazebo 모델 자세)
- 회차 끝 실제 정지: 마지막 0.5 s 유지 구간(그 시작 시각 이전 표본 1개 포함)에서 0.1 s 창마다 Gazebo 자세의 경로 길이(인접 표본 이동·|Δyaw| 합)를 창 시간으로 나눈 속도의 최대값이 D-06 값(0.01 m/s·0.02 rad/s) 안이어야 한다. 순변위가 아니라 경로 길이라 왕복·중간 초과가 상쇄되지 않는다. `end_speed`의 `linear_m_s`·`angular_rad_s`는 이 최대값이고, `stop_hold_check`에 방식을 남긴다. 표본 2개 미만·유지 구간 부족·표본 간격 0.1 s 초과·시각 중복/역행·비유한 값은 `problem`에 사유를 남기고 정지 미확인으로 중단한다. 0.1 s 간격 상한은 자료 연속성 확인값이며 정지 임계값이 아니다.
- odom·실제 회전각·이동 거리 차이
- 회전 중 병진·회전 중심

기울기가 `--tilt-abort`(기본 0.15 rad, A1 mobility 시험과 같은 값)를 넘거나 정지가 확인되지 않으면 측정을 중단한다. 이상 감지용 값이며 성능 기준이 아니다.

회전 감속 제한(0.20 rad/s²)으로 0.5 rad/s에서 정지까지 약 2.5 s가 걸린다. 그래서 기본 `--settle 2.0`에서는 정지 확인이 실패할 수 있으므로 `--settle 4.0`을 쓴다.

팔 로봇은 `--arm-pose`로 정지 상태에서 팔 자세를 먼저 바꾸고, `joint_states`가 `--arm-tolerance`(기본 0.02 rad, A1 `joint_tolerance_rad`) 안에 1 s 머문 뒤 주행한다. 주행 중 팔 자세 편차(`joint_states` 기준)를 회차마다 기록한다. 차체 충돌 센서가 없어 접촉은 기울기·궤적으로만 간접 확인한다.
`check_tool_attach.py`는 A3·A4 모의 상대를 띄우고 상자를 만들어 부착·거절·재전송·안전 정지 중 유지·해제를 시험한다. 시뮬레이션을 새로 띄운 뒤 한 번만 실행한다. 상자 질량은 `--box-mass`(kg, 기본 1.0)로 바꾼다. 결과는 `log/a2_tool_attach/<KST 시각>/result.json`이며 판정별 측정값(`values`: 상승량·이동량·높이·거부 사유)과 시험 배치(`scene`: TCP·상자·받침대 위치)를 남긴다.
`check_payload_tracking.py`는 흡착으로 붙인 상자로 단계 하중(기본 0·0.5·1·2 kg) 추종·유지·운반 중 취소 정지를 시험한다.

- 자세: A1 검증 자세 3개와 공구 하향으로 어깨 축에서 TCP까지 수평 약 1.0 m(`reach_side`)·1.29 m(`reach_max`)인 자세. 충돌 선별(매단 상자 포함)과 관절 한계 여유 0.1 rad 이상으로 골랐다.
- 독립 관측: `joint_states` 위치·속도와 함께 Gazebo 링크 자세로 다시 계산한 관절각(`gazebo_observer.py`), TCP 기준 상자 상대 위치(미끄러짐), 상자 실제 높이(들어 올림·낙하), 차체 기울기를 기록한다.
- 구조 판정만 PASS/FAIL이다: Action 성공, 두 관측 일치(1e-3 rad), 부착 신호와 실제 위치 일치, effort가 URDF 한계의 95% 미만, 차체 기울기 0.25 rad 이하. 추종 오차·정지 시간·effort 비율은 기준 없이 기록만 한다.
- 결과는 `log/a2_payload/<KST 시각>/`의 단계별 JSON과 `summary.json`이다. 물리 설정이 잠정값이므로 기록값을 AX 하중 성능으로 쓰지 않는다. 부착은 고정 관절이라 상자가 떨어지지 않은 것은 흡착력 검증이 아니다.

### S005 반영 모델 구동 재검증 (2026-11-09)

조건: 커밋 `6ac7ec8`(+ 이 도구 보완), 바퀴 `slip1 0.05`·`mu1 0.1`·`mu2 1.0`(실행 중 `robot_description`에서 확인), `ax_test.world`, headless, 공구 `epick`, 하중 0 kg. 결과는 `log/a2_s005_revalidation/`(Git 제외). 판정 기준이 없는 수치이며 설정은 바꾸지 않았다.

| 시험 | 결과 |
|---|---|
| robot_2 무부하, 좌·우 회전·전·후진 각 3회(`--settle 4.0`) | 실행 완료, 구조 판정 실패 0, 회차 끝 정지 12회 확인(당시 순변위 기준). 회전 비율(odom/실제) 0.9996(표준편차 < 1e-5), 회전 오차 ±0.0008 rad, 직진 비율 1.0000, 회전 중 실제 병진 왼쪽 0.009·오른쪽 0.004 m, 회전 중심 차체 앞 -0.006·-0.001 m, 권장 multiplier 1.1314(현재 1.1318) |
| robot_1 흡착판·무부하, 주행 자세 `[-1,0,0,0,0]` rad, 같은 시험 | 정지 상태 자세 전환 6.0 s 뒤 정착(오차 4e-7 rad), 주행 중 팔 편차 < 5e-6 rad(`joint_states` 기준, Gazebo 링크 독립 확인 아님). 구조 판정 실패 0, 정지 12회 확인(당시 순변위 기준). 회전 비율 1.0000, 직진 1.0000, 회전 중 병진 0.025·0.030 m, 권장 multiplier 1.2315(현재와 같음) |
| `check_stop_feedback.py --robot-id robot_1 --repeats 3`(새 실행, 팔 neutral) | 6개 시나리오 18회 정지 확인. 직진 0.3 m/s 0.956 s, 직진 0.67 m/s 1.541 s(실제 1.530), 회전 0.5 rad/s 2.941 s(실제 2.919), 호 1.942 s, 팔 이동 0.338 s(실제 0.371), 팔 복귀 0.339 s(실제 0.466). odom 정지 뒤 실제 추가 이동 최대 4.5 mm |
| robot_2 정지 시간 | 이 표 작성 당시 미검증이었다. 도구 확장 뒤 측정 결과는 아래 「robot_2 주행 정지 시간」 |
| robot_2 적재 주행(A1 `verify_a1_envelope.py --mobility`, 저장소 밖 복사본에서 실행) | 0·0.5·1·2 kg × 회전·전후진·곡선 6동작 24건과 robot_1 주행 자세 6건 통과(A1 도구 기준). 상자 높이 오차 < 1e-6 m, 수평 이동 최대 6 mm, 기울기 3e-6 rad 수준. 정지 시간은 재지 않는다 |

표의 `check_wheel_calibration.py` 회차 끝 정지 확인은 당시 구현으로 마지막 0.5 s 양끝 자세의 순변위/시간만 본 값이다. 그 구간 안의 왕복·중간 초과는 걸러지지 않았다. 원본 결과에는 회차별 요약(`end_speed`)만 있고 연속 자세 표본이 없어 수정된 유지 판정으로 다시 계산할 수 없다. 그래서 이 정지 확인을 수정된 판정의 통과로 보지 않는다. 보정 비율·권장 multiplier는 정지 판정과 별개이며 그대로 둔다. `check_stop_feedback.py`는 표본마다 0.1 s 창 양끝 차분 속도가 0.5 s 동안 기준 안인지 보는 별도 판정이라 이번 수정 범위가 아니다(0.1 s보다 짧은 왕복은 그 방식에서도 상쇄될 수 있다).

A1의 S005 시험과는 조건이 다르다. A1 도구는 두 로봇 동시 spawn, 센서 끔, 소프트웨어 렌더링에서 주행 동작별 변위·기울기를 보고, 이 시험은 로봇 1대, 센서 켬, odom·실제 비율과 정지 확인을 본다. 두 결과를 같은 값으로 비교하지 않는다.

### 바퀴 보정 회차 끝 정지 유지 재확인 (2026-11-09)

수정된 유지 판정(`stop_hold_check.method` `max_short_window_path`)으로 새로 실행한 조건만 적는다. 조건은 위 S005 재검증과 같고(`ax_test.world`, headless, `epick`, 0 kg, 바퀴 `slip1 0.05`, 설정 변경 없음), 회차는 방향별 1회(`--repeats 1 --settle 4.0`)다. 결과는 `log/a2_stop_hold_recheck/`(Git 제외)에 있다.

| 조건 | 결과 |
|---|---|
| robot_2 무부하, 좌·우 회전·전·후진 각 1회 | `completed`, 회차 끝 정지 4/4 확인. 유지 구간 0.1 s 창 최대 속도: 회전 2.1e-5 m/s·1.7e-12 rad/s, 직진 0. 표본 52~58개, 최대 간격 0.010 s. 회전 비율 0.9996, 직진 1.0000, 권장 multiplier 1.1314 |
| robot_1 주행 자세 `travel_pose_rad` `[-1,0,0,0,0]`, 무부하, 같은 시험 | `completed`, 정지 4/4 확인. 최대 속도: 회전 3.8e-4 m/s·1.6e-10 rad/s, 직진 0. 표본 33~34개, 최대 간격 0.018 s. 회전·직진 비율 1.0000, 권장 multiplier 1.2315 |

앞 S005 재검증 표의 `check_wheel_calibration.py` 각 3회 결과는 다시 돌리지 않았으며, 그 회차 끝 정지 확인은 당시 순변위 기준으로 남는다. `check_stop_feedback.py`의 하중·정지 방식 시험은 별도 측정 방식(시각별 0.1 s 창 속도의 0.5 s 유지)을 쓰며 그 결과는 이번 수정과 관계없이 그대로 둔다.

### robot_2 주행 정지 시간 (2026-11-09)

조건: 커밋 `6ac7ec8` + 작업트리(`check_stop_feedback.py` 확장), `ax_test.world`, headless, 로봇 1대(`robot_ids:=robot_2`, 센서 켬), 바퀴 `mu1 0.1`·`mu2 1.0`·`slip1 0.05`(실행 중 `robot_description`), 실행 중 diff_drive multiplier 1.1318·주행 제한은 `common_controllers.yaml`과 같음. `--repeats 3`, 시나리오당 3회, 하중 조건마다 새 시뮬레이션. 적재는 A1 이동 시험과 같은 20 cm 상자를 `cargo_link` 중심에 수동 적재(고정 없음). 결과는 `log/a2_stop_feedback/20261109_*_robot_2_*`(Git 제외), 조건은 각 `result.json`의 `conditions`와 `session/conditions.json`.

정지 시간은 `command_loss` 방식(명령 발행 중단부터, `cmd_vel_timeout` 0.5 s 대기 포함)으로 D-06 값 안으로 들어오기 시작한 시점(평균±표준편차, 3회)이다. 0.5 s 유지 확인은 여기에 0.5 s를 더한 `*_hold_confirmed_s`다. 차이는 실제−odom이며 표본 간격(약 0.03 s) 단위로 양자화된다. 합격 기준이 없어 시간만 기록한다.

| 하중 | 직진 0.3 m/s odom / 실제 | 직진 0.67 m/s | 제자리 회전 0.5 rad/s | 호 주행 0.3 m/s·0.3 rad/s |
|---|---|---|---|---|
| 0 kg | 0.978±0.005 / 0.978±0.005 s | 1.550±0.014 / 1.527±0.011 s | 2.947±0.003 / 2.940±0.009 s | 1.950±0.013 / 1.933±0.007 s |
| 0.5 kg | 0.988±0.013 / 0.978±0.017 s | 1.552±0.009 / 1.542±0.008 s | 2.949±0.007 / 2.940±0.010 s | 1.946±0.006 / 1.946±0.006 s |
| 1 kg | 0.991±0.016 / 0.981±0.003 s | 1.537±0.004 / 1.537±0.004 s | 2.951±0.008 / 2.933±0.011 s | 1.950±0.003 / 1.932±0.012 s |
| 2 kg | 0.982±0.009 / 0.973±0.007 s | 1.556±0.010 / 1.527±0.008 s | 2.953±0.006 / 2.935±0.014 s | 1.945±0.006 / 1.945±0.006 s |

- 실행·구조: 네 조건 모두 `completed`, 시나리오 48회 정지 확인, `failure_details` 없음, 팔 시나리오 `not_applicable`. 기록 구간은 직진·호 3.0 s, 회전 4.0 s이고 표본 최대 간격 0.031 s였다.
- 두 정지 시각 차이: 0~−30 ms(실제가 먼저이거나 같음). odom 정지 뒤 실제 추가 이동은 직진 ≤ 0.001 mm, 회전 0.37~0.40 mm, 호 0.32~0.35 mm.
- 보호 조건: 차체 기울기 최대 4.5e-6 rad, 높이 변화 최대 1.1e-6 m. 적재 상자는 안착 뒤 중심 수평 오차 < 1e-10 m, 시험 중 `cargo_link` 기준 수평 이동 최대 0.021 mm·높이 차 최대 0.0007 mm였고 시험 뒤 제거됐다.
- robot_1 회귀(같은 확장 도구, `--repeats 3`, 새 실행): 6개 시나리오 18회 정지 확인, 기존 결과 형식·시나리오 유지. 직진 0.3 m/s 0.975 s, 0.67 m/s 1.544 s(실제 1.533), 회전 2.929 s(실제 2.919), 호 1.927 s, 팔 이동 0.331 s(실제 0.364), 팔 복귀 0.324 s(실제 0.478). 같은 날 이전 실행과 같은 범위다.
- 한계: 평면 시험 월드·이 상자 크기·중심 배치·수동 적재판만이다. 다른 상자 배치·마찰·경사, A4 게이트 경유 정지는 재지 않았다. 두 로봇 동시 주행은 11-13 1세트(같은 동작 병렬)만 쟀다. 무부하 결과를 적재 정지 성능으로, 개발 물리 설정 결과를 실물 성능으로 쓰지 않는다.

### 주행 정지 방식 비교 (2026-11-09)

`--stop-mode`를 추가한 뒤 실제로 실행한 결과다. 조건은 위 절과 같고(`ax_test.world`, 로봇 1대, 실행 중 `cmd_vel_timeout` 0.5 s), 실행마다 새 시뮬레이션이다. 결과는 `log/a2_stop_feedback/20261109_12{2,3}*_robot_*`(Git 제외). 합격 기준이 없어 시간만 기록한다.

정지 시간(odom / 실제, 기준 시각부터 D-06 진입까지):

| 조건 | 방식 | 회수 | 직진 0.67 m/s | 제자리 회전 0.5 rad/s | 직진 0.3 m/s | 호 주행 |
|---|---|---|---|---|---|---|
| robot_2 0 kg | `command_loss` | 3 | 1.538±0.010 / 1.528±0.011 s | 2.951±0.011 / 2.935±0.005 s | 0.979 / 0.971 s | 1.952 / 1.933 s |
| robot_2 0 kg | `zero_command` | 3 | 1.077±0.011 / 1.054±0.011 s | 2.464±0.007 / 2.456±0.010 s | 0.513 / 0.496 s | 1.484 / 1.466 s |
| robot_2 2 kg | `zero_command` | 1 | 1.079 / 1.053 s | 2.470 / 2.444 s | 0.510 / 0.510 s | 1.486 / 1.461 s |
| robot_1 0 kg | `command_loss` | 1 | 1.545 / 1.545 s | 2.922 / 2.922 s | 0.982 / 0.982 s | 1.940 / 1.940 s |
| robot_1 0 kg | `zero_command` | 1 | 1.079 / 1.079 s | 2.495 / 2.461 s | 0.514 / 0.514 s | 1.479 / 1.448 s |

- 실행·구조: 다섯 실행 모두 `completed`, 실패 0, `failure_details` 없음. robot_2는 팔 시나리오 `not_applicable`, robot_1 팔 이동·복귀는 두 방식에서 기존 기준으로 실행됐다(0.322·0.328 s / 0.344·0.353 s, 1회).
- 감속 시작(`deceleration_onset_s`, 기준 직전 속도의 95 % 아래로 처음 내려간 시점): `command_loss` 직진 0.55~0.61 s·회전 0.65~0.68 s, `zero_command` 직진 0.07~0.14 s·회전 0.15~0.22 s. 값에는 5 % 감속에 걸리는 시간(감속 제한으로 직진 0.3 m/s 약 0.02 s, 0.67 m/s 약 0.05 s, 회전 약 0.13 s)과 관측 지연이 포함된다. robot_1 `command_loss` 0.67 m/s 1회는 odom 0.070 s·실제 0.589 s로 odom 쪽이 일시 변동에 걸렸다. 기록용이며 판정에 쓰지 않는다.
- 두 방식의 같은 시나리오 정지 시간 차이는 robot_2 0.46~0.49 s, robot_1 0.43~0.47 s(회전 odom 0.427 s)로 timeout 0.5 s와 비슷하지만 정확히 같게 맞추지 않았다(기준 시각은 도구 발행 시각이고 제어기 timeout 판정 시점은 관측하지 않음).
- `zero_command` 0 발행: 기록 구간 3.0 s에 60회, 4.0 s에 80회, 최대 간격 0.052 s. 기준 시각은 마지막 비영 명령 발행 뒤 0.025~0.050 s였다.
- robot_2 2 kg `zero_command`: 상자 안착 오차 < 1e-10 m, 시험 중 `cargo_link` 기준 수평 이동 최대 0.005 mm, 기울기 최대 3.2e-6 rad, 시험 뒤 제거.
- 한계: 1회 실행 조건(robot_2 2 kg, robot_1)은 반복 편차가 없다. 0.5·1 kg `zero_command`와 두 로봇 동시 주행(1세트)은 11-13에 다시 쟀다([아래 절](#s005ax_2-갱신-뒤-재확인-2026-11-13)). A4 게이트 경유 정지는 재지 않았다. `zero_command`는 도구가 0을 직접 보내는 시험 방식이며 A4 최종 구동 차단 방식이 아니다.

TCP 정렬(흡착판 장착 원점 13.04 mm 이동, PR #37) 뒤 격리 시뮬레이션 robot_1·다중 흡착판으로 다시 잰 기록이다(2026-11-06, 기본 인자).
부착 시험(2 kg)은 14개 판정, 하중 시험(0·0.5·1·2 kg, 자세 5개 × 3회, 취소 정지 3회)은 284개 판정 모두 통과했다.

| 2 kg 유지 자세 | 어깨 effort | 팔꿈치 effort | 손목 effort |
|---|---|---|---|
| neutral | 24% | 37% | 1% |
| reach_left | 37% | 35% | 8% |
| reach_right | 11% | 39% | 10% |
| reach_side | 67% | 46% | 2% |
| reach_max | 84% | 57% | 19% |

- effort는 URDF 한계(어깨 100, 팔꿈치 65, 손목 25 N·m, 개발용 잠정값) 대비 비율이며 3회 반복값이 같았다. TCP 정렬 전 기록(reach_max 어깨 84%, 팔꿈치 57%)과 차이가 1% 미만이다. 시험 자세에서 공구가 아래를 향해 TCP 이동이 거의 수직 방향이기 때문이다.
- 2 kg에서 TCP 기준 상자 미끄러짐 최대 0.0014 mm, 차체 기울기 최대 1.8e-6 rad였다. 운반 중 취소 뒤 정지 시간은 `joint_states` 약 0.10 s, Gazebo 약 0.14 s이고 정지 뒤 TCP 이동은 0.42~0.47 mm였다.
- 부착 시험의 TCP 높이는 1.1704 m에서 1.1573 m로 13.0 mm 낮아졌고, 들어 올림 0.125 m·팔 추종 0.399 m·해제 뒤 바닥 높이 0.075 m였다.

### S005·AX_2 갱신 뒤 재확인 (2026-11-13)

조건: 커밋 `e4dbaaa`, 격리 headless(domain 211~216, 실행마다 새 시뮬레이션), 공구 `epick`, 센서 켬. `ax_test.world`가 3dd8948에서 삭제돼 `tools/empty_test_fixture.py`의 빈 바닥(현 창고 월드 physics 1 ms, 바닥 마찰은 SDF 기본값, 이전 월드는 `mu 1.0` 명시)을 `test_fixture_world`로 썼다. robot_2는 11-11 갱신 모델(3dd8948·32e1031)이라 11-09 robot_2 값과는 다른 모델 사이의 비교다. 측정 중 RTF(Δsim/Δreal)는 0.40~0.91, load1 평균 8~12였다. 결과는 `log/a2_payload/20261113_*`, `log/a2_stop_feedback/20261113_*`, `log/a2_tcp_drift_20261113/`(Git 제외). 판정 기준이 없는 수치는 기록만 했다.

robot_1 하중 시험(0·0.5·1·2 kg, 자세 5개 × 3회, 취소 정지 3회)은 284개 판정 모두 통과했고, 2 kg effort는 위 표와 같다(차이 1%p 이하, 3회 반복값 같음). 운반 중 취소 뒤 정지 시간은 `joint_states` 0.09~0.10 s, Gazebo 0.14~0.16 s로 11-06과 같다. 정지 뒤 TCP 이동은 3.9~5.4 mm(0 kg 3.9~4.0, 2 kg 5.3~5.4)로 11-06(0.33~0.45 mm)보다 크고, 1 s 유지 구간의 TCP 이동도 최대 5.4 mm(11-06 0.13 mm 이하)였다.

정지 피드백 `zero_command`(odom / 실제, 기준 시각부터 D-06 진입까지):

| 조건 | 회수 | 직진 0.3 m/s | 직진 0.67 m/s | 제자리 회전 0.5 rad/s | 호 주행 |
|---|---|---|---|---|---|
| robot_2 0.5 kg | 3 | 0.514±0.014 / 0.502±0.013 s | 1.085±0.009 / 1.063±0.012 s | 2.492±0.007 / 2.458±0.007 s | 1.472±0.009 / 1.461±0.010 s |
| robot_2 1 kg | 3 | 0.522±0.014 / 0.501±0.011 s | 1.071±0.015 / 1.060±0.009 s | 2.482±0.013 / 2.461±0.006 s | 1.472±0.015 / 1.450±0.005 s |
| 동시 주행 robot_2 0 kg | 1 | 0.513 / 0.513 s | 1.075 / 1.075 s | 2.487 / 2.452 s | 1.476 / 1.476 s |
| 동시 주행 robot_1 | 1 | 0.531 / 0.499 s | 1.073 / 1.073 s | 2.499 / 2.467 s | 1.471 / 1.471 s |

- 실행·구조: 모두 `completed`, 실패 0, `failure_details` 없음. robot_2는 팔 시나리오 `not_applicable`. 동시 주행의 robot_1 팔 이동·복귀는 0.328 / 0.378 s, 0.365 / 0.496 s였다.
- robot_2 적재: 기울기 최대 6.7e-6 rad, 높이 변화 1.5e-6 m 이하, `cargo_link` 기준 수평 이동 최대 0.017 mm, 높이 차 0.0004 mm 이하, 시험 뒤 제거. odom 정지 뒤 실제 이동은 회전 0.24~0.26 mm, 호 0.21 mm였다.
- 11-09 값(robot_2 0 kg 3회, 2 kg 1회, robot_1 단독 1회)과 같은 시나리오 차이는 ±35 ms 안이다. 0.5·1 kg 사이의 질량 경향은 보이지 않았다.
- 동시 주행: 한 시뮬레이션에 두 로봇을 띄우고 기존 도구 두 개를 같은 시각에 시작했다(`--repeats 1`, robot_2 무부하, 도구 변경 없음). 시나리오별 정지 기준 시각 차이는 sim 0.43~0.45 s, 주행 3 s라 한 로봇이 정지할 때 다른 로봇은 주행 중이었다. 출발 위치는 옆 1.5 m·앞 2.5 m 떨어져 있고 접촉은 없었다. 교차·근접 주행이나 상호 간섭 시나리오는 아니다.

정지 뒤 TCP 이동 분리(robot_1, 0·2 kg, 취소 정지 각 3회, `reach_side` 쪽 이동 중 취소, Gazebo 정지 시각부터 약 2.9 s). 차체 자세(`base_link` 모델 자세)를 함께 기록해 차체 이동으로 생긴 TCP 이동과 차체 기준 팔 상대 이동으로 나눴다. 비교 모델은 바퀴 `slip1` 줄만 뺀 저장소 밖 사본이다(실행 중 `robot_description`의 `slip1` 0개 확인).

| 모델 | 하중 | 정지 뒤 TCP 이동 | 차체 이동 | 차체 기인 TCP 이동 | 팔 상대 이동 | 차체 yaw 변화 |
|---|---|---|---|---|---|---|
| S005 (`slip1 0.05`) | 0 kg | 3.85~3.95 mm | 4.16~4.29 mm | 4.16~4.29 mm | 0.33~0.35 mm | ≤ 6e-8 rad |
| S005 (`slip1 0.05`) | 2 kg | 5.31~5.36 mm | 5.67~5.75 mm | 5.67~5.75 mm | 0.38~0.42 mm | ≤ 3.3e-7 rad |
| `slip1` 없음 | 0 kg | 0.35~0.37 mm | 0.00 mm | 0.01 mm | 0.35~0.38 mm | ≤ 3.4e-9 rad |
| `slip1` 없음 | 2 kg | 0.41~0.43 mm | 0.00 mm | 0.00 mm | 0.42~0.43 mm | ≤ 1.5e-8 rad |

- 정지 뒤 TCP 이동 증가는 팔 자체 정지 오차가 아니라 차체 미끄러짐이다. 팔 상대 이동(0.33~0.43 mm)은 두 모델과 11-06 값이 같고, S005에서는 차체가 정지 뒤에도 옆(+Y)으로만 4.2~5.7 mm 계속 움직였다(전방·yaw 변화 거의 없음). 팔이 옆으로 뻗은 자세의 옆방향 하중을 받는 동안 `slip1` 순응(힘에 비례하는 옆 미끄럼 속도)으로 차체가 밀리는 것으로 해석한다. 유지 구간 1 s의 TCP 이동도 S005 최대 5.7 mm(자세·회차별 0~5.7 mm), `slip1` 없음 0.12 mm 이하였다. 정지 시간은 두 모델이 같다.
- 열린 문제(설정 변경 없음): 정지 중 팔 하중이 옆으로 걸리면 S005 모델의 차체는 멈춰 있지 않는다. 운반·작업 중 차체 정지 유지가 필요한 시험을 위해 `slip1` 값 재선정, 정지 중 옆 미끄럼을 막는 바퀴 제동 모델 추가를 A1(S005) 담당과 검토할 후보로 남긴다. 이 값들은 개발 물리 설정 결과이며 실물 성능이 아니다.
모두 운영 경로가 아니므로 격리된 개발 시뮬레이션에서만 실행한다.
