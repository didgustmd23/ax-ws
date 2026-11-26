# A4·B4 안전 패키지

ROS 패키지 루트는 `src/ax_safety/`다. A4 C++ 주행 게이트·보호 팔 서버와
B4 Python 위험 판정 노드를 제공한다. 기본 설정은 구동 비허용이다.

## 폴더와 역할

| 경로 (패키지 루트 기준) | 역할 |
|---|---|
| `src/a4/safety_gate.cpp` | Nav2 → A4 → A2 주행 명령 게이트 |
| `src/a4/protected_arm_server.cpp` | 보호 팔 Action·A3 권한 확인·A2 Action 중계 |
| `include/ax_safety/a4/` | A4 헤더 |
| `ax_safety/b4/risk_rules.py` | ROS 독립 위험 판정 규칙 |
| `ax_safety/b4/b4_risk_node.py` | 센서·위치·통로 관측 → SafetyState |
| `launch/safety.launch.py` | B4·A4 주행·A4 팔 노드 실행 |
| `config/safety_limits.yaml` | 개발 기본 설정 |
| `CMakeLists.txt`, `package.xml` | 빌드·설치·의존성 |

Python의 `ax_safety/b4/`와 C++의 `include/ax_safety/`는 모듈·헤더 namespace이므로 유지한다.
과거 ZIP의 `src/ax_safety/src/ax_safety/` 중첩은 제거했다. 과거 검증 기록의 경로는 당시 실행 경로다.

공통 시험 파일은 작업공간의 `tools/tests/`에 있다.
`test_a4_safety_gate.cpp`, `test_b4_risk_rules.py`, `mock_check_command_lease.py`를 사용한다.
마지막 파일은 시험용 서버이며 운영 launch에서 실행하지 않는다.

## Schema 1과 Schema 2

Schema 1은 `ActuationState.state`와 `drive_allowed`·`arm_allowed`를 함께
해석하던 기존 계약이다. Schema 2에서는 의미가 분리된다. `STOPPED`는 허가
여부가 아니라 최신 odom과 필요한 관절 피드백이 실제 저속 상태로 0.5초 이상
유지되었는지를 나타낸다. `drive_allowed`와 `arm_allowed`는 다음 동작의
시작 허가이며, B4 `SafetyState`가 무효·stale·STOP_REQUIRED이면 별도로 false가
된다. 따라서 배터리 때문에 허가가 차단되어도 실제 피드백이 멈추면 STOPPED는
독립적으로 보고할 수 있다.

`meta.valid`는 동작 허가를 뜻하지 않는다. Schema 2에서는 metadata, clock,
state freshness가 계약상 유효한지를 뜻하며, 소비자는 `robot_id`,
`source_session_id`, 증가하는 `revision`, 미래가 아닌 `timestamp`와 stale
조건을 함께 검증해야 한다. 보호 팔 서버는 Schema 1을 거부하고 Schema 2만
수락한다. A2·B2·D3 소비자는 `drive_allowed`/`arm_allowed`와 `STOPPED`를
서로 대체하지 않도록 전환이 필요하다. 상세 규칙은
[`INTERFACE_CONTRACT.md`](../../docs/design/INTERFACE_CONTRACT.md)를 따른다.

## 배터리 입력

B4는 종수님 시뮬레이터의 표준 `sensor_msgs/BatteryState`와 상대 토픽
`battery_state`를 사용한다. `/ax/robot_<n>` namespace에서 실행하면 로봇별
토픽이 분리된다. `battery_required=false`는 기존 B4 동작을 유지하고,
`true`일 때만 timestamp, `present`, 잔량 `[0, 1]`, 충전 상태와 stale를
검증한다. LOW는 22% 이하에서 경고만 남기며, CRITICAL과 LOW의 감속·정지
정책은 아직 활성화하지 않았다. 누락·무효·stale 입력은 기존 UNKNOWN 안전
차단 경로를 따른다. 원격 시뮬레이터가 현재 브랜치에 병합되지 않았다면
Gazebo 실제 연동 완료로 간주하지 않는다. 상세 계약은
[`BATTERY_CONTRACT_AND_PLAN.md`](../../docs/design/BATTERY_CONTRACT_AND_PLAN.md)를 따른다.

## 빌드·실행

Ubuntu 24.04 / ROS Jazzy 작업공간 루트에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ax_safety ax_control ax_task_executor
source install/setup.bash
ros2 launch ax_safety safety.launch.py robot_id:=robot_1
```

매개변수 파일(W2 보완):

- `params_file`(기본 `config/safety_limits.yaml`)은 로봇 공통값이다. 노드가 `/ax/<robot_id>` namespace에서 돌기 때문에
  키는 `/**/<노드>` 형식이다. bare 키(`a4_safety_gate:`)는 namespaced 노드에 적용되지 않는다(조회로 확인).
- `robot_params_file`(기본 없음)은 로봇별 주행 자세 `travel_joint_names`·`travel_joint_positions`와 팔 한계 `arm_*`를 준다.
  이 launch는 로봇 하나만 띄우므로 다른 로봇 값과 섞이지 않는다. 없으면 빈 기본값이라 주행 자세·팔 목표가 잠긴다.
- 빈 배열(`[]`)은 파일에 쓰지 않는다. 적용되면 C++ 노드가 `No parameter value set`으로 시작 즉시 종료한다.
- 적용 순서: `params_file` → `robot_params_file` → launch 인자(허가 플래그는 파일 값과 무관하게 launch 인자, 기본 false).
- 패키지에는 검증된 로봇별 파일이 없다. A1 `config/a1_validation.yaml`의 robot_1 `travel_pose_rad`는
  개발용 정의이며 주행 허용 근거로 승인된 값이 아니다. robot_2(팔 없음)의 주행 자세 처리는 계약 결정 전이다.

D-06 정지 판정(STOPPED)은 수신한 피드백 표본의 stamp(시뮬레이션은 sim time)로 잰다. wall tick·/clock 진행은
근거가 되지 않는다. 피드백·SafetyState 무수신 만료는 steady 시계로 따로 잰다(계약 2.3절).

- odom·joint_states를 표본마다 바로 판정해 채널별 연속 저속 구간을 만든다. 이동·비유한·필수 관절 누락·이름 중복
  표본은 그 자리에서 구간을 끊는다(다음 tick 전에 저속으로 덮여도 이동이 사라지지 않음).
- 현재 ROS 시각보다 미래인 stamp는 정지 근거로도, 신선한 표본으로도 쓰지 않는다(보류 없이 버림, 이동이면 구간 끊음).
  판정에 쓰는 구간 끝도 현재 시각을 넘지 않는다.
- 같은 채널 연속 표본 간격이 0.3 s를 넘으면 구간을 끊는다. 0.3 s는 기존 피드백 나이·무수신 한계(D-05 구동 상태)와
  같은 수치지만, 연속성 판정에 쓰는 것은 새 개발 규칙이며 실측 타당성·운영 승인은 미검증이다.
- 같은 stamp 재수신은 정규화한 내용(관절은 이름별 위치·속도, odom은 linear.x·angular.z와 frame 이름)을 원 값으로
  직접 비교한다. 같으면 무시하고 신선도도 늘리지 않는다(배열 순서만 다른 경우 포함). 다르면 구간을 끊는다.
  더 오래된 stamp는 최신 상태를 덮지 않고, 그 표본이 저속이 아니면 구간을 끊는다.
- 팔 정지 조건 역할(`stop_arm_presence`)은 정지 판정에만 쓴다. 기본 `unconfirmed`이면 STOPPED를 확인하지 않는다.
  - `present`: `stop_arm_joint_names`(비어 있지 않고 중복 없음)의 관절이 모두 있고 |속도| ≤ 0.01 rad/s여야 한다.
    바퀴 등 다른 관절 속도는 넣지 않는다. 두 채널(odom·필수 관절) 구간이 함께 덮는 길이가 stop_hold_s 이상이면 STOPPED.
  - `absent`: 팔 조건 해당 없음. odom 구간 길이로 판정한다.
  - 값은 launch `stop_evidence_file`로 모델 파일을 명시한다: `config/stop_evidence_ax1.yaml`(AX_1, 출처
    ax1_controllers.yaml arm_controller 5축), `config/stop_evidence_ax2.yaml`(AX_2, 팔 없음). robot_id로 추측하지 않는다.
  - 역할 설정은 주행 자세 검사·drive_allowed·arm_allowed·운영 허가를 열지 않는다. robot_2 주행 허가 계약은 미해결이다.
- /clock 0·역행(리셋, 채널 상태 전체 초기화)·D-09 정체(3 s 벽시계 미진행)는 시간 무효다.
  이미 STOPPED여도 UNKNOWN·valid=false로 바꾸고 근거를 버리며, block_reasons에 계약 D-12의 `TIME_INVALID`를 넣는다.
  `meta.validity_reason`은 자유 문자열 필드이며 여기에는 `CLOCK_INVALID`를 쓴다(ActuationState용 목록은 계약에 없음).
  RTF 범위 판정과 /ax/fleet/time_validity 소비는 감독기(미구현) 몫이라 하지 않는다.
- STOPPED는 주행 허용(`drive_allowed`)과 별개이며 주행 허가나 위험 해제의 근거를 대신하지 않는다.
- 표본 사이 실제 움직임은 관측하지 못한다. 이 판정은 실제 물리 정지 검증이 아니다.
- 진단(`stop_diagnostics`, 기본 false): 켜면 노드 로그에 `A4_STOP_DIAG {json}` 한 줄을 남긴다.
  STOPPED 진입·이탈 때 판정 근거(sim now, 채널별 구간 start/end, 유지 길이, freshness·SafetyState·시계 유효성,
  역할·joint 사용 여부)와, 1 s(벽시계)마다 채널별 수용·중복·상충·늦음·거부(시계·stamp·미래) 누적 수를 쓴다.
  판정·발행 내용은 바꾸지 않으며 공개 토픽이 아니다. 무수신 나이(age_*)는 진단 시점에 따로 읽은 값이다.
- W3 A4 진단은 D2 `/ax/fleet/tasks`, D3 `/ax/fleet/corridor/state`, B3
  `/ax/<robot>/perception/corridor_observation`을 구독해 `valid`, 출처,
  session/revision 순서와 상태 freshness를 별도로 표시한다. `A4_STOP_DIAG`의
  `gate_block_detail`, `d2_tasks`, `d3_corridor`, `b3_corridor_observation`은 실제 입력별
  차단 원인을 구분하기 위한 진단 필드이며, 공통 `ActuationState` 차단 사유나 주행 허가를
  임의로 바꾸지 않는다. D2/D3 허가 정책과 AX_2 무관절 주행 자세 계약이 확정되기 전까지
  게이트는 기존 보수적인 parameter 기본값(false)을 유지한다.

설치·연결 절차는 [W1 안내](../../docs/guides/W1_A4_B4_SETUP.txt)를 따른다.
A2 시뮬레이션에서 함께 시작하려면 `simulation.launch.py safety:=true`를 사용한다.
시뮬레이션이나 실제 입력 없이 기본 잠금 상태인 것은 정상이다.

## 검사

```bash
python3 -m unittest discover -s tools/tests -p test_b4_risk_rules.py
python3 -m unittest tools/tests/test_b4_battery_rules.py
mkdir -p /tmp/ax_w1_test
g++ -std=c++17 -Wall -Wextra -Wpedantic -I src/ax_safety/include \
  tools/tests/test_a4_safety_gate.cpp -o /tmp/ax_w1_test/test_a4
/tmp/ax_w1_test/test_a4
python3 -m unittest tools/tests/test_safety_launch_config.py
# 실ROS(가짜 /clock·피드백, domain 90~99 자동 예약, 작업공간 install 사용). 실제 물리 정지 검증이 아니다.
python3 -m unittest tools/tests/test_a4_b4_sim_clock_params_ros.py tools/tests/test_a4_stop_confirmation_ros.py
```

[W1 기록](../../docs/evidence/w1_a4_b4/)에는 당시 빌드·모의 상태·A3 개발 정책·Gazebo 검증 범위가 있다.
경로 정리는 안전 동작 결함 수정이나 운영 승인과 별개다. 실제 입력 유효성, 권한·기한,
최종 속도 제한·취소·중복 실행·실제 정지는 각각 계약에 맞게 검증해야 한다.
