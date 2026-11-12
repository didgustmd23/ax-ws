# A3 첫 과제: 모의 Action 결과를 Gazebo에서 보기

이 예제는 실제 적재 확인이 아니다. 모의 서버가 반환한 결과를 충돌 없는 정적
표시등으로 보여준다. EMPTY=빨강, STOWED=초록, UNKNOWN=노랑이다.
표시등 위치는 월드 좌표 (0, -3, 2) m이며 Entity Tree 이름은
`a3_mock_indicator_EMPTY`처럼 표시된다. 기존 로봇·월드·제어 코드는 변경하지 않는다.
시험 표시등은 Gazebo 종료 시 사라진다. 노드만 종료하면 마지막 표시등은 남는다.

## 준비

프로젝트 루트에서 기존 설명·제어 패키지를 Ubuntu에 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /home/freem/ax_main_logs build \
  --base-paths src --build-base /home/freem/ax_main_build \
  --install-base /home/freem/ax_main_install \
  --packages-up-to ax_description ax_interfaces ax_control ax_task_executor
```

## 실행 (Ubuntu 터미널 3개)

기존 A3 모의 서버와 Gazebo를 먼저 종료한다. 각 터미널에서 프로젝트 루트로
이동하고 아래 환경을 설정한다. Gazebo와 표시 노드의 GZ_PARTITION은 같아야 한다.
위 빌드에는 A3 패키지도 포함된다. 빌드한 설치 환경을 source하면 아래 `ros2 run` 명령을 사용할 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/freem/ax_main_install/setup.bash
export GZ_PARTITION=a3_first_demo
```

터미널 1: 기존 창고와 과제 로봇 실행.

```bash
ros2 launch ax_control simulation.launch.py \
  world:=planned/ax_warehouse.world robots:=2 sensors:=false
```

터미널 2: 모의 결과 표시 노드 실행.

```bash
ros2 run ax_task_executor gazebo_mock_display
```

터미널 3: A3 모의 서버 실행.

```bash
ros2 run ax_task_executor mock_executor \
  --ros-args -p mock_load:=EMPTY
```

터미널 2와 3은 준비 후 계속 실행해 둔다.

## 요청 및 확인 (추가 Ubuntu 터미널)

같은 ROS 환경을 source한 후 요청한다.

```bash
ros2 action send_goal /ax/robot_2/executor/execute_local_task \
  ax_interfaces/action/ExecuteLocalTask \
  "{task_kind: 5, object_id: 'test_box', execution: {execution_id: 'visual-empty-001', operation: 'CONFIRM_LOAD'}}" \
  --feedback
```

응답은 `outcome: 2`, `load_state: 2`, `MOCK_EMPTY`, `ABORTED`가 정상이다.
표시 노드 로그에 `MOCK result displayed: EMPTY`가 나오면 Gazebo Entity Tree에서
`a3_mock_indicator_EMPTY`를 찾아 더블클릭해 해당 위치로 시야를 이동한다.
Gazebo 버전에 따라 우클릭 메뉴의 Move To 기능을 사용할 수 있다.
빨간 표시등은 빈 적재함이라는 **시험 응답**을 뜻한다. 실제 적재함이 비었음을
관측했다는 의미가 아니다. Gazebo에서 상자를 옮겨도 이 모의 결과는 바뀌지 않는다.

다른 결과는 서버의 `mock_load`를 `STOWED` 또는 `UNKNOWN`으로 변경해 재시작한 뒤
새 execution_id로 요청한다. 실제 관측·물리 작업 취소·운영 lease 검증은 이 모의 구현 범위 밖이다.

## W1: 기다리기·취소·동시 요청

실제 Action 실행은 `execute_action`에서 처리하며 기본 `mock_delay_s=0.5`초 동안
단조 시계로 기다린다. 대기 중에도 ROS 콜백을 처리하므로 상태 재발행과 취소가 동작한다.
`-p mock_delay_s:=5.0`으로 관찰 시간을 늘릴 수 있다(허용 0~30초).

- 한 로봇에서 다른 실행 ID의 작업이 진행 중이면 새 작업은 거절한다. 단 이미 끝난 실행 ID의
  재요청은 다른 작업 처리 중에도 받아 저장 결과만 돌려준다.
- 같은 실행 ID·동일 요청(재요청 Goal)은 기존 처리를 기다린 뒤 저장된 결과만 재사용한다.
  재요청 Goal은 상태 Topic(단계·revision)과 결과 표시를 다시 발행하지 않는다. 그래서 상태가
  `MOCK_VALIDATING`으로 되돌아가거나, 다른 작업 처리 중에 이전 작업으로 바뀌지 않는다.
- 취소는 Goal 단위다. 실제로 작업을 처리 중인 원래 Goal만 취소할 수 있고, 재요청 Goal의 취소
  요청은 거절한다(원래 작업은 계속된다). 원래 Goal을 취소하면 Action 상태와 결과가 모두 CANCELED다.
- 취소 시 `MOCK_CANCELING` → `MOCK_CANCELED`로 종료하고 CANCELED 결과를 SQLite에 저장한다.
- 취소된 실행 ID의 재요청은 취소 결과를 반환한다. 다시 시험하려면 새 ID가 필요하다.
- 취소 요청 중인 Action Goal은 CANCELED로 끝난다. 이미 완료된 취소 결과를 재조회하는
  새 Goal은 Action ABORTED와 ResultStatus.CANCELED를 반환한다.
- 완료 결과가 이미 저장됐으면 이를 뒤늦은 취소로 바꾸지 않는다.

현재는 외부 로봇 구동이 없는 모의 작업이라 취소 확인이 가능하다. C4·A4의 물리 정지나
안전한 해제까지 구현한 것은 아니다. 공통 Action 타입은 변경하지 않았다.
설치된 ros2 CLI에는 `--cancel-after`가 없으므로 실제 취소 검증은 테스트의
ActionClient.cancel_goal_async 경로로 수행한다.

## 중복 요청과 재시작

수락 전에 SQLite 원장에 로봇 ID·실행 ID·요청 내용·최초 `mock_load`를 저장한다.
같은 로봇의 같은 실행 ID와 같은 요청은 거절하지 않고 기존 실행 기록을 사용한다.
완료 결과가 있으면 그대로 반환하며, 수락 후 결과 저장 전에 재시작한 경우 최초 시험 입력으로
모의 결과를 완성한다. 이 복구 방식은 물리 명령이 없는 모의 서버에만 적용한다.
동일 ID로 물체·배정·작업 인자 등을 바꾸면 거절한다. 계약상 재전송 시 달라질 수 있는
`execution.task_revision`은 비교에서 제외한다. 다른 결과를 시험하려면 새 실행 ID를 사용한다.

기본 원장은 로봇별 보존 경로 `${ROS_HOME:-~/.ros}/ax_task_executor/<robot_id>/executions.sqlite3`다(시작 로그에 경로 출력).
**기록은 계속 남는다.** 재시작해도 같은 기본 경로를 쓰므로 이전에 수락한 실행·저장 결과가 그대로 유지된다.
그래서 예전에 쓴 `execution_id`를 다시 보내면 새로 처리하지 않고 저장된 결과를 돌려준다(요청 내용이 다르면 거절).
시험마다 새 `execution_id`를 쓰고, 기록 없이 새로 시작하려면
`--ros-args -p execution_db:=/절대/경로/executions.sqlite3`로 다른 경로를 지정한다.
원장 안 기록은 로봇 ID별로도 분리된다. 다른 경로를 쓰면 이전 기록은 그 원장에서 조회되지 않는다. 원장을 쓰지 못하면 요청을 거절하고, 결과 저장 실패 시 성공을 반환하지 않는다.
운영용 배정 검증·lease·물리 작업 복구는 별도 구현 대상이다.

## 로봇별 RobotTaskStatus 발행

각 모의 서버는 선택한 `robot_id`의 계약 Topic에 `RobotTaskStatus`를 발행한다.

| 서버 | 상태 Topic |
|---|---|
| `robot_id:=robot_1` | `/ax/robot_1/executor/status` |
| `robot_id:=robot_2` (기본) | `/ax/robot_2/executor/status` |
| `robot_id:=robot_3`·`robot_4` | `/ax/robot_3/executor/status`·`/ax/robot_4/executor/status` |

Action은 계약 경로 `/ax/<robot_id>/executor/execute_local_task`다(이전 `/a3_mock/<robot_id>/execute_local_task`에서 변경).
상태는 계속 `valid=false`·`MOCK_ONLY`이고 결과 오류 코드는 `MOCK_*`이므로, 같은 경로라도
소비자는 모의 결과임을 구분할 수 있다. 운영 A3와 같은 로봇에 동시에 띄우지 않는다.

상태 변경 시 즉시 발행하고, 단조 시계 타이머로 1초마다 재발행한다.
QoS는 reliable·transient_local·keep_last 1이다. 늦게 구독해도 마지막 상태를 받을 수 있다.
각 로봇 서버와 ROS 환경을 실행한 후 아래처럼 확인한다.

```bash
ros2 topic echo /ax/robot_1/executor/status ax_interfaces/msg/RobotTaskStatus \
  --qos-reliability reliable --qos-durability transient_local
# 다른 터미널: 로봇 2
ros2 topic echo /ax/robot_2/executor/status ax_interfaces/msg/RobotTaskStatus \
  --qos-reliability reliable --qos-durability transient_local
```

`local_phase`는 시작 시 `MOCK_IDLE`, 실행 검사 시 `MOCK_VALIDATING`, 확인 중 `MOCK_CONFIRM_LOAD`,
결과에 따라 `MOCK_SUCCEEDED`·`MOCK_FAILED`·`MOCK_UNCERTAIN`이며,
취소는 `MOCK_CANCELING` → `MOCK_CANCELED`다.
`execution`에는 처리 중이거나 마지막으로 처리한 Goal의 식별 정보가 들어간다.
`last_result`는 마지막 처리 결과이며, 다음 작업 처리 중에는 이전 결과를 유지한다.
첫 결과 전에는 기본값(UNSPECIFIED)이다. `paused=false`는 이 모의 서버에 일시 정지
기능이 없다는 뜻이며 실제 로봇의 정지 여부가 아니다. 취소와 일시 정지는 서로 다르다.

`meta.schema_version=1`, `source_id=a3_mock_executor_<robot_id>`이며
`source_session_id`는 프로세스 시작마다 새로 생성한다. `revision`은 상태 변경 때만
증가하고 주기 재발행에서는 유지한다. 기본 `use_sim_time=true`로 ROS 시각을 기록하며,
`/clock`이 없으면 stamp는 0일 수 있다. Gazebo를 일시 정지해도 주기 발행은 계속된다.

**운영 상태로 사용하지 않는다.** 모든 메시지는 `meta.valid=false`,
`meta.validity_reason=MOCK_ONLY`로 표시한다. `last_result`가 SUCCEEDED여도
시험 결과일 뿐이다. 운영 A3 발행기와 같은 로봇·Topic에 동시에 띄우지 않는다.
재시작 시 상태는 새로운 세션의 `MOCK_IDLE`부터 시작하며, SQLite의 이전 결과는
해당 실행 ID로 재요청할 때 조회·반영한다. 거절된 요청은 현재 실행 상태를 바꾸지 않는다.

표시가 없으면 Gazebo가 재생 중인지, world 이름(기본 ax_test)과 GZ_PARTITION이 같은지
확인한다. 서비스 연결 실패 시 표시 노드는 경고를 출력하고 재시도한다.

## 로봇 1에서도 실행하기

기본 `robot_id`는 `robot_2`이므로 기존 로봇 2 명령은 그대로 사용할 수 있다.
로봇 1은 위와 같은 빌드·환경 설정 후 별도 터미널 두 개에서 실행한다.
두 로봇을 함께 시험할 때도 Gazebo는 한 번만 실행한다.

```bash
# 로봇 1 표시 노드 (별도 터미널)
ros2 run ax_task_executor gazebo_mock_display --ros-args \
  -r __node:=a3_gazebo_mock_display_robot_1 -p robot_id:=robot_1
```

```bash
# 로봇 1 모의 서버 (별도 터미널)
ros2 run ax_task_executor mock_executor --ros-args \
  -r __node:=a3_mock_executor_robot_1 -p robot_id:=robot_1 -p mock_load:=EMPTY
```

```bash
# 요청 (추가 터미널)
ros2 action send_goal /ax/robot_1/executor/execute_local_task \
  ax_interfaces/action/ExecuteLocalTask \
  "{task_kind: 5, object_id: 'test_box', execution: {execution_id: 'robot1-empty-001', operation: 'CONFIRM_LOAD'}}" \
  --feedback
```

로봇 1 결과는 `/a3_mock/robot_1/result_display`로 전달하며,
표시등은 `a3_mock_indicator_robot_1_EMPTY`처럼 생성된다. 위치는 (-1, -3, 2) m다.
로봇 3·4도 같은 방식이다(`/a3_mock/robot_3/result_display`, `a3_mock_indicator_robot_3_*`).
표시 위치 x는 robot_3 −2 m, robot_4 1 m이며 0.6 m 상자가 겹치지 않는다.
로봇 2의 기존 결과 Topic·표시등 이름·위치는 유지된다. 각 표시 노드는 자기 로봇의
표시등만 제거하므로 두 결과가 서로 덮어쓰이지 않는다.
로봇 1도 STOWED·EMPTY·UNKNOWN 시험 입력만 사용하며 실제 로봇 구동·적재 관측은 하지 않는다.

## 확인한 범위

분리된 headless Gazebo 시험 월드에서 EMPTY Action 응답과
scene/info의 `a3_mock_indicator_EMPTY` 생성 확인. 실제 GUI의 색상·시야는 수동 확인 대상이다.

로봇 1 추가 후 ROS 환경에서 `python3 tools/tests/test_a3_mock_robots.py`로 두 로봇의
Action 경로·결과 Topic과 STOWED·EMPTY·UNKNOWN 결과를 확인했다. Gazebo 서비스는
모의 호출로 대체하여 표시등 이름·위치·삭제 범위의 분리를 검사했다.
두 로봇의 Gazebo GUI 동시 표시는 별도 수동 확인 대상이다.

RobotTaskStatus 추가 검증은 같은 테스트 파일의
`test_status_late_subscriber_heartbeat_and_transitions`에 작성했다.
로봇별 늦은 구독·시계 정지 중 재발행·단계·결과·revision·세션 변경·저장 오류를 검사한다.
2026-11-09 WSL 자원 설정 조정 후 ROS 환경에서 상태 발행 검증을 포함한
4개 테스트가 모두 통과했다. 이후 현재 실행기 단계 머신을 연결하고 잘못된 전환·입력
차단 검증을 추가하여 6개, 이후 실제 Action 취소·진행 중 중복·다른 작업 차단 검증까지
추가하여 총 7개 테스트가 통과했다. 시뮬레이션 시계가 진행되지 않는 조건에서도
주기 발행과 늦은 구독을 확인했다. Gazebo GUI 동시 표시는 별도 확인 대상이다.
2026-11-12 Goal 단위 취소·재요청 비재발행·다른 작업 끼어들기·로봇 4대·계약 Action 경로·보존 원장(재시작 유지) 회귀 시험 6개를
추가했다(`MockExecutorRegressionTests`). 수정 전 코드에서는 이 중 5개가 실패했다. lease 서버는
`tools/tests/test_a3_lease_check.py`(15개, A2 `suction_tool_node.check_lease` 왕복 포함)로 확인했다.
Gazebo·실제 A2/A4 노드와의 실행 시험은 하지 않았다.

## CheckCommandLease 서버 (개발 정책)

`lease_check`는 계약 4.3·5.2절의 `/ax/<robot_id>/executor/lease/check_command`
(`CheckCommandLease`) 서버다. A2 `suction_tool_node`·A4가 명령 수락 전에 호출한다.
**실제 lease 판정은 없다.** 계약의 형식·권한·멱등·수락 기록 규칙만 구현하고,
lease 판정 자리는 개발 정책 파라미터가 대신한다.

```bash
# 기본: dev_deny(형식이 맞아도 LEASE_EXPIRED로 거절, 안전 쪽)
ros2 run ax_task_executor lease_check --ros-args -p robot_id:=robot_1
# 개발 시험: 형식·권한 검사를 통과하면 허용
ros2 run ax_task_executor lease_check --ros-args -p robot_id:=robot_1 -p policy:=dev_allow
```

| 파라미터 | 기본 | 뜻 |
|---|---|---|
| `robot_id` | `robot_2` | robot_1~robot_4. 요청의 `robot_id`가 다르면 거절 |
| `policy` | `dev_deny` | `dev_allow`(검사 통과 시 허용) 또는 `dev_deny` |
| `deny_reason` | `LEASE_EXPIRED` | `dev_deny`의 거절 사유. `LEASE_EXPIRED`·`LEASE_REVOKED`·`GOAL_MISMATCH`·`ASSIGNMENT_STALE`·`LEASE_RESYNC_UNCERTAIN` |
| `valid_for_s` | 2.0 | 허용 응답의 유효 시간(s). 0 초과 10 이하만 받는다(개발용 상한) |
| `lease_db` | 빈 값 → `${ROS_HOME:-~/.ros}/ax_task_executor/<robot_id>/lease_checks.sqlite3` | 확인 응답·수락 기록 SQLite(WAL 저널 + synchronous=FULL). 시작 로그에 경로 출력, WAL을 못 쓰는 파일 시스템이면 경고 |

정책과 관계없이 항상 하는 검사:
- `phase`·`command_source`·`authority`가 UNSPECIFIED이거나 `robot_id`가 다르면 거절한다.
- 권한별 필드: C 조작은 `execution_id`·`goal_id`(소문자 16진수 32자)·`lease_generation`>0, `local_task_kind`=0.
  A3 로컬은 `local_task_kind`=RETRACT_ARM, `goal_id` 빈 문자열, `lease_generation`=0, `SOURCE_A4_ARM`만.
- `request_id`·`execution_id`·`command_id`는 계약 문자 집합, `command_digest`는 SHA-256 소문자 16진수여야 한다.
- 형식·필드 오류는 모두 `AUTHORITY_INVALID`다(계약 D-13에서 요청 형식 오류를 포함하도록 정의).
- 멱등:
  - 같은 `request_id` 재전송은 기존 응답을 그대로 돌려준다(정책을 바꿔 재시작해도 같은 원장이면 같다).
  - 같은 `request_id`인데 내용이 다르면 `DUPLICATE_REQUEST`다.
  - 같은 `command_id`에 권한 필드·출처·digest가 다르면 `DUPLICATE_REQUEST`다.
  - 같은 `command_id`의 새 `request_id` ACCEPT는 기존 수락 기록 ID를 재사용하고 새 기록을 만들지 않는다.
  - `CONTINUE`는 그 `command_id`의 ACCEPT 수락 기록이 있을 때만 허용하며(없으면 `AUTHORITY_INVALID`) 새 기록을 만들지 않는다.
- 허용이면 수락 기록을 원장에 먼저 쓰고 응답한다. 기록에 실패하면 `A3_RECORD_FAILED`로 거절한다.
  이 코드는 계약 D-13의 `A3_RECORD_FAILED`다(요청자는 실행하지 않음).
- 허용 응답은 `reason_code=DEV_POLICY_ALLOW`이고 수락 기록 ID가 `devpolicy-`로 시작한다. 개발 정책 결과임을 구분할 수 있다.

lease 원장도 **계속 남는다.** 같은 `request_id`를 재사용하면 재시작·정책 변경 뒤에도 예전 응답이 돌아오고,
같은 `command_id`는 예전 수락 기록을 재사용한다. 시험마다 새 ID를 쓰고, 새로 시작하려면 `-p lease_db:=` 로 다른 경로를 지정한다.

**알려진 한계(응답 지연):** 단일 스레드 `lease_check`에 4개 클라이언트가 1000회 요청한 측정에서 10회 중 3회,
요청 1~2개가 0.8~2.5 s 늦게 서버에 도착했다(서버 처리는 0.03 s 이하, 측정 당시 loadavg 1~2).
A2의 lease 대기(0.5 s)를 넘기면 무응답 거절(안전 쪽)로 나타날 수 있다.
원인은 rclpy/rmw 서비스 처리 쪽으로 추정하지만 확정하지 못했다. 측정 자료는 `log/a3_executor_latency/20261112_210838`(작성 PC 기록, Git 제외)이다.
- 서버 언어와는 무관하다(C++ 서버도 같음). 서버 처리 시간이 길고 클라이언트가 여럿일 때 잦다.
- 원장을 WAL 저널(synchronous=FULL 유지)로 바꾼 뒤 처리 시간 p50이 22.8 → 6.1 ms로 줄었다.
  같은 시간대 비교에서 기한 초과 회차가 2/20 → 0/20이었다(표본이 작다).
- 클라이언트가 같은 `request_id`를 짧은 간격으로 재전송하면 안 된다. 재전송이 일어난 13회 중 12회에서
  서버가 같은 요청을 약 1만 6천 번 반복 수신해 다른 요청이 더 늦어졌다(rmw 중복 수신). 재전송이 먼저 응답받은 경우는 0건이었다.
  자료: `log/lease_latency_compare/20261112_215030/REPORT.md`·`REPORT_R.md`(작성 PC 기록, Git 제외).

구현하지 않은 것: lease 표·세대 발급·갱신·철회, 활성 `ManipulateObject` Goal과 `goal_id` 대조,
배정 유효성(`tA + H`)·위임 예산 허용 목록, 재동기화(`LEASE_RESYNC_UNCERTAIN` 판정), A3 로컬 작업의
"현재 실행 중 RETRACT_ARM" 대조, `valid_for_s`를 남은 lease 기한 이하로 줄이는 처리.
`dev_allow` 허용은 **운영 권한이 아니다.** 이 서버를 통과해도 실제 lease가 유효하다는 뜻이 아니다.

## 로컬 작업 실행기 한 프로세스 (local_task_executor)

`local_task_executor`는 한 노드 안에 `ExecuteLocalTask` Action 서버(모의 실행기와 같은 로직)와
`CheckCommandLease` 서비스(lease_check와 같은 개발 정책)를 함께 둔다(설계 11.1절).
lease 판정은 여전히 개발 정책(`dev_deny` 기본)이며 운영 lease 판정이 아니다.

```bash
ros2 run ax_task_executor local_task_executor --ros-args -p robot_id:=robot_1 -p policy:=dev_allow
```

- 구조
  - MultiThreadedExecutor(`threads`, 기본 4)가 콜백 그룹 3개를 돌린다.
    - lease: Reentrant 그룹. 요청 직렬 처리(계약 5.2절)는 콜백 그룹이 아니라 원장 잠금 + SQLite `BEGIN IMMEDIATE`가 보장한다.
    - Action: Reentrant 그룹(Goal 수락·취소·실행, 실행 대기 타이머).
    - 상태: MutuallyExclusive 그룹(1 Hz 상태 재발행).
  - 공유 상태 `ExecutorState`(현재 실행·진행 중 처리·취소 표시, 앞으로 C4 활성 Goal·lease 세대 자리)는 잠금 하나로 보호한다.
  - 잠금 안에서는 SQLite 쓰기·ROS 호출·await를 하지 않는다. 한 스레드가 두 잠금을 함께 잡지 않는다(순서는 `local_task_executor.py` 머리말).
  - lease 판정은 정책 객체 `DevPolicyAuthority`가 맡는다. 운영 판정은 같은 `decide(fields, state)` 인터페이스로 교체한다.
  - 원장은 `${ROS_HOME:-~/.ros}/ax_task_executor/<robot_id>/`의 `executions.sqlite3`·`lease_checks.sqlite3`(분리)다.
- 기존 실행 파일과의 관계
  - `mock_executor`·`lease_check`는 호환용으로 그대로 둔다.
  - **같은 로봇에 local_task_executor와 동시에 띄우지 않는다.**
  - local_task_executor는 시작 때 같은 lease 서비스·Action 이름을 가진 다른 노드를 `name_check_s`(기본 1 s) 동안 찾고, 있으면 시작을 거절한다.
  - 반대 방향(기존 실행 파일이 나중에 시작)은 감지하지 않는다.
- 비교용 파라미터 `lease_mode`
  - `reentrant`(기본)
  - `mutex`: 같은 노드의 MutuallyExclusive 그룹
  - `dedicated`: 전용 노드 + 단일 스레드 실행기
  - WAL 이전 비교에서 mutex 4/6, dedicated 2/8 회차에 0.8~2.5 s 지연이 났고, reentrant는 0/8이었다.
- 지연 측정(2026-11-12, `tools/tests/test_a3_local_task_executor.py`의 지연 시험 그대로)
  - 조건: Action Goal이 계속 도는 동안 lease 1000회(클라이언트 4개), 새 command_id ACCEPT, WAL+FULL 기록.
  - 기준(미리 고정): p99 < 0.1 s, max < 0.5 s, 무응답 0.

| 부하 | 대상 | 기준 통과 | p50 | p99 최대 | max 최대 | loadavg |
|---|---|---|---|---|---|---|
| 낮음 | local_task_executor | 10/10 | 14.8 ms | 32 ms | 54 ms | 1.0~2.0 |
| 낮음 | 기존 lease_check 단독(대조) | 10/10 | 6.5 ms | 20 ms | 39 ms | 1.4~2.0 |
| 높음(자기 CPU 루프 2개) | local_task_executor | 10/10 | 14.6 ms | 34 ms | 48 ms | 1.7~3.6 |
| 높음 | 기존 lease_check 단독(대조) | 9/10 | 6.5 ms | 20 ms | 1.06 s | 1.9~3.5 |

  - 같은 시간대 번갈아 측정했다. 자료: `log/a3_local_executor_latency/20261112_233223`(작성 PC 기록, Git 제외).
  - 표본이 작아 "통합이 지연을 없앤다"고 단정하지 않는다. local_task_executor는 Action 처리와 함께 돌아 p50이 두 배쯤 길다.

## A4·A2 개발자용 연결 안내

A4(`ProtectedArmTrajectory`)·A2(`ToolCommand`)를 개발할 때 상대 A3로 `lease_check`를 띄운다.
**개발 정책 서버이므로 응답은 운영 lease 판정이 아니다.** 허용을 받아도 실제 lease·세대·배정이 유효하다는 뜻은 아니다.

```bash
ros2 run ax_task_executor lease_check --ros-args -p robot_id:=robot_1 -p policy:=dev_allow   # 허용 경로 시험
ros2 run ax_task_executor lease_check --ros-args -p robot_id:=robot_1                        # 기본 dev_deny: 거절 경로 시험
```

서비스는 `/ax/<robot_id>/executor/lease/check_command`(`ax_interfaces/srv/CheckCommandLease`)다. 요청 필드 예:

| 필드 | C 조작 (`AUTH_C_MANIPULATION`, A2·A4) | A3 로컬 후퇴 (`AUTH_A3_LOCAL_TASK`, A4만) |
|---|---|---|
| `request_id` | 확인 요청마다 새 ID. 재전송은 같은 ID | 같음 |
| `robot_id` | 서버의 `robot_id`와 같아야 함 | 같음 |
| `execution_id` | 받은 `ManipulateObject` 실행 ID 그대로 | A3가 보낸 `ExecuteLocalTask` 실행 ID |
| `goal_id` | 부모 `ManipulateObject` Goal UUID(소문자 16진수 32자) 그대로 | 빈 문자열 |
| `lease_generation` | 받은 값 그대로(> 0) | 0 |
| `local_task_kind` | 0 | `ExecuteLocalTask.Goal.RETRACT_ARM` |
| `phase` | `ACCEPT`(새 명령) 또는 `CONTINUE`(진행 중) | 같음 |
| `command_source` | `SOURCE_A2_TOOL` 또는 `SOURCE_A4_ARM` | `SOURCE_A4_ARM`만 |
| `command_id` | 물리 명령 ID(`ToolCommand.request_id`·`ProtectedArmTrajectory.request_id`) | 같음 |
| `command_digest` | 명령 인자의 SHA-256 소문자 16진수 64자 | 같음 |

- digest를 만드는 방식은 클라이언트가 정한다. 같은 `command_id`에는 항상 같은 값을 보내야 한다.
  예: A2 `suction_tool_node.command_digest`는 `robot_id|execution_id|goal_id|object_id|lease_generation|command`의 SHA-256이다.
- 응답 해석
  - `allowed=false`, 무응답, 기한 경과 중 하나라도 해당하면 명령을 실행하지 않는다.
  - `reason_code`는 거절 사유다(`AUTHORITY_INVALID`·`DUPLICATE_REQUEST`·`LEASE_EXPIRED`·`A3_RECORD_FAILED` 등).
    개발 정책 허용은 `DEV_POLICY_ALLOW`이고, 운영 A3 허용은 빈 문자열이다(계약 5.2절).
  - `accept_record_id`는 ACCEPT 허용일 때만 있다. 개발 정책은 `devpolicy-`로 시작한다.
  - `valid_for_s`: 허용 기한은 **요청을 처음 보낸 시각 `t_first`(CLOCK_BOOTTIME) + `valid_for_s`**다.
    응답을 받은 시각부터 세지 않는다. 기한이 지나면 새 `request_id`로 다시 확인한다.
  - `CONTINUE`는 그 `command_id`를 ACCEPT로 허용받은 뒤에만 허용된다.
- 기록은 `${ROS_HOME:-~/.ros}/ax_task_executor/<robot_id>/lease_checks.sqlite3`에 계속 남는다.
  예전 `request_id`·`command_id`를 다시 쓰면 예전 응답·수락 기록이 돌아온다.
  시험마다 새 ID를 쓰고, 새로 시작하려면 `-p lease_db:=<다른 경로>`를 지정한다.

## 모의 통과와 운영 완료의 구분 · 다음 작업 (미구현·예정)

이 패키지의 모의 실행기와 lease 서버를 통과해도 운영 기능이 완료된 것은 아니다.
현재 모의 실행기는 `CONFIRM_LOAD`만 받고, 결과는 시험 입력(`mock_load`)이다.

| 운영 작업 종류 | 현재 | 연결할 하위 대상(예정) |
|---|---|---|
| `LOCK_HANDOFF`·`UNLOCK_HANDOFF` | 미구현(Goal 거절) | 인계 잠금 상태(`transfer_state`)·A4 차단 조건, D4 heartbeat |
| `PICK_AND_STOW`·`UNLOAD_AND_PLACE` | 미구현(Goal 거절) | C4 `ManipulateObject`(`/ax/<robot_id>/manipulation/execute`) + lease 요청·발급·철회 Topic, 적재 확인 |
| `RETRACT_ARM` | 미구현(Goal 거절) | A4 `ProtectedArmTrajectory`(`/ax/<robot_id>/arm/execute_trajectory`, `AUTH_A3_LOCAL_TASK`), `joint_states`로 완료 확인 |
| `CONFIRM_LOAD` | 모의(시험 입력) | 실제 적재 판정(`load/state`) |
| 배정 확인 | 미구현 | D4 `assignment_check/request·response` heartbeat |

C4·A4·D4의 모의 상대(가짜 서버)도 아직 없다. 위 연결은 담당 계약 확인 뒤 진행한다.

