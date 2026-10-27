# AX 공통 인터페이스 계약 초안 (K0)

> 상태: **초안**. 생산자·소비자 승인 전이며 구현·동작을 뜻하지 않는다.
> 기준 설계는 [통합 상세설계](DETAILED_DESIGN.md) 6·7절, 일정은 [팀 공통 일정](../management/TEAM_SCHEDULE.md)의 K0(W0~W1, 승인 11-08)이다.
> 타입 정의는 `src/ax_interfaces/`의 `.msg`·`.srv`·`.action`이며, 이 문서는 이름·방향·QoS·최신성·소유 작업을 정한다.
> 구성도: [인터페이스 관계](../assets/diagrams/ax-interfaces.html)(4절 묶음 개요) · [lease·명령 확인 순서](../assets/diagrams/ax-lease.html)(5.2절). 구성도는 이름·방향만 보이며 값의 기준은 이 문서다.

## 1. 범위와 승인 절차

- 이 문서는 기능 A~E 사이의 공유 Topic·Service·Action과 공통 타입을 정한다. 기능 내부 노드 사이의 통신과 Gazebo·Nav2·ros2_control 표준 인터페이스는 해당 담당이 정한다. 한 기능 안이라도 여러 작업 담당(예: D1·D3·D4)이 나눠 구현하는 Service는 이 문서에 넣는다.
- 각 인터페이스는 **생산자와 소비자가 함께 승인**한다(본설계 11절). 진행 방식으로 7절의 "이의 없으면 승인"을 제안하며, 이를 공동 승인으로 인정할지는 7.3절대로 팀이 합의한다. 승인 전에는 모의 상대와 계약 시험으로 개발한다.
- 승인 후 의미가 바뀌면 `StateMeta`를 가진 상태 메시지와 `AgentProposal`·`AgentPlan`은 `schema_version`을 올린다. 이 필드가 없는 타입(lease·배정 확인·사건·Goal·Service 등)은 `ax_interfaces` 패키지 버전을 올리고 ROS 2 타입 해시로 불일치를 검사한다(D-14). 공개한 enum 숫자는 다른 의미로 재사용하지 않는다.
- 아래 수치(주기·만료 시간·lease 상한)는 개발용 초깃값이며 P1 계약 시험과 P2 실측으로 확정한다.

## 2. 공통 규칙

### 2.1 식별자

| 식별자 | 형식 초안 | 발급 | 비고 |
|---|---|---|---|
| `team_id` | `team_<n>` | D | 협업 그룹 |
| `robot_id` | `robot_<n>` | 구성 | 실제 로봇. 역할(`MANIPULATOR`·`TRANSPORTER`)은 별도 필드 |
| `task_id` | `task-<UUIDv4>` | D | 그룹 작업 |
| `assignment_id` / `assignment_generation` | `asg-<UUIDv4>` / 증가 정수 | D | 이전 배정 결과 차단(TC-03) |
| `transfer_id` | `xfer-<UUIDv4>` | D | 적재·하역 인계 |
| `root_execution_id` | `exec-<UUIDv4>` | D | 상위 작업 실행 |
| `execution_id` | `derive(root_execution_id, step_key, operation, attempt_index)` | D 규칙 | D-03. 규칙 버전은 `ExecutionRef.derive_rule_version`에 기록 |
| `step_key` | 계획에 저장한 문자열 | D | 반복문 순번 사용 금지 |
| `receipt_event_id`·`object_id`·`location_id` | `rcpt-…`·`obj-…`·`loc-…` | D·구성 | 등록 지점은 지도 변경 시에도 ID 유지 |
| `hold_id`·`recovery_context_id`·`budget_grant_id` | `hold-…`·`rctx-…`·`bgt-…` | D | 원장 영속 대상 |
| `failure_event_id` | `fail-<UUIDv4>` | 사건 소유자(A·B·C·D·감독기) | D 무응답 중에도 로컬 기록 후 같은 ID로 재전송(멱등) |
| `sender_agent_id` | `agent_central`·`agent_<robot_id>`(예: `agent_robot_1`) | Agent 어댑터 | LLM이 채우지 않음. Topic 경로에 쓰므로 소문자·숫자·`_`만 사용 |
| `request_id`·`proposal_id`·`plan_id` | `<prefix>-<UUIDv4>` | 요청자 | 재전송은 같은 ID로 멱등 처리 |
| `goal_id` | Action goal UUID의 소문자 16진수 32자 | Action 클라이언트(ROS 2) | lease 요청·응답·철회와 명령 확인을 Goal 인스턴스에 묶는다(5.2절) |

메시지 필드의 문자열 ID는 영문·숫자·`_ - . : @`만 쓴다. ROS Topic·노드·namespace 이름에 들어가는 ID(`robot_id`, `team_id`, `sender_agent_id`, 조건 소유 영역)는 소문자·숫자·`_`만 쓴다. ID를 파싱해 의미를 꺼내지 않는다.

### 2.2 단위·좌표·시계

- 거리 m, 속도 m/s, 각도 rad, 각속도 rad/s, 질량 kg, 시간 s.
- 공유 위치는 `map` 기준 `geometry_msgs/PoseStamped`. 로봇 프레임은 `robot_<n>/odom`, `robot_<n>/base_link`. 공구 목표는 `robot_<n>/epick_tcp`, 공구 ToF는 `robot_<n>/tool_tof_link`, 운송 로봇 적재함 기준은 `robot_<n>/cargo_link`(A1 모델이 제공). 로봇 안의 모든 frame은 `robot_<n>/` 접두사를 붙인다.
- TF는 공용 `/tf`·`/tf_static`을 쓰고 로봇별 frame 접두사로 구분한다(로봇별 TF namespace를 쓰지 않는다). 발행 소유: `map→robot_<n>/odom`은 B2, `robot_<n>/odom→robot_<n>/base_link`는 A2(차체 제어기 odometry), 로봇 관절·고정 frame(`epick_tcp`·`tool_tof_link`·`cargo_link` 등)은 A2의 robot_state_publisher(A1 모델 기준). 다른 노드는 이 변환을 발행하지 않는다.
- `StateMeta.header.stamp`는 ROS sim time(`use_sim_time=true`)이다. 통신 만료·lease·예산은 수신 측이 `CLOCK_BOOTTIME`으로 직접 잰다. 다른 프로세스의 단조 시계 값을 메시지로 비교하지 않는다(본설계 6·7.6절).
- 상태 Topic의 발행 주기는 생산자의 단조 시계로 잰다. 낮은 RTF에서도 무수신 만료를 잘못 일으키지 않게 하기 위해서다. 표본 나이는 sim time 기준이다.

### 2.3 상태 메시지 공통 머리 `StateMeta`

모든 상태 Topic 메시지는 `StateMeta`를 첫 필드로 가진다. 목록 메시지(`GroupTaskList` 등)는 목록 전체에 하나를 둔다. 안쪽 항목이 자기 `StateMeta`(또는 `RobotSummary`의 영역별 meta)를 가지면 **안쪽 meta가 우선**하고, 없는 항목만 바깥 meta를 따른다. 바깥 meta가 무효면 안쪽과 관계없이 전체를 무효로 본다.

| 필드 | 규칙 |
|---|---|
| `header.stamp` | 관측·판정 시각(sim time). 소비자는 표본 나이로 최신성을 검사 |
| `schema_version` | 메시지 계약 버전 |
| `source_id` | 발행 노드 식별자 |
| `source_session_id` | 생산자 프로세스가 시작할 때마다 새로 발급. 세션이 바뀌면 소비자는 revision 비교를 새로 시작 |
| `revision` | 같은 세션에서 상태 변경마다 1 증가. 0은 발행 전 |
| `valid` / `validity_reason` | 생산자의 유효성 판단. `false`는 정상·성공으로 해석하지 않음 |

소비자 규칙:

- (`source_session_id`, `revision`)으로 비교한다. 같은 세션에서 revision이 줄면 버린다. 같은 revision이 다시 오면 내용은 갱신하지 않고 **`header.stamp`와 수신 시각을 갱신**한다(주기 재발행). 생산자는 재발행할 때마다 상태를 다시 판정하고 그 시각으로 stamp를 찍는다.
- **표본 나이(sim time)와 무수신 시간(CLOCK_BOOTTIME) 둘 다** 검사한다. 만료된 상태는 `UNKNOWN`으로 취급한다. 모든 STATE Topic은 "변경 시"에 더해 주기 재발행과 만료 시간을 가진다(D-05).
- 세션끼리는 선후를 비교하지 않는다. 새 세션의 첫 메시지를 받은 뒤 이전 세션 메시지가 오면 버린다.
- `valid=false`, `revision=0`, 받은 적 없는 상태는 `UNKNOWN`이다. 본설계 7.2·7.3절이 숫자를 정한 `SafetyState.CLEAR=0`, `CorridorState.FREE=0`, `CorridorObservation.OUTSIDE=0`은 이 규칙으로만 보호된다. 통로 관측은 `position_valid=true`일 때만 해제 근거가 된다.
- enum 값이 `UNKNOWN`·`UNSPECIFIED`(0)이면 정상·성공·허용으로 해석하지 않는다. Goal의 `UNSPECIFIED`는 거절한다.

### 2.4 QoS 프로필

| 프로필 | 설정 | 용도 |
|---|---|---|
| `STATE` | reliable · transient_local · keep_last 1 | 최신 상태가 중요한 Topic. 늦게 접속한 노드도 마지막 값을 받음. 대상이 여러 개면 목록 메시지 하나로 보낸다 |
| `EVENT` | reliable · volatile · keep_last 50 | 관측 사건·제안·결정·lease 메시지 |
| `SENSOR` | best_effort · volatile · keep_last 5 | 센서 데이터(흡착 공구 ToF 등 표준 타입) |

### 2.5 결과와 멱등성

- Action 결과는 `ResultStatus.outcome`으로 `SUCCEEDED / FAILED / CANCELED / TIMED_OUT / UNCERTAIN`을 구분한다. `UNCERTAIN`과 채우지 않은 `UNSPECIFIED`는 성공으로 처리하지 않고 본설계 9.6절 수명주기로 보낸다.
- Service 응답의 수락(`accepted`·`granted`)은 실제 동작 완료가 아니다.
- 같은 `request_id`·`execution_id` 재전송은 새 동작을 만들지 않고 기존 결과를 돌려준다. Action goal UUID는 통신 식별자일 뿐이며 `execution_id`와 연결해 기록한다. 단 lease와 명령 확인은 Goal UUID(`goal_id`)에 묶는다(5.2절).

## 3. 공통 타입

| 타입 | 내용 | 주 사용처 |
|---|---|---|
| `StateMeta` | 상태 출처·세션·버전·유효성 | 모든 상태 Topic |
| `ExecutionRef` | 팀·작업·배정·인계·실행 ID 묶음 | Action Goal, 실행 상태 |
| `BudgetGrant` | D가 위임한 재시도 예산과 허용 하위 ID | ExecuteLocalTask·ManipulateObject Goal |
| `AttemptUsage` | 물리 시도 1회 사용 기록(시작·종료·미확정) | Action Feedback·Result |
| `ResultStatus` | 결과 4종 + 미확정, 오류 코드 | Action Result, 로컬 실행 상태 |
| 목록 메시지 | `GroupTaskList`·`CorridorStateList`·`HoldStatusList`·`AgentReceiveStateList`·`CandidateSetList`·`CorridorObservationList`·`FailureEventList` | 대상이 여러 개인 STATE Topic |
| `PlanCondition`·`RobotStopPose` | 계획 전제조건(소유 영역·판정식), 지점의 역할별 정차 자세 | `PlanStep`, `RegisteredLocation` |

## 4. 인터페이스 목록

`<n>`은 로봇 번호다. "작업"은 [팀 공통 일정](../management/TEAM_SCHEDULE.md)의 작업 ID로, 생산자 쪽 담당 작업이다.

### 4.1 운영·작업 (D 중심)

| 이름 | 형태 · 타입 | 생산자 → 소비자 | QoS · 주기 | 작업 |
|---|---|---|---|---|
| `/ax/fleet/mode` | Topic · `OperationMode` | D → 전체 | STATE · 변경 시 + 1 Hz · 3.0 s | D4 |
| `/ax/fleet/mode/set` | Service · `SetOperationMode` | 운영자·시험 하네스 → D | — | D4 |
| `/ax/fleet/safe_stop/request` | Service · `RequestSafeStop` | 운영자·시험 하네스 → D | 수락은 정지 완료 아님. 해제는 이 Service로 하지 않음 | D4 |
| `/ax/fleet/tasks` | Topic · `GroupTaskList` | D → A·C4·E·관제 | STATE · 변경 시 + 1 Hz · 3.0 s | D2 |
| `/ax/fleet/tasks/register` | Service · `RegisterTask` | D 입고 처리·운영자 → D | — | D2 |
| `/ax/fleet/inventory` | Topic · `InventorySnapshot` | D → E·관제 | STATE · 변경 시 + 1 Hz · 3.0 s | D2 |
| `/ax/fleet/receipt/observations` | Topic · `ReceiptObservation` | B3·C1 → D | EVENT · 관측 시 | B3, C1 |
| `/ax/fleet/holds` | Topic · `HoldStatusList` | D → 관제·E | STATE · 변경 시 + 1 Hz · 3.0 s | D4 |
| `/ax/fleet/failures` | Topic · `FailureEvent` | D → 관제·E | EVENT · 원장 기록 시(영속 원장이 기준) | D4 |
| `/ax/fleet/failures/history` | Topic · `FailureEventList` | D → 관제·E | STATE · 변경 시 + 1 Hz · 3.0 s. 최근 100건, 재시작 후 원장에서 복원 | D4 |
| `/ax/fleet/budget/query` | Service · `QueryBudget` | 관제·E·시험 하네스 → D | 읽기 전용 | D4 |
| `/ax/robot_<n>/failures/report` | Topic · `FailureEvent` | 사건 소유자(A3·A4·B·C4·감독기) → D4 | EVENT · 발생 시. D 무응답이면 로컬 영속 기록 후 재전송 | A3, A4, B4, C4 |
| `/ax/fleet/time_validity` | Topic · `TimeValidity` | A 공용 감독기 → D·A | STATE · 1 Hz · 3.0 s | A4 |
| `/ax/fleet/corridor/state` | Topic · `CorridorStateList` | D → A·B·E·관제 | STATE · 변경 시 + 1 Hz · 3.0 s. 통로 조회는 이 Topic으로 한다(별도 조회 Service 없음) | D3 |
| `/ax/fleet/corridor/release` | Service · `ReleaseCorridor` | D 주행 관리자 → D3 | 해당 로봇의 `corridor_observation`이 `OUTSIDE`·`position_valid=true`일 때만 수락 | D3 |
| `/ax/fleet/corridor/request_entry` | Service · `RequestCorridorEntry` | D 주행 관리자 → D 통로 관리자 | — | D3 |
| `/ax/robot_<n>/conditions/<a·b·c>/recheck` | Service · `RecheckCondition` | D → 해당 로봇의 A4·B4·C4 서버 | — | A4·B4·C4 |
| `/ax/fleet/conditions/d/recheck` | Service · `RecheckCondition` | D 보류 관리 → D 조건 서버 | — | D4 |

### 4.2 로봇 안전·위치 (A·B)

| 이름 | 형태 · 타입 | 생산자 → 소비자 | QoS · 주기 · 만료 초안 | 작업 |
|---|---|---|---|---|
| `/ax/robot_<n>/safety/state` | Topic · `SafetyState` | B4 → A4·D | STATE · 10 Hz · 0.5 s | B4 |
| `/ax/robot_<n>/safety/actuation_state` | Topic · `ActuationState` | A4 → A2(`arm_allowed`)·B4·D·**상대 로봇 A4** | STATE · 20 Hz · 0.3 s | A4 |
| `/tf`·`/tf_static` | Topic · `tf2_msgs/TFMessage` | B2(`map→odom`)·A2(`odom→base_link`, 관절·고정 frame) → 전체(인계 상대 로봇 C4 포함) | ROS 2 TF 기본 QoS | B2, A2 |
| `/ax/robot_<n>/safety/clear_fault` | Service · `ClearFault` | 시험 하네스·사후 조치 경로 → A4. D는 본설계 14절 리셋 범위 확정 전 자동 호출하지 않음 | 실제 정지·SAFE_STOP 아님·원인 해소·진행 중 작업 없음일 때만 수락. 다른 차단 사유는 유지 | A4 |
| `/ax/robot_<n>/localization/health` | Topic · `LocalizationHealth` | B2 → B3·B4·D(D1·D4 snapshot)·인계 상대 로봇 C4 | STATE · 5 Hz · 1.0 s | B2 |
| `/ax/robot_<n>/localization/reinitialize` | Service · `std_srvs/Trigger` | D → B2 | 접수만 응답, 완료는 `localization/health`로 확인. request_id가 없으므로 진행 중 재요청은 같은 재초기화로 처리(멱등) | B2 |
| `/ax/robot_<n>/perception/corridor_observation` | Topic · `CorridorObservationList` | B3 → D3·A4 | STATE · 5 Hz · 1.0 s | B3 |

### 4.3 로컬 실행·조작·적재 (A·C)

| 이름 | 형태 · 타입 | 생산자 → 소비자 | QoS · 주기 | 작업 |
|---|---|---|---|---|
| `/ax/robot_<n>/executor/execute_local_task` | Action · `ExecuteLocalTask` | D2(클라이언트) → A3(서버) | Action 기본 | A3 |
| `/ax/robot_<n>/executor/status` | Topic · `RobotTaskStatus` | A3 → D2 | STATE · 변경 시 + 1 Hz · 3.0 s | A3 |
| `/ax/robot_<n>/executor/assignment_check/request` | Topic · `AssignmentCheckRequest` | A3 → D4 | EVENT · lease 갱신 주기 | A3 |
| `/ax/robot_<n>/executor/assignment_check/response` | Topic · `AssignmentCheckResponse` | D4 → A3 | EVENT · 요청 시 | D4 |
| `/ax/robot_<n>/manipulation/execute` | Action · `ManipulateObject` | A3(클라이언트) → C4(서버) | Action 기본 | C4 |
| `/ax/robot_<n>/manipulation/lease/request` | Topic · `ExecutionLeaseRequest` | C4 → A3 | EVENT · 첫 시도 전·갱신 시 | C4 |
| `/ax/robot_<n>/manipulation/lease/grant` | Topic · `ExecutionLeaseGrant` | A3 → C4 | EVENT · 요청 시 | A3 |
| `/ax/robot_<n>/manipulation/lease/revoke` | Topic · `ExecutionLeaseRevoke` | A3 → C4 | EVENT · 철회 시(Action cancel과 함께) | A3 |
| `/ax/robot_<n>/manipulation/tool_command` | Service · `ToolCommand` | C4 → A2(부모 `goal_id`를 바꾸지 않고 `CheckCommandLease`로 확인, `arm_allowed` 확인 후 수락) | 수락은 부착·해제 완료 아님 | A2 |
| `/ax/robot_<n>/executor/lease/check_command` | Service · `CheckCommandLease` | A2·A4(클라이언트) → A3(서버) | 상위 명령 수락·진행 중 확인용. 관절 제어 주기마다 호출하지 않음(5.2절) | A3 |
| `/ax/robot_<n>/manipulation/attach_state` | Topic · `AttachState` | A2 → C4 | STATE · 20 Hz · 0.3 s | A2 |
| `/ax/robot_<n>/arm/execute_trajectory` | Action · `ProtectedArmTrajectory` | C4(조작, `AUTH_C_MANIPULATION`) 또는 A3(일반 팔 후퇴, `AUTH_A3_LOCAL_TASK`) → A4 보호 경로(서버). A4가 내부에서 팔 제어기(`FollowJointTrajectory`)로 넘김 | Action 기본. 제어기 Action은 A4만 호출. 이 Action의 UUID는 부모 `goal_id`와 다름 | A4 |
| `/ax/robot_<n>/sensors/tool_tof/raw` | Topic · `sensor_msgs/Range` | A1(센서 모델) → B1 | SENSOR · 30 Hz · frame `robot_<n>/tool_tof_link`(8절) | A1 |
| `/ax/robot_<n>/sensors/tool_tof` | Topic · `sensor_msgs/Range` | B1(전처리·무효값 검사) → C | SENSOR · raw와 같은 주기 | B1 |
| `/ax/robot_<n>/manipulation/hold_state` | Topic · `ToolHoldState` | C4 → D2 | STATE · 10 Hz · 0.5 s | C4 |
| `/ax/robot_<n>/manipulation/candidates` | Topic · `CandidateSetList` | C3 → D4 | STATE · 관측 갱신 시 + 1 Hz · 3.0 s | C3 |
| `/ax/robot_<n>/load/state` | Topic · `LoadState` | A3(운송 로봇) → D2·**상대 로봇 A4** | STATE · 10 Hz · 0.5 s | A3 |

인계 중 상호 배제(DEC-19)는 각 로봇의 A4가 강제한다.

- **대상 작업:** `/ax/fleet/tasks`에서 자기 `robot_id`가 `manipulator_robot_id` 또는 `transporter_robot_id`인 모든 작업. D는 `transfer_state`가 `NONE`·`UNLOCKED`가 아닌 작업을 작업 상태(FAILED·CANCELED 포함)와 관계없이 잠금 해제를 확인할 때까지 목록에 남긴다. 재배정할 때도 이전 로봇의 잠금 해제를 확인한 뒤 로봇 ID를 바꾼다.
- **잠금:** 대상 작업 중 하나라도 `transfer_state`가 `TRANSFER_UNKNOWN`·`LOCKED`·`IN_PROGRESS`면 `HANDOFF_LOCK`으로 주행을 막는다. 실패 뒤 남은 `LOCKED`도 D가 명시적으로 해제할 때까지 그 로봇을 막는다.
- **상대 상태:** 상대 상태 만료 검사는 `transfer_state`가 `TRANSFER_UNKNOWN`·`LOCKED`·`IN_PROGRESS`인 작업에만 적용한다. 상대는 그 작업의 다른 역할 로봇이다. 상대의 `actuation_state`는 항상 검사하고, `load_state`는 상대가 운송 역할(`transporter_robot_id`)일 때만 검사한다. 만료되면 `PEER_STATE_STALE`로 주행을 막는다. `ExecuteLocalTask` Goal의 `peer_robot_id`가 작업의 상대 로봇과 다르면 Goal 서버인 A3가 수락 전에 거절한다. A4는 같은 정보로 상대 상태를 구독하고 구동 차단 검사를 그대로 한다.
- **목록 미수신·만료:** A4가 시작·재시작 후 `/ax/fleet/tasks`를 아직 받지 못했거나, D 세션이 바뀐 직후이거나, 목록이 만료되면 배정 여부를 `UNKNOWN`으로 보고 `HANDOFF_LOCK`으로 주행을 막는다(배정 없는 대기 로봇 포함, 보수 정책). D 세션이 바뀐 뒤에는 새 세션의 `/ax/fleet/tasks`를 받아 자기 로봇의 배정·`transfer_state`·상대 로봇을 다시 확인해야 차단을 푼다.
- 받은 목록에서 배정된 작업이 없는 로봇에는 인계 규칙을 적용하지 않는다.

통로 미허가 차단(`CORRIDOR_NOT_GRANTED`)은 A4가 자기 로봇의 `corridor_observation`과 `/ax/fleet/corridor/state`로 판정한다.

- 통로 형상과 접근 구역은 B3가 지도 구성으로 관리한다. `APPROACHING`은 "통로 진입선 앞 접근 구역 안에서 통로 쪽을 향함"을 뜻한다. 접근 구역은 통로 입구에만 두고 일반 통행로와 겹치지 않게 구성한다(B3·D1). 접근 구역을 지나는 경로는 D1이 통로 진입으로 취급해 허가를 먼저 받는다.
- 관측이 `APPROACHING`·`INSIDE`·`UNKNOWN`이거나 `position_valid=false`인 통로에 이 로봇의 활성 허가(`owner_robot_id`·`active_grant_id`)가 없으면, A4는 선속도를 0으로 막고 제자리 회전만 허용한다.
- 관측 목록이 만료되면 모든 통로를 `UNKNOWN`으로 본다. 통로 상태 목록이 만료되면 활성 허가가 없다고 본다.
- D3는 해당 로봇의 관측이 `APPROACHING`·`INSIDE`·`UNKNOWN`인 동안 허가를 만료·철회하지 않는다. 재시작 복원으로 `BLOCKED_UNKNOWN`을 만들 때도 원장의 `owner_robot_id`·`active_grant_id`를 유지한다. 그래서 허가를 가진 로봇은 통로 밖으로 빠져나올 수 있다.
- D는 대기 지점(`WAIT`)을 접근 구역 밖에 등록한다.

A2(`ToolCommand`)와 A4(`ProtectedArmTrajectory`)의 명령 수락·진행 중 확인·정지 규칙은 5.2절 "lease 소유와 명령 수락"을 따른다. 팔 제어기의 `FollowJointTrajectory`는 A4만 호출한다.
흡착 공구 ToF 값은 접촉·흡착·보유 판정이 아니다(본설계 3.1절).

주행은 D1이 로봇별 Nav2 `NavigateToPose` Action을 직접 호출한다. Nav2의 속도 출력은 A4 최종 구동 게이트를 거친 뒤 ros2_control 제어기로 간다. Nav2 출력은 `/ax/robot_<n>/nav/cmd_vel`(`geometry_msgs/TwistStamped`)이고, A4가 `drive_allowed`, `SafetyState`의 방향 허용(`allow_forward`·`allow_backward`·`allow_rotate`)과 선속도·각속도 상한(`max_linear_speed`·`max_angular_speed`)을 적용해 `/ax/robot_<n>/diff_drive_controller/cmd_vel`(TwistStamped)로 넘긴다. 차단 중에는 0 속도를 보낸다. Nav2 입력을 0.2 s(단조 시계) 동안 받지 못하거나 `header.stamp`가 0.2 s(sim time)보다 오래된 명령은 버리고 0 속도를 보낸다. A4는 마지막 Nav2 명령을 다시 발행해 명령 수명을 늘리지 않는다. 다른 노드는 제어기 입력 Topic에 직접 발행하지 않는다.

### 4.4 Agent (E ↔ D)

| 이름 | 형태 · 타입 | 생산자 → 소비자 | QoS · 주기 | 작업 |
|---|---|---|---|---|
| `/ax/fleet/agent/snapshot` | Topic · `FleetSnapshot` | D4 → E1(중앙) | STATE · 변경 시(최대 2 Hz) + 1 Hz · 3.0 s | D4 |
| `/ax/robot_<n>/agent/snapshot` | Topic · `FleetSnapshot`(그 로봇 범위만) | D4 → E2(로봇별) | STATE · 변경 시(최대 2 Hz) + 1 Hz · 3.0 s | D4 |
| `/ax/fleet/agent/<sender_agent_id>/proposal` | Topic · `AgentProposal` | Agent 어댑터(E3) → D4 | EVENT · 제안 시 | E3 |
| `/ax/fleet/agent/agent_central/plan` | Topic · `AgentPlan` | 중앙 Agent 어댑터(E3) → D4 | EVENT · 계획 제출 시 | E1, E3 |
| `/ax/fleet/agent/decision` | Topic · `AgentDecision` | D4 → E·로그 | EVENT · 결정 시 | D4 |
| `/ax/fleet/agent/receive_state` | Topic · `AgentReceiveStateList` | D4 → 관제·E | STATE · 변경 시 + 1 Hz · 3.0 s | D4 |
| `/ax/fleet/agent/<sender_agent_id>/reasoning_state` | Topic · `AgentReasoningState` | Agent 어댑터(E3) → D4·관제 | STATE · 변경 시 + 1 Hz · 3.0 s | E3 |

전체 계획은 `/plan` Topic의 `AgentPlan`으로만 제출한다(제안 종류에 계획 제출은 없다, D-17). `AgentProposal.plan_id`는 제안의 근거 계획 참조다. 경로를 Agent별로 나눠도 발신자 인증을 대신하지 않는다. SROS2 권한(A4)과 D4의 `sender_agent_id`·세션·세대·대상 대조를 함께 쓴다(본설계 5.1·7.6절).

## 5. Action·Service 계약

### 5.1 `ExecuteLocalTask` (D → A)

- **Goal:** `ExecutionRef`, 작업 종류(`LOCK_HANDOFF`·`UNLOCK_HANDOFF`·`PICK_AND_STOW`·`UNLOAD_AND_PLACE`·`CONFIRM_LOAD`·`RETRACT_ARM`, 0은 `UNSPECIFIED`로 거절), 인계 상대 로봇 `peer_robot_id`, 물체 ID와 재고 revision, 출발·도착 지점 ID와 각 revision, 지도 revision(revision은 D가 전송 직전 원장과 대조하고 A는 기록·보고), 작업별 `pick_pose`·`place_pose`(5.2절 표, pose가 필요 없는 작업은 비움), `BudgetGrant[]`, 선택적 후보 선호.
- **후보 선호 전달 조건(본설계 7.13절):** D는 Goal 전송 시 ASSISTED이고 해당 Agent 수신 상태가 ACTIVE이며 이미 도착한 유효 제안이 있을 때만 `has_candidate_preference=true`로 넣는다. 의견을 기다리지 않으며, 실행 중 도착한 의견은 그 실행에 반영하지 않는다.
- **Feedback:** 단계, 진행률, 시도별 사용 기록.
- **Result:** `ResultStatus`, 보유 판정(`object_hold_state`, `ToolHoldState` 상수)·적재 상태, 시도 사용 기록, `need_reobservation_move`와 C의 관측 요구(`observation_reason`·`observation_target`). 재관측 위치는 D가 등록 지점 중에서 고른다.
- 주행은 포함하지 않는다. A는 수락 전에 `execution_id`를 원장에 기록하고 같은 ID 재요청에는 기존 결과를 돌려준다.
- `RETRACT_ARM`은 C의 조작 Goal 없이 A3가 직접 수행하는 로컬 작업이다. A3는 이 작업의 실행을 수락·기록한 뒤 `ProtectedArmTrajectory`를 `AUTH_A3_LOCAL_TASK`로 A4에 보낸다(5.2절 권한 종류).

### 5.2 `ManipulateObject` (A → C)

lease·명령 수락 규칙의 기준은 이 절이다. 다른 절은 이 절을 참조한다.

- **Goal:** `ExecutionRef`, 물체, `PICK`·`PLACE`(0은 `UNSPECIFIED`로 거절), 목표 pose, 인계 상대 로봇 `peer_robot_id`, `BudgetGrant[]`(집기 시도와 재관측을 구분), A3 원장의 마지막 lease 세대(첫 Goal은 0, 기록용), 선택적 후보 선호. Goal만으로 물리 시도를 허가하지 않는다.
- **Result:** `ResultStatus`, `object_hold_state`(0은 미보유가 아니라 불명), 시도 수·사용 기록, `need_reobservation_move`와 관측 요구. 차체 이동이 필요하면 C는 Action을 끝내고 이 값을 돌려준다. C는 차체 이동을 직접 실행하지 않는다.
- **팔 동작과 공구:** C4는 팔 궤적을 `arm/execute_trajectory`(`ProtectedArmTrajectory`, `AUTH_C_MANIPULATION`)로, 흡착·해제를 `tool_command`(`ToolCommand`)로 요청한다. 두 요청에는 C4가 처리 중인 `ManipulateObject` Goal의 UUID를 부모 `goal_id`로 넣는다. 실제 부착은 `attach_state`, 보유 판정은 C4의 `hold_state`로 확인한다.

#### lease 소유와 명령 수락

- **단일 소유:** A3가 로봇별 lease·세대·명령 수락 기록의 유일한 소유자다. 같은 `execution_id`의 lease 발급·갱신·철회와 명령 확인은 A3가 하나씩 순서대로(직렬로) 처리한다. A2·A4는 lease 표를 따로 갖지 않는다.
- **명령 확인 Service:** A2(`ToolCommand`)와 A4(`ProtectedArmTrajectory`)는 실제 명령을 수락하기 전에 A3의 `CheckCommandLease`(`/ax/robot_<n>/executor/lease/check_command`)를 `ACCEPT`로 호출한다. A2·A4는 요청으로 받은 권한 필드(`authority`·`execution_id`·부모 `goal_id`·`lease_generation`)를 바꾸지 않고 그대로 전달하며, 명령 출처·명령 ID(`command_id`)·명령 인자 digest를 덧붙인다. 부모 `goal_id`는 `ManipulateObject` Goal의 UUID이며 보호 팔 Action 자체의 UUID가 아니다.
- **권한 종류:** 확인 요청은 두 권한 중 하나다. 권한마다 필수 식별자와 적용하지 않는 필드가 다르며, 적용하지 않는 필드에 값이 있으면 A3는 거절한다(`AUTHORITY_INVALID`).

| 권한 (`authority`) | 쓰는 경우 | 필수 식별자 | 적용하지 않는 필드(비움) | A3 검사 |
|---|---|---|---|---|
| `AUTH_C_MANIPULATION` | C4의 조작(팔 궤적·흡착·해제) | `execution_id`(ManipulateObject 실행), 부모 `goal_id`, `lease_generation` | `local_task_kind`=0 | `goal_id`가 A3가 보낸 현재 활성 Goal과 같은지, `execution_id`·`lease_generation`이 그 Goal의 유효 lease와 같은지, 현재 배정이 유효한지(`tA + H`), `execution_id`가 위임 예산의 허용 목록에 있는지 |
| `AUTH_A3_LOCAL_TASK` | A3의 일반 팔 후퇴(`RETRACT_ARM`) | `execution_id`(A3가 수락·기록한 현재 `ExecuteLocalTask` 실행), `local_task_kind`=`RETRACT_ARM` | `goal_id`=빈 문자열, `lease_generation`=0 | 그 `execution_id`가 A3의 현재 실행 중 로컬 작업이고 작업 종류가 `RETRACT_ARM`인지, 현재 배정이 유효한지, 출처가 `SOURCE_A4_ARM`인지 |

  누락·형식 오류(`goal_id`가 소문자 16진수 32자가 아님)·종료된 부모 Goal·이전 세대·다른 배정은 거절한다. `AUTH_A3_LOCAL_TASK`는 `RETRACT_ARM` 외의 로컬 작업에는 쓸 수 없어, 다른 로컬 작업이 이 경로로 팔을 움직이지 못한다. C의 조작 없이 하는 후퇴에 가짜 `ManipulateObject` Goal이나 임의 lease를 쓰지 않는다. 허용이면 A3는 **수락 기록을 먼저 남긴 뒤** 응답한다.
- **멱등:** 같은 `request_id`의 재전송은 기존 결과만 돌려주며 세대 증가·추가 수락 기록·물리 실행을 만들지 않는다. 같은 `request_id`나 같은 `command_id`에 인자(digest)가 다르면 거절한다(`DUPLICATE_REQUEST`). 기한이 지나 새 `request_id`로 다시 확인하더라도 `command_id`가 같으면 같은 물리 명령이다. A3는 기존 수락 기록을 재사용하고 새 기록을 만들지 않으며, A2·A4는 그 명령을 다시 실행하지 않는다.
- **실행 조건과 기한:** A2·A4는 `request_id`마다 **최초 송신 시각 `t_first`**(CLOCK_BOOTTIME)를 기록한다. 같은 요청을 재전송해도 `t_first`는 바꾸지 않는다. 허용 기한은 항상 `t_first + valid_for_s`이며, 응답 수신 시각이나 재전송 시각부터 권한을 늘리지 않는다. `allowed=false`, 무응답, 기한 경과 중 하나라도 해당하면 명령을 실행하지 않는다. 기한이 지났으면 캐시된 `allowed=true` 응답도 실행 권한으로 쓰지 않고, 새 `request_id`로 A3가 현재 권한을 다시 검사하게 한다. 프로세스 재시작 등으로 `t_first`를 잃으면 과거 응답을 재사용하지 않고 새 요청으로 확인한다. A3는 `valid_for_s`를 그 권한의 남은 기한(lease 또는 배정 확인 기한)보다 길게 주지 않는다.
- **진행 중 확인:** 실행이 허용 기한보다 길면 A4는 lease 갱신 주기(D-07의 0.5 s)마다 `CONTINUE`로 다시 확인한다. `CONTINUE`는 새 수락 기록을 만들지 않는다. 거절·무응답으로 기한이 지나거나 `lease/revoke`를 받으면 A4는 진행 중 궤적을 검증된 방식으로 정지한다. 이 Service는 상위 명령 단위로 쓰며 관절 제어 주기마다 호출하지 않는다.
- **`arm_allowed` 경로:** A2는 자기 로봇의 `safety/actuation_state`(A4)를 구독해 `arm_allowed`를 확인한다. 메시지가 무효·만료·미수신이면 허용으로 보지 않는다.
- **일반 후퇴와 보호 정지의 구분:** 일반 팔 후퇴(`RETRACT_ARM`)는 A3가 `ProtectedArmTrajectory`를 `AUTH_A3_LOCAL_TASK`로 보내는 상위 명령이며, A4의 보호 경로와 구동 허용 조건(`arm_allowed` 등)을 모두 통과해야 한다. A3가 응답하지 않으면 일반 후퇴는 할 수 없다. 후퇴 완료는 Action 결과가 아니라 실제 관절 상태(`joint_states`)로 확인한다. 로컬 보호 정지는 A4가 lease·A3·이 Action과 관계없이 항상 할 수 있으며, 정지할 때 흡착은 자동 해제하지 않는다.
- **기록의 목적 구분:** A3의 수락 기록은 "그 세대로 물리 명령이 나갔을 수 있다"는 A 측 사실이며 재동기화 판정에 쓴다. 예산 차감은 C의 `AttemptUsage` 기록이다. lease 발급·재동기화 자체는 예산을 쓰지 않고 새 물리 시도를 만들지 않는다.
- **A 측 기한:** A3는 lease 기한을 자기 CLOCK_BOOTTIME 발급 시각 + `lease_s`로 관리한다. C 측 기한은 C의 요청 시각 `tC + lease_s`이며, `0 < lease_s <= min(상한, tA + H - 현재)`.

#### lease 요청·응답

- **Goal 인스턴스 결합:** 모든 lease 요청·응답·철회는 C가 받은 `ManipulateObject` Goal의 UUID(`goal_id`)를 담는다. A3는 자기가 보낸 현재 활성 Goal과 같은 `goal_id`만 처리하고, 끝났거나 취소된 이전 Goal의 지연 요청은 거절한다. C는 다른 `goal_id`의 응답을 버린다. `c_session_id`는 기록용이다.
- **`FIRST`:** 이 Goal의 lease를 받는 요청이다. A 원장에 그 `execution_id`의 이전 세대나 수락 기록이 **없으면** 마지막 세대+1을 발급한다. **있으면** `goal_id`가 새 것이어도 아래 재동기화 절차를 거친다.
- **`RENEW`:** 보유한 유효 세대의 기한만 새로 정한다. 만료·철회된 세대면 거절한다. RENEW 응답을 잃으면 C는 마지막으로 받은 기한까지만 실행한다. 이미 만료된 lease는 늦게 온 응답으로 되살리지 않으며, C는 재동기화 `FIRST`로 넘어간다.
- **재전송:** 같은 `request_id`의 `FIRST`·`RENEW` 재전송은 기존 응답만 돌려주며 추가 철회·세대 증가를 일으키지 않는다.
- **재동기화 절차** (응답 유실·만료 뒤, 또는 같은 `execution_id`에 새 Goal이 왔을 때): C는 같은 `goal_id`로 새 `request_id`의 `FIRST`를 보낸다. 재동기화에 들어간 C는 그 뒤 도착하는 이전 `request_id`의 grant를 모두 버린다. A3는 다음을 직렬로 수행한다.
  1. **이전 세대 차단:** 이전 세대를 철회로 표시한다. 이후 그 세대의 `CheckCommandLease`는 모두 거절된다.
  2. **수락 기록 확인:** A3 자신의 수락 기록에서 이전 세대의 `ACCEPT` 허용 기록을 찾는다.
  3. **판정:** 기록이 없음을 확인하면 새 세대를 발급한다. 기록이 있거나 원장을 신뢰할 수 없어 확인하지 못하면 `granted=false`, `LEASE_RESYNC_UNCERTAIN`으로 응답한다. C는 Action을 `UNCERTAIN`으로 끝내고 D의 보류(본설계 9.6절)로 보낸다.
- **재동기화 한도:** Goal당 2회(개발용 출발값, 8절). 넘으면 시작된 시도가 없으므로 C는 Action을 `FAILED`·`LEASE_EXPIRED`로 끝낸다.
- **철회:** A는 `lease/revoke`와 Action cancel을 함께 보낸다. C는 철회 메시지의 `execution_id`·`lease_generation`·`goal_id`가 모두 현재 것과 같을 때만 처리하고, 다르면 무시하고 기록한다. 둘 다 오지 않아도 lease 만료가 같은 차단을 한다.
- **같은 `execution_id`의 새 Goal (C4):** C4는 같은 `execution_id`의 활성 실행이 있으면 새 Goal을 `DUPLICATE_REQUEST`로 거절한다. 이미 끝난 실행이면 물리 동작 없이 저장된 결과를 돌려준다. 같은 `execution_id`에 Goal 인자가 다르면 거절한다.

#### 목표 pose와 상대 로봇

D는 `ExecuteLocalTask` Goal의 `pick_pose`·`place_pose`를 검증해 넣고, A3는 PICK에는 `pick_pose`, PLACE에는 `place_pose`를 `ManipulateObject.target_pose`로 그대로 넘긴다. 인계 잠금·해제·적재 확인·팔 후퇴처럼 pose가 필요 없는 작업은 두 pose를 비우며 판정에 쓰지 않는다.

| 로컬 작업 | PICK 대상 (`pick_pose`) | PLACE 대상 (`place_pose`) |
|---|---|---|
| `PICK_AND_STOW` | 출발지 물체. `map` 기준, D 원장의 물체 마지막 확인 pose(C 관측 → D 원장) | 운송 로봇 적재함. `robot_<상대>/cargo_link` 기준 적재 위치(A1 모델 상수) |
| `UNLOAD_AND_PLACE` | 적재함 물체. `robot_<상대>/cargo_link` 기준 적재 위치(관측 초기값) | 목적지 배치 위치. `map` 기준 도착 지점 `RegisteredLocation.pose` |

- **상대 로봇 확인:** 상대 로봇은 Goal의 `peer_robot_id`다. C4는 `/ax/fleet/tasks`에서 Goal의 `ExecutionRef.task_id`·`transfer_id`·`assignment_generation`이 같은 작업을 찾아, `peer_robot_id`가 그 작업의 다른 역할 로봇과 같은지 확인한다. frame 문자열에서 로봇 ID를 꺼내 판단하지 않는다.
- **`map` 기준 pose:** D가 Goal 전송 직전 재고·지점 revision으로 대조한다.
- **`cargo_link` 기준 pose:** C4는 실행 직전과 각 하위 명령 전에 다음을 모두 확인한다.
  - 자기 로봇과 상대 로봇의 `LocalizationHealth`가 `OK`
  - 필요한 TF(자기 `map→odom→base_link`, 상대 `map→odom→base_link→cargo_link`)가 유효하고 최신(8절)
  - 위 작업의 `transfer_state`가 `TRANSFER_LOCKED` 또는 `TRANSFER_IN_PROGRESS`

  하나라도 맞지 않으면 시작 전에는 Goal을 거절한다(시도 없음).
- **재관측(PICK 공통, 하역 포함):** C는 PICK 직전 물체를 다시 관측해 최종 공구 목표를 계산하고 관측 물체가 `object_id`와 같은지 확인한다. 하역의 고정 적재 위치는 관측 초기값일 뿐이다. 가림·식별 실패·적재 위치 변화로 목표를 정할 수 없으면 `need_reobservation_move`와 관측 요구로 Action을 끝내거나, 재관측 예산이 없으면 실패로 끝낸다(9.7절 재관측 절차). 적재 판정용 Gazebo 객체 위치는 C의 관측에 쓰지 않는다. 적재함 안 이동 허용 오차는 시험 단계에서 정한다(미정).

#### 수락 후 조건 상실

Goal을 수락한 뒤 위치 신뢰성·TF·인계 잠금·lease 중 하나라도 무효가 되면 C는 다음을 따른다.

1. 새 하위 명령(팔 궤적·흡착·해제)을 보내지 않는다.
2. 진행 중 하위 동작은 검증된 안전 정지·취소 절차로 멈춘다(A4 보호 경로).
3. 들고 있는 물체는 계속 보유하며 흡착을 자동 해제하지 않는다.
4. 실제 결과(시작 안 함, 보유 유지 등)를 확인하면 `FAILED`, 확인할 수 없으면 `UNCERTAIN`으로 Action을 끝낸다.
5. 오류 코드는 원인에 따라 `LOCALIZATION_INVALID`·`TF_STALE`·`HANDOFF_LOCK_LOST`·`LEASE_EXPIRED`·`LEASE_REVOKED`(D-13)를 쓰고, A는 실패 사건(`failures/report`)을 올려 D의 보류로 연결한다.
6. C Action 종료는 실제 정지 완료가 아니다. 실제 정지는 A4의 `actuation_state`로 확인한다.

### 5.3 배정 유효성 확인 (A ↔ D)

A가 자기 세션 ID(`a_session_id`)를 넣어 `assignment_check/request`를 보내고 요청 시각 `tA`를 기록한다. D는 재시작 전 A 세션의 요청을 거절한다. D 응답의 `validity_s`(H)로 권한 기한 `tA + H`를 계산한다. 늦게 온 응답, 이전 세션·세대 응답, 중복 응답은 버린다. 일반 상태 heartbeat로는 기한을 늘리지 않는다.

### 5.4 Service

| Service | 요청 | 응답 | 규칙 |
|---|---|---|---|
| `RequestCorridorEntry` | 통로·로봇·request_id | 허가·grant_id·사유·revision | 같은 request_id는 같은 응답. 점유 확인 없이 해제 불가 |
| `RegisterTask` | request_id·입고 사건·물체·출발/도착 지점 | 수락·task_id·사유 | 입고 사건당 작업 1개. 도착 지점이 비면 D 규칙 또는 중앙 계획 |
| `RecheckCondition` | hold·조건·로봇 | 충족/불충족/불명·근거 상태 ID | 계획 조건의 `condition_owner`(경로에는 소문자) 영역 서버가 판정. A·B·C는 `PlanCondition.robot_id` 로봇의 서버, D는 fleet 서버. 응답 0(UNKNOWN)이면 보류 유지. LLM 판단으로 대신하지 않음 |
| `ToolCommand` | request_id·로봇·실행 ID·부모 `goal_id`·물체·lease 세대·부착/해제 | 수락·사유 | A2가 `CheckCommandLease`(`AUTH_C_MANIPULATION`) 허용과 `arm_allowed`를 확인한 뒤 수락. 수락은 부착·해제 완료가 아님. 같은 request_id는 같은 응답 |
| `SetOperationMode` | 모드·사유 | 수락·사유 | 시작 전 설정이 원칙. D는 운영 중 스스로 상향하지 않는다. 운영 중 운영자 상향 요청은 D-18 조건에서만 수락. 인증 검증 전 ASSISTED는 거절. 장애 시 하향은 D가 자동 수행 |
| `RequestSafeStop` | request_id·사유 | 수락·사유 | 수락은 진입 요청 접수이며 정지 완료가 아님. 해제는 본설계 9.5절 절차로만 |
| `ReleaseCorridor` | request_id·통로·로봇·grant_id | 수락·사유·revision | 완전 이탈(`OUTSIDE`·`position_valid=true`) 확인 전에는 거절, 허가 유지 |
| `QueryBudget` | request_id·복구 문맥 또는 작업 | 예산·사용·미확정·동결 여부 | 읽기 전용. 조회로 예산이 바뀌지 않음 |
| `ClearFault` | request_id·로봇·고장 코드·사유 | 수락·사유·남은 차단 사유 | A4가 해제 조건을 직접 확인. 수락해도 다른 차단 사유는 유지 |

## 6. 상태 enum 대응

| 메시지 | 상태 | 본설계 |
|---|---|---|
| `SafetyState.level` | CLEAR(0) · SLOWDOWN_REQUIRED · STOP_REQUIRED · UNKNOWN — 설계 숫자 유지, 2.3절 규칙으로 보호 | 7.2 |
| `ActuationState.state` | UNKNOWN(0) · RUNNING · STOP_REQUESTED · STOPPING · STOPPED · FAULT | 8 안전 실행 |
| `LoadState.state` | UNKNOWN(0) · STOWED · EMPTY · LOST | 7.8 |
| `ToolHoldState.state` | UNKNOWN(0) · HELD · RELEASED | 7.7 |
| `InventoryEntry.state` | UNKNOWN(0) · AT_SOURCE · HELD_BY_MANIPULATOR · STOWED_ON_TRANSPORTER · AT_DESTINATION · LOST | 7.8 그룹 물체 상태(역할 이름으로 일반화) |
| `CorridorState.state` | FREE(0) · RESERVED · OCCUPIED · BLOCKED_UNKNOWN — 설계 숫자 유지, 2.3절 규칙으로 보호 | 7.3 |
| `GroupTaskStatus.state` | UNSPECIFIED(0) · QUEUED · ASSIGNED · RUNNING · COMPLETED · FAILED · CANCELED · RETRY_PENDING | 8 |
| `GroupTaskStatus.transfer_state` | TRANSFER_UNKNOWN(0) · NONE · LOCKED · IN_PROGRESS · UNLOCKED. 0·만료는 두 로봇 주행 금지 | 9.3 |
| `HoldStatus.state` | HOLD_PENDING(0) · HELD · RECHECK · RELEASED · FAILED_ISOLATED | 9.6 |
| `OperationMode` | OFF(0) · ADVISORY · ASSISTED / SYSTEM_UNKNOWN(0) · SYSTEM_NORMAL · SYSTEM_SAFE_STOP | 5.1 · 9.5 |
| `AgentReceiveState.state` | UNKNOWN(0) · ACTIVE · QUARANTINED · RECOVERY_SHADOW | 5.3 |
| `AgentReasoningState.state` | DISABLED(0) · IDLE · REASONING · VALIDATING · ACCEPTED · REJECTED · FALLBACK | 8 |
| `AgentProposal.action` | UNSPECIFIED(0, 형식 오류) · NO_ACTION · ASSIGN_TASK · DEFER_TASK · RECOMMEND_RETRY · RECOMMEND_REASSIGN · RECOMMEND_PRIORITY_CHANGE · REQUEST_HOLD · REQUEST_RESUME_REVIEW · PREFER_CANDIDATE · MOVE_TO_LOCATION · WAIT_AT_LOCATION | 5.2 · 7.6 |

그 밖에 `ResultStatus`·`AgentDecision`·`AttemptUsage`·`FailureEvent`·`ToolCommand`·`ExecutionLeaseRequest.kind`·`CheckCommandLease`(`phase`·`command_source`)·Goal 작업 종류는 0이 `UNSPECIFIED`, `LocalizationHealth`·`TimeValidity`·`RegisteredLocation`·`RecheckCondition` 응답·`LoadState.source`·`ReceiptObservation.source`는 0이 `UNKNOWN`이다. `HoldStatus` `HOLD_PENDING`, `AgentReasoningState` `DISABLED`, `ReceiptEvent` `PENDING_IDENTIFICATION`의 0은 보류·비활성·미확정이라는 보수적 의미라 그대로 둔다. `GroupTaskStatus.priority` 0은 미설정이며 D가 등록 때 기본값을 채운다. `InventoryEntry.isolated=false`만으로 정상 물품으로 보지 않고 `state`·보류로 확인한다.

설계와 이름이 달라진 필드: `SafetyState.recommended_max_linear_speed` → `max_linear_speed`(각속도 상한 `max_angular_speed` 추가), `CorridorState.revision` → `corridor_revision`, 그룹 물체 상태 `HELD_BY_ROBOT_1`·`STOWED_ON_ROBOT_2` → 역할 이름(D-15), `ManipulateObject` Result `success`·`error_code`·`object_held` → `ResultStatus`·`object_hold_state`, `SafetyState`의 `header` → `StateMeta`, `CorridorState`의 `header` → 목록 `CorridorStateList`의 `StateMeta`.

## 7. 승인 방식과 결정 항목

### 7.1 승인 방식 — 이의 없으면 승인

- 팀이 정할 것은 아래 **결정 항목 19개**, 8절 세부 결정, `src/ax_interfaces` 타입의 필드 구성이다. 모두 정해 둔 값이 있으므로 이의가 없으면 그대로 구현한다. 필드 구성은 4~6절과 타입 파일 주석이 설명한다.
- **이의 마감 11-06(금)**: 그때까지 이의가 없으면 추천값으로 확정하고 **11-08(일)** K0 승인으로 기록한다.
- 이의가 있으면 "이의" 칸에 작업 ID와 대안을 적는다. 이의가 있는 항목만 30분 회의로 정한다.
- 숫자 값은 출발값이다. P2 실측(11-22까지) 결과로 바꾸며, 바꿀 때는 이 표와 타입을 함께 고치고 관련 생산자·소비자에게 알린다.
- 7.2절 규칙은 이미 설계에서 정해진 내용이므로 정하지 않고 지키기만 한다.

| ID | 결정 항목 | 추천값 | 관련 작업 | 이의 |
|---|---|---|---|---|
| D-01 | 인터페이스 이름 | 4절 표의 이름 그대로. 로봇별은 `/ax/robot_<n>/…`, 공용은 `/ax/fleet/…`. namespace 목록의 기준은 이 문서 4절이며 본설계 6절은 이를 참조한다 | 전원 | |
| D-02 | ID 형식 | 2.1절 표 그대로(`접두사-UUIDv4`, `robot_<n>`, `team_<n>`) | D2·D4 | |
| D-03 | `execution_id` 생성 규칙 | 키 정렬·공백 없는 UTF-8 JSON `{attempt_index, operation, root_execution_id, step_key}`의 SHA-256 **소문자** 16진수 앞 32자. `operation`은 Goal enum 상수 이름 문자열(예: `PICK_AND_STOW`, `PICK`), `attempt_index`는 0부터 시작하는 정수. 주행처럼 Goal enum이 없는 단계는 D-11의 operation 이름(예: `NAVIGATE_TO`). 직렬화는 RFC 8785(JCS), 결과에 접두사 없음. 규칙 버전 `derive-v1`을 `derive_rule_version`에 기록. 실행 측은 `ExecutionRef.operation`과 Goal의 작업 종류가 같은지 검사 | D4·A3·C4 | |
| D-04 | QoS 프로필 | 2.4절의 STATE·EVENT·SENSOR 3종 | 전원 | |
| D-05 | 상태 Topic 주기·만료 | 변경 시 발행 Topic은 + 1 Hz 재발행·3.0 s, 안전 10 Hz·0.5 s, 구동 상태 20 Hz·0.3 s, 위치 신뢰성 5 Hz·1.0 s, 통로 관측 5 Hz·1.0 s, 부착 신호 20 Hz·0.3 s, 보유 판정 10 Hz·0.5 s, 적재 판정 10 Hz·0.5 s, 시간 유효성 1 Hz·3.0 s | 각 생산자 | |
| D-06 | 실제 정지 판정 | 선속도 ≤ 0.01 m/s, 각속도 ≤ 0.02 rad/s, 팔 관절 속도 ≤ 0.01 rad/s가 0.5 s 이상 유지 | A4·B4·D2 | |
| D-07 | 배정 확인·lease 시간 | 배정 확인 유효 시간 H = 2.0 s(요청 0.5 s마다), lease 상한 1.5 s(C 갱신 0.5 s마다) | A3·C4·D4 | |
| D-08 | 적재 LOST 판정 | 접촉이 끊겨도 영역 안이면 `UNKNOWN`. 접촉 끊김 1.0 s 이상 + 허용 영역 이탈이면 `LOST`. 허용 영역 치수는 A1 모델 확정 후 | A3·D2 | |
| D-09 | sim time 감시 | 측정 창 5 s, 허용 RTF 0.5~1.5, `/clock`이 3 s 진행하지 않으면 `PAUSED` | A4·D4 | |
| D-10 | 로컬 작업 종류 | `LOCK_HANDOFF`·`UNLOCK_HANDOFF`·`PICK_AND_STOW`·`UNLOAD_AND_PLACE`·`CONFIRM_LOAD`·`RETRACT_ARM` 6종 | A3·D2 | |
| D-11 | 계획 허용 operation | `NAVIGATE_TO`·`REQUEST_CORRIDOR`·`WAIT`·D-10의 6종 | D4·E1 | |
| D-12 | 구동 차단 사유 코드 | `SAFETY_STOP`·`HANDOFF_LOCK`·`PEER_STATE_STALE`·`CORRIDOR_NOT_GRANTED`·`ASSIGNMENT_INVALID`·`TIME_INVALID`·`SYSTEM_SAFE_STOP`·`FAULT_LOCK` | A4·D | |
| D-13 | 오류 코드 초기 목록 | 본설계 9.1절 코드 + `NEED_REOBSERVATION_MOVE`·`LEASE_EXPIRED`·`LEASE_REVOKED`·`ASSIGNMENT_STALE`·`DUPLICATE_REQUEST`·`BUDGET_EXHAUSTED`·`LOAD_UNKNOWN`·`LOAD_LOST`·`PEER_STATE_STALE`·`LEASE_RESYNC_UNCERTAIN`(재동기화 때 이전 세대의 시작 여부를 확인할 수 없음)·`LOCALIZATION_INVALID`·`TF_STALE`·`HANDOFF_LOCK_LOST`(수락 후 조건 상실, 5.2절)·`GOAL_MISMATCH`(부모 Goal 누락·불일치·종료)·`AUTHORITY_INVALID`(권한 종류와 필드 조합 오류). 본설계 9.1절의 적재 불명·상실 코드는 `LOAD_UNKNOWN`·`LOAD_LOST`, 인계 상대 상태 만료 코드는 `PEER_STATE_STALE`이다. 코드는 `ResultStatus.error_code` 등 문자열로 보내며 숫자 매핑은 두지 않음 | 전원 | |
| D-14 | 버전 관리 | `schema_version` 초깃값 1, 필드 추가·삭제·의미 변경 시 1 증가. 이 필드가 없는 타입은 `ax_interfaces` 패키지 버전(초깃값 0.1.0)과 ROS 2 타입 해시로 추적 | 전원 | |
| D-15 | 이름 일반화 | 물체 상태를 역할 이름(`HELD_BY_MANIPULATOR`·`STOWED_ON_TRANSPORTER`)으로, Agent 이동 제안을 `MOVE_TO_LOCATION`·`WAIT_AT_LOCATION`으로 | D2·D4·E | |
| D-16 | 계약에서 새로 정한 이름 | `source_session_id`·`a_session_id`·`d_session_id`·`sequence`·`lease_generation`, 목록 메시지 7종, `missing_sources`, `RECORDED_ONLY`, `ToolCommand`, `PlanCondition`, `RobotStopPose`, 인계 상태 `TRANSFER_*`, `TimeValidity`(`REVERSED`·`RTF_OUT_OF_RANGE` 등), `LocalizationHealth`(`OK`·`DEGRADED`·`LOST`), `RegisteredLocation` 상태, `FailureEvent.classification`, `ActuationState` `RUNNING`·`FAULT`, `object_hold_state`, `observation_reason`·`observation_target`, `failures/report`, 보조 Service `ReleaseCorridor`·`QueryBudget`·`ClearFault`·`RequestSafeStop`, `FailureEventList`, lease 요청 `kind`(`FIRST`·`RENEW`)·`c_session_id`, 작업 `priority`·`deferred`·`schedule_revision`, 계획 단계 `task_id`, 로컬 작업 `pick_pose`·`place_pose`, lease `goal_id`, 적재함 frame `robot_<n>/cargo_link`, 보호 팔 궤적 `ProtectedArmTrajectory`(`arm/execute_trajectory`), 명령 확인 `CheckCommandLease`(`executor/lease/check_command`)와 권한 종류 `AUTH_C_MANIPULATION`·`AUTH_A3_LOCAL_TASK`, 명령의 부모 `goal_id`, lease 철회 `goal_id`, snapshot `open_failures`, 계획 `task_ids`. 설계 숫자 `SafetyState.CLEAR=0` 등은 유지 | 전원 | |
| D-17 | 전체 계획 제출 경로 | 제안 종류가 아니라 별도 `AgentPlan` Topic(`/ax/fleet/agent/agent_central/plan`). 본설계 5.2·7.6절에 반영됨 | D4·E1·E3 | |
| D-18 | 운영 중 모드 상향 | D는 자동 상향하지 않는다. 운영자 상향 요청은 `SYSTEM_NORMAL`이고 진행 중 그룹 작업이 없으며(작업 경계) ASSISTED는 인증 검증 완료일 때만 수락. 그 밖에는 거절 | D4·E3 | |
| D-19 | 사유 코드 목록 | **안전(`SafetyState.reason_code`)**: `NONE`·`OBSTACLE_SLOWDOWN`·`OBSTACLE_STOP`·`SENSOR_STALE`·`SENSOR_INVALID`·`REAR_SENSOR_INVALID`·`LOCALIZATION_INVALID`. **Agent 결정(`AgentDecision.reason_code`)**: `OK`·`MODE_RECORD_ONLY`·`RECEIVER_NOT_ACTIVE`·`AUTH_FAILED`·`SCHEMA_INVALID`·`STALE_SNAPSHOT`·`STALE_GENERATION`·`TARGET_OUT_OF_SCOPE`·`PERMISSION_DENIED`·`EVIDENCE_MISMATCH`·`PRECONDITION_FAILED`·`BUDGET_EXHAUSTED`(접수 예산 소진)·`RATE_LIMITED`(빈도 초과). 그 밖의 `reason_code`는 D-12 구동 차단 사유 또는 D-13 오류 코드를 쓴다 | B4·D4·E3 | |

### 7.2 지켜야 할 규칙 (확인만)

본설계에서 이미 정한 내용이다. 구현과 계약 시험에서 지킨다.

**공통**
- 단위·프레임은 2.2절, 상태 머리는 `StateMeta`, 만료된 상태는 `UNKNOWN`으로 취급한다.
- 같은 `request_id`·`execution_id` 재전송은 새 동작을 만들지 않는다. 늦은 응답·이전 세션 응답은 버린다.
- 생산자는 모의 발행기·모의 서버를 제공하고, 계약 시험(정상·만료·중복·취소·권한 위반)을 통과해야 통합한다.

**운영·작업**
- 입고 사건 1건에 작업 1개. 같은 상자의 반복 관측은 새 작업이 아니다. 관측 출처(센서·정답 기반)를 구분한다.
- 물체 상태가 C 보유 판정·A 적재 판정과 모순되면 성공으로 처리하지 않는다.
- 통로 활성 허가는 1 이하. `OCCUPIED`·`BLOCKED_UNKNOWN`에서 신규 허가 없음. 점유 확인 없이 해제하지 않는다.
- 운영 중 모드 자동 상향 금지(운영자 상향은 D-18), 인증 검증 전 ASSISTED 금지. SAFE_STOP 해제는 개별 잠금 해제가 아니다.
- 실패는 사람 응답 없이 종료·격리·알림으로 기록하고 재시작 후에도 조회된다.

**안전·위치**
- 전체 정지와 상태 만료는 방향별 허가보다 우선한다. 후방 센서 이상이면 후진 금지.
- 정지 요청과 실제 정지를 구분한다. 위치 이상 점수는 진단 보조이며 안전 판단을 대체하지 않는다.
- 통로 완전 이탈 판정에 접힌 팔·적재물 외형을 포함한다. 재판정이 `UNKNOWN`이면 보류를 유지한다.

**로컬 실행·조작·적재**
- 로컬 작업에 주행은 없다. A·C는 D가 허용한 목록 밖의 `execution_id`를 만들지 않는다.
- 시작된 시도는 환불하지 않고 `UNCERTAIN` 시도는 예약을 유지한다.
- 배정 권한 기한 `tA + H`, lease 기한 `tC + L`, `0 < L <= min(상한, tA + H - 현재)`. heartbeat로 연장하지 않는다.
- 첫 물리 시도 전에 lease를 받는다. 철회와 Action cancel을 함께 보내고, 둘 다 없으면 lease 만료로 차단한다.
- 차체 이동이 필요하면 C는 Action을 끝내고 `need_reobservation_move`로 알린다.
- 부착 신호(A)와 보유 판정(C)을 나누고 fake 압력값을 보유 근거로 쓰지 않는다. 안전 정지 중 흡착 자동 해제 금지.
- 후보 좌표는 C가 계산한다. 후보 의견이 없거나 늦어도 기본 순위로 진행한다. 선호는 Goal 전송 시 ASSISTED·ACTIVE·이미 도착한 유효 제안일 때만 전달하고 실행 중 도착한 의견은 그 실행에 반영하지 않는다.

**Agent**
- snapshot의 누락 관측은 정상으로 채우지 않는다(표시 필드는 D-16).
- 발신 필드는 어댑터가 채우고 LLM이 만들지 않는다. 인증은 SROS2 권한과 D4 대조를 함께 쓴다.
- 로봇 Agent는 자기 로봇·하위 작업, 중앙은 배정된 그룹만 대상으로 한다. 계획에 좌표·관절·속도 명령을 넣지 않는다.
- ADVISORY·RECOVERY_SHADOW의 제안은 기록·검증만 하고 실행하지 않는다(결정 표기는 D-16). 권한·근거·대상·만료 위반은 발신자 위반으로 누적하고 기준을 넘으면 그 Agent만 격리한다. 형식 오류·지연은 FALLBACK이며 위반으로 세지 않고, 반복되면 일정 기간 사용 제한·재평가한다.
- `explanation`은 기록용이며 판정 근거로 쓰지 않는다.

### 7.3 승인 현황

| 묶음 | 생산자 | 소비자 | 상태 | 확정일 |
|---|---|---|---|---|
| 운영·작업(4.1) | D2·D3·D4·A3·A4·B3·B4·C1·C4·감독기 | A·B·C·D·E·관제 | 초안 | |
| 안전·위치(4.2) | A4·B2·B3·B4 | A4·B3·B4·C4·D | 초안 | |
| 로컬 실행·조작·적재(4.3) | A1·A2·A3·A4·B1·C3·C4·D4 | A3·A4·B1·C·D2·D4 | 초안 | |
| Agent(4.4) | D4·E3(E1·E2 출력은 E3 어댑터 경유) | D4·E | 초안 | |

작업 ID의 담당자는 [번호별 작업자 배정](../management/TEAM_SCHEDULE.md#번호별-작업자-배정)을 따른다.
이의 마감 후 결정 항목과 타입 필드 구성이 확정되면 네 묶음을 함께 "승인"으로 바꾸고 확정일을 적는다. "이의 없으면 승인"을 본설계 11절의 생산자·소비자 공동 승인으로 인정할지는 팀이 이 절차를 시작할 때 합의한다.

## 8. 세부 결정

결정 항목과 같은 방식(이의 없으면 이대로)으로 정한 세부 값이다. 구현은 이 값을 따르고, 숫자는 P2 실측 후 바꿀 수 있다.

**경로·조회**
- 주행 명령 경로: Nav2 → `/ax/robot_<n>/nav/cmd_vel` → A4 게이트 → `/ax/robot_<n>/diff_drive_controller/cmd_vel`(모두 `TwistStamped`, 4.3절). Nav2 입력 무수신·오래된 stamp 기준은 0.2 s이며 넘으면 0 속도.
- 관제 화면: 범용 조회 API는 두지 않는다. 상태는 4절의 STATE·EVENT Topic을 구독하고, 예산은 읽기 전용 `QueryBudget`, 실패 이력은 `/ax/fleet/failures/history`로 본다. 웹 화면이 필요하면 `rosbridge_server`로 중계하되 **읽기 전용 Topic만** 노출하고 Service와 쓰기 Topic(모드 설정·정지·고장 해제 등)은 노출하지 않는다. 정지 요청 버튼은 SROS2 권한을 가진 관제 노드가 `RequestSafeStop`을 호출한다.
- 보조 Service: 통로 해제 `ReleaseCorridor`(D3), 예산 조회 `QueryBudget`(D4), 고장 해제 `ClearFault`(A4, 시험 하네스·사후 조치 경로만 호출), 수동 전체 정지 `RequestSafeStop`(D4). 통로 조회는 `/ax/fleet/corridor/state` 구독으로 한다.

**크기·센서**
- `FleetSnapshot` 상한: 로봇 8, 그룹 작업 32, 물품 200, 등록 지점 100, 후보 묶음 로봇당 8, 보류 32, 미종료 실패 사건 32. 넘으면 진행 중 작업과 관련된 항목을 먼저 넣고 잘린 영역을 `missing_sources`에 `TRUNCATED:<영역>`으로 적는다. 로봇별 snapshot은 그 로봇과 배정 작업 관련 항목만 넣는다.
- 흡착 공구 ToF: 측정 범위 0.02~0.40 m, 30 Hz, frame `robot_<n>/tool_tof_link`. 범위 밖은 `sensor_msgs/Range` 규약대로 `+inf`(너무 멂)·`-inf`(너무 가까움)로 보낸다.
- TF 최신성(5.2절 적재함 pose 검사): 필요한 변환의 시각이 sim time 기준 1.0 s 이내. 개발용 출발값. 적재함 안 물체 이동 허용 오차는 센서·모델에 따라 시험 단계에서 정한다(미정).

**실행·예산**
- 로컬·주행 Action `TIMED_OUT`: D는 실패 사건을 기록하고 잠금을 유지한다. 재시도(`RETRY_PENDING`)는 보유·적재 판정이 확정이고, 본설계 9.7절의 원인 분류·검증된 복구 절차가 있으며, 본설계 8절 재개 전 확인(하위 Action 종료·실제 정지·인계 잠금)을 통과하고, 해당 예산이 남을 때만 한다. 예산은 조작이 `GRASP_ATTEMPT`, 일반 주행이 `NAV_RETRY`, 승인된 복구 주행이 `RECOVERY_MOVE`이며 서로 섞지 않는다(복구 주행의 시간초과는 `NAV_RETRY`를 쓰지 않는다). 예산이 소진되면 `FAILED`. 재시도 예산 종류가 없는 로컬 작업(`LOCK_HANDOFF`·`UNLOCK_HANDOFF`·`CONFIRM_LOAD`·`RETRACT_ARM`)은 재시도하지 않고 보류(본설계 9.6절)로 보낸다. 판정이 `UNKNOWN`이거나 Result 자체를 받지 못했으면 `UNCERTAIN`과 같이 보류로 보낸다. `TIMED_OUT`은 잠금 해제가 아니다.
- lease·명령 수락(5.2절): 재동기화 Goal당 2회, `CONTINUE` 확인 주기 0.5 s(D-07 갱신 주기와 같음), `valid_for_s`는 남은 lease 기한 이하. 모두 개발용 출발값.
- `task_revision`: 물체·지점·계획 단계 정의·배정이 바뀔 때 1 증가한다. 우선순위·연기는 `schedule_revision`을 올린다. 진행 상태 전이는 `GroupTaskList`의 `StateMeta.revision`만 올린다. A는 `ExecutionRef.task_revision`을 Goal 최초 수락 때만 대조하고, 같은 `execution_id` 재전송은 revision이 달라도 기존 실행에 연결한다(2.5절). 상충하는 일정 제안은 먼저 반영된 쪽이 `schedule_revision`을 올리므로 같은 근거의 다른 제안은 거절된다. 보류 검사는 revision 검사보다 우선하며, D는 보류 범위 로봇의 이동·대기 제안을 거절한다.
- `BudgetGrant.kind`: `GRASP_ATTEMPT`(집기 물리 시도)·`REOBSERVATION`(재관측 주기)·`RECOVERY_MOVE`(복구 주행)·`NAV_RETRY`(일반 주행 재시도) 4종. `time_budget_s` 0 이하는 무효 예산이다.
- `PlanStep.on_failure`: 실패 분기 `step_key` 또는 `FAIL_TERMINATE`(이 계획에 후속 분기가 없다는 표시). 실패 종료 여부는 계획이 아니라 D가 본설계 9.5·9.6절로 판정한다. `UNCERTAIN` 결과는 분기 대상이 아니며 보류로 간다. 비어 있으면 계획 등록 거절.

**값 목록·형식**
- `evidence_state_ids`: `state:<source_id>:<source_session_id>:<revision>` 또는 원장 ID(`hold-…`·`fail-…`·`rcpt-…`). 근거 ID는 제안이 가리키는 snapshot 안에 있어야 한다. 실패 사건은 `FleetSnapshot.open_failures`, 보류는 `holds`, 입고 사건은 재고 항목의 `receipt_event_id`로 확인한다. 없으면 거절한다.
- `HoldStatusList`: 종료된 보류는 60 s 동안 목록에 남기고, 그 뒤는 원장에만 둔다.
- `RegisteredLocation.purpose`: `RECEIPT`·`STORAGE`·`STATION`(작업대)·`HANDOFF`·`WAIT`·`SHIPPING` 6종. `allowed_object_classes`가 비면 모든 종류 배치 불가, 제한이 없으면 `"*"` 하나.
- `RobotSummary.availability`: `AVAILABLE`·`BUSY`·`FAULT`·`STANDBY_SPARE` 4종. 빈 문자열은 가용이 아니다. `role`은 `MANIPULATOR`·`TRANSPORTER`.
- `ManipulateObject.target_pose`: 작업별 PICK·PLACE 대상과 프레임·최신성 검사는 5.2절 표를 따른다.
- `LocalizationHealth.anomaly_score`: 0~1 정규화, 클수록 이상. 모델 불가면 0이고 `anomaly_model_available=false`.
- `GraspCandidate.success_probability`: 0~1, 모델 불가면 -1이고 `rule_score`로만 순위를 정한다.
- `AgentProposal.priority`: 1~9, 클수록 먼저. 해당 없는 제안은 0.

**Agent·상태**
- 계획 단계 소속: `PlanStep.task_id`는 `AgentPlan.task_ids` 중 하나여야 한다. 여러 작업에 걸친 대기·이동 단계는 그 단계를 수행하는 로봇이 배정된 작업에 넣고, 실행 ID·예산은 그 작업에 귀속한다. 예비 로봇이 이어받는 단계는 기존 작업의 실행 문맥(`root_execution_id`)·복구 문맥(`recovery_context_id`)·예산을 승계하며 새 예산을 만들지 않는다.
- `agent_receive_generation`: 격리 진입과 ACTIVE 복귀 때 1 증가하고, 이전 세대의 제안·계획은 거절한다. `RECOVERY_SHADOW` 중 제안은 격리 진입 때 발급한 세대를 쓰며 기록·검증만 한다. ACTIVE 복귀 뒤에는 그 세대의 제안을 거절한다.
- `AgentReasoningState`: D의 `RECORDED_ONLY` 결정을 받으면 `IDLE`로 돌아간다.
- `snapshot_id`: D가 마지막 값을 원장에 저장해 재시작 후에도 이어 증가한다. 내용이 같은 재발행은 같은 ID다.
- 인계 잠금 해제(`TRANSFER_UNLOCKED`): 적재는 적재 확인·흡착 해제·팔 후퇴 확인 후. 하역은 배치 확인·흡착 해제·팔 후퇴·운송 로봇 적재함 `EMPTY` 확인 후.

## 9. 구현 시 계약 시험 항목

아래 항목은 노드를 구현할 때 모의 상대와 함께 수행할 계약 시험이다. 타입 빌드 통과는 이 규칙들이 실제로 지켜진다는 뜻이 아니다.

| 시험 | 확인할 것 | 관련 절 |
|---|---|---|
| 명령 수락과 철회의 경합 | A3가 철회를 표시한 뒤 도착한 `CheckCommandLease`가 모두 거절되고, 철회 전 허용분은 수락 기록으로 남는지 | 5.2 |
| 지연·유실·중복 응답 | `FIRST`·`RENEW`·`CheckCommandLease` 응답의 지연·유실·중복에서 세대가 한 번만 오르고, 늦은 허용 응답이 기한 뒤 실행되지 않으며, RENEW 유실 시 기존 기한에 멈추는지 | 5.2 |
| 같은 실행 ID의 새 Goal·인자 변경 | 새 `goal_id`라도 재동기화 절차를 거치고, C4가 활성 실행을 중복 실행하지 않으며, 인자가 다른 재요청을 거절하는지 | 2.5, 5.2 |
| A3 무응답·재시작 | A2·A4가 명령을 실행하지 않고, A4 보호 정지가 A3 없이 동작하며, 흡착이 유지되고, 재시작 뒤 원장을 신뢰할 수 없으면 `LEASE_RESYNC_UNCERTAIN`이 되는지 | 5.2 |
| 부모 Goal UUID 검사 | `goal_id` 누락·형식 오류·다른 Goal·종료된 Goal·이전 세대 명령이 `GOAL_MISMATCH`·`LEASE_REVOKED` 등으로 거절되는지 | 5.2 |
| A3 로컬 후퇴 승인 | 현재 `RETRACT_ARM` 실행의 후퇴는 허용되고, 다른 작업 종류·다른 배정·종료된 실행·`goal_id`가 채워진 요청은 거절되는지 | 5.2 |
| A3 무응답 때 후퇴와 보호 정지 | A3가 응답하지 않으면 일반 후퇴는 실행되지 않고, A4 보호 정지는 동작하며 흡착이 유지되는지 | 5.2 |
| 재전송의 최초 기한 유지 | 같은 `request_id` 재전송이 `t_first`를 바꾸지 않고, 기한은 `t_first + valid_for_s`로 유지되는지 | 5.2 |
| 기한 뒤 캐시 응답 폐기 | 기한이 지난 캐시된 `allowed=true`로 명령이 실행되지 않고, 재시작으로 `t_first`를 잃으면 과거 응답을 쓰지 않는지 | 5.2 |
| 새 확인 요청의 중복 실행 방지 | 새 `request_id`로 다시 확인해도 같은 `command_id`의 물리 명령이 다시 실행되지 않고 수락 기록이 하나뿐인지 | 5.2 |
| 정상 `IN_PROGRESS`의 PLACE | 같은 작업·인계·배정의 `TRANSFER_IN_PROGRESS`에서 적재함 PLACE가 통과하는지 | 5.2 |
| 이전 인계·만료 TF·위치 손실 차단 | 다른 `transfer_id`·이전 배정, TF 1.0 s 초과, 자기·상대 위치 `DEGRADED`·`LOST`에서 시작이 거절되고, 수락 후에는 조건 상실 절차로 끝나는지 | 4.3, 5.2 |
