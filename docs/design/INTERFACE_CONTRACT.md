# AX 공통 인터페이스 계약 초안 (K0)

> 상태: **초안**. 생산자·소비자 승인 전이며 구현·동작을 뜻하지 않는다.
> 기준 설계는 [통합 상세설계](DETAILED_DESIGN.md) 6·7절, 일정은 [팀 공통 일정](../management/TEAM_SCHEDULE.md)의 K0(W0~W1, 승인 11-08)이다.
> 타입 정의는 `src/ax_interfaces/`의 `.msg`·`.srv`·`.action`이며, 이 문서는 이름·방향·QoS·최신성·소유 작업을 정한다.

## 1. 범위와 승인 절차

- 이 문서는 기능 A~E 사이의 공유 Topic·Service·Action과 공통 타입을 정한다. 기능 내부 노드 사이의 통신과 Gazebo·Nav2·ros2_control 표준 인터페이스는 해당 담당이 정한다.
- 각 인터페이스는 **생산자와 소비자가 함께 승인**한다(본설계 11절). 승인은 7절의 "이의 없으면 승인" 방식으로 한다. 승인 전에는 모의 상대와 계약 시험으로 개발한다.
- 승인 후 의미가 바뀌면 해당 메시지의 `schema_version`을 올린다. 공개한 enum 숫자는 다른 의미로 재사용하지 않는다.
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
| `hold_id`·`failure_event_id`·`recovery_context_id`·`budget_grant_id` | `hold-…`·`fail-…`·`rctx-…`·`bgt-…` | D | 원장 영속 대상 |
| `sender_agent_id` | `agent_central`·`agent_<robot_id>`(예: `agent_robot_1`) | Agent 어댑터 | LLM이 채우지 않음. Topic 경로에 쓰므로 소문자·숫자·`_`만 사용 |
| `request_id`·`proposal_id`·`plan_id` | `<prefix>-<UUIDv4>` | 요청자 | 재전송은 같은 ID로 멱등 처리 |

메시지 필드의 문자열 ID는 영문·숫자·`_ - . : @`만 쓴다. ROS Topic·노드·namespace 이름에 들어가는 ID(`robot_id`, `team_id`, `sender_agent_id`, 조건 소유 영역)는 소문자·숫자·`_`만 쓴다. ID를 파싱해 의미를 꺼내지 않는다.

### 2.2 단위·좌표·시계

- 거리 m, 속도 m/s, 각도 rad, 각속도 rad/s, 질량 kg, 시간 s.
- 공유 위치는 `map` 기준 `geometry_msgs/PoseStamped`. 로봇 프레임은 `robot_<n>/odom`, `robot_<n>/base_link`. 공구 목표는 `epick_tcp` 기준.
- `StateMeta.header.stamp`는 ROS sim time(`use_sim_time=true`)이다. 통신 만료·lease·예산은 수신 측이 `CLOCK_BOOTTIME`으로 직접 잰다. 다른 프로세스의 단조 시계 값을 메시지로 비교하지 않는다(본설계 6·7.6절).

### 2.3 상태 메시지 공통 머리 `StateMeta`

모든 상태 Topic 메시지는 `StateMeta`를 첫 필드로 가진다. 목록 메시지(`GroupTaskList` 등)는 목록 전체에 하나를 두고, 목록 안 항목과 `FleetSnapshot` 안 항목은 바깥 `StateMeta`의 유효성을 따른다.

| 필드 | 규칙 |
|---|---|
| `header.stamp` | 관측·판정 시각(sim time). 소비자는 표본 나이로 최신성을 검사 |
| `schema_version` | 메시지 계약 버전 |
| `source_id` | 발행 노드 식별자 |
| `source_session_id` | 생산자 프로세스가 시작할 때마다 새로 발급. 세션이 바뀌면 소비자는 revision 비교를 새로 시작 |
| `revision` | 같은 세션에서 상태 변경마다 1 증가. 0은 발행 전 |
| `valid` / `validity_reason` | 생산자의 유효성 판단. `false`는 정상·성공으로 해석하지 않음 |

소비자 규칙:

- (`source_session_id`, `revision`)으로 비교한다. 같은 세션에서 revision이 줄면 버린다. 같은 revision이 다시 오면 내용은 갱신하지 않지만 **수신 시각은 갱신**한다(주기 재발행).
- **표본 나이(sim time)와 무수신 시간(CLOCK_BOOTTIME) 둘 다** 검사한다. 만료된 상태는 `UNKNOWN`으로 취급한다.
- `valid=false`, `revision=0`, 받은 적 없는 상태는 `UNKNOWN`이다. 본설계 7.2·7.3절이 숫자를 정한 `SafetyState.CLEAR=0`, `CorridorState.FREE=0`, `CorridorObservation.OUTSIDE=0`은 이 규칙으로만 보호된다.
- enum 값이 `UNKNOWN`·`UNSPECIFIED`(0)이면 정상·성공·허용으로 해석하지 않는다. Goal의 `UNSPECIFIED`는 거절한다.

### 2.4 QoS 프로필

| 프로필 | 설정 | 용도 |
|---|---|---|
| `STATE` | reliable · transient_local · keep_last 1 | 최신 상태가 중요한 Topic. 늦게 접속한 노드도 마지막 값을 받음. 대상이 여러 개면 목록 메시지 하나로 보낸다 |
| `EVENT` | reliable · volatile · keep_last 50 | 관측 사건·제안·결정·lease 메시지 |
| `SENSOR` | best_effort · volatile · keep_last 5 | 센서 데이터(흡착 공구 ToF 등 표준 타입) |

### 2.5 결과와 멱등성

- Action 결과는 `ResultStatus.outcome`으로 `SUCCEEDED / FAILED / CANCELED / TIMED_OUT / UNCERTAIN`을 구분한다. `UNCERTAIN`과 채우지 않은 `UNSPECIFIED`는 성공으로 처리하지 않고 9.6절 수명주기로 보낸다.
- Service 응답의 수락(`accepted`·`granted`)은 실제 동작 완료가 아니다.
- 같은 `request_id`·`execution_id` 재전송은 새 동작을 만들지 않고 기존 결과를 돌려준다. Action goal UUID는 통신 식별자일 뿐이며 `execution_id`와 연결해 기록한다.

## 3. 공통 타입

| 타입 | 내용 | 주 사용처 |
|---|---|---|
| `StateMeta` | 상태 출처·세션·버전·유효성 | 모든 상태 Topic |
| `ExecutionRef` | 팀·작업·배정·인계·실행 ID 묶음 | Action Goal, 실행 상태 |
| `BudgetGrant` | D가 위임한 재시도 예산과 허용 하위 ID | ExecuteLocalTask·ManipulateObject Goal |
| `AttemptUsage` | 물리 시도 1회 사용 기록(시작·종료·미확정) | Action Feedback·Result |
| `ResultStatus` | 결과 4종 + 미확정, 오류 코드 | Action Result, 로컬 실행 상태 |
| 목록 메시지 | `GroupTaskList`·`CorridorStateList`·`HoldStatusList`·`AgentReceiveStateList` | 대상이 여러 개인 STATE Topic |
| `PlanCondition`·`RobotStopPose` | 계획 전제조건(소유 영역·판정식), 지점의 역할별 정차 자세 | `PlanStep`, `RegisteredLocation` |

## 4. 인터페이스 목록

`<n>`은 로봇 번호다. "작업"은 [팀 공통 일정](../management/TEAM_SCHEDULE.md)의 작업 ID로, 생산자 쪽 담당 작업이다.

### 4.1 운영·작업 (D 중심)

| 이름 | 형태 · 타입 | 생산자 → 소비자 | QoS · 주기 | 작업 |
|---|---|---|---|---|
| `/ax/fleet/mode` | Topic · `OperationMode` | D → 전체 | STATE · 변경 시 + 1 Hz | D4 |
| `/ax/fleet/mode/set` | Service · `SetOperationMode` | 운영자·시험 하네스 → D | — | D4 |
| `/ax/fleet/tasks` | Topic · `GroupTaskList` | D → A·E·관제 | STATE · 변경 시 + 1 Hz | D2 |
| `/ax/fleet/tasks/register` | Service · `RegisterTask` | D 입고 처리·운영자 → D | — | D2 |
| `/ax/fleet/inventory` | Topic · `InventorySnapshot` | D → E·관제 | STATE · 변경 시 | D2 |
| `/ax/fleet/receipt/observations` | Topic · `ReceiptObservation` | B3·C1 → D | EVENT · 관측 시 | B3, C1 |
| `/ax/fleet/holds` | Topic · `HoldStatusList` | D → 관제·E | STATE · 변경 시 + 1 Hz | D4 |
| `/ax/fleet/failures` | Topic · `FailureEvent` | D → 관제·E | EVENT · 발생 시(영속 원장이 기준) | D4 |
| `/ax/fleet/time_validity` | Topic · `TimeValidity` | A 공용 감독기 → D·A | STATE · 1 Hz · 3.0 s | A4 |
| `/ax/fleet/corridor/state` | Topic · `CorridorStateList` | D → A·B·E | STATE · 변경 시 + 1 Hz | D3 |
| `/ax/fleet/corridor/request_entry` | Service · `RequestCorridorEntry` | D 주행 관리자 → D 통로 관리자 | — | D3 |
| `/ax/fleet/conditions/<a·b·c·d>/recheck` | Service · `RecheckCondition` | D → 조건 소유 영역(A·B·C·D 각 서버) | — | A4·B4·C4·D4 |

### 4.2 로봇 안전·위치 (A·B)

| 이름 | 형태 · 타입 | 생산자 → 소비자 | QoS · 주기 · 만료 초안 | 작업 |
|---|---|---|---|---|
| `/ax/robot_<n>/safety/state` | Topic · `SafetyState` | B4 → A4·D | STATE · 10 Hz · 0.5 s | B4 |
| `/ax/robot_<n>/safety/actuation_state` | Topic · `ActuationState` | A4 → B4·D·**상대 로봇 A4** | STATE · 20 Hz · 0.3 s | A4 |
| `/ax/robot_<n>/localization/health` | Topic · `LocalizationHealth` | B2 → A4·D | STATE · 5 Hz · 1.0 s | B2 |
| `/ax/robot_<n>/localization/reinitialize` | Service · `std_srvs/Trigger` | D → B2 | 접수만 응답, 완료는 `localization/health`로 확인 | B2 |
| `/ax/robot_<n>/perception/corridor_observation` | Topic · `CorridorObservation` | B3 → D3 | STATE · 5 Hz · 1.0 s | B3 |

### 4.3 로컬 실행·조작·적재 (A·C)

| 이름 | 형태 · 타입 | 생산자 → 소비자 | QoS · 주기 | 작업 |
|---|---|---|---|---|
| `/ax/robot_<n>/executor/execute_local_task` | Action · `ExecuteLocalTask` | D2(클라이언트) → A3(서버) | Action 기본 | A3 |
| `/ax/robot_<n>/executor/status` | Topic · `RobotTaskStatus` | A3 → D2 | STATE · 변경 시 + 1 Hz | A3 |
| `/ax/robot_<n>/executor/assignment_check/request` | Topic · `AssignmentCheckRequest` | A3 → D4 | EVENT · lease 갱신 주기 | A3 |
| `/ax/robot_<n>/executor/assignment_check/response` | Topic · `AssignmentCheckResponse` | D4 → A3 | EVENT · 요청 시 | D4 |
| `/ax/robot_<n>/manipulation/execute` | Action · `ManipulateObject` | A3(클라이언트) → C4(서버) | Action 기본 | C4 |
| `/ax/robot_<n>/manipulation/lease/request` | Topic · `ExecutionLeaseRequest` | C4 → A3 | EVENT · 첫 시도 전·갱신 시 | C4 |
| `/ax/robot_<n>/manipulation/lease/grant` | Topic · `ExecutionLeaseGrant` | A3 → C4 | EVENT · 요청 시 | A3 |
| `/ax/robot_<n>/manipulation/lease/revoke` | Topic · `ExecutionLeaseRevoke` | A3 → C4 | EVENT · 철회 시(Action cancel과 함께) | A3 |
| `/ax/robot_<n>/manipulation/tool_command` | Service · `ToolCommand` | C4 → A2(A의 허용 검사 포함) | 수락은 부착·해제 완료 아님 | A2 |
| `/ax/robot_<n>/manipulation/attach_state` | Topic · `AttachState` | A2 → C4 | STATE · 20 Hz · 0.3 s | A2 |
| `/ax/robot_<n>/arm/follow_joint_trajectory` | Action · `control_msgs/FollowJointTrajectory` | C4(클라이언트) → A4 보호 경로(서버) | Action 기본 | A4 |
| `/ax/robot_<n>/sensors/tool_tof` | Topic · `sensor_msgs/Range` | A1(센서 모델) → C4 | SENSOR · 갱신율은 A1 모델 기준 · frame `epick_tcp` 근처 공구 프레임 | A1 |
| `/ax/robot_<n>/manipulation/hold_state` | Topic · `ToolHoldState` | C4 → D2 | STATE · 10 Hz · 0.5 s | C4 |
| `/ax/robot_<n>/manipulation/candidates` | Topic · `CandidateSet` | C3 → D4 | STATE · 관측 갱신 시 | C3 |
| `/ax/robot_<n>/load/state` | Topic · `LoadState` | A3(운송 로봇) → D2·**상대 로봇 A4** | STATE · 10 Hz · 0.5 s | A3 |

인계 중 상호 배제(DEC-19)를 위해 각 로봇의 A4는 상대 로봇의 `actuation_state`·`load_state`와 `/ax/fleet/tasks`의 인계 상태를 구독하고, 만료되면 `PEER_STATE_STALE`로 주행을 막는다.
흡착 공구 ToF 값은 접촉·흡착·보유 판정이 아니다(본설계 3.1절).

주행은 D1이 로봇별 Nav2 `NavigateToPose` Action을 직접 호출한다. Nav2의 속도 출력은 A4 최종 구동 게이트를 거친 뒤 ros2_control 제어기로 간다. 이 경로의 Topic 이름은 A2·A4·D1이 P2에서 정한다.

### 4.4 Agent (E ↔ D)

| 이름 | 형태 · 타입 | 생산자 → 소비자 | QoS · 주기 | 작업 |
|---|---|---|---|---|
| `/ax/fleet/agent/snapshot` | Topic · `FleetSnapshot` | D4 → E1·E2 | STATE · 변경 시(최대 2 Hz) | D4 |
| `/ax/fleet/agent/<sender_agent_id>/proposal` | Topic · `AgentProposal` | Agent 어댑터(E3) → D4 | EVENT · 제안 시 | E3 |
| `/ax/fleet/agent/agent_central/plan` | Topic · `AgentPlan` | 중앙 Agent 어댑터(E3) → D4 | EVENT · 계획 제출 시 | E1, E3 |
| `/ax/fleet/agent/decision` | Topic · `AgentDecision` | D4 → E·로그 | EVENT · 결정 시 | D4 |
| `/ax/fleet/agent/receive_state` | Topic · `AgentReceiveStateList` | D4 → 관제·E | STATE · 변경 시 + 1 Hz | D4 |
| `/ax/fleet/agent/<sender_agent_id>/reasoning_state` | Topic · `AgentReasoningState` | E → D4·관제 | STATE · 변경 시 | E1, E2 |

전체 계획은 `/plan` Topic의 `AgentPlan`으로만 제출한다(제안 종류에 계획 제출은 없다). 경로를 Agent별로 나눠도 발신자 인증을 대신하지 않는다. SROS2 권한(A4)과 D4의 `sender_agent_id`·세션·세대·대상 대조를 함께 쓴다(본설계 5.1·7.6절).

## 5. Action·Service 계약

### 5.1 `ExecuteLocalTask` (D → A)

- **Goal:** `ExecutionRef`, 작업 종류(`LOCK_HANDOFF`·`UNLOCK_HANDOFF`·`PICK_AND_STOW`·`UNLOAD_AND_PLACE`·`CONFIRM_LOAD`·`RETRACT_ARM`, 0은 `UNSPECIFIED`로 거절), 물체 ID와 재고 revision, 출발·도착 지점 ID와 각 revision, 지도 revision, `BudgetGrant[]`, 선택적 후보 선호.
- **후보 선호 전달 조건(본설계 7.13절):** D는 Goal 전송 시 ASSISTED이고 해당 Agent 수신 상태가 ACTIVE이며 이미 도착한 유효 제안이 있을 때만 `has_candidate_preference=true`로 넣는다. 의견을 기다리지 않으며, 실행 중 도착한 의견은 그 실행에 반영하지 않는다.
- **Feedback:** 단계, 진행률, 시도별 사용 기록.
- **Result:** `ResultStatus`, 보유·적재 상태, 시도 사용 기록, `need_reobservation_move`와 요구 지점.
- 주행은 포함하지 않는다. A는 수락 전에 `execution_id`를 원장에 기록하고 같은 ID 재요청에는 기존 결과를 돌려준다.

### 5.2 `ManipulateObject` (A → C)

- **Goal:** `ExecutionRef`, 물체, `PICK`·`PLACE`(0은 `UNSPECIFIED`로 거절), 목표 pose, `BudgetGrant`, lease 세대, 선택적 후보 선호. Goal만으로 물리 시도를 허가하지 않는다.
- **물리 시도 전:** C가 `lease/request`를 보내고 A의 `lease/grant`를 받는다. lease 기한은 C의 요청 시각 `tC + lease_s`, `0 < lease_s <= min(상한, tA + H - 현재)`.
- **중단:** A는 `lease/revoke`와 Action cancel을 함께 보낸다. 둘 다 오지 않아도 lease 만료가 같은 차단을 한다.
- **Result:** `ResultStatus`, `object_held`, 시도 수·사용 기록, `need_reobservation_move`. 차체 이동이 필요하면 C는 Action을 끝내고 이 값을 돌려준다.
- **팔 동작과 공구:** C4는 팔 궤적을 A4 보호 경로의 `arm/follow_joint_trajectory`로, 흡착·해제를 `tool_command`로 요청한다. 실제 부착은 `attach_state`, 보유 판정은 C4의 `hold_state`로 확인한다.

### 5.3 배정 유효성 확인 (A ↔ D)

A가 `assignment_check/request`를 보내고 요청 시각 `tA`를 기록한다. D 응답의 `validity_s`(H)로 권한 기한 `tA + H`를 계산한다. 늦게 온 응답, 이전 세션·세대 응답, 중복 응답은 버린다. 일반 상태 heartbeat로는 기한을 늘리지 않는다.

### 5.4 Service

| Service | 요청 | 응답 | 규칙 |
|---|---|---|---|
| `RequestCorridorEntry` | 통로·로봇·request_id | 허가·grant_id·사유·revision | 같은 request_id는 같은 응답. 점유 확인 없이 해제 불가 |
| `RegisterTask` | request_id·입고 사건·물체·출발/도착 지점 | 수락·task_id·사유 | 입고 사건당 작업 1개. 도착 지점이 비면 D 규칙 또는 중앙 계획 |
| `RecheckCondition` | hold·조건·로봇 | 충족/불충족/불명·근거 상태 ID | 계획 조건의 `condition_owner` 영역 서버가 판정. 응답 0(UNKNOWN)이면 보류 유지. LLM 판단으로 대신하지 않음 |
| `ToolCommand` | request_id·로봇·실행 ID·물체·부착/해제 | 수락·사유 | A의 허용 검사 후 수락. 수락은 부착·해제 완료가 아님. 같은 request_id는 같은 응답 |
| `SetOperationMode` | 모드·사유 | 수락·사유 | 시작 전 설정이 원칙. 운영 중 상향 요청과 인증 검증 전 ASSISTED는 거절. 장애 시 하향은 D가 자동 수행 |

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
| `HoldStatus.state` | HOLD_PENDING · HELD · RECHECK · RELEASED · FAILED_ISOLATED | 9.6 |
| `OperationMode` | OFF(0) · ADVISORY · ASSISTED / SYSTEM_UNKNOWN(0) · SYSTEM_NORMAL · SYSTEM_SAFE_STOP | 5.1 · 9.5 |
| `AgentReceiveState.state` | UNKNOWN(0) · ACTIVE · QUARANTINED · RECOVERY_SHADOW | 5.3 |
| `AgentReasoningState.state` | DISABLED · IDLE · REASONING · VALIDATING · ACCEPTED · REJECTED · FALLBACK | 8 |
| `AgentProposal.action` | NO_ACTION · ASSIGN_TASK · DEFER_TASK · RECOMMEND_RETRY · RECOMMEND_REASSIGN · RECOMMEND_PRIORITY_CHANGE · REQUEST_HOLD · REQUEST_RESUME_REVIEW · PREFER_CANDIDATE · MOVE_TO_LOCATION · WAIT_AT_LOCATION | 5.2 · 7.6 |

그 밖에 `ResultStatus`·`AgentDecision`·`AttemptUsage`·`FailureEvent`·`ToolCommand`·Goal 작업 종류는 0이 `UNSPECIFIED`, `LocalizationHealth`·`TimeValidity`·`RegisteredLocation`·`RecheckCondition` 응답·`LoadState.source`·`ReceiptObservation.source`는 0이 `UNKNOWN`이다.

설계와 이름이 달라진 필드: `SafetyState.recommended_max_linear_speed` → `max_linear_speed`(각속도 상한 `max_angular_speed` 추가), `CorridorState.revision` → `corridor_revision`, 그룹 물체 상태 `HELD_BY_ROBOT_1`·`STOWED_ON_ROBOT_2` → 역할 이름(D-15).

## 7. 승인 방식과 결정 항목

### 7.1 승인 방식 — 이의 없으면 승인

- 팀이 정할 것은 아래 **결정 항목 16개**와 `src/ax_interfaces` 타입의 필드 구성이다. 각 결정 항목에는 추천값이 있고, 필드 구성은 4~6절이 설명한다.
- **이의 마감 11-06(금)**: 그때까지 이의가 없으면 추천값으로 확정하고 **11-08(일)** K0 승인으로 기록한다.
- 이의가 있으면 "이의" 칸에 작업 ID와 대안을 적는다. 이의가 있는 항목만 30분 회의로 정한다.
- 숫자 값은 출발값이다. P2 실측(11-22까지) 결과로 바꾸며, 바꿀 때는 이 표와 타입을 함께 고치고 관련 생산자·소비자에게 알린다.
- 7.2절 규칙은 이미 설계에서 정해진 내용이므로 정하지 않고 지키기만 한다.

| ID | 결정 항목 | 추천값 | 관련 작업 | 이의 |
|---|---|---|---|---|
| D-01 | 인터페이스 이름 | 4절 표의 이름 그대로. 로봇별은 `/ax/robot_<n>/…`, 공용은 `/ax/fleet/…` | 전원 | |
| D-02 | ID 형식 | 2.1절 표 그대로(`접두사-UUIDv4`, `robot_<n>`, `team_<n>`) | D2·D4 | |
| D-03 | `execution_id` 생성 규칙 | 키 정렬·공백 없는 UTF-8 JSON `{attempt_index, operation, root_execution_id, step_key}`의 SHA-256 **소문자** 16진수 앞 32자. `operation`은 Goal enum 상수 이름 문자열(예: `PICK_AND_STOW`, `PICK`), `attempt_index`는 정수. 규칙 버전 `derive-v1`을 `derive_rule_version`에 기록 | D4·A3·C4 | |
| D-04 | QoS 프로필 | 2.4절의 STATE·EVENT·SENSOR 3종 | 전원 | |
| D-05 | 상태 Topic 주기·만료 | 안전 10 Hz·0.5 s, 구동 상태 20 Hz·0.3 s, 위치 신뢰성 5 Hz·1.0 s, 통로 관측 5 Hz·1.0 s, 부착 신호 20 Hz·0.3 s, 보유 판정 10 Hz·0.5 s, 적재 판정 10 Hz·0.5 s, 시간 유효성 1 Hz·3.0 s | 각 생산자 | |
| D-06 | 실제 정지 판정 | 선속도 ≤ 0.01 m/s, 각속도 ≤ 0.02 rad/s, 팔 관절 속도 ≤ 0.01 rad/s가 0.5 s 이상 유지 | A4·B4·D2 | |
| D-07 | 배정 확인·lease 시간 | 배정 확인 유효 시간 H = 2.0 s(요청 0.5 s마다), lease 상한 1.5 s(C 갱신 0.5 s마다) | A3·C4·D4 | |
| D-08 | 적재 LOST 판정 | 접촉이 끊겨도 영역 안이면 `UNKNOWN`. 접촉 끊김 1.0 s 이상 + 허용 영역 이탈이면 `LOST`. 허용 영역 치수는 A1 모델 확정 후 | A3·D2 | |
| D-09 | sim time 감시 | 측정 창 5 s, 허용 RTF 0.5~1.5, `/clock`이 3 s 진행하지 않으면 `PAUSED` | A4·D4 | |
| D-10 | 로컬 작업 종류 | `LOCK_HANDOFF`·`UNLOCK_HANDOFF`·`PICK_AND_STOW`·`UNLOAD_AND_PLACE`·`CONFIRM_LOAD`·`RETRACT_ARM` 6종 | A3·D2 | |
| D-11 | 계획 허용 operation | `NAVIGATE_TO`·`REQUEST_CORRIDOR`·`WAIT`·D-10의 6종 | D4·E1 | |
| D-12 | 구동 차단 사유 코드 | `SAFETY_STOP`·`HANDOFF_LOCK`·`PEER_STATE_STALE`·`CORRIDOR_NOT_GRANTED`·`ASSIGNMENT_INVALID`·`TIME_INVALID`·`SYSTEM_SAFE_STOP`·`FAULT_LOCK` | A4·D | |
| D-13 | 오류 코드 초기 목록 | 본설계 9.1절 코드 + `NEED_REOBSERVATION_MOVE`·`LEASE_EXPIRED`·`LEASE_REVOKED`·`ASSIGNMENT_STALE`·`DUPLICATE_REQUEST`·`BUDGET_EXHAUSTED`·`LOAD_UNKNOWN`·`LOAD_LOST`·`PEER_STATE_STALE`. 9.1의 "코드 이름 미정" 두 항목은 적재 `LOAD_UNKNOWN`·`LOAD_LOST`, 인계 상대 상태 만료 `PEER_STATE_STALE`로 정함 | 전원 | |
| D-14 | `schema_version` | 초깃값 1. 필드 추가·삭제·의미 변경 시 1 증가 | 전원 | |
| D-15 | 이름 일반화 | 물체 상태를 역할 이름(`HELD_BY_MANIPULATOR`·`STOWED_ON_TRANSPORTER`)으로, Agent 이동 제안을 `MOVE_TO_LOCATION`·`WAIT_AT_LOCATION`으로 | D2·D4·E | |
| D-16 | 계약에서 새로 정한 이름 | `source_session_id`(상태 세션), 목록 메시지 4종, `missing_sources`(snapshot 누락 출처), `RECORDED_ONLY`(ADVISORY·shadow 결정), `ToolCommand`, `PlanCondition`, `RobotStopPose`. 설계 숫자 `SafetyState.CLEAR=0` 등은 유지 | 전원 | |

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
- 운영 중 모드 자동 상향 금지, 인증 검증 전 ASSISTED 금지. SAFE_STOP 해제는 개별 잠금 해제가 아니다.
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
- ADVISORY·RECOVERY_SHADOW의 제안은 기록·검증만 하고 실행하지 않는다(결정 표기는 D-16). 권한·근거·대상·만료 위반은 발신자 위반으로 누적하고 기준을 넘으면 그 Agent만 격리한다. 형식 오류·지연은 FALLBACK이며 위반으로 세지 않는다.
- `explanation`은 기록용이며 판정 근거로 쓰지 않는다.

### 7.3 승인 현황

| 묶음 | 생산자 | 소비자 | 상태 | 확정일 |
|---|---|---|---|---|
| 운영·작업(4.1) | D2·D3·D4·A4·B3·C1 | A3·D·E·관제 | 초안 | |
| 안전·위치(4.2) | A4·B2·B3·B4 | A4·D | 초안 | |
| 로컬 실행·조작·적재(4.3) | A2·A3·C3·C4·D4 | A3·C4·D2·D4 | 초안 | |
| Agent(4.4) | D4·E1·E2·E3 | D4·E | 초안 | |

작업 ID의 담당자는 [번호별 작업자 배정](../management/TEAM_SCHEDULE.md#번호별-작업자-배정)을 따른다.
이의 마감 후 결정 항목과 타입 필드 구성이 확정되면 네 묶음을 함께 "승인"으로 바꾸고 확정일을 적는다. "이의 없으면 승인"을 본설계 11절의 생산자·소비자 공동 승인으로 인정할지는 팀이 이 절차를 시작할 때 합의한다.

## 8. 미정 항목

결정 항목에 넣지 않은 것으로, 해당 단계에서 정한다.

- 오류 코드의 숫자 enum 매핑(코드 이름은 D-13).
- Nav2 속도 출력 → A4 게이트 → 제어기 경로의 Topic 이름(P2, A2·A4·D1).
- 관제 화면 조회 API(Service 또는 별도 브리지) 형태(P5, D2).
- `FleetSnapshot` 크기 상한과 분할 여부, 목록·배열 필드의 최대 길이(로봇·물품 수 증가 시, D4).
- 일정표에 있는 보조 Service: D3 통로 조회·해제 요청, D4 상태·예산 조회, A4 조건부 고장 해제(P1 계약 시험 전, 각 담당).
- 흡착 공구 ToF의 측정 범위·갱신율·frame 이름(A1 모델 확정 시).
- Action `TIMED_OUT`과 그룹 작업 상태의 대응(본설계 15절, D2).
