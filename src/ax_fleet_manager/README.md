# ax_fleet_manager — D2 그룹 단계 머신(W3 개발 기반 + W4 인계 잠금·자동 입고 + W5 재고·완료·종단 연결 초안)

명시적으로 주어진 그룹 운반 작업 하나의 단계·실행 식별자·두 역할 결과를 관리하고, 근거가 부족하거나 실패·중복·재시작이 있으면 다음 물리 동작 의도를 내지 않는 **D2 개발 기반**이다. 범위는 순수 엔진, D2 전용 원장, 모의 상대 포트, 개발 이름공간 상태 출력이다.

W4에서 입고 관측 → 입고 사건 대조 → 작업 자동 등록(QUEUED)을, W5에서 재고 갱신·완료 판정 결합·목적지 예약·주행 UUID D1 생성·live 어댑터 초안·개발 이름공간 입고 노드를 더했다. 이 패키지는 **실제 인계 프로토콜·자동 배정·운영 종단 완료가 아니며, 운영 이름 ROS 연결과 실제 A3/C4/D1 연결도 없다.** 실제 Action·주행·팔·공구·안전 해제를 호출하거나 발행하지 않는다. 운영 모드는 시작을 거부하고, `/ax/fleet/tasks` 같은 운영 Topic에는 발행하지 않는다.

## 구성

| 모듈 | 역할 |
|---|---|
| `group_model.py` | 입력 schema·계획 검증·내부 자료형. 공개 enum 값은 `ax_interfaces` 정의를 그대로 쓴다 |
| `group_events.py` | 엔진 입력 사건과 출력(`CommandIntent`·`Decision`) |
| `group_machine.py` | 순수 엔진 `apply_event(state, event, now) -> Decision(next_state, effects, reasons)`, 복원 규칙 `restore_state` |
| `registry.py` | 작업 간 충돌 검사(작업·물체·로봇·배정·인계·root 실행 ID 중복, 지도·지점 revision 불일치) |
| `execution_ids.py` | D-03 `derive-v1` 실행 ID 최소 구현 |
| `task_store.py` | D2 전용 SQLite 원장(schema 4: 입고·재고·ETA 주행 표본 표 포함) |
| `reassignment.py` | 예비 재배정 사전 검사(9.10, 순수 함수, 실행 없음) |
| `receipt.py` | 입고 규칙(순수): 관측 검증·식별·반복/재입고/구역 충돌 판정, 등록 요청 형식·목적지 규칙 |
| `intake.py` | 입고 접수(`ReceiptIntake`): 관측·사건·등록 요청·재고 항목·목적지 예약·작업 생성을 원장 한 트랜잭션으로 기록 |
| `inventory.py` | 등록 지점 카탈로그(개발 설정) 검증, 지점 파생 상태, `InventorySnapshot` 필드 |
| `live_adapters.py` | live 연결 초안(기본 비활성): D1 `NavigationClient` 어댑터, D1 결과 변환, A4 `ActuationState`·C4 `ToolHoldState` 관측 변환 |
| `intake_node.py`·`config/dev_intake.json` | 개발 입고 노드(`d2_dev_intake`)와 개발 설정(입고 규칙·지점 카탈로그, 실환경 값 아님) |
| `runtime.py` | 평가 → 원장 기록 → goal UUID 예약 → 포트 전송 결합 |
| `ports.py` | live 포트(전부 미지원), 개발 모의 포트, Goal 필드 변환 |
| `snapshot.py` | 상태 → `GroupTaskStatus` 필드, 목록 유지 규칙 |
| `node_config.py`·`group_node.py` | 개발 출력 노드(`group_dev_publisher`) |
| `dev_fixtures.py`·`config/dev_fixture_transfer.json` | 개발 fixture(실환경 값 아님) |

## 입력 schema(부족·추가 키 거부)

작업(`parse_spec`)은 아래 키를 **정확히** 가진 사전이다. 좌표·자동 선택값은 받지 않는다.

| 키 | 형식·규칙 |
|---|---|
| `team_id`·`task_id`·`object_id`·`root_execution_id`·`source_transfer_id`·`destination_transfer_id`·`input_source_id` | 식별자 `^[A-Za-z0-9][A-Za-z0-9_.:/-]{0,127}$`. 두 인계 ID는 서로 달라야 한다 |
| `source_location_id`·`destination_location_id` | 식별자, 서로 달라야 한다. 등록 지점 목록(호출자가 명시)에 있어야 한다 |
| `inventory_revision`·`source_location_revision`·`destination_location_revision`·`map_revision`·`schedule_revision` | 0 이상 uint64(bool 불가). 지도·지점 revision은 registry의 명시값과 같아야 한다 |
| `task_revision` | 1 이상 |
| `priority` | 1~9 |
| `steps` | 단계 사전 목록(최대 32개): `step_key`·`operation`·`robot_role`(MANIPULATOR·TRANSPORTER)·`location`(SOURCE·DESTINATION)·`depends_on`(최대 8개)·`terminal` |

계획 검증 사유 코드: `DUPLICATE_STEP_KEY`, `OPERATION_NOT_ALLOWED`(D-11 밖), `OPERATION_UNSUPPORTED`(`REQUEST_CORRIDOR`·`WAIT`는 W3 미지원), `ROLE_OPERATION_MISMATCH`, `UNKNOWN_DEPENDENCY`, `PLAN_CYCLE`, `NO_TERMINAL_STEP`, `PLAN_SHAPE_UNSUPPORTED`. W3 엔진은 단계별 근거 조건을 아는 **표준 운반 템플릿**(`standard_transfer_plan()`, 18단계)만 실행한다.

배정(`parse_assignment`)은 `assignment_id`·`assignment_generation`(1 이상)·`manipulator_robot_id`·`transporter_robot_id`(서로 다름, `^[a-z][a-z0-9_]{0,63}$`)다. robot_id는 코드에 고정하지 않는다. 다른 진행 중 작업 또는 잠금이 남은 종료 작업이 쓰는 로봇·물체는 거부한다. 실행 시작 뒤 재배정은 `REASSIGNMENT_UNSUPPORTED`다.

개발 출력 노드 fixture는 `fixture_id`·`map_revision`·`locations`(`{지점 ID: revision}`)·`tasks`(최대 32)·`assignments`(`{task_id: 배정}`)를 정확히 가진 JSON(최대 256 KiB)이다.

## 단계 근거 규칙

단계 진행에는 선행 단계의 Action 결과 `SUCCEEDED`와, **그 결과 이후에 받은** 최신·유효·출처 허용 관측이 함께 필요하다. 관측 만료는 D-05(정지 0.3 s, 보유·적재 0.5 s)이고, 팔 후퇴·목적지 배치 관측은 생산자가 없어 개발 가정값(0.5 s·3.0 s)이다. 정지 판단은 `ActuationState.state == STOPPED`만 쓰고 허가 필드는 근거로 쓰지 않는다.

| 전이 | 필요 근거 |
|---|---|
| 출발 인계 `UNKNOWN`(잠금 대기) | 출발 `LOCK_HANDOFF` goal UUID 하나라도 원장 예약(W4) |
| 출발 인계 `LOCKED` | 출발 xfer의 `UNKNOWN`, 두 `LOCK_HANDOFF` 성공, 둘째 결과 뒤 받은 두 로봇 실제 정지 |
| `PICK_AND_STOW` 의도(→ `IN_PROGRESS`) | `LOCKED`, 두 로봇 정지 |
| 팔 후퇴 의도 | `CONFIRM_LOAD` 결과 `STOWED`와 적재 관측 `STOWED` |
| 출발 인계 `UNLOCKED`·운송 의도 | 후퇴 성공, 보유 `RELEASED`, 적재 `STOWED`, 팔 후퇴 관측, 두 로봇 정지. `HELD`와 `STOWED`가 함께 최신이면 `EVIDENCE_CONFLICT` |
| 목적지 `UNKNOWN`(잠금 대기) | 출발 xfer `UNLOCKED`에서 목적지 `LOCK_HANDOFF` goal UUID 예약(W4) |
| 목적지 `LOCKED`·하역 | 목적지 xfer의 `UNKNOWN`, 두 로봇 목적지 주행 성공, 두 잠금 성공, 둘째 결과 뒤 정지. 하역은 최신 정지·적재 `STOWED` |
| `COMPLETED` | 하역·후퇴·`CONFIRM_LOAD` 결과 `EMPTY`, 배치 관측, 보유 `RELEASED`, 적재 `EMPTY`, 팔 후퇴, 두 로봇 정지, 두 `UNLOCK_HANDOFF` 성공 |

### 인계 잠금(W4)

- `transfer_state`는 D2가 두 로봇의 `LOCK_HANDOFF`·`UNLOCK_HANDOFF` 결과로 정한다(계약 509~510행). 잠금 요청 goal UUID를 원장에 예약하는 순간 `TRANSFER_UNKNOWN`과 그 `transfer_id`를 내며, 이는 계약 4.3절에서 두 로봇 주행 금지(`HANDOFF_LOCK`)다. 의도만 기록되고 예약 전에 철회·전송 실패한 경우는 A3에 보낸 것이 없으므로 이전 상태(출발 `NONE`, 목적지 전 출발 xfer `UNLOCKED`)를 유지한다.
- 예약 뒤에는 한쪽 실패·시간초과·미확정·Goal 거절·실패 종료·취소와 무관하게 `UNKNOWN` 이상을 유지한다. 명시적 인계 중단·잠금 해제·정차 보정(설계 9.7)은 없으며, 잠금이 남은 채 차단·종료된 작업 판정에는 `HANDOFF_RECOVERY_UNSUPPORTED` 사유를 붙인다.
- `LOCKED`는 같은 xfer의 `UNKNOWN`에서만 가며 다른 xfer의 `UNKNOWN`에서는 가지 않는다. 결과는 goal UUID로 결합한 뒤 단계 기록의 배정 세대(`ASSIGNMENT_STALE`)와 잠금·해제 단계의 `transfer_id`(`TRANSFER_MISMATCH`)를 지금 상태와 대조하고, 다르면 쓰지 않고 억제한다.
- 재시작 복원(runtime·개발 노드 읽기 전용 복원 모두)은 단계 기록의 goal UUID와 실행 기록 표의 예약 행으로 잠금 요청 예약을 다시 확인해 `UNKNOWN`을 세운다. 상태 기록 전에 멈춘 예약과 W3 원장(예약 뒤 `NONE`)도 포함한다. `LOCKED`·`UNLOCKED`는 되돌리지 않는다.

실패·취소·시간초과·미확정·Goal 거절·원장 오류·sim time 역행·복원은 실행 억제(`inhibit`)로 남고 W3에는 해제 사건이 없다. 실패는 잠금 해제·재시도·`attempt_index` 증가가 아니다. 취소 ACK만으로 `CANCELED`가 되지 않는다. 모든 하위 실행이 확실히 끝나야 하고(미확정이 남으면 완료 불가), 한 번이라도 goal UUID를 예약·전송한 작업은 취소 요청 시각과 마지막 하위 종료 시각보다 **뒤에** 받은 두 로봇의 최신·유효·출처 허용 `STOPPED`가 있어야 `CANCELED`다(같은 시각 수신은 인정하지 않음). 전송이 전혀 없던 작업(QUEUED·ASSIGNED, 또는 의도만 기록되고 예약 전 철회)의 취소는 물리 근거를 요구하지 않는다. 취소 요청 시각은 원장에 남고, 재부팅 등으로 시계가 그보다 앞서면 완료되지 않는다. 취소 완료는 inhibit·hold·인계 상태를 바꾸지 않는다. `FailTerminate`는 개발 운영자의 명시 실패 종료이며 인계 상태를 바꾸지 않는다. `HoldApplied`는 보류 ID를 남기고, 해제는 `HOLD_RELEASE_UNSUPPORTED`다(D4 9.6 절차 미지원). 운영 모드에서는 `MOCK` 출처를 근거로 받지 않는다(`provenance_acceptable`). 다만 운영 모드는 지금 생성 단계에서 거부된다. 혼합 개발 모드의 종류별 출처 규칙은 아래 [혼합 개발 모드](#혼합-개발-모드development_mixed)를 본다.

Action 결과에는 `ExecutionRef`가 없으므로 원장에 먼저 기록한 goal UUID로 결과를 결합한다(로컬 작업). 주행 단계는 goal UUID를 D1이 만들므로 D1이 돌려준 UUID 또는 `StepResult.execution_id`로 결합한다(W5, 주행 단계에만 허용). `RobotTaskStatus`는 `task_id`·`assignment_id`·`assignment_generation`·`execution_id`가 맞을 때만 쓴다. 실패 보고는 진행을 막지만, 성공 보고는 Action 결과를 대신하지 않는다.

## 자동 입고 등록(W4)

`ReceiptIntake.observe(관측 사전)`과 `ReceiptIntake.register_task(요청 사전)`만 제공한다. 관측 사전 키는 `ReceiptObservation`에서 D 판정에 쓰는 필드(`observation_id`·`zone_id`·`object_id`·`object_class`·`identity_confidence`·`source`·`source_id`·`valid`·`stamp_sim_s`)이고 pose는 받지 않는다. 요청 사전 키는 `RegisterTask` 요청 필드 그대로다. 타입(`ax_interfaces`)은 바꾸지 않았다.

| 관측 판정 | 조건 | 결과 |
|---|---|---|
| 거부(`REJECT`, 기록) | `valid=false`(`OBSERVATION_INVALID`), `SOURCE_UNKNOWN`, 입고 구역 아님(`ZONE_NOT_RECEIPT`), stamp 나이 > 2.0 s(`OBSERVATION_STALE`), 미래 stamp(`OBSERVATION_FROM_FUTURE`), 정답 기반인데 개발 모드 아님·비허용(`PROVENANCE_REJECTED`) | 사건·작업 없음 |
| 거부(기록 안 함) | 형식 오류(`INVALID_*`), 시각 무효(`TIME_INVALID`), runtime 고장 | 회복 뒤 같은 관측을 다시 판정 |
| 식별 불명(`PENDING`) | `object_id` 빈 값 또는 `identity_confidence` < 0.8 | 입고 구역마다 열린 `PENDING_IDENTIFICATION` 사건 1개(`object_id` 빈 값, 원장 부분 UNIQUE)에 관측 ID를 덧붙인다(최대 64, 초과분은 관측 기록에만). 없을 때만 새로 만든다. 식별 사건·물체 ID와 합치지 않음, 작업 0 |
| 신규 입고(`TASK_CREATED`) | 식별됨, 같은 물체 사건 없음 | `rcpt-<UUIDv4>` 사건(회차 1) + 내부 `RegisterTask` 한 번 |
| 반복 관측(`DUPLICATE`) | 같은 물체의 열린 사건이 같은 구역에 있음 | 그 사건에 관측 ID 덧붙임(최대 64), 새 사건·작업 없음 |
| 구역 충돌(`REJECT`) | 같은 물체의 열린 사건이 다른 구역에 있음 | `RECEIPT_ZONE_CONFLICT`, 사건 불변 |
| 재입고 미확인(`REJECTED`) | 같은 물체의 이전 사건 작업이 종료(COMPLETED·FAILED·CANCELED) | 새 회차 사건을 `REJECTED`·`RERECEIPT_UNCONFIRMED`로만 남김, 작업 0 |

- **열린 사건**: 작업이 종료 전인 `TASK_CREATED` 사건과, 작업 없이 남은 `REJECTED` 사건이다. `REJECTED`도 열린 것으로 보는 이유는 거절된 물품의 반복 관측이 새 사건·새 재입고 판정을 계속 만들지 않게 하기 위해서다. 확인 절차(식별 불명·재입고·거절 사건 해소)는 범위 밖이다. 대기 사건도 해소 수단이 없으므로 **한 구역에 식별 불명 상자가 여러 개여도 대기 사건은 1개로 보이며**, 사건의 `provenance`는 처음 관측의 출처다.
- **같은 관측 ID 재전송**: 같은 내용이면 처음 판정을 그대로 돌려주고(`replayed`), 내용이 다르면 `DUPLICATE_REQUEST`로 거부하며 기록을 덮어쓰지 않는다.
- **작업 등록**: 같은 `request_id`·같은 내용은 저장한 응답, 같은 `request_id`·다른 내용은 `DUPLICATE_REQUEST`(D-13). 입고 사건당 작업 1개: 이미 작업이 있는 사건을 다른 `request_id`로 보내면 `accepted=false`·기존 `task_id`·`DUPLICATE_REQUEST`. 작업 없는 사건은 `RECEIPT_EVENT_NOT_REGISTRABLE`, 없는 사건은 `RECEIPT_EVENT_UNKNOWN`. 거절 응답도 요청 ID에 저장되므로 설정을 바꾼 뒤에는 새 `request_id`가 필요하다.
- **내부 요청 ID**: `req-<uuid5(RECEIPT_REQUEST_NAMESPACE, receipt_event_id)>`. namespace는 고정 상수 `5f96e85c-52b2-5215-8fd5-4232de1a4c72`(`receipt.py`)이며 바꾸면 같은 사건의 요청 ID가 달라진다.
- **출발·목적지**: 입고 사건 작업은 출발 = 입고 구역(다르면 `SOURCE_MISMATCH`). 목적지가 비면 설정 표 `{입고 zone_id: destination_location_id}`, 없으면 `NO_DESTINATION_RULE`. 수동 요청(`receipt_event_id` 빈 값)도 같은 경로를 쓰며 출발이 비면 `SOURCE_MISMATCH`다. 목적지의 용량·점유·예약은 검사하지 않는다(W5).
- **작업 생성**: `task-`·`exec-`·`xfer-` UUIDv4를 발급하고 표준 운반 계획, `task_revision=1`, `inventory_revision`=등록 시점 재고 항목 revision(W5), registry의 지도·지점 revision을 쓴다. `input_source_id`는 `d2_receipt:<receipt_event_id>` 또는 `d2_register:<request_id>`다. 기존 수용 검사를 그대로 거치며, 진행 중이거나 잠금이 남은 작업이 쓰는 물체는 `OBJECT_BUSY`다. 등록은 QUEUED 생성일 뿐 운반 완료가 아니고, 배정은 명시 입력(`AssignRoles`)이다.
- **원자성**: 관측 기록·사건 생성/갱신·요청 기록·작업 행은 한 SQLite 트랜잭션이다. 실패하면 모두 되돌리고 메모리 registry에도 넣지 않으며 runtime을 고장 상태로 둔다(이후 접수·전송 거부).
- **개발 가정값(설정 `ReceiptConfig`)**: 식별 신뢰 ≥ 0.8, 관측 나이 ≤ 2.0 s, 사건당 관측 ID 64(식별·대기 사건 공통), 정답 기반 관측은 개발 모드에서만(사건 `provenance`에 `SENSOR`·`GROUND_TRUTH` 기록), `team_id`·`priority` 기본값. 시험 전 고정할 운영 기준이 아니다.
- **사유 코드**: D-13 `DUPLICATE_REQUEST`와 D2 내부 코드 `RERECEIPT_UNCONFIRMED`·`NO_DESTINATION_RULE`·`SOURCE_MISMATCH`·`ZONE_NOT_RECEIPT`·`OBJECT_BUSY`·`RECEIPT_ZONE_CONFLICT`·`RECEIPT_EVENT_UNKNOWN`·`RECEIPT_EVENT_NOT_REGISTRABLE`, W5 `LOCATION_FULL`·`OBJECT_CLASS_NOT_ALLOWED`·`INVENTORY_MISMATCH`·`LOCATION_INACTIVE`, 관측 검사 코드(`OBSERVATION_*`·`SOURCE_UNKNOWN`·`PROVENANCE_REJECTED`·`TIME_INVALID`). **ROS 연결 시 계약 D-13 목록 갱신이 필요하다.**
- **범위 밖**: `/ax/fleet/receipt/observations` 구독·`/ax/fleet/tasks/register` 서버·`ReceiptEvent` 발행(필드 변환 `receipt_event_fields`만 있음), 자동 배정·확인 절차, 관측 기록의 보존 상한(입고 관측 표는 계속 커진다).

## 재고 갱신·완료 판정·예약(W5)

그룹 물체 상태는 엔진이 정하고(`GroupState.object_*`), 같은 판정을 기록하는 `record_decision` 트랜잭션에서 그 작업이 소유한 재고 항목에 반영한다. 계획·전송·수락만으로 바꾸지 않는다.

| 재고 상태 | 언제·근거 | location / carrier |
|---|---|---|
| `AT_SOURCE` | 등록 수락(입고·수동 공통, 같은 트랜잭션). 집기 요청 예약 전 실패·취소도 유지 | 출발 지점 / '' |
| `HELD_BY_MANIPULATOR` | 집기(또는 하역) 요청 예약 뒤 받은 조작 로봇 HOLD `HELD`(대상 물체) 최신 관측 | '' / 조작 로봇 |
| `STOWED_ON_TRANSPORTER` | `CONFIRM_LOAD` 결과 STOWED + 그 결과 뒤 적재 관측 STOWED(보유 관측이 없으면 HELD를 건너뜀). 하역 요청 예약 뒤에는 되돌아가지 않음 | '' / 운송 로봇 |
| `AT_DESTINATION` | **COMPLETED 판정과 같은 사건·같은 트랜잭션에서만**(E-31) | 도착 지점 / '' |
| `LOST` | 집기 시작 뒤 적재 감시 `LOAD_LOST` 또는 적재 확인 결과 LOST | '' / 마지막 carrier |
| `UNKNOWN`(`isolated=true`) | 집기 요청 예약 뒤 실패·시간초과·미확정·Goal 거절·모순·실패 종료·취소 | '' / 마지막 carrier |

- 복원 미대조(`RESTORED_UNRECONCILED`) 중에는 재고를 바꾸지 않는다(승격·강등 없음). 실제 관측과 재대조하는 절차는 없다.
- 재고 항목 revision은 필드가 바뀔 때마다 +1(원장 영속, 세션 무관). 재고 snapshot revision(`InventorySnapshot.meta.revision`)은 재고·예약·지점 설정이 바뀔 때마다 +1(원장 영속).
- 재고 항목이 없는 작업(명시 작업 입력 `runtime.admit`, 이전 원장)은 재고를 바꾸지 않는다.
- **목적지 예약**: 등록과 같은 트랜잭션에서 `rsv-<UUIDv4>`를 만든다. 목적지가 사용 중지면 `LOCATION_INACTIVE`, 물체 종류가 허용 목록(`"*"` 또는 일치, 종류 없는 물체는 `"*"`만)에 없으면 `OBJECT_CLASS_NOT_ALLOWED`, 도착 물품 + 활성 예약 ≥ 용량이면 `LOCATION_FULL`. COMPLETED에서 소진(CONSUMED), **집기 요청 예약 전** 실패 종료·취소만 해제(RELEASED). 그 밖의 실패·미확정은 유지한다(만료로 해제하지 않음). `max_load_kg`는 물품 질량 원천이 없어 검사하지 않는다.
- 지점 상태는 파생값이다: 사용 중지 INACTIVE, 물품으로 용량 도달 OCCUPIED, 활성 예약 RESERVED, 그 밖 AVAILABLE. `reservation_id`는 최근 활성 예약 1개라 용량 > 1이면 정보가 줄어든다.
- **수동 등록과 재고(K7)**: 항목이 없으면 `AT_SOURCE`로 만든다. 있으면 격리되지 않았고 `AT_SOURCE`·`AT_DESTINATION`이며 위치가 요청 출발과 같을 때만 수락하고 항목을 새 작업으로 넘긴다. 그 밖은 `INVENTORY_MISMATCH`. 진행 중이거나 잠금이 남은 작업의 물체는 `OBJECT_BUSY`가 먼저다.
- **완료 판정(E-31)**: AT_DESTINATION 경로는 COMPLETED 블록 하나뿐이다. 배치 관측 누락, 해제 결과 유실(미확정), 완료 전 재시작에서는 AT_DESTINATION이 되지 않고 예약이 남는다(시험 고정). 입고 사건은 TASK_CREATED를 유지한다.
- **개발 가정값**: 지점 메타(용도·용량·허용 종류·최대 하중·pose·역할별 정차 자세·대기 지점)는 개발 설정 파일(`config/dev_intake.json`, 시험은 `dev_fixtures.catalog_dict`)이다. 등록 지점 원장 값이 아니다.

## 주행 goal UUID와 D1(W5)

- 주행(NAVIGATE_TO)은 D2가 goal UUID를 만들지 않는다(계약 ID 표: UUID는 Action 클라이언트가 만듦). 순서: 의도 기록(execution_id) → `begin_submit`(실행 기록 `SUBMITTING`, UUID 없음) → 포트 `dispatch(intent, None)`(D1 submit, 같은 execution_id) → 돌려받은 UUID를 `record_submitted`로 참고 기록(`RESERVED`) → 상태 반영 → `SENT`.
- UUID 기록 전에 멈추면 재시작에서 재전송하지 않고 단계를 `UNCERTAIN`으로 둔다(D1 공개 조회 API가 없어 대조 불가). 전송 중 미확정 오류도 `UNCERTAIN`(`DISPATCH_UNCERTAIN`)이다.
- 로컬 작업(`ExecuteLocalTask`)은 D2가 Action 클라이언트이므로 지금처럼 D2가 UUID를 만들고 전송 전에 예약한다.
- 모의 주행 포트(`MockNavigationPort`)는 D1처럼 execution_id마다 UUID 하나를 만들어 돌려준다.

## live 연결 초안(W5, 기본 비활성)

- `D1NavigationPort`: D1 `NavigationClient`(같은 프로세스 객체)의 `submit`·`cancel`을 감싼다. 등록 지점 revision·지도 revision을 대조하고 카탈로그의 역할별 정차 자세(x, y, yaw)를 보낸다. D2 UUID를 넘기면 거부한다. D1 예외는 예약 전 거절과 전송 실패를 밖에서 구분할 수 없어 모두 미확정으로 본다. `enabled=False`가 기본이며 runtime에 연결하지 않는다. 실제 D1 실행 원장(`src/ax_navigation/scripts/navigation_execution.py`, 수정 없이 시험에서 읽기만)으로 같은 execution_id 재호출 시 같은 UUID·전송 1회를 확인했다.
- `d1_result_event`: D1 원장 행 상태(ARRIVED·CANCELED·ABORTED·REJECTED·UNKNOWN) → `StepResult`(execution_id 결합). ARRIVED는 도착일 뿐이며 정지 근거는 A4 관측으로 따로 본다.
- `actuation_observation`: A4 `ActuationState` 필드 → D2 관측. 정지 근거는 `state`만, 허가 필드는 `permits`.
- `hold_observation`: C4 `ToolHoldState` 필드 → D2 관측(`HOLD`), `(관측, None)` 또는 `(None, 사유)`를 돌려준다(입고 `observation_rejection`과 같은 사유 문자열 관례). `hold_rejection`이 형식만 거절한다: state가 0·1·2 밖(`HOLD_STATE_UNKNOWN_VALUE`), `robot_id`(`HOLD_ROBOT_ID_EMPTY`)·`object_id`(`HOLD_OBJECT_ID_EMPTY`)·`source_id`/`source_session_id`(`HOLD_SOURCE_EMPTY`) 빈 문자열, revision 0(`HOLD_REVISION_UNSET`), evidence가 `MOCK`으로 시작(`HOLD_MOCK_EVIDENCE`). `valid=false`·만료·물체 불일치·앵커 이전 수신은 어댑터가 보지 않고 `group_machine._fresh`가 거른다. `UNKNOWN`(0)은 관측으로 넘기되 미보유로 해석하지 않는다. `hold_fields`는 메시지 속성만 읽어(rclpy 불필요) 필드 사전을 만든다. 구독·노드는 만들지 않는다.
  - **모의 근거 거절은 임시 규칙이다.** 현재 C4 `mock_execution`은 `evidence='MOCK_ATTACH_STATE'`, `valid=true`로 HELD·RELEASED를 낸다. C4가 meta·출처로 모의 판정을 구분하기 전까지 evidence 접두어로 거른다.
- live 출처(`LIVE`)는 모의 모드에서 판정 근거가 아니며, 아래 혼합 개발 모드에서 `live_kinds`로 연 종류만 받는다. `D1NavigationPort.supported`는 `enabled=True`이고 혼합 모드에서 `NAVIGATION`을 열었을 때만 참이다(결과가 LIVE라 모의 모드에서는 받을 수 없으므로).

## 혼합 개발 모드(`DEVELOPMENT_MIXED`)

개발 중 live 입력을 근거 종류별로 하나씩 열기 위한 모드다. `DEVELOPMENT_MOCK` 동작은 그대로이고 `OPERATIONAL`은 계속 `OPERATIONAL_MODE_UNSUPPORTED`로 거부한다. runtime에 live 구독·연결을 새로 추가하지 않았다(어댑터는 함수·객체뿐).

1. **모드:** `gm.MODE_DEVELOPMENT_MIXED`. `check_mode`가 두 개발 모드(`DEVELOPMENT_MODES`)를 받는다.
2. **live 허용 종류(`live_kinds`):** 열 수 있는 이름은 `ACTUATION`(A4 관측), `NAVIGATION`(D1 주행 단계의 결과·수락 응답·상태 사건), `HOLD`(C4)뿐이다. `LOAD`(A3 값이 모의값)·`PLACEMENT`·`ARM_RETRACTED`(생산자 없음)와 그 밖의 이름은 `LIVE_KIND_NOT_OPENABLE`로 설정 단계에서 거절한다. 기본값은 빈 값이며(혼합 모드여도 전부 MOCK), 첫 시험 대상(ACTUATION+NAVIGATION)을 코드 기본값으로 넣지 않았다. 모의 모드에 `live_kinds`를 주면 `LIVE_KINDS_REQUIRE_MIXED_MODE`, 중복은 `DUPLICATE_LIVE_KIND`, 목록이 아니거나 문자열이 아닌 항목은 `INVALID_LIVE_KINDS`다.
3. **종류마다 출처 하나:** `provenance_acceptable(mode, provenance, kind, live_kinds)`. 혼합 모드에서 `live_kinds`에 든 종류는 `LIVE`만, 나머지는 `MOCK`만 받고 반대 출처는 `PROVENANCE_REJECTED`다. 단계 결과·Goal 응답·상태 사건의 종류는 그 단계 operation이 `NAVIGATE_TO`면 `NAVIGATION`, 그 밖(A3 로컬 작업)은 `LOCAL_TASK`이며 `LOCAL_TASK`는 열 수 없어 항상 MOCK이다.
4. **상태·원장:** `live_kinds`는 작업 생성 때 정해 `GroupState`에 두고 원장 상태 JSON에 저장·복원한다(관측 버퍼처럼 버리지 않는다). 상태 JSON 안 필드라 **schema 이주는 없다**: 필드가 없는 이전 원장은 빈 값(=전부 MOCK)으로 읽는다(W5 `object_*` 필드와 같은 방식). runtime의 mode·`live_kinds`와 원장 작업의 값이 하나라도 다르면 원장에 쓰기 전에 `STORE_MODE_MISMATCH`로 거부한다. 따라서 기존 모의 원장은 혼합 모드로 열 수 없다.
5. **포트:** 모의 포트 `supported(mode, live_kinds)`는 두 개발 모드에서 참이되, 포트가 담당하는 근거 종류(`evidence_kind`: 모의 주행 `NAVIGATION`, 모의 로컬 작업 `LOCAL_TASK`)가 `live_kinds`에 있으면 거짓이다(종류마다 출처 하나). 그래서 혼합 모드 + `NAVIGATION`에서 모의 주행 포트는 `PORT_UNSUPPORTED`이고, `LOCAL_TASK`는 열 수 없으므로 모의 로컬 작업 포트는 계속 참이다. `D1NavigationPort`는 위 조건에서만 참이다. `provenance_acceptable`은 혼합 모드에서 종류가 없거나 문자열이 아니면 거절한다(호출부 누락은 닫힌 쪽). runtime은 `port.supported(mode, live_kinds)`로 판정하며, 거짓이면 보내지 않고 `PORT_UNSUPPORTED`로 실패 처리한다.
6. **완료 표기:** 계약 상태값은 늘리지 않았다.
   - (a) 판정에 쓴 근거의 (종류, 출처) 요약(`evidence_used`)을 작업 상태에 누적·영속한다. 누적 시점은 단계 조건에서 참으로 확인된 관측, 적재 확인의 모순 검사에 쓴 관측, 수용한 단계 결과다. **기록 범위는 과대(보수적)다:** `_fresh_is`가 참인 관측은 그 판정 전체(다른 조건과의 AND)가 거짓이어도 누적된다. 요약은 `persistent_view`에 없어 요약만 바뀌면 revision을 올리지 않는다(원장에는 판정마다 상태 전체를 쓰므로 함께 저장된다). COMPLETED로 바뀐 판정에서 원장 사건 `Completed`(`mode`·`live_kinds`·`evidence_used`)를 한 번 남긴다.
   - (b) `group_dev_publisher`의 `source_id`는 노드 파라미터가 아니라 작업 상태의 모드로 정한다. 혼합 모드 작업이 하나라도 있으면 `d2_dev_mixed:<label>`, 아니면 `d2_dev_mock:<label>`이다. 원장을 다시 읽어 바뀌면 revision을 올린다.
   - 두 노드의 `mode` 파라미터는 MIXED를 받고, `live_kinds` 파라미터(문자열 배열, 기본 빈 값)를 받는다. 입고 노드는 이 값으로 runtime을 만든다. 출력 노드는 값을 검사만 한다.
7. **입고:** 혼합 모드도 입고 관측은 모의 모드와 같게 다룬다(정답 기반 관측은 `allow_ground_truth`를 따름).
8. **재고:** 재고 전이는 작업과 같은 원장 트랜잭션에 기록되고, 4번 규칙으로 원장 하나에는 한 모드·한 `live_kinds`의 작업만 있으므로 별도 처리를 하지 않았다.

**혼합 모드의 COMPLETED는 종단 완료가 아니다.** 열지 않은 종류(LOAD·PLACEMENT·ARM_RETRACTED·로컬 작업)는 모의 근거이며, 실제 집기·배치·적재를 확인하지 않는다. 완료 사건의 `evidence_used`로 어떤 근거가 live였는지 구분한다.

내부 사유 코드(계약 D-13 미반영): `LIVE_KIND_NOT_OPENABLE`, `LIVE_KINDS_REQUIRE_MIXED_MODE`, `DUPLICATE_LIVE_KIND`, `INVALID_LIVE_KINDS`, `STORE_MODE_MISMATCH`.

## ETA 주행 표본과 예비 재배정 사전 검사(W6 선행)

ETA 학습·모델·관제는 하지 않는다(W7 ETA XGBoost 몫, 설계 4.4·10절). 이번에는 표본 기록과 사전 검사만 한다.

### ETA 주행 표본(`eta_samples`, schema 4)

- **기록 조건:** `NAVIGATE_TO` 단계 상태가 `SUCCEEDED`·`FAILED`·`CANCELED`·`TIMED_OUT`·`UNCERTAIN` 중 하나로 처음 바뀐 판정에서, 그 판정을 원장에 쓰는 트랜잭션(`record_decision`) 안에서 실행당 1행을 남긴다(`execution_id` 기본 키). 표본 쓰기가 실패하면 그 판정 전체가 기록되지 않는다.
- **제외:** `REJECTED`·`NOT_SENT`·`WITHDRAWN`은 주행하지 않아 제외한다. 중복 결과는 엔진이 상태를 바꾸지 않으므로(`DUPLICATE_RESULT`) 행이 늘지 않는다. 재시작 복원으로 진행 중 주행이 `UNCERTAIN`이 되면 종료 시각 없는 행을 남긴다.
- **이주:** v1·v2·v3 → v4는 한 트랜잭션이며 실패하면 원장이 바뀌지 않는다. 과거 단계는 백필하지 않는다.
- **시각:** 단계 기록(`StepRun`)에 `sent_sim_s`·`accepted_boottime_s`·`accepted_sim_s`·`finished_sim_s`·`result_provenance`를 더했다. 상태 JSON에 없으면 기본값(-1·빈 값)으로 읽는다.

| 열 | 단위·값 | 설명 |
|---|---|---|
| `execution_id`·`task_id`·`step_key`·`robot_id`·`role`·`attempt_index` | 문자열·정수 | 실행 식별. `role`은 `MANIPULATOR`·`TRANSPORTER` |
| `mode`·`live_kinds` | 문자열·JSON 목록 | 작업 모드와 live 허용 종류 |
| `provenance` | `MOCK`·`LIVE`·NULL | 결과 사건 출처. 결과 없이 미확정이면 NULL |
| `status`·`outcome`·`error_code` | 문자열·정수·문자열 | 단계 종료 상태, `ResultStatus.outcome`(없으면 NULL), 오류 코드 |
| `from_location_id`·`to_location_id` | 등록 지점 ID | 목적지 주행만 출발 지점이 있다. 출발지 주행은 출발 위치 원천이 없어 NULL |
| `to_location_revision`·`map_revision` | 정수 | 작업 정의의 지점·지도 revision |
| `object_state` | `INV_*` 정수 | 결과 판정 시점의 D2 그룹 물체 상태 |
| `carrying_object` | 1·0·NULL | 그 로봇이 물체를 들거나 싣고 있으면 1. 물체 상태 `UNKNOWN`·`LOST`면 NULL |
| `sent_*`·`accepted_*`·`finished_*` | s | `_boottime_s`는 D2 프로세스 CLOCK_BOOTTIME, `_sim_s`는 ROS sim time. 모르면 NULL |
| `duration_s`·`duration_sim_s` | s | 전송 예약 → 결과(boottime 차·sim 차). 한쪽 시각이 없으면 NULL |
| `path_length_m`·`turn_count`·`corridor_wait_s`·`obstacle_density` | m·회·s·(미정) | 원천(D1·D3·B3)이 없어 **항상 NULL**. 값을 지어내지 않는다 |

내보내기(원장 읽기 전용, ROS 불필요):

```bash
python3 tools/export_d2_eta_samples.py /tmp/<폴더>/d2.sqlite -o /tmp/<폴더>/eta.csv
```

- 기본은 `mode=DEVELOPMENT_MIXED`·`provenance=LIVE`·`status=SUCCEEDED` 표본만 낸다(학습 표본). MOCK 표본은 학습에 쓰지 않는다.
- `--include-failed`를 주면 그 밖의 종료 상태도 넣고, `--include-mock`을 주면 모드·출처 필터를 끈다(개발 확인용).
- `finished_boottime_s`가 NULL이거나 `duration_s`·`duration_sim_s`가 음수·비유한인 행은 항상 제외한다.
- CSV 첫 줄은 위 열 이름이고 NULL은 빈 칸이다. schema 4 미만 원장은 종료 코드 2로 거부한다.

### 예비 재배정 사전 검사(`reassignment.check_spare_reassignment`)

- **입력:** 대상 작업 상태, 교체할 역할, 후보 `RobotSummary` 필드 사전, 기존 로봇의 `ActuationState` 필드 사전(없으면 `None`), 필요 capabilities(호출자 입력), 시각, 다른 작업 상태 목록.
- **결과:** `(허용 여부, 사유 코드 tuple)`. 재배정을 실행하거나 원장·상태를 바꾸지 않는다. 사유는 모두 모으고, 하나라도 있으면 거절이다. 허용은 사전 조건 충족일 뿐 재배정 승인이 아니다.
- **D2에는 로봇 풀이 없다.** 후보는 D4 `FleetSnapshot`의 `RobotSummary`다.

| 사유 코드 | 조건 |
|---|---|
| `INVALID_ROLE`·`TASK_NOT_ASSIGNED`·`CANDIDATE_INVALID` | 역할 이름 오류, 배정 없음, 후보 형식·robot_id 오류 |
| `TASK_TERMINAL` | 종료 작업(실패 작업을 되살리지 않음, 후속 작업 승인 절차 몫) |
| `CANDIDATE_IS_DEPARTING_ROBOT`·`CANDIDATE_IS_PARTNER` | 후보가 기존 로봇이거나 상대 역할 로봇 |
| `CANDIDATE_ROLE_MISMATCH` | 다른 역할 대체 금지 |
| `CANDIDATE_CAPABILITY_MISSING` | 필요 capabilities 미포함 |
| `CANDIDATE_NOT_AVAILABLE` | `availability`가 `AVAILABLE`·`STANDBY_SPARE`가 아님(빈 문자열·`BUSY`·`FAULT`) |
| `CANDIDATE_ALREADY_ASSIGNED` | 후보 `assignment_id`가 비어 있지 않음 |
| `CANDIDATE_META_STALE:<영역>` | `actuation`·`safety`·`localization`(운송 역할은 `load`도) meta가 무효·revision 0·만료(D-05: 0.3·0.5·1.0·0.5 s, sim 나이)·미래 |
| `CANDIDATE_IN_ACTIVE_TASK` | 다른 진행 중·잠금 잔존 작업이 후보를 씀 |
| `DEPARTING_STEP_ACTIVE`·`DEPARTING_STEP_UNCERTAIN` | 기존 로봇 단계가 의도·예약·전송·수락 중이거나 미확정. 상대 로봇 단계는 보지 않는다 |
| `DEPARTING_ACTUATION_UNCONFIRMED` | 기존 로봇의 최신·유효 `ActuationState`가 `STOPPED`이고 `drive_allowed`·`arm_allowed`가 모두 거짓인 것을 확인할 수 없음 |
| `TRANSFER_NOT_RELEASED` | `transfer_state`가 `NONE`·`UNLOCKED`가 아님(물품 보유·인계 중일 수 있음) |
| `OBJECT_HELD_NO_RECOVERY_PROCEDURE` | 재고 상태 `HELD_BY_MANIPULATOR`·`STOWED_ON_TRANSPORTER`(검증된 회수·인계 절차 없음) |
| `OBJECT_STATE_UNKNOWN` | 재고 상태 `UNKNOWN`·`LOST`(보유 여부를 배제할 수 없음) |

`TASK_TERMINAL`·`CANDIDATE_ALREADY_ASSIGNED`·`CANDIDATE_IN_ACTIVE_TASK`·`OBJECT_STATE_UNKNOWN`과 정지 판정의 허가 필드 확인은 9.10 "확인할 수 없으면 재배정하지 않는다"에 따른 보수적 추가다.

판정하지 않는 미정 항목: (1) 실행·예산 이력 승계 방식(DEC-24), (2) 기존 권한 철회와 새 배정 세대 부여 절차, (3) 재시작한 고장 로봇의 과거 명령 재실행 방지(A·D 검증), (4) 역할별 필요 capabilities 목록(호출자가 넘긴다).

내부 사유 코드(계약 D-13 미반영): 위 표의 코드 전부.

## 개발 입고 노드(W5)

```bash
ros2 run ax_fleet_manager d2_dev_intake --ros-args -p store_path:=/tmp/<폴더>/d2.sqlite
ros2 run ax_fleet_manager group_dev_publisher --ros-args -p store_path:=/tmp/<폴더>/d2.sqlite  # 작업 목록
```

- 입력 `/ax/dev/d2/receipt/observations`(`ReceiptObservation`), 서비스 `/ax/dev/d2/tasks/register`(`RegisterTask`). 출력 `/ax/dev/d2/receipt/events`(`ReceiptEvent`, reliable·volatile), `/ax/dev/d2/inventory`(`InventorySnapshot`, STATE QoS, 변경 시 + 1 Hz).
- 관측 메시지에서 pose는 쓰지 않는다. `meta.source_id`·`meta.valid`·`meta.header.stamp`를 관측 출처·유효성·시각으로 쓰고, float32 신뢰도는 소수 6자리로 맞춘다.
- 원장은 쓰기 가능하게 연다(작업 목록 출력 노드는 같은 원장을 읽기 전용으로 읽는다). 작업은 QUEUED 등록만 하며 포트가 없어 배정·전송하지 않는다.
- 운영 모드, `/ax/dev/` 밖 이름, `/ax/fleet`·`/ax/robot_` 포함 이름(remap 결과 포함)은 시작 거부(종료 코드 2). 운영 이름 연결은 하지 않는다.

## 상대 확인 필요

1. 등록 지점 원장 소유: D2(계약상 `InventorySnapshot` 발행자) vs D4(일정표 "FleetSnapshot·지도/지점 revision"). W5는 D2 개발 설정 파일로 대신한다.
2. D1 복원 조회 API(execution_id → goal_uuid·상태) 공개 여부와 D1 패키지 import 방식(현재 `scripts/` 스크립트, 패키지 모듈 아님). D1 `ExecutionLedger`는 robot_1~robot_4만 허용한다.
3. PLACEMENT·ARM_RETRACTED 관측 생산자(계약에 Topic 없음). 없으면 live로 COMPLETED에 도달하지 못한다.
4. W4·W5 내부 사유 코드의 계약 D-13 반영 시점(운영 이름 연결 때 제안).

## 종단 연결 막힘 목록(W5 시점)

| 연결 | 상태 | 막힘 |
|---|---|---|
| (a) 순수 엔진 + 모의 포트 전 흐름 | 완료(시험) | 없음 |
| (b1) D1 주행 | 어댑터 초안·실제 D1 원장 시험 | 위 2번(조회 API·import), 지점 pose 원천(1번), runtime live 연결 없음(출처 규칙은 혼합 개발 모드로 결정) |
| (b2) A3 로컬 작업 | 미연결 | A3 서버는 `CONFIRM_LOAD`만 수락하고 나머지는 거절(mock_executor). 기본 `RuntimeAuthority`는 dispatcher 등록이 없으면 거절 |
| (b3) 관측 | `ActuationState`·`ToolHoldState` 변환 함수(구독 없음) | C4 `hold_state`는 조작 단계에서만 발행(주기 재발행 없음), revision이 1씩 증가하지 않음, 모의 판정을 출처로 구분하지 않음(아래 운영 발행 전환 조건). 배치·팔 후퇴 생산자 없음(3번), A3 LoadState는 모의값 |
| (c) 격리 Gazebo 종단 | 미실행 | (b1)~(b3). 혼합(live 주행 + 모의)도 실제 집기·배치가 아니므로 종단 완료가 아님 |

## 운영 발행 전환 조건(`/ax/fleet/tasks`)

현재 상태와, 실제 운영으로 발행하기 전에 필요한 결정·전제를 적는다. 상대 기능 부족은 [종단 연결 막힘 목록](#종단-연결-막힘-목록w5-시점)·[미지원 상대 기능](#미지원-상대-기능운영-실행-차단), 미결 소유권은 [상대 확인 필요](#상대-확인-필요)를 본다. 아래 항목은 모두 미정이며 이 문서가 정하지 않는다.

**현재 상태**

- `check_mode`가 `OPERATIONAL`을 `OPERATIONAL_MODE_UNSUPPORTED`로 거부한다(개발 모드 `DEVELOPMENT_MOCK`·`DEVELOPMENT_MIXED`만 받음). 개발 노드(`group_dev_publisher`·`d2_dev_intake`)는 운영 모드, `/ax/dev/` 밖 이름, `/ax/fleet`·`/ax/robot_` 포함 이름(remap 결과 포함)을 시작 거부한다(종료 코드 2). 출력은 `/ax/dev/` 아래뿐이다.
- live 입력을 받는 runtime 노드가 없다. live 어댑터(`D1NavigationPort`·`d1_result_event`·`actuation_observation`·`hold_observation`)는 함수·객체만 있고 연결하지 않는다.

**D2 쪽 결정**

1. live 출처 정책: **결정됨(혼합 개발 모드).** 모의 모드는 `MOCK`만, 운영 모드는 `LIVE`만 받고, 개발 중 live 입력은 `DEVELOPMENT_MIXED`에서 `live_kinds`(ACTUATION·NAVIGATION·HOLD 중)로 연 종류만 `LIVE`로 받는다. 종류마다 출처는 하나다. 규칙은 [혼합 개발 모드](#혼합-개발-모드development_mixed)를 본다. 운영 모드의 출처 규칙(운영 모드 허용 조건, 2번)은 그대로 미정이다.
2. 운영 모드 허용 조건: `check_mode` 거부를 풀 조건(어느 상대 포트가 live여야 하는지, 위 미지원 상대 기능 중 어디까지)을 정해야 한다.
3. 원장 경로·쓰기 주체: 개발 노드는 `store_path` 파라미터로 원장을 받고, 입고 노드는 쓰기, 출력 노드는 읽기 전용으로 연다. 운영에서 원장 위치와 쓰기 프로세스(단일 writer 여부)는 미정이다.
4. 세션·revision 규칙(D-05): 출력 노드는 시작마다 새 `source_session_id`, 내용 변경 시 revision +1, 1 Hz 재발행·3.0 s 만료를 쓴다. 운영 이름에서 같은 규칙을 쓸지, 재시작 뒤 소비자 재대조 방법은 정해야 한다.

**소비자 측 전제**

- A4: 계약 4.3절 인계 잠금(`HANDOFF_LOCK`·`PEER_STATE_STALE`)을 `/ax/fleet/tasks`로 판정해야 한다. 현재 A4 감독기는 `/ax/fleet/tasks`를 진단(`A4_STOP_DIAG`의 `d2_tasks`)으로만 구독하고, 게이트는 D2/D3 허가 정책 확정 전까지 parameter 기본값 차단(`HANDOFF_LOCK:PARAMETER_OR_D2_D3_POLICY_NOT_VERIFIED`)을 유지한다([ax_safety 안내](../ax_safety/README.md)).
- C4: Goal의 `peer_robot_id`를 D2 작업 상태와 대조하지 않는다. 현재 `mock_execution`이 아니면 상대 로봇이 있는 Goal을 `PRECONDITION_FAILED`로 거절한다.

**`hold_state` 입력 상태**

- D2: `hold_observation` 변환 함수만 추가했다(구독·runtime 연결 없음). LIVE 보유 관측은 혼합 모드에서 `HOLD`를 열었을 때만 근거가 된다.
- C4 쪽 필요 사항(계약 STATE 10 Hz·0.5 s 기준): 현재 C4는 PICK·PLACE 처리 중에만 발행하고 주기 재발행이 없다. revision은 `time.monotonic_ns()` 값이라 같은 세션에서 1씩 증가하지 않는다. 모의 판정은 `evidence='MOCK_ATTACH_STATE'`·`valid=true`로만 드러나 출처 구분이 필요하다.

## 원장과 실행 ID

- D2 전용 SQLite 파일(`tasks`·`dispatches`·`events`·`meta`, schema 2에서 `receipt_events`·`receipt_observations`·`register_requests`, schema 3에서 `inventory`·`locations`·`reservations`와 `receipt_events.object_class`, schema 4에서 `eta_samples` 추가)이다. schema 1·2·3 원장은 쓰기 가능하게 열 때 한 트랜잭션으로 이주하고(1·2는 종료 전 작업의 물체를 근거가 없어 `UNKNOWN` 항목으로 백필, 종료 작업은 생략. ETA 표본은 백필하지 않음), 실패하면 원장을 바꾸지 않고 거부한다. 개발 노드의 읽기 전용 열기는 이주하지 않는다. 원장은 D4 원장 DB를 열거나 바꾸지 않는다. 상태와 의도는 한 트랜잭션에 기록하고, `expected_revision` 불일치·같은 실행 ID의 다른 인자는 거부한다.
- 전송 순서는 의도 기록 → `reserve_dispatch`(goal UUID 기록) → 포트 전송이다. 이미 예약된 실행은 다시 예약하지 않으며 재전송하지 않는다. 같은 판정의 앞 의도가 실패하면 남은 의도도 보내지 않는다.
- 재시작하면 관측을 버리고, 진행 중이거나 예약된 단계는 `UNCERTAIN`, 작업은 `RESTORED_UNRECONCILED`로 원장에 남긴다. 과거 세션·lease는 복원하지 않는다. 실제 상대 재대조 절차가 없어 차단은 풀리지 않는다.
- 원장 손상(무결성·schema·JSON·단계 불일치)은 열기를 거부하고, 쓰기 실패는 runtime을 고장 상태로 두어 이후 전송을 막는다. 사건 기록은 작업별 500건(기본)·4 KiB로 제한한다.
- 실행 ID는 D-03 규칙을 표준 json으로 구현했다. `ax_d4.common_contract.make_execution_id`는 `rfc8785` 외부 모듈이 필요한데, rosdep 키가 없고 ROS 시스템 python에도 없어서 직접 의존하지 않았다. 대신 고정 벡터 시험으로 두 함수의 결과가 같은지 대조한다.

## 미지원 상대 기능(운영 실행 차단)

`ports.UNSUPPORTED_EXTERNAL_PORTS`에 적는다. 실 A3 `ExecuteLocalTask` 연결, D1 주행 실행 API(실행 ID·goal UUID·취소), C4 조작, D3 통로 허가, D4 보류 생명주기·배정 확인, 등록 지점·물품 pose 조회, 목적지 배치·팔 후퇴 확인 생산자, A4 `ActuationState` live 어댑터(schema 전환 확정 전)가 여기에 해당한다. live 포트는 확정 미전송(`NOT_SENT`)으로 실패한다. 모의 포트의 `source_id`는 `d2_mock_<종류>:<pid>:<번호>:<label>` 형식이다. Goal의 `pick_pose`·`place_pose`는 원장 조회 포트가 없어 만들지 않는다.

## 개발 출력 노드

```bash
source /opt/ros/jazzy/setup.bash && source <격리 install>/setup.bash
ros2 launch ax_fleet_manager group_manager_dev.launch.py            # 패키지 fixture
ros2 run ax_fleet_manager group_dev_publisher --ros-args -p store_path:=/tmp/<폴더>/d2.sqlite
```

- 출력은 `/ax/dev/` 아래(기본 `/ax/dev/d2_mock/fleet_tasks`) `GroupTaskList`뿐이며 STATE QoS(reliable·transient_local·keep_last 1)를 쓴다. 내용이 바뀌면 `StateMeta.revision`을 1 올려 바로 내고, 1 Hz로 같은 revision을 새 stamp와 함께 다시 낸다(소비자 만료 3.0 s).
- 재발행·원장 폴링 타이머(입고 노드의 재고 1 Hz 재발행 포함)는 `use_sim_time`이어도 STEADY 시계로 돈다(소비자의 무수신 판정이 수신 측 CLOCK_BOOTTIME 기준이고, 시스템 시계가 역행하면 ROS 시계 타이머 간격이 2 s 넘게 벌어진 기록이 있다). 메시지 stamp는 지금처럼 `get_clock().now()`(ROS 시간, sim이면 sim)다.
- `source_id`는 `d2_dev_mock:<fixture_id>` 또는 `d2_dev_mock:store`, `source_session_id`는 시작마다 새로 만든다. 단계 진행은 `task_revision`을 올리지 않는다(`task_revision`·`schedule_revision`은 입력값 그대로).
- 원장은 읽기 전용 URI로 열고 복원 규칙은 메모리에서만 적용한다. 원장이 바뀌면(0.2 s 확인) 다시 읽고, 읽지 못하면 `valid=false`·`D2_STORE_UNREADABLE`로 낸다.
- 목록 유지: 진행 중 작업과, 종료됐어도 `transfer_state`가 `NONE`·`UNLOCKED`가 아닌 작업을 낸다(계약 4.3).
- 다음 경우 종료 코드 2로 시작을 거부한다: `mode:=OPERATIONAL`, `/ax/dev/` 밖이거나 `/ax/fleet`·`/ax/robot_`를 포함한 출력 Topic, remap 결과가 그런 이름인 경우, 입력이 없는 경우. 구독·Action client·서버를 만들지 않으며 A4 입력 Topic을 remap하지 않는다.

## 시험

```bash
.venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros tools/tests/test_d2_group_machine.py \
  tools/tests/test_d2_execution_ids.py tools/tests/test_d2_task_store.py tools/tests/test_d2_ports.py \
  tools/tests/test_d2_workflow_scenarios.py tools/tests/test_d2_group_node.py tools/tests/test_d2_receipt.py \
  tools/tests/test_d2_inventory.py tools/tests/test_d2_mixed_mode.py \
  tools/tests/test_d2_eta_samples.py tools/tests/test_d2_reassignment.py
# 격리 install을 source한 셸, 빈 domain 80~89 자동 예약
python3 -m pytest -q -p no:cacheprovider tools/tests/test_d2_group_node_ros.py \
  tools/tests/test_d2_intake_node_ros.py
```

시험은 개발 fixture와 모의 관측만 쓴다. Gazebo·실제 A3/C4/D1 연결·물리 시험은 포함하지 않는다.
