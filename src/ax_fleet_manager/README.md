# ax_fleet_manager — D2 그룹 단계 머신(W3 개발 기반)

명시적으로 주어진 그룹 운반 작업 하나의 단계·실행 식별자·두 역할 결과를 관리하고, 근거가 부족하거나 실패·중복·재시작이 있으면 다음 물리 동작 의도를 내지 않는 **D2 개발 기반**이다. 범위는 순수 엔진, D2 전용 원장, 모의 상대 포트, 개발 이름공간 상태 출력이다.

이 패키지는 **자동 입고·실제 인계 프로토콜·재고 갱신·운영 종단 완료가 아니다.** 실제 Action·주행·팔·공구·안전 해제를 호출하거나 발행하지 않는다. 운영 모드는 시작을 거부하고, `/ax/fleet/tasks` 같은 운영 Topic에는 발행하지 않는다.

## 구성

| 모듈 | 역할 |
|---|---|
| `group_model.py` | 입력 schema·계획 검증·내부 자료형. 공개 enum 값은 `ax_interfaces` 정의를 그대로 쓴다 |
| `group_events.py` | 엔진 입력 사건과 출력(`CommandIntent`·`Decision`) |
| `group_machine.py` | 순수 엔진 `apply_event(state, event, now) -> Decision(next_state, effects, reasons)`, 복원 규칙 `restore_state` |
| `registry.py` | 작업 간 충돌 검사(작업·물체·로봇·배정·인계·root 실행 ID 중복, 지도·지점 revision 불일치) |
| `execution_ids.py` | D-03 `derive-v1` 실행 ID 최소 구현 |
| `task_store.py` | D2 전용 SQLite 원장 |
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
| 출발 인계 `LOCKED` | 두 로봇 주행 성공, 두 `LOCK_HANDOFF` 성공, 두 로봇 실제 정지 |
| `PICK_AND_STOW` 의도(→ `IN_PROGRESS`) | `LOCKED`, 두 로봇 정지 |
| 팔 후퇴 의도 | `CONFIRM_LOAD` 결과 `STOWED`와 적재 관측 `STOWED` |
| 출발 인계 `UNLOCKED`·운송 의도 | 후퇴 성공, 보유 `RELEASED`, 적재 `STOWED`, 팔 후퇴 관측, 두 로봇 정지. `HELD`와 `STOWED`가 함께 최신이면 `EVIDENCE_CONFLICT` |
| 목적지 `LOCKED`·하역 | 두 로봇 목적지 주행 성공·정지, 적재 `STOWED` |
| `COMPLETED` | 하역·후퇴·`CONFIRM_LOAD` 결과 `EMPTY`, 배치 관측, 보유 `RELEASED`, 적재 `EMPTY`, 팔 후퇴, 두 로봇 정지, 두 `UNLOCK_HANDOFF` 성공 |

실패·취소·시간초과·미확정·Goal 거절·원장 오류·sim time 역행·복원은 실행 억제(`inhibit`)로 남고 W3에는 해제 사건이 없다. 실패는 잠금 해제·재시도·`attempt_index` 증가가 아니다. 취소 ACK만으로 `CANCELED`가 되지 않는다. 모든 하위 실행이 확실히 끝나야 하고(미확정이 남으면 완료 불가), 한 번이라도 goal UUID를 예약·전송한 작업은 취소 요청 시각과 마지막 하위 종료 시각보다 **뒤에** 받은 두 로봇의 최신·유효·출처 허용 `STOPPED`가 있어야 `CANCELED`다(같은 시각 수신은 인정하지 않음). 전송이 전혀 없던 작업(QUEUED·ASSIGNED, 또는 의도만 기록되고 예약 전 철회)의 취소는 물리 근거를 요구하지 않는다. 취소 요청 시각은 원장에 남고, 재부팅 등으로 시계가 그보다 앞서면 완료되지 않는다. 취소 완료는 inhibit·hold·인계 상태를 바꾸지 않는다. `FailTerminate`는 개발 운영자의 명시 실패 종료이며 인계 상태를 바꾸지 않는다. `HoldApplied`는 보류 ID를 남기고, 해제는 `HOLD_RELEASE_UNSUPPORTED`다(D4 9.6 절차 미지원). 운영 모드에서는 `MOCK` 출처를 근거로 받지 않는다(`provenance_acceptable`). 다만 운영 모드는 지금 생성 단계에서 거부된다.

Action 결과에는 `ExecutionRef`가 없으므로 원장에 먼저 기록한 goal UUID로 결과를 결합한다. `RobotTaskStatus`는 `task_id`·`assignment_id`·`assignment_generation`·`execution_id`가 맞을 때만 쓴다. 실패 보고는 진행을 막지만, 성공 보고는 Action 결과를 대신하지 않는다.

## 원장과 실행 ID

- D2 전용 SQLite 파일(`tasks`·`dispatches`·`events`·`meta`)이며 D4 원장 DB를 열거나 바꾸지 않는다. 상태와 의도는 한 트랜잭션에 기록하고, `expected_revision` 불일치·같은 실행 ID의 다른 인자는 거부한다.
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
- `source_id`는 `d2_dev_mock:<fixture_id>` 또는 `d2_dev_mock:store`, `source_session_id`는 시작마다 새로 만든다. 단계 진행은 `task_revision`을 올리지 않는다(`task_revision`·`schedule_revision`은 입력값 그대로).
- 원장은 읽기 전용 URI로 열고 복원 규칙은 메모리에서만 적용한다. 원장이 바뀌면(0.2 s 확인) 다시 읽고, 읽지 못하면 `valid=false`·`D2_STORE_UNREADABLE`로 낸다.
- 목록 유지: 진행 중 작업과, 종료됐어도 `transfer_state`가 `NONE`·`UNLOCKED`가 아닌 작업을 낸다(계약 4.3).
- 다음 경우 종료 코드 2로 시작을 거부한다: `mode:=OPERATIONAL`, `/ax/dev/` 밖이거나 `/ax/fleet`·`/ax/robot_`를 포함한 출력 Topic, remap 결과가 그런 이름인 경우, 입력이 없는 경우. 구독·Action client·서버를 만들지 않으며 A4 입력 Topic을 remap하지 않는다.

## 시험

```bash
.venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros tools/tests/test_d2_group_machine.py \
  tools/tests/test_d2_execution_ids.py tools/tests/test_d2_task_store.py tools/tests/test_d2_ports.py \
  tools/tests/test_d2_workflow_scenarios.py tools/tests/test_d2_group_node.py
# 격리 install을 source한 셸, 빈 domain 80~89 자동 예약
python3 -m pytest -q -p no:cacheprovider tools/tests/test_d2_group_node_ros.py
```

시험은 개발 fixture와 모의 관측만 쓴다. Gazebo·실제 A3/C4/D1 연결·물리 시험은 포함하지 않는다.
