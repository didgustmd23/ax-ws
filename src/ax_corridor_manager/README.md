# D3 통로 허가·점유 — 김성민 W3

`RequestCorridorEntry`, `ReleaseCorridor`, `CorridorStateList`의 기존 계약을 사용한다.
다른 담당자의 Nav2·작업 배정·A4 게이트를 변경하거나 구동 명령을 보내지 않는다.

## 동작

**현재 운영 상태: 허가 정책 미합의로 새 허가 발급 보류.** 영역 승인이 있어도 별개로 보류한다.
새 요청은 `AUTHORITY_INVALID`로 거절하고 상태는 `valid=false` 및 `BLOCKED_UNKNOWN`으로 게시한다.
기존 owner/grant와 과거 요청 응답은 보존한다. 과거 granted=true는 현재 권한이 아니며,
유효한 완전 이탈 증거가 있는 기존 허가의 정상 해제는 가능하다. 다중 허가 기능을 추가한 것은 아니다.
아래 단일 소유권 알고리즘은 비활성 개발 구현으로 남겨 두며 합의된 운영 정책으로 간주하지 않는다.
정책 활성화는 팀 합의 후 코드 검토를 거쳐 진행하며 현재 launch에는 활성화 옵션을 제공하지 않는다.

## 합의를 가정한 개발 구현

- 전체 관리 로봇의 신선한 B3 관측이 있어야 통로 상태를 확정한다. 미수신·UNKNOWN·만료면 BLOCKED_UNKNOWN이다.
- 발급이 활성화됐다고 가정한 시험에서는 D2 현재 배정과 A4 실제 STOPPED를 검사한다.
- 비활성 개발 구현은 통로당 owner/grant 하나를 저장하며, 요청 판정·상태 전이·영속 응답을 SQLite 트랜잭션으로 직렬화한다.
  로봇당 통로 허가 1개 제한은 미합의 사항으로 적용하지 않는다.
- 비활성 개발 구현은 B3 INSIDE → OCCUPIED로 판정한다. 현재 보류 상태에서는 계속
  BLOCKED_UNKNOWN을 게시하며 OUTSIDE만으로 기존 허가를 자동 해제하지 않는다.
- 해제는 현재 owner/grant와 일치하고 허가 이후의 신선한 완전 이탈 관측이 있어야 한다.
  다른 로봇이 통로 안이거나 불명이어도 해제하지 않는다.
- 통신 단절·시간초과·재시작은 이탈 증거가 아니다. 재시작은 BLOCKED_UNKNOWN으로 복원하되 owner/grant를 보존한다.
- 대기 요청은 로봇별 FIFO 순번을 유지한다. 새 요청이 없으면 대기표만 5초 뒤 만료한다(개발값).
  활성 허가는 이 시간으로 만료시키지 않는다. 대기 중이면 새 request_id로 재요청한다.
- 같은 서비스·request_id는 저장된 같은 응답을 돌려주며 내용 변경은 DUPLICATE_REQUEST다.
  과거의 granted=true 응답은 현재 진입 권한 증명이 아니다. 소비자는 최신 state의 owner/grant/revision을 확인해야 한다.
- 정책 보류 중 기존 owner의 새 request_id도 AUTHORITY_INVALID로 거절한다.
  합의를 가정한 시험에서는 배정·정지·통로 상태를 다시 검사한다. 만료된 배정은
  ASSIGNMENT_STALE, 정지 미확인은 STOP_NOT_CONFIRMED, 불명 통로는 CORRIDOR_BLOCKED로 거절한다.
  거절해도 기존 owner/grant는 보존하며, 동일 request_id 재전송은 과거 응답을 재현한다.

## ROS 연결

| 방향 | 경로 | 타입 |
|---|---|---|
| 입력 | `/ax/<robot>/perception/corridor_observation` | CorridorObservationList |
| 입력 | `/ax/<robot>/safety/actuation_state` | ActuationState |
| 입력 | `/ax/fleet/tasks` | GroupTaskList |
| 서비스 | `/ax/fleet/corridor/request_entry` | RequestCorridorEntry |
| 서비스 | `/ax/fleet/corridor/release` | ReleaseCorridor |
| 출력 | `/ax/fleet/corridor/state` | CorridorStateList |

관측 1초·구동 상태 0.3초·작업 목록 3초의 sim 시각 최신성과 벽시계 무수신 기한을 함께 확인한다.
STATE QoS는 reliable/transient_local/depth 1이다. 상태는 변경 시 + 1Hz로 발행하며,
내용 변경 때 영속 revision이 증가한다. 원장은 기본 `~/.ros/ax_corridor_manager/corridors.sqlite3`다.
DB 오류는 무효 상태/거절로 고정하며 정상으로 자동 복귀시키지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash  # 작업공간에서 colcon build 후 실행
ros2 launch ax_corridor_manager corridor_manager.launch.py use_sim_time:=true
python3 tools/tests/test_d3_corridor_manager.py
python3 tools/tests/test_w3_contracts_ros.py
```

기본 영역은 B3 `ax_warehouse_regions.json`이며 미승인 설정에서는 허가를 발급하지 않는다.
`regions_file`, `robot_ids`(기본 robot_1·robot_2), `database` 파라미터로 구성한다.
관리 로봇을 누락시키면 그 로봇을 관측할 수 없으므로 실제 fleet 구성과 같아야 한다.
Gazebo와 함께 실행할 때 `/clock`이 필요하다. 실제 시계를 쓰는 환경에서는 `use_sim_time:=false`로
명시하고 모든 입력 노드도 같은 시계를 사용해야 한다. 0.2초 내부 타이머는 만료 감시용이며
변경 없는 상태를 5Hz로 내보내지 않는다.

## 검증 경계

`test_d3_corridor_manager.py`와 기존 허가·해제 회귀 시험은 **시험 인스턴스에서만**
`grant_policy_approved=True`를 지정한다. 이는 실제 팀 합의나 운영 노드 활성화를 뜻하지 않는다.
`test_d3_contract_review.py`의 PendingPolicyTests는 실제 기본값인 발급 보류, 기존 기록 보존,
정상 이탈 해제를 검사한다. ROS 계약 시험도 시험 인스턴스만 활성화하며 실제 기본값과 구분한다.

W3 단독·계약 시험이며 실제 D1 요청, D2 배정, B3 실주행, A4 게이트까지의 종단 통과는 아니다.
시험의 승인 영역·B3·D2·A4 입력은 임시 데이터다. 운영 영역 승인을 대신하지 않는다.
통로 진입 전 검증된 안전 대기 지점 정차와 경로 계획은 D1/D2가 담당한다.
SROS2 노드 권한 구성과 다른 담당자의 점유·허가 소비 경로는 공동 통합 시 확인해야 한다.
다른 로봇 관측이 만료됐을 때 해제를 보류하는 것은 의도한 보수적 정책이다. owner가 OUTSIDE여도
통로 전체가 비었다고 확정하지 않으며 owner/grant를 보존한다. 전체 관리 로봇의 신선한 관측이
회복되고 다른 로봇이 INSIDE/UNKNOWN이 아니면 새 request_id로 해제를 재시도한다.
완전 이탈과 통로 비움을 분리하는 정책 변경은 D1·D2·A4와 별도로 합의해야 한다.

사유 코드 추가 제안과 상대 구현 경계는 [계약 인계 문서](../../docs/guides/C3_D3_CONTRACT_HANDOFF.md)를 따른다.

## 2026-11-23 상대 변경 연동

A4 `ActuationState`만 Schema 2로 받는다. 구형·미지원 schema 또는 출처 없는 입력은 이전 정지
근거를 무효화한다. 신선한 STOPPED와 실측 속도 기준을 함께 확인하며, 만료 시 내부 정지 표시도
내린다. `drive_allowed`·`arm_allowed`는 명령 허가이므로 정지 판정으로 대신 사용하지 않는다.
예를 들어 HANDOFF_LOCK으로 명령이 차단됐어도 실제 정지 증거는 유효할 수 있다.

새 D2의 개발 fixture와 실제 `task_list_fields` 결과를 내 수신 콜백에 직접 전달해 형식 호환을
검사한다. 이는 오프라인 시험이다. D2 개발 토픽을 운영 `/ax/fleet/tasks`로 remap하거나 실제
A3 실행 포트로 연결하지 않는다. D2의 live 포트는 여전히 상대 패키지에서 미지원이다.

`tools/tests/test_d3_team_inputs.py`는 새 정지 형식·만료·잘못된 입력과 D2 직렬화 연결을,
`tools/tests/test_b3_b4_battery_handoff.py`는 내 B3 출력과 상대 B4 배터리 차단의 결합을 검사한다.
정상 배터리도 미승인 영역을 우회하지 않으며 LOW 강제 정지 같은 새 정책은 추가하지 않는다.

검증: 관련 13개 패키지 재빌드 후 연동·회귀·스타일 78건, A3/B3/C3 추가 검사 92건,
상대 A2/B2/B4 원본 33건, A4 원본 정지 2건이 통과했다. A4는 최신 시험이 요구하는
`AX_GATE_BIN`을 현재 설치본 실행파일로 명시했다. 합의 domain이 없는 C3 ROS 검사 8건은
건너뛰었다. 전체 저장소·Gazebo 종단 시험을 새로 모두 실행한 결과는 아니다.

### D3(W2) 정지 메타 추가 검증

`node.py`의 `stop_evidence_remaining`은 A4 메시지만 대상으로 Schema 2, 공백이 아닌
source_id·source_session_id, revision > 0, valid=true를 검사한다. 정지 증거는 양의 ROS stamp와
0~999,999,999 범위의 nanosec를 가져야 하고, 미래 표본 또는 나이 300ms 이상이면 무효다.
초 단위 float 대신 정수 ns로 판정해 정확한 만료 경계와 큰 ROS 시각에서도 1ns를 구분한다.
B3 관측·D2 배정 메시지의 기존 schema 규칙은 바꾸지 않았다.

추가 시험에서 공백 식별자, 정확히 300ms 된 표본의 float 오차, 비정규 nanosec 합산이라는
세 누락을 재현하고 수정했다. 무효 입력은 이전 정지 근거를 지우며 정상 세션 이력은 교체하지
않는다. 기존 세션·revision 검사는 세션 재시작 후 옛 세션 재등장과 낮은 revision을 차단한다.

최종 검사: `tools/tests/test_d3_stop_metadata.py`의 9개 시험을 포함해 D3 회귀·스타일 57건,
현재 A4 실행파일을 사용한 원본 정지 시험 2건이 통과했다. D3 빌드도 성공했다.
원본 로그는 로컬 docs/reports/d3_stop_metadata_*_20261123 및 d3_metadata_*_20261123에 있다.
첫 재현 결과 3 failed/4 passed와 수정 후 결과를 별도로 보존한다.

D3는 속도 세 개와 STOPPED 보고를 검증한다. 0.5초 연속 정지의 표본 축적은 생산자 A4가
담당하며 D3가 대신 수행하지 않는다. source_id 문자열 검사는 출처 필드 검증이지 노드 인증이
아니다. 이번 검사는 직접 콜백·격리 원장과 A4 합성 입력 시험이며 실제 Gazebo 종단 검증 또는
미합의 domain에서의 D3 DDS 시험 완료를 뜻하지 않는다. 영역·허가 정책 승인은 유지 보류다.

### 후속: 실제 ROS2/Gazebo Schema 2·배터리 연결 확인

2026-11-23 `run_kim_gazebo_schema2_review.py`로 설치된 원본 simulation launch, 두 로봇의
Gazebo 물리·제어기·sim_battery, A4/B4와 내 D3를 실제 실행했다. 기존 김성민 시험 domain
138~140에서 잠금·그래프 충돌 검사를 거쳤고 Gazebo partition도 매번 분리했다.
원본 설정 파일은 수정하지 않았으며 임시 설정에서 battery_required=true만 지정했다.
모든 주행·팔·운영 허가 플래그와 영역·통로 승인 보류는 유지했다.

최종 run_3: robot_1 배터리 27개, Schema 2 STOPPED 수신·D3 수락 85회; robot_2 배터리 28개,
Schema 2 STOPPED 148회·D3 수락 147회. 종료 직전에는 두 로봇 모두 유효한 STOPPED이고
D3 정지 근거도 미만료였다. robot_2는 격리 Gazebo set_pose로 패드에 배치한 뒤 DISCHARGING
→ CHARGING → FULL 전환을 확인했다. 자율 주행·자동 도킹 성공을 뜻하지 않는다.

LiDAR·B2 위치 추정·승인된 B3 실제 입력은 이번 Gazebo 실행에 포함하지 않았다(sensors=false).
따라서 B4의 UNKNOWN과 A4 명령 차단은 예상 결과이며, 배터리 정상만으로 주행이 열리지 않았다.
실제 주행 중 LOW/CRITICAL 대응과 전체 센서 경로까지 통과했다고 볼 수 없다.

상대 원본의 보호팔 Schema 2·상위 launch 매개변수 실제 ROS 시험 6개, A2/B2/B4 및 내 B3/D3
경계값·스타일 시험 59개도 통과했다. 후자는 모의 입력을 포함하며 실제 Gazebo 시험과 구분한다.
run_2에서는 controller_manager 조회의 일시적 DDS timeout 경고가 있었다. 원인을 확정하거나
완전히 해결한 것은 아니며, run_3의 기동·최종 상태 확인과 별도로 남긴다.

첫 실행에서 내 시험 도구가 ROS_HOME을 자식 종료 전에 지우는 정리 오류를 발견해 종료 순서를
수정했다. SIGINT 종료 뒤의 exit -2·KeyboardInterrupt 로그는 의도한 시험 종료와 구분한다.
실제 결과·명령·임시 설정·원본 로그는 로컬 docs/reports/gazebo_schema2_battery_20261123/
run_1~3와 schema2_real_ros_consumers_20261123.*, schema2_battery_boundary_regression_20261123.*에 있다.

### 2026-11-27 D2 개발 입력 선택과 상대 호출 호환

`fleet_tasks_topic` 파라미터로 내 D3가 읽을 작업 목록을 고른다. 기본값은 기존 운영 경로
`/ax/fleet/tasks`이며, 명시하면 D2가 사용하는 `/ax/dev/d2_mock/fleet_tasks` 같은 개발 경로도
읽을 수 있다. 상대 D2의 발행 코드·토픽을 바꾸거나 운영 경로로 remap하지 않는다.
경로 선택은 배정 승인·통로 정책 승인이 아니며 기본 발급 보류와 영역 미승인은 유지한다.
상대 구동 토픽·상대 경로·상위 디렉터리 표기 등은 시작 전에 거절한다.

기존 Gazebo/ROS 시계와 D2 발행자가 별도로 준비된 환경에서 내 실행 설정만 선택한다.

```bash
ros2 launch ax_task_executor team_handoff.launch.py with_w3:=true \
  fleet_tasks_topic:=/ax/dev/d2_mock/fleet_tasks
```

이 명령은 상대 D2·D1·C4 노드를 시작하지 않는다. 양쪽 시계 설정은 같아야 한다.
`test_d3_d2_selected_topic_ros.py`는 원본 D2 개발 발행자에서 실제 DDS로 두 로봇의 작업
목록을 수신하고, 발행 중단 후 만료와 통로 발급 보류·운영 토픽 미발행을 확인한다.
실제 주행·조작이나 운영 배정 승인 시험은 아니다.

`test_a3_c4_tool_handoff.py`는 원본 C4의 ToolCommand 생성 → 원본 A2의 CheckCommandLease
생성 → 내 A3 판정·SQLite 기록을 직접 연결한다. 부모 Goal·실행 ID·세대 보존과 배정 미확인,
철회·만료·다른 Goal 거절을 검사한다. 전송·C4 내부 runtime 준비·배정 응답·lease 등록은 시험
대역이므로 실제 dispatcher·D4·lease 발급기·C4 조작 종단 완료를 의미하지 않는다.
C3 후보·IK·궤적 공급자는 아직 별도 연결 대상이며 새 C4가 생겼다고 자동 허가하지 않는다.

### B3/D3 실제 ROS2 발행 확인 — 2026-11-27

격리 Gazebo와 원본 미승인 영역 설정으로 내 B3(robot_2/AX_2)·D3를 실행했다.
`/ax/robot_2/perception/corridor_observation` 101개와 `/ax/fleet/corridor/state` 20개를
실제 DDS로 수신했다. 세 통로 모두 B3 UNKNOWN·position_valid=false이며 사유는
REGIONS_NOT_APPROVED다. D3는 meta.valid=false, BLOCKED_UNKNOWN, owner·active_grant_id가
빈 상태였다. robot_2의 실제 request_entry 서비스 응답은 granted=false·grant_id 빈 값·
REGIONS_NOT_APPROVED였다. 빈 owner가 물리 비점유를 확정하는 것은 아니다.

이는 현재 승인 상태의 정상 차단 결과다. 운영 OUTSIDE·position_valid=true를 확인한 것은 아니다.
별도 `test_b3_outside_evidence_ros.py`에서는 원본을 수정하지 않은 임시 승인 사본과 합성 B2
원점 pose·health로 세 통로 OUTSIDE·position_valid=true가 실제 ROS로 발행되는 것을 확인한다.
이 사본 시험을 실제 지도·TF·로봇 외곽·경계 공동 승인 근거로 사용하지 않는다.

새 grant는 영역 승인, 합의된 허가 정책, 현재 배정, 신선한 실제 STOPPED, 신선한 통로 관측과
빈 통로·대기 순서 조건이 모두 필요하다. 현재 영역 승인과 정책 승인은 둘 다 보류 상태다.
현장·실제 Gazebo 주행으로 경계를 검증하기 전 승인값을 바꾸지 않았다.
근거는 로컬 docs/reports/lease_corridor_review_20261127/gazebo_configured/summary.json,
b3_outside_fixture_20261127.json과 해당 ROS 시험 로그에 보존했다.
