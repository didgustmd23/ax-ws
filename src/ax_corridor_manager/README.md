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
