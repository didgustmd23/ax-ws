# D3 통로 허가·점유 — 김성민 W3

`RequestCorridorEntry`, `ReleaseCorridor`, `CorridorStateList`의 기존 계약을 사용한다.
다른 담당자의 Nav2·작업 배정·A4 게이트를 변경하거나 구동 명령을 보내지 않는다.

## 동작

- 전체 관리 로봇의 신선한 B3 관측이 있어야 통로 상태를 확정한다. 미수신·UNKNOWN·만료면 BLOCKED_UNKNOWN이다.
- 새 허가는 D2의 현재 배정과 A4의 실제 STOPPED 확인이 있는 로봇에만 발급한다.
- 통로마다 활성 owner/grant는 하나이며, 요청 판정·상태 전이·영속 응답을 SQLite 트랜잭션으로 직렬화한다.
- B3 INSIDE → OCCUPIED. OUTSIDE만으로 허가를 자동 해제하지 않는다.
- 해제는 현재 owner/grant와 일치하고 허가 이후의 신선한 완전 이탈 관측이 있어야 한다.
  다른 로봇이 통로 안이거나 불명이어도 해제하지 않는다.
- 통신 단절·시간초과·재시작은 이탈 증거가 아니다. 재시작은 BLOCKED_UNKNOWN으로 복원하되 owner/grant를 보존한다.
- 대기 요청은 로봇별 FIFO 순번을 유지한다. 새 요청이 없으면 대기표만 5초 뒤 만료한다(개발값).
  활성 허가는 이 시간으로 만료시키지 않는다. 대기 중이면 새 request_id로 재요청한다.
- 같은 서비스·request_id는 저장된 같은 응답을 돌려주며 내용 변경은 DUPLICATE_REQUEST다.
  과거의 granted=true 응답은 현재 진입 권한 증명이 아니다. 소비자는 최신 state의 owner/grant/revision을 확인해야 한다.

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
STATE QoS는 reliable/transient_local/depth 1이다. 상태는 5Hz 재발행(계약 1Hz 이상),
내용 변경 때 영속 revision이 증가한다. 원장은 기본 `~/.ros/ax_corridor_manager/corridors.sqlite3`다.
DB 오류는 무효 상태/거절로 고정하며 정상으로 자동 복귀시키지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/freem/ax_main_install/setup.bash
ros2 run ax_corridor_manager corridor_manager --ros-args -p use_sim_time:=true
python3 tools/tests/test_d3_corridor_manager.py
python3 tools/tests/test_w3_contracts_ros.py
```

기본 영역은 B3 `ax_warehouse_regions.json`이며 미승인 설정에서는 허가를 발급하지 않는다.
`regions_file`, `robot_ids`(기본 robot_1·robot_2), `database` 파라미터로 구성한다.
관리 로봇을 누락시키면 그 로봇을 관측할 수 없으므로 실제 fleet 구성과 같아야 한다.

## 검증 경계

W3 단독·계약 시험이며 실제 D1 요청, D2 배정, B3 실주행, A4 게이트까지의 종단 통과는 아니다.
시험의 승인 영역·B3·D2·A4 입력은 임시 데이터다. 운영 영역 승인을 대신하지 않는다.
통로 진입 전 검증된 안전 대기 지점 정차와 경로 계획은 D1/D2가 담당한다.
SROS2 노드 권한 구성과 다른 담당자의 점유·허가 소비 경로는 공동 통합 시 확인해야 한다.
