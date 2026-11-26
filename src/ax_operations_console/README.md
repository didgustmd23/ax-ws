# ax_operations_console — D2 통합 관제 화면 (1단계)

이 패키지는 상태 Topic을 **구독만** 하는 노드와, **GET·HEAD만** 받는 웹 화면으로 이루어진다.
화면은 하나이고 정상 운영에서는 조작할 일이 없다(설계 5.5절, DEC-24).
중계 방식은 계약 8절(표시 중계 노드, 응답은 표시 전용)을 따른다.

## 실행

```bash
# 개발: 작업 목록은 /ax/dev/d2_mock/fleet_tasks. with_fixture:=true면 D2 개발 출력(fixture)을 같이 띄운다
ros2 launch ax_operations_console console_dev.launch.py with_fixture:=true
# 운영 이름: 작업 목록은 /ax/fleet/tasks(D2 운영 발행자는 아직 없다), use_sim_time 기본 true
ros2 launch ax_operations_console console.launch.py
```

브라우저에서 `http://localhost:18780/` 을 연다. WSL이면 윈도 브라우저에서 같은 주소로 연다.
종료는 launch 터미널에서 Ctrl+C로 한다.

| 인자 | 기본 | 설명 |
|---|---|---|
| `port` | 18780 | 8765(tandem 관제)·9090(rosbridge 기본)은 거부한다 |
| `bind_address` | 127.0.0.1 | IP 주소만 받는다. 루프백이 아니면 시작할 때 외부 노출 경고를 남긴다 |
| `use_sim_time` | 운영 true / 개발 false | 표본 나이 판정에 쓰는 sim time |
| `robots_file` | `config/console_robots_dev.yaml` | 로봇 목록. 화면에 "출처: 개발 설정"으로 표시한다 |
| `map_yaml` | ax_localization `maps/ax_warehouse/map.yaml` | 지도 파일은 읽기만 한다 |
| `with_fixture`·`fixture_file` | false·ax_fleet_manager `dev_fixture_transfer.json` | 개발 launch 전용 |

노드 매개변수 `future_tolerance_s`(0.1)와 `display_grace_s`(0.5)는 launch 인자로 내놓지 않았다.
바꿀 때는 `--ros-args -p`로 준다.

## 입력과 생산자 현황 (1단계)

| 화면 항목 | Topic | 만료(s) | 생산자 |
|---|---|---|---|
| 작업·배정·인계 | `/ax/fleet/tasks` (개발 `/ax/dev/d2_mock/fleet_tasks`) | 3.0 | 운영 발행자 없음, 개발은 `group_dev_publisher` |
| 구동·허가 | `/ax/<robot>/safety/actuation_state` | 0.3 | A4 `safety_gate` |
| 안전 | `/ax/<robot>/safety/state` | 0.5 | B4 `b4_risk_node` |
| 위치 신뢰 | `/ax/<robot>/localization/health` | 1.0 | B2 감독기 |
| 로컬 작업 | `/ax/<robot>/executor/status` | 3.0 | A3(개발 모의 실행기 포함) |
| 지도 위 위치 | `/ax/<robot>/amcl_pose` | 5.0(표시 기준) | B2 AMCL. 계약 4절 표 밖 표시 전용 |
| 배터리 | `/ax/<robot>/battery_state` | 5.0(표시 기준) | 모의 배터리. 계약 4절 표 밖 표시 전용(best effort로 구독) |

원천이 없는 항목(운영 모드, 보류, 실패 이력, 적재 `load/state`, `FleetSnapshot.robots`)은 화면 아래에 "원천 없음"으로 적는다.
값을 지어내지 않는다.
로봇 목록과 가동 로봇 정의는 `RobotSummary`(D4)가 생기면 그것으로 바꾼다.

## 판정 규칙 (`console_state.py`, 계약 2.3)

- **상태 5종:** `missing`(미수집)·`ok`·`stale`(만료)·`invalid`(무효)·`no_source`(원천 없음).
  정상이 아닌 값은 0이나 빈칸으로 표시하지 않는다. 서버도 정상일 때만 `value`를 보낸다.
  위치가 없거나 만료되면 좌표를 보내지 않는다.
- **만료:** 표본 나이(노드 clock, `use_sim_time`)와 무수신 시간(`CLOCK_BOOTTIME`) 가운데 하나라도 만료 기준(계약값)을 넘으면 만료다.
  표본 나이가 음수(시스템 시계 역행)이면 판정은 그대로 두고, 화면은 `stamp_age_shown_s`(0으로 자름)와 `clock_regressed` 플래그로 "시계 역행" 라벨을 보인다(원값 `stamp_age_s`는 상세 표에 남긴다).
- **미래 stamp:** 현재 sim time보다 `future_tolerance_s` = **0.1 s** 넘게 앞선 stamp는 받지 않는다(횟수만 센다).
  0.1 s는 같은 sim clock을 쓰는 발행자와 이 노드 사이의 `/clock` 전달 지연을 넘는 여유로 잡았다.
  stamp 0도 받지 않는다.
- **세션·순서:**
  - 같은 세션에서 revision이 줄거나, 새 revision인데 stamp가 줄면 버린다.
  - 같은 revision이 다시 오면 내용은 두고 stamp·수신 시각만 갱신한다(주기 재발행).
    시스템 시계가 역행해 같은 revision의 stamp가 줄면 버리지 않고 수신 시각만 갱신한다. stamp는 큰 값을 유지한다(`stamp_regressed_same_revision`).
  - 새 세션을 받은 뒤에는 이전 세션 메시지를 버린다.
  - `valid=false`, `revision=0`, 시계 0은 무효다.
- **표시 여유:** 판정 `status`는 계약 만료값 그대로 낸다.
  화면 깜박임을 줄이려고 `display_status`만 만료 기준을 넘은 지 `display_grace_s` = **0.5 s** 미만일 때 직전 정상 표시를 유지한다.
  이때 화면에서는 신선도 점에 테두리가 붙고, 상세 표에 "(판정 만료)"가 나온다.
  KPI `running`은 표시 기준으로 세고, `running_strict`는 판정 기준으로 센다.
- **가동 로봇:** 구동 상태가 신선·유효하고 FAULT·UNKNOWN이 아닌 로봇.

## 읽기 전용 보장

- 노드: publisher·service client·action client를 만들지 않는다.
  파라미터 Service도 끈다(`start_parameter_services=False`).
  graph에는 rclpy 기본 `/rosout`·`/parameter_events` 발행만 남는다.
- HTTP 서버가 다루는 요청:
  - 경로 화이트리스트(`/`, `/static/*`, `/api/state`, `/map/info.json`, `/map/map.pgm`)만 응답한다.
  - 쓰기 메서드는 405, 허용 Host(127.0.0.1·localhost·바인딩 주소 + 포트)가 아니면 403이다.
  - 오류 본문은 고정 문구이고 예외 내용은 노드 로그에만 남긴다. CORS 헤더는 보내지 않는다.
- `/api/state`는 화면 표시 전용이다. 다른 노드·Agent가 상태 원천으로 쓰지 않는다(계약 8절).
  `QueryBudget`·`RequestSafeStop` 같은 Service 기능은 이 패키지에 없다. SROS2 권한을 가진 별도 관제 노드가 2단계 이후 맡는다.

## 화면 (시안 b "Graphite Minimal")

- 구성: 상단 띠(출처·작업 Topic·시계), 왼쪽 KPI(가동 로봇 링 게이지·2분 추이, 작업 상태별 막대), 가운데 2D 지도(로봇 위치·방향, 역할별 모양, 만료는 반투명), 오른쪽 작업 목록, 아래 로봇 카드.
- 로봇 카드를 누르면 상세가 열린다. 상세에는 항목별 값·판정·나이·source·session·revision·valid가 나오고, 원본 JSON은 접혀 있다.
- 외부 자산은 쓰지 않는다. 글꼴은 시스템 글꼴이고, 차트는 SVG·canvas로 직접 그린다.
- 2분 추이는 화면이 열린 동안 브라우저가 기록한 것이다. 원장 이력이 아니며 새로고침하면 비워진다.
- 색 대비(패널 #202125 기준): 본문 13.6, 보조 5.9, 상태색 4.7~8.2(모두 4.5 이상).
- 3D 표시와 TF 기반 위치는 2단계다.

## 시험

```bash
source <격리 install>/setup.bash
python3 -m pytest tools/tests/test_console_state.py tools/tests/test_console_config.py \
  tools/tests/test_console_http.py tools/tests/test_console_ros.py tools/tests/test_console_web.py -q
```

`test_console_web.py`는 node로 문법을 검사하고 가짜 DOM 렌더 점검을 한다(브라우저 대체 아님, node가 없으면 실패).

`test_console_ros.py`는 ROS_DOMAIN_ID 60~69 중 빈 곳을 잡는다.
렌더링은 자동 시험이 없으므로 사람이 브라우저로 확인한다.

## 제한

- 운영 작업 목록(`/ax/fleet/tasks`) 발행자가 아직 없다. 운영 launch에서는 작업 칸이 "미수집"이다.
- amcl_pose는 이동이 있을 때만 갱신된다. 오래 정지한 로봇은 위치가 만료로 보일 수 있다(표시 기준 5 s). TF 합성은 2단계다.
- 화면 레이아웃은 1920×1080·1366×768을 기준으로 맞췄다. 실제 브라우저 확인은 사용자 수동 확인에 맡긴다.
