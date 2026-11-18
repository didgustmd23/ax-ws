# 김성민 담당과 현재 상대 구현의 연결 점검 — 2026-11-17

후속: 11-18 main 776773c에서 A4의 피드백 기반 STOPPED 판정과 C2 표면 계산·어댑터가 추가됐다.
아래 A4 UNKNOWN 고정·C2 미구현 설명은 346d7bf 당시 기록이다. 현재 상태는
[11-18 점검](KIM_MAIN_776773C_REVIEW_2026-11-18.md)으로 구분한다.

기준 main 346d7bf 및 로컬 B3/A3 기록·핵심 주석 보완 위에서 작업했다.
다른 담당자의 소스·설정·시험은 수정하지 않았다. 전체 물리 작업의 통합 완료를 주장하지 않는다.

## 이번에 내 쪽에서 연결한 범위

- team_handoff.launch.py의 기존 A3/B3 실행을 유지하고 with_w3=true 선택 시 내 C3/D3를 추가했다.
  같은 로봇 ID·영역 파일·시뮬레이션 시계를 사용한다. D3는 로봇별 복제 없이 전체 목록을 받는 하나다.
  C3는 명시된 AX_1 모델에 시작한다. 이는 현재 개발 조작 모델 선택이며 실제 D2 역할 배정을 대신하지 않는다.
- 로봇·모델 조합과 옵션을 노드 생성 전에 검사한다. 기본 policy=operational, 지도 revision 2를 유지한다.
- 실제 B3 생성 메시지를 D3 원본 수신 콜백에 전달해 정상/미승인/만료 관측을 확인했다.
- A4 ActuationState와 D2 GroupTaskList 계약 fixture를 내 D3 수신 콜백으로 검사했다.
  STOP_REQUESTED/UNKNOWN/속도 초과는 정지로 보지 않고, 보류·연기·만료 작업은 유효 배정으로 보지 않는다.
  이 입력은 시험 fixture이며 상대 A4/D2가 실제로 정상 발행한다는 증거가 아니다.
- 내 package.xml에 선택 실행에 필요한 ax_grasp_planner/ax_corridor_manager 의존성을 추가했다.
- 기존의 실제 B4/D4 소비 함수·A2 lease 클라이언트 연동 검사를 함께 사용한다.

## 연결 경로와 현재 상대 구현에서 필요한 것

| 경계 | 현재 연결/검증 범위 | 추가로 필요한 것 |
|---|---|---|
| A1→B3/C3 | 현재 모델 외곽·기하·제한 파일을 읽어 정적 검사 | 종수 A1의 실제 자세/적재 외곽·관절 제한 검증 범위와 승인 근거 |
| B1→B2/B4→내 B3 | 내 B3는 B2 health/TF를 소비. B1 센서를 임의 직접 배선하지 않음 | 실제 센서 시각·좌표·불명 조건을 담당자와 공동 검증 |
| B2→B3 | /ax/<robot>/localization/health와 map→<robot>/base_link, revision 2 | 현승 B2의 실제 정합·오차/지연 근거. 정적 초기 변환은 운영 승인 아님 |
| B3→B4→A4 | 실제 B3 출력과 실제 B4 소비 함수 검사 통과 | 실제 B4/A4 노드·같은 시계·정지까지 종단 검사 |
| A4→D3 | safety/actuation_state를 기존 타입으로 수신 | 승연 A4의 실제 STOPPED 발행 및 정지 독립 검증 |
| D2→D3 | /ax/fleet/tasks에서 유효한 현재 배정만 수신 | 현승 D2의 실제 작업 목록 발행·배정/보류 원장 |
| D3→D1/D2/A4 | corridor/state와 request_entry/release 서비스 제공 | 종수 D1·현승 D2·승연 A4가 실제 허가·점유·만료를 소비하는 연결 |
| A2/A4→A3 | check_command 서비스와 실제 A2 클라이언트 왕복 검사 | 실제 공구·팔 구동 및 실제 운영 권한 체계 공동 검사 |
| A3→D4 | executor/status 실제 D4 콜백·원장 직접 연결 검사 | 실제 DDS 전체 연결 확인 |
| D4→A3 | assignment_check 요청/응답 수신 경로 준비 | 승연 D4의 실제 배정 확인 응답·세션/순번/기한 규칙과 원장 |
| D2→A3→C4/A4 | 내 모의 Action과 operational 권한 검사만 있음 | 내 A3 실제 dispatcher·lease 발급/철회·운영 작업 구현, 현승 D2와 승연 C4의 실제 실행기 |
| C2/C4↔C3 | 표면·제약을 받는 Python 계산/선택 함수와 후보 게시 제공 | 현승 C2의 실제 표면/관측 입력, 내 C3와 승연 C4의 내부 호출 연결·도달/충돌 검증기 |
| C3→D4 | candidates 게시 제공 | 현재 D4는 A3 상태 수신만 확인됨. 후보 소비·검증 연결 필요 |
| E1/E2/E3/E4 | 현재 작업에서 연결할 실제 Agent 기능 확인 안 됨 | 이후 주차의 구현·인증/계약 확인. 규칙/권한을 임의 우회하지 않음 |

## 소스에서 확인한 실제 차단 조건

1. B3 영역은 approved=false이며 REGIONS_NOT_APPROVED/UNKNOWN을 내는 것이 현재 의도다.
2. D3는 grant_policy_approved=False다. 정상 입력 수신만으로 정책을 승인하거나 새 허가를 만들지 않는다.
3. A4 safety_gate.cpp는 낮은 속도가 0.5초 유지된 분기에서도 독립 정지 검증 전에는 UNKNOWN을 낸다.
   따라서 현재 A4만으로 내 D3의 STOPPED 선행 조건이 충족됐다고 판단할 수 없다.
4. 같은 A4는 operational_permits_verified 파라미터를 사용하며 현재 구현에서 D2/D3 메시지 직접 구독은 없다.
   내 D3 게시만으로 실제 허가가 최종 구동 게이트에 자동 반영되는 구조가 아니다. 파라미터를 임의 true로 켜지 않았다.
5. 현재 D4에는 AssignmentCheckRequest/Response 처리 구현이 없다. 실제 배정 승인 시험은 아직 불가하다.
6. 내 A3도 운영 집기·하역·후퇴·인계 작업이 미구현이다. 이 부분을 상대 미구현 탓으로만 분류하지 않는다.
7. C3의 reachability/collision 미제공은 VALIDATORS_UNAVAILABLE로 빈 결과다. 후보 게시 노드 실행이 집기 연결은 아니다.

이 항목은 런타임 예외와 미구현·미승인에 따른 정상 거절을 구분한 결과이며, 타 담당자의 실제 PC 완료 상태는 별도다.

## 검사와 실행

추가 검사는 tools/tests/test_kim_team_launch.py와 test_d3_team_inputs.py에만 만들었다.
로봇 ID·한 개의 D3·AX_1 C3 선택·동일 시계·정책 미승인 유지와 실제 수신 콜백의 입력 처리를 검사한다.
다른 상대를 가짜 승인 노드로 대체해 정상 운영이라고 표시하지 않는다.

- 내 전체 관련 단위·회귀·소비자·스타일 실행: 147개 통과. 가져온 fixture 클래스의 반복 사례 포함.
- 내 담당 패키지 4개 빌드 성공, 설치된 launch --show-args에서 새 옵션 확인. 이 명령은 노드를 시작하지 않았다.
- 신규 통합 launch 전체의 실제 DDS 기동 시험은 새 domain 합의가 없어 미실행.
- 기존 전용 domain을 예약한 A3/B3 ROS 재검사 11개 통과. 실제 A2 lease 클라이언트 왕복과
  모의 TF/health B3 관측이며 실제 Gazebo 종단은 아니다. 개인 로그 ros.log/xml에 기록했다.
- 합계 158개 통과. 형식 검사 경고(다중 스레드 프로세스의 fork 경고)는 있었으나 실패는 없다.

팀이 배정한 ROS_DOMAIN_ID와 상대 노드·/clock 준비 후 실행한다. 중복 실행기를 먼저 종료하고
시험 원장은 corridor_database로 분리한다. 이 문서의 준비 기록 자체가 주행/영역/허가 승인은 아니다.

```bash
# ROS와 최신 자신의 설치본을 source한 셸에서 실행. domain은 팀 합의값을 별도 설정.
ros2 launch ax_task_executor team_handoff.launch.py with_w3:=true
```

이번 개인 로그: docs/reports/team_integration_20261117/{unit,ros}.{log,xml}, build.log,
launch_args.log, latency.json. 승인 상태·실제 공동 시험 절차는
[B3/A3 검증 기록](KIM_B3_A3_VALIDATION_2026-11-17.md)을 따른다.
