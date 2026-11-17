# 김성민 B3/A3 검증 기록과 공동 시험 절차 — 2026-11-17

기준 코드: main `346d7bf`. 이 문서는 기존 검증 근거를 정리하며 타 담당자의 확인·승인을 대신하지 않는다.
다른 담당자에게 메시지를 전송하거나 답변을 받은 기록은 없다. 아래 확인 요청은 전달·회신용 초안이다.

## 1. 지금 확인된 상태

| 항목 | 분류 | 근거와 한계 |
|---|---|---|
| B3 revision 2·지도/월드 해시·초기 변환 대조 | 정적 검증 완료 | 생성 파일 일치와 정적 점검. 실제 좌표 정합 완료가 아님 |
| 두 로봇·3통로·양방향·6경계 단계 | 정적 계산 72개 통과 | SDF와 개발 외곽으로 계산. 실제 센서/주행 관측 아님 |
| B3 TF·health·시각·관측 메시지 | 모의 입력 ROS 검증 | 실제 관측 노드, 가짜 TF/시계/health. 실제 B2 AMCL과 구분 |
| B3→B4 정상·미승인·만료·빈 관측 | 실제 소비 함수 검증 | B4 원본 함수 연결. 실제 A4 구동 차단까지의 종단 시험 아님 |
| B3 실제 진입·완전 이탈·불명 시 차단 | 미검증 | 공동 Gazebo 시험 필요. approved=false 유지 |
| A3 배정·세대·활성 Goal·lease 만료/철회 | 단위/회귀 검증 | 제어된 시계와 시험 응답·권한 등록 사용. 실제 D4 배정 승인 아님 |
| A2→A3 명령 확인 왕복 | 개발 정책 ROS 검증 | 실제 A2 클라이언트 코드 사용. 실제 공구 구동·D4 권한 체계 검증 아님 |
| A3→D4 초기 대기·원장 보존 | 실제 소비 코드 검증 | D4 main 콜백·검증/원장 함수 직접 호출. 전체 DDS 종단 아님 |
| A3 실제 배정·lease 승인 | 미검증 | 실제 D4 assignment_check 응답 및 실제 작업 dispatcher 연결 필요 |

검증 완료는 해당 열의 범위에만 적용한다. 미승인은 결함을 숨긴 성공이 아니라 정상 관측·권한으로
사용할 근거가 아직 없다는 뜻이다. B3 영역 승인과 D3 허가 정책 승인은 별도다.

## 2. 기존 결과와 재현 위치

2026-11-15: 내 단위/소비자/스타일 162 통과, 기존 격리 ROS 40 통과.
상대 검사 37 통과/설치 경로 조건 1 실패. 상세는 [revision 2 기록](KIM_REVISION2_INTEGRATION_2026-11-15.md).

2026-11-17 기존 재검사: 계약·소비자 코드 121 통과, 기존 격리 ROS 11 통과, 신규 domain 미설정 27 skip.
이 27개는 합의할 기능 27개가 아니라 아직 실행하지 않은 시험 건수다.
A3 개발 부하 시험: 4클라이언트·4스레드·모의 Action·WAL/FULL·새 요청 1000개와 수락 기록 1000개,
p99 36.8ms·최대 59.2ms. 과거 지연 초과 원인이 해결됐다는 판정은 아니다.

개인 PC 원본 로그(현재 GitHub 업로드 제외):

- docs/reports/revision2_handoff_20261115/{unit,ros,peer}.{log,xml}, readiness.json.
- docs/reports/handoff_review_20261117/{contracts,ros}.{log,xml}, latency.json, RESULTS.md.

실제 소비 코드 검사: tools/tests/test_team_consumers_offline.py,
test_kim_d4_entrypoint.py, test_a3_d4_idle_receiver.py.
배정/lease 검사: test_a3_runtime_authority.py, test_a3_idempotency_timer_review.py,
test_a3_assignment_monitor.py. 새 시험도 tools/tests 안에만 둔다.

이번 문서·핵심 주석 보완 후 재검사: B3 정적 점검은 72개 경계 사례 통과,
approved=false/ready_for_approval=false 유지. 아래 단위·회귀·실제 소비 함수 검사 58개 통과.
새로운 실제 ROS/Gazebo·D4 배정 승인 시험을 수행한 결과가 아니다.

```bash
# Ubuntu에서 ROS Jazzy와 자신의 최신 설치본 setup.bash를 source한 뒤 저장소 루트에서 실행
python3 tools/generate_b3_regions.py --check
python3 tools/check_b3_region_readiness.py
python3 -m pytest -q \
  tools/tests/test_b3_revision2_handoff.py \
  tools/tests/test_b3_region_readiness.py \
  tools/tests/test_b3_review_regressions.py \
  tools/tests/test_a3_runtime_authority.py \
  tools/tests/test_a3_assignment_monitor.py \
  tools/tests/test_a3_idempotency_timer_review.py \
  tools/tests/test_team_consumers_offline.py \
  tools/tests/test_a3_d4_idle_receiver.py \
  tools/tests/test_kim_d4_entrypoint.py
```

이번 원본 결과는 개인 docs/reports/validation_record_20261117/의
readiness.json, pytest.log, results.xml에 저장했다. 주석 추가 파일의 실행 로직은 변경하지 않았다.

## 3. 담당자에게 확인할 내용 — 회신 전에는 미확인

| 담당 | 전달할 질문 | 받아야 할 근거 | 현재 |
|---|---|---|---|
| 종수 A1 | 현재 두 로봇의 접힌 팔·공구·선반·적재 외곽 및 관절 제한의 검증/승인 범위는? | 모델 commit/해시, 외곽 수치·자세/적재 조건, 제한 파일, 결과 문서·승인 범위 | 회신 미수신 |
| 현승 B2 | revision 2 실제 map→odom→base TF 정합·오차와 적용 월드는? | 지도/월드 해시, 같은 시각의 실제/추정 자세, 오차·지연, 지원 조건 | 회신 미수신 |
| 현승 A2 + 승연 A4 | 바퀴 정지 실패 원인과 실제 STOPPED 후속 결과는? | 측정 창·시계·기준값, 실제 속도/odom, STOPPED 토픽, 실패/수정/재검사 로그 | 회신 미수신 |
| 승연 D4 | 실제 assignment_check 요청 소비·응답 발행 구현이 준비됐는가? | 실행 명령, 경로/QoS, D세션/sequence/기한 규칙, 배정 원장과 거절 사례 | 회신 미수신 |
| 현승 D2 + 성민 A3 | 실제 배정과 로컬 작업의 접수·영속화 연결이 준비됐는가? | 배정/실행 ID·세대, 로봇 ID, dispatcher 연결, 활성 Goal/lease 발급·철회 경로 | 공동 확인 대기 |

추가로 현승 님과 시험 domain 배정을 확인한다. AX_KIM_TEST_DOMAINS에 임의 번호를 넣지 않는다.
main의 현재 자료만으로 타 담당자의 개인 PC 작업 완료 여부를 단정하지 않는다.

## 4. B3 실제 좌표·주행 공동 검증 — 아직 실행하지 않음

1. 정적 대조와 기록은 지금 진행한다. 실제 주행은 정지 실패 후속 결과와 시험 환경을 먼저 확인한다.
2. 기준은 revision 2의 근거가 있는 질감 월드로 고정한다. 단색 월드는 지도 적용 pending이므로 따로 검증한다.
   같은 /clock·use_sim_time과 합의한 격리 domain, 코드·지도·영역·모델 해시를 기록한다.
3. 종수/현승/성민이 Gazebo 실제 자세와 B2 map 자세·B3 경계를 같은 시각에 대조한다.
   T_world_map 초기값 (-6.8,0,0)은 검증용이며 운영 정합 승인 근거가 아니다.
   허용 오차·외곽 여유·허용 지연은 측정 전에 공동 확정하고 미정이면 not_judged로 남긴다.
4. 두 모델·통로 3개·양방향에서 접근 전/접근/최초 접촉/진입/출구 겹침/완전 이탈을 기록한다.
   중심점만 빠져나간 상태를 완전 이탈로 보지 않는다. 실제 적재·팔 자세와 외곽도 함께 기록한다.
5. B2 위치 손실·TF/health 만료·시계 이상에서 UNKNOWN 전이, B4 무효 판단, A4 차단을 공동 확인한다.
6. 운영 파일은 approved=false로 유지한다. 동작 경계 시험에는 별도 임시 영역 사본을 사용하고,
   사본의 approved=true는 관측 시험 조건일 뿐 운영 승인이나 구동 허가가 아님을 로그에 명시한다.
7. 사례별 실제/추정 위치·관측 상태·기대 상태·전이 지연·오차·판정과 검토자를 남긴다.
   실제 근거와 공동 검토가 끝난 경우에만 파일 변경을 검토한다. 이번에는 승인하지 않는다.

## 5. A3 실제 배정·lease 공동 시험 — 아직 실행하지 않음

구동 없는 권한 통신 시험은 B3 승인과 병렬로 준비할 수 있다. 실제 D4 응답과 접수·영속화한
실제 작업의 dispatcher 연결이 선행 조건이다. 현재 CONFIRM_LOAD 모의 Action이 이를 대신하지 않는다.

- policy=operational을 사용한다. dev_allow의 허용이나 시험에서 만든 응답을 실제 승인으로 기록하지 않는다.
- 실제 D2/D4 배정의 robot/assignment ID·generation과 A3 execution/세션을 기록한다.
- C 조작 권한은 부모 Goal UUID·lease 세대·만료/철회와 연결한다. A3 로컬 작업 권한과 구분한다.
- 배정 응답 전 거절 → 올바른 배정·활성 Goal·lease가 준비된 새 요청의 허용 → CONTINUE 확인.
- 다른 로봇/배정/세대/Goal, 지연·옛 응답, 만료·철회·D4 단절/재시작은 거절되는지 검사한다.
- 같은 request_id·같은 digest는 계약상 결과/수락 기록 규칙을 지키고, digest 변경은 DUPLICATE_REQUEST인지 확인한다.
  배정 전 거절한 ID를 배정 후 재사용해 허용을 기대하지 않는다. 회복 검사는 새 ID를 사용한다.
- 각 요청·응답·수락 원장 수·기대/실제 사유를 남긴다. 실제 팔·공구 구동은 별도 단계로 검증한다.

## 6. 실제 시험 결과 기입 양식

아래는 미작성 양식이다. 빈칸을 통과나 승인으로 해석하지 않는다.

| 항목 | 기록 |
|---|---|
| 시험 ID·일시·실행자·관련 담당자 | 미실행 |
| 코드/지도/월드/영역/모델 commit·해시 | 미기록 |
| domain·시계·모델/하중/팔 자세·설치 경로 | 미기록 |
| 합의한 허용 오차·지연·정지 기준 | 미합의 |
| 실행 명령·입력·기대/실제 결과 | 미기록 |
| 원본 로그/rosbag/원장 위치 | 미기록 |
| PASS/FAIL/not_judged·실패 원인·재검사 | 미판정 |
| 검토자·승인 범위·남은 제한 | 미승인 |
