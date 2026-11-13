# A3 초기 모의 대기 → D4 처리 방향 초안

2026-11-14 · 상태: **A3 표현·D4 최소 수신 구분 로컬 적용, 담당자 검토·실제 ROS 연결 시험 대기**

## 문제와 선행 처리

A3가 작업 접수 전에 보내는 MOCK_IDLE에는 실행 정보가 없다. 수정 전 D4의 check_status는
ExecutionRef의 derive_rule_version과 실행 번호를 필수 검사해 오류 로그가 났다.
현재는 구조·로봇·schema 검사 뒤 정확한 초기 모의 대기를 구분하도록 D4 common_contract.py를
최소 수정했다. 기존 accept_status는 valid=false를 UNKNOWN으로 반환하므로 작업 원장을 쓰지 않는다.
A3가 가짜 task/assignment/execution ID를 넣으면 미배정 상태가 실제 작업처럼 보이므로 사용하지 않는다.

내 파트에서는 status_contract.py에 생성과 분류 함수를 추가하고 MockExecutor 초기 게시에 연결했다.
기존 메시지 타입·필드·토픽·코드 이름은 바꾸지 않았다. 다음 형태를 명시적으로 유지한다.

| 필드 | 초기 모의 대기 표현 |
|---|---|
| robot_id | 실제 대상 로봇 ID |
| meta.schema_version / revision | 1 / 1부터 시작, 내용이 바뀔 때만 증가 |
| meta.source_id | a3_mock_executor_<robot_id> |
| meta.source_session_id | 프로세스 시작마다 새 세션 |
| meta.header.stamp | 실제 게시 함수에서 현재 ROS 시각으로 갱신 |
| meta.valid / validity_reason | false / MOCK_ONLY |
| local_phase / paused | MOCK_IDLE / false |
| execution | ExecutionRef의 **모든 필드가 메시지 기본값**: 문자열 빈 값, 정수 0 |
| last_result | ResultStatus 기본값. 성공·실패 결과를 만들지 않음 |

이 형태는 ‘A3 모의 실행기에 현재 실행 정보가 없다’는 heartbeat다. **로봇 사용 가능·배정 해제·
물리 정지·작업 성공·실행 권한을 뜻하지 않는다.** 실제 운영 IDLE의 표현은 별도 합의 사항이다.

## D4 수신 순서와 적용 범위

아래 3번의 정확한 형태 구분을 실행 번호 검사 전에 적용했다. 기존 원장의 무효 상태 처리로
4~6번의 ‘권한·작업 원장에 반영하지 않음’을 확인했다. 별도 heartbeat 저장·표시·세션 재동기화는
추가 구현하지 않았다. 아래 운영 통합 방향과 공동 계약 확정은 여전히 담당자 검토 대상이다.

1. 공통 메시지 형식, 토픽의 expected_robot_id, 지원 schema, source/session/revision을 검사한다.
2. 같은 시계의 표본 나이와 무수신 기한을 확인한다. 오래된 메시지를 새 대기로 처리하지 않는다.
3. 위 표와 정확히 일치하는지 확인한다. execution_id만 비어 있거나 phase 이름만 MOCK_IDLE이면
   예외로 인정하지 않는다. execution의 일부 필드나 결과가 남은 메시지는 기존 오류 검사로 넘긴다.
4. 정확한 초기 모의 대기는 실행 번호 derive 검증 대상으로 보내지 않고 별도 모의/미확정 관측으로
   분류한다. 일반적인 meta.valid=false 전체를 검증 우회시키는 예외를 만들지 않는다.
5. 작업 원장의 latest/history, 배정, lease, 진행 중 작업 결과를 이 heartbeat로 갱신하거나 삭제하지 않는다.
   D4에 활성 배정이 있다면 ‘대기라서 가용’으로 바꾸지 않고 재시작·불일치 확인 대상으로 남긴다.
6. 정상 실행 메시지는 기존 실행 번호·세대·세션 검증을 그대로 유지한다.

별도 heartbeat 저장소나 화면 표시가 필요한지, 세션 전환 및 활성 배정 충돌을 어떻게 보고할지는
D4 담당자가 결정한다. 신규 사유 코드를 이 초안에서 임의로 추가하지 않는다.
김성민의 is_unassigned_mock_idle은 ROS 메시지 형태 분류용 참조 구현이며 D4의 전체 수신 검증이나
권한 판정을 대신하지 않는다. D4가 A3 패키지를 의존하도록 강제하지 않으며 dict 기반 구현을 검토할 수 있다.

## 함께 확인할 시험

| 입력 | 제안 기대 결과 |
|---|---|
| robot_1·robot_2의 신선한 정상 MOCK_IDLE | 실행 번호 오류 없이 모의 대기로 분류, 배정·작업 결과 변경 없음 |
| 같은 세션의 반복 heartbeat | 시각만 갱신, 추가 실행·결과·수락 기록 없음 |
| 새 세션의 MOCK_IDLE + D4에 이전 활성 배정 있음 | 이전 작업 완료·배정 해제로 단정하지 않고 재동기화 필요 표시 |
| execution_id만 비었지만 task/assignment 정보가 남음 | 단순 대기 예외로 우회하지 않고 기존 검증·충돌 처리 |
| MOCK_IDLE이지만 valid=true 또는 결과가 SUCCEEDED | 초기 모의 대기로 인정하지 않음 |
| 다른 robot_id·출처, 옛 세션·만료 표본 | 정상 대기로 원장에 반영하지 않음 |
| 정상 실행 상태와 결과 | 기존 D4 검증과 원장 기록 동작 유지 |

## 현재 검증 범위와 남은 작업

- tools/tests/test_a3_idle_contract.py: 두 로봇의 초기 표현, 부분 실행 정보 거절, 메타·결과 오분류 방지,
  실제 A3 publish_task_status의 시각 갱신을 ROS 그래프 없이 검사한다.
- tools/tests/test_a3_d4_idle_receiver.py: 실제 D4 검증·원장 함수에 A3 메시지를 직접 전달한다.
  두 로봇 대기, 기존 원장 보존, 부분 실행 정보·잘못된 메타/결과·손상 형식 거절, 정상 작업 검증을 확인했다.
- tools/tests/check_a3_idle_d4_handoff.py: 현재 로컬 D4 검증에서 초기 상태를 정상 구분한다.
- 오류 해결 요청에 따라 승연님 담당 D4의 common_contract.py 한 파일을 최소 수정했다.
  공통 ROS 메시지·토픽·코드 이름과 D4 배정 확인 구현은 변경하지 않았다.
- 합의된 domain의 실제 ROS 연결과 담당자 검토는 남아 있다. 함수 직접 연결 통과가 운영 완료는 아니다.
