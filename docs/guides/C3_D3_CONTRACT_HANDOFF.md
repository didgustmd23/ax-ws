# C3·D3 계약 인계 — 2026-11-13

발생 조건·상대 대응·복구 조건·시험 기준과 결정 기록 양식은
[A3·C3·D3 연결·사유 코드 검토 초안](A3_C3_D3_INTEGRATION_REVIEW_DRAFT.md)에 정리했다.
관련 담당자 검토 대기 자료이며, 코드 15종 합의나 실제 연결 완료를 뜻하지 않는다.
초기 MOCK_IDLE의 A3 표현 정리와 D4 최소 수신 구분은 로컬에 적용했다.
[초기 대기 인계 문서](A3_D4_IDLE_HANDOFF_PROPOSAL.md)에 적용 범위를 적었으며 실제 ROS 연결·담당 검토는 남아 있다.

## D-13 사유 코드 추가 제안

다음 15종은 서로 다른 복구 조치가 필요하므로 이름을 유지하여 D-13에 추가할 것을 제안한다.
아래 목록은 계약 담당자 검토용이며 D-13 승인 완료를 뜻하지 않는다.
D3 코드는 서비스 응답의 reason_code, C3 코드는 내부 Plan.reason 및 후보 상태의
meta.validity_reason에 사용한다. CORRIDOR_LEDGER_FAILED와 REGIONS_NOT_APPROVED는 D3 상태에도 사용한다.

| 담당 | 코드 | 뜻과 소비자 조치 |
|---|---|---|
| D3 | STOP_NOT_CONFIRMED | 신선한 A4 STOPPED 또는 실제 속도 한계 확인이 없음. 진입하지 말고 정지 확인을 기다림. |
| D3 | WAITING_TURN | 통로 FIFO에서 다른 로봇이 선순위임. 대기 후 새 요청 ID로 재시도. |
| D3 | GRANT_MISMATCH | 해제 대상의 owner/grant가 현재 원장과 다르거나 요청 필드가 누락됨. 최신 상태와 ID 확인. |
| D3 | EXIT_NOT_CONFIRMED | 허가 발급 이후의 신선한 완전 OUTSIDE 관측이 없음. 관측을 회복하고 해제 재요청. |
| D3 | CORRIDOR_LEDGER_FAILED | SQLite 판독·저장 실패. 서비스 거절과 상태 무효화를 고정하고 원장을 점검. |
| D3 | REGIONS_NOT_APPROVED | 통로 영역 설정이 미승인임. 운영 영역 승인 전 허가를 발급하지 않음. |
| C3 | CANDIDATE_STALE | 후보의 관측 기준 기한 또는 수신 후 벽시계 기한이 만료됐거나 현재 시계보다 미래임. 새 관측·계산 필요. |
| C3 | NO_VALID_CANDIDATES | 입력 후보가 없거나 명시적 IK·충돌 검사를 통과한 후보가 없음. 실행 금지. |
| C3 | INVALID_GEOMETRY | 관측 중심 또는 계산 스칼라의 차원·유한수 조건이 잘못됨. 관측·제약 값 확인. |
| C3 | OBSERVATION_STALE_OR_INVALID | 로봇·물체·좌표계·revision·시각이 누락/무효이거나 관측이 미래/만료임. 새 관측 필요. |
| C3 | CONSTRAINTS_INVALID | 공구 크기·하중·거리·기한이 양수가 아니거나 여유·신뢰도·평탄도 한계가 부적합함. 제약 수정. |
| C3 | TOOL_CONDITIONS_NOT_MET | 흡착 적합성·관측 신뢰도·질량·평탄도 조건을 충족하지 못함. 다른 전략 또는 재관측 필요. |
| C3 | VALIDATORS_UNAVAILABLE | 실제 도달성 또는 전체 경로 충돌 검증기가 전달되지 않음. 검증기 연결 전 실행 금지. |
| C3 | SURFACE_AXES_INVALID | 법선·접선이 유한한 3차원 비영벡터가 아니거나 서로 직교하지 않음. 표면 축 보정. |
| C3 | CONTACT_SURFACE_TOO_SMALL | 공구 접촉 폭·깊이와 가장자리 여유를 모두 담을 표면 공간이 없음. 다른 표면/공구 필요. |

## 허가와 해제의 의미

- 통로당·로봇당 허가 개수 정책은 미합의다. 로봇당 제한은 제거했으며 운영 노드는 새 허가 발급을
  보류한다. 영역 승인은 허가 정책 합의를 대신하지 않는다. 새 요청은 기존 `AUTHORITY_INVALID`로
  거절하고 상태는 `valid=false` 및 `BLOCKED_UNKNOWN`으로 게시한다. 기존 owner/grant·요청 이력은
  보존하며 정상 이탈 해제는 가능하다. 기존 단일 소유권 개발 구현과 합의 가정의 회귀 시험은
  남겨 두되 확정된 운영 정책으로 표현하지 않는다. 실제 정책 활성화는 팀 합의 후 별도 수정한다.
- 같은 요청 ID의 저장 응답은 역사적 응답이다. granted=true만으로 지금 진입할 수 없다.
  새 요청 ID의 owner 재확인은 신선한 배정·STOPPED·통로 상태를 검사한다. 관제 상태와
  A4 게이트·B3 최신 관측도 함께 확인해야 하며 허가 응답으로 안전 게이트를 우회하지 않는다.
- 다른 로봇의 관측이 UNKNOWN/만료일 때 owner의 완전 이탈 해제도 CORRIDOR_BLOCKED로 보류한다.
  이는 의도한 보수적 정책이다. 전체 관측 회복 뒤 새 요청 ID로 재시도하며, 자동 해제하지 않는다.

## 실행 및 검증 경계

- assignment_check EVENT QoS는 reliable/volatile/keep_last 50이다.
- corridor/state는 변경 시 발행하고 변경이 없으면 마지막 발행 1초 뒤 재발행한다.
  내부 0.2초 만료 감시는 출력 발행 주기가 아니다.
- 새 launch는 use_sim_time=true를 기본으로 전달한다. Gazebo의 /clock이 필요하며 실제 시계를
  쓸 때는 입력 노드도 함께 false로 설정한다. 개인 PC 설치 경로는 실행 예시에서 제외한다.
- A3의 assignment_check 요청 발행·응답 대조·주기 확인·만료/오류 무효화는 구현되어 있다.
  D4의 실제 응답 구현은 아직 없다. `test_a3_d4_handoff_ros.py`에는 모의 D4가 있으며,
  A3↔D4 결과는 이 시험 상대와의 검사다. 실제 D4 원장 연결 완료로 해석하지 않는다.
- C3에는 실제 IK·충돌 검증기와 C2/C4 호출 연결이 아직 없다. 단독 노드는 빈 후보를 발행하며
  launch 추가로 이 경계가 바뀌지 않는다. 실제 집기 성공은 검증하지 않았다.
- ROS 시험은 담당자와 합의한 domain 범위를 AX_KIM_TEST_DOMAINS에 넣었을 때만 실행한다.
  0·팀 Gazebo 범위·다른 시험에 배정된 범위를 대신 사용하지 않는다.

## 변경 전달

### 상대 소비 함수 직접 연결 시험

`tools/tests/test_team_consumers_offline.py`는 실제 ROS 메시지 생성 함수와 상대 원본 소비 함수를
직접 연결한다. ROS 노드·DDS 그래프·토픽 발행 없이 시계·센서·영역 승인 사본만 시험값을 사용한다.

- B3 실제 영역 판정·메시지 생성 → B4 원본 소비: 정상 OUTSIDE 수용, 영역 미승인 차단,
  과거 timestamp 차단의 3개 시나리오 통과.
- A3 모의 결과·상태 생성 → D4 원본 형식 검사·원장: 형식 호환 및 MOCK_ONLY의 UNKNOWN 처리 통과.
- 직접 연결 4개와 D3·lease·C3 회귀를 합쳐 47개 통과. 이 결과는 ROS 통신·전체 작업 실행·실물
  센서·Gazebo 주행·실제 집기 성공을 대신하지 않는다. D4 검사는 requirements.txt의 rfc8785가 필요하다.
- 초기 대기 진단에서 발생했던 D4 실행 ID 규칙 오류는 로컬 수신 검증 수정으로 해결했다.
  별도 test_a3_d4_idle_receiver.py에서 정확한 모의 대기는 UNKNOWN, 기존 작업 원장 변경 없음,
  손상·부분 실행 정보는 여전히 거절됨을 확인했다. 실제 ROS 연결과 계약 검토는 별도다.

앞으로 main에 직접 push하지 않는다. 작업 브랜치에서 변경과 검증 근거를 묶어 main 대상 PR로
제출한다. 기존 작업은 GK `1cda53d` 및 [PR #58](https://github.com/didgustmd23/ax-ws/pull/58)로
main(`5ebbeae`)에 반영했다. PR #56 → #57 되돌림 → #58 재반영 이력이 있다.
2026-11-14의 D3 정책 보류, C3 만료 후보 정리·재수신 방어, B3 승인 전 정적 검증 보완,
A3/D4 초기 대기 처리와 검토 문서는 별도 후속 변경이다. 반영 상태는 해당 커밋과 main 대상
PR 이력으로 확인하며 실제 ROS 연결·공동 계약 승인이 완료됐다는 뜻으로 해석하지 않는다.
