# 김성민 시험 실행 환경 보완 — 2026-11-18

다른 담당자의 C2·매핑 소스와 원본 시험 파일은 수정하지 않았다.
이번 변경은 내 실행 환경의 외부 install 선택과 시험 프로세스 분리다.
C2 DDS 전송 순서의 근본 원인을 고쳤다는 의미는 아니다.

## 매핑 설치 경로

원본 test_world_selection.py는 --install-base를 주지 않아 저장소 내부 install을 선택한다.
현재 환경은 /home/freem/ax_main_install이므로 의도한 robot_id 검사 전에 다른 설치본을 발견해 거절한다.
정상 경로 선택은 내 호출부에서 --install-base를 명시하는 것으로 해결한다.

tools/tests/test_kim_mapping_external_install.py는 다음을 실제 상대 스크립트로 검사한다.

1. 명시한 설치본에서 월드를 읽고 robot_id 불일치 사전검사까지 도달한다.
2. 지원하지 않는 월드는 그대로 거절한다.
3. 없는 설치본이면 셸의 다른 설치본으로 우회하지 않는다.

robot_2 경로에 robot_1을 의도적으로 지정해 Gazebo 시작 전에 종료한다.
기대 종료 코드는 2이며, 실제 주행 성공을 확인한 시험이 아니다.
원본 시험의 기본 install 전제는 바꾸지 않았으므로 외부 설치 셸에서 그 명령만 그대로 실행하면 여전히 실패할 수 있다.

## C2와 ROS context의 분리

이전 같은 프로세스 전체 실행에서 C2가 10.1초 빈 검출을 먼저 처리한 뒤 10.0초 검출을
OUT_OF_ORDER_DETECTION_ARRAY로 거절했다. ASSEMBLY_TIMEOUT도 기록됐다.
단독 C2 시험은 3개 통과했지만 시간·전송·실행 순서의 근본 원인은 확정되지 않았다.

tools/tests/run_kim_team_review.py는 core → C2 원본 설치 시험 → A4 원본 정지 시험을
각각 별도 Python 프로세스로 순차 실행한다. ROS context·import·환경 누적의 영향을 분리하고,
core에서 분리한 C2/A4 사례를 별도 단계에서 다시 실행한다. 자동 재시도는 하지 않는다.
한 단계라도 실패하면 전체 종료 코드가 1이며, 실패 로그와 단계별 종료 코드·skip 수를 summary.json에 남긴다.
별도 단계가 통과해도 원래 같은 프로세스/부하 조건의 C2 문제가 해결됐다고 기록하지 않는다.

core의 원본 매핑 1개는 외부 설치 경로를 명시하는 내 세 검사로 구분해 실행한다.
원본 기대 결과나 상대 입력 검사·관측 만료·역순 거절 기준을 완화하지 않는다.
A4 원본은 기존 run_external_a4_stop_review.py로 바이너리 경로만 외부 설치본에 맞춘다.
기본 domain을 쓰는 스캔 노드 1개는 계속 제외하며, 신규 합의 domain 없는 27개는 skip한다.

## 실행 방법

ROS Jazzy와 검증할 최신 설치본을 source한 Ubuntu 셸에서 저장소 루트로 이동해 실행한다.
기존 로그를 덮지 않도록 실행마다 새 output 경로를 준다.

```bash
python3 tools/tests/run_kim_team_review.py \
  --install-prefix /home/freem/ax_main_install \
  --c2-prefix /home/freem/ax_c2_review_install \
  --output docs/reports/failure_resolution_20261118/new_run \
  --stage all
```

경로는 이 PC의 예시다. 다른 PC에서는 본인의 실제 설치 경로를 지정한다.
C2 prefix는 원본 시험이 요구하는 격리 설치본이어야 한다.
--stage mapping/c2/a4/core는 각 단계를 단독 진단할 때만 사용한다. 부분 실행을 전체 실행으로 표시하지 않는다.
all_stages_passed는 실행한 단계의 종료 코드 판정이며, skip된 시험·실제 Gazebo·운영 승인까지 통과했다는 뜻이 아니다.

## 확인 결과

- 외부 설치 경로를 명시한 매핑 검사 3개 통과.
- 전체 단계 실행: core 1127 통과/29 skip, 독립 C2 원본 3 통과, 외부 설치 경로 A4 원본 2 통과.
  각 단계 종료 코드 0, 실행한 검사 합계 1132 통과/실패 0. 위 단독 매핑 3개는 core에도 포함되므로 중복 합산하지 않는다.
- core에서 분리한 7개 중 C2 3개/A4 2개는 별도 단계에서 실행했다. 남은 원본 기본-install 매핑 1개는
  내 외부-install 검사로 대조했고 기본-domain 스캔 노드 1개는 미실행이다. 신규 domain 27개와
  기존 copyright/선택 texture 각 1개의 skip은 통과가 아니다.
- summary.json은 all_stages_passed=true와 has_skipped_tests=true를 함께 기록한다.
- 이 결과는 프로세스 분리·외부 install 명시라는 변경된 실행 조건의 결과다. 원본 같은 프로세스 조건에서
  실패한 C2 DDS 순서/조립 시간초과 원인을 확정하거나 코드로 제거했다는 뜻은 아니다.
- 전체 단계 실행의 결과와 원본 로그는 docs/reports/failure_resolution_20261118/all_1의
  summary.json, core.log/xml, c2.log/xml, a4.log에 기록했다.
- B3 approved=false와 D3 허가 정책 보류는 그대로 유지한다.

## 추가 경계·반복 검사

내 c2_handoff.py에 observation_ttl_s=10**400을 입력하면 math.isfinite의 float 변환에서
OverflowError가 발생하는 사례를 재현했다. 정상 운영 입력에서 발생한 실패와 구분한다.
내 연결부의 유한값 검사에서 OverflowError도 CONSTRAINTS_INVALID 보류로 처리하도록 수정하고
핵심 주석을 추가했다. 다른 담당자의 C2·매핑 소스/시험은 변경하지 않았다.

- 추가 TTL·시각 누락·검증기 장애를 포함한 관련 회귀·소비자·스타일 76개 통과.
- 내 실행 도구의 C2 실패 전달·기존 로그 보존 검사 2개 통과.
- 독립 C2 원본 시험을 3회차 실행해 회차별 3개, 총 9회 모두 통과. 실패 후 자동 재시도한 결과가 아니다.
- 원본 기록은 docs/reports/additional_review_20261118/의 edge_before.log,
  regression.log/xml, runner.log와 c2_round_1~3에 남겼다.

이는 전체 1132개 검사를 다시 실행한 결과가 아니라 추가 입력 경계·반복 검사다.
C2 같은 프로세스 조건의 근본 원인과 실제 Gazebo 종단 검증은 여전히 미완료다.
