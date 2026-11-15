# 김성민 담당 — 지도 revision 2 연동 점검

기준 main: `492b504`. B2 지도 revision 2와 이전 B3 영역·기본 실행 인자 revision 1의 불일치를 수정했다.
다른 담당자 구현·지도·월드·시험은 수정하지 않았다. 관련 설치본은 외부 빌드 폴더에서 갱신했다.

## 수정한 동작

- B3 생성기·생성 영역·관측 노드와 A3/B3 팀 launch의 기본 지도 번호를 2로 맞췄다.
- B2 초기 변환은 revision 1과 같은 (-6.8, 0, 0)이다. 기존 통로·접근·작업대·대기 지점의 수치 경계는 보존했다.
  지도 이미지 원점과 최적 정합값을 영역 변환으로 대신 사용하지 않았다.
- 생성기는 B2 map_info와 applicable_worlds의 지도 번호 및 초기 변환을 검사한다.
  다음 지도 교체 때 번호만 자동 추종하지 않고 불일치하면 재검증을 요구한다.
- 오래된 revision 1 입력은 유효한 위치를 주더라도 UNKNOWN으로 거절한다.
- 운영 영역은 approved=false, D3 허가는 기존 합의 전 보류를 유지한다.
  지도 번호 일치는 실제 TF·경계 관측·운영 승인을 뜻하지 않는다.
- 단색 월드는 영역 형상만 같으며, B2의 revision 2 지도 적용 근거는 아직 pending이다.

## 확인 결과

- 정적 점검: 지도·월드 해시, revision, 초기 변환, 두 로봇 외곽 기준 72개 경계 사례 통과.
- 내 단위·회귀·실제 상대 소비 함수·스타일 시험 162개 통과.
- 기존 전용 domain을 잠금 예약한 B3 관측 및 A3 Action·lease·다중 로봇 ROS 시험 40개 통과.
  B3는 가짜 TF/시계/health를 사용하며 실제 B2 AMCL·Gazebo 주행 시험이 아니다.
- A3 지연: 1,000회 요청·수락 기록 1,000개, p99 31.5ms, 최대 40.1ms. 기존 기준 통과이며 과거 원인 확정은 아니다.
- 상대 원본 매핑 설치 경로·지도 적용 목록·B4·D4 시험: 37개 통과, 설치 경로 조건 실패 1개.
- 외부 빌드 갱신: 관련 ROS 패키지 9개 성공. 제품 소스 변경과 설치본 갱신은 구분한다.

새 시험은 tools/tests/test_b3_revision2_handoff.py에 두었다. 미래 지도 번호·변환 변경 거절,
두 로봇 launch 기본 인자 전달, 옛 지도 관측 거절, 미승인 유지와 생성 파일 일치를 검사한다.
기존 B3 기하 시험은 현재 영역 번호를 사용하도록 고쳤으며, 불일치 차단 시험은 계속 유지한다.

## 남은 설치 경로 조건

tools/tests/test_world_selection.py:115는 매핑 스크립트에 --install-base를 주지 않아 저장소 내부
install을 선택한다. 이 환경의 외부 설치본은 /home/freem/ax_main_install이므로 다음 오류로 실패한다.

```text
선택한 설치 폴더 밖의 ax_description을(를) 찾음(다른 설치본이 source된 셸?):
/home/freem/ax_main_install/ax_description/share/ax_description
(설치 폴더 /mnt/c/Users/freem/OneDrive/바탕 화면/ax-ws-main/install)
```

최신 상대 스크립트의 --install-base /home/freem/ax_main_install을 명시하면 이 검사를 통과한다.
robot_2 경로에 robot_1을 지정한 안전한 사전검사로, 실제 시뮬레이터를 띄우기 전에 다음 정상 거절까지
도달함을 확인했다. 해당 상대 소스나 시험을 임의로 변경하지 않았다.

```text
경로·월드 불일치: 경로 robot_id 'robot_2' ≠ 실행 로봇 'robot_1'
```

## 미완료 범위

신규 ROS 시험 domain 합의, 실제 D4 배정 응답·C3 IK/충돌 검증기 연결, 두 로봇 Gazebo 종단 시험,
B3 실제 관측 승인과 D3 허가 정책 합의는 별도다. 과거 A3 지연 초과의 원인 해결도 완료로 선언하지 않는다.
이 기록은 전체 프로젝트의 모든 시험을 다시 실행한 결과가 아니다.

개인 원본 로그: docs/reports/revision2_handoff_20261115/{unit,ros,peer}.{log,xml},
build.log, latency.json, mapping_explicit_install.log.
