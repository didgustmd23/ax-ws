# 이종수 W2 개발 검증 — 2026-11-09

W2의 B1 보정·동기화 개발 검증과 D1 Nav2 기본 구성을 수행했다.
결과는 아래 조건에 한정한다. W3 주행·정차, 실제 B2/A4 통합, 운영 지도 채택 또는 G1 통과가 아니다.

## 보존 파일

| 파일 | 조건과 결과 |
|---|---|
| `robot_1_calibration.json` | 전방 RGB-D/점군·LiDAR, 1·2 m × 서로 다른 stamp 3개, 비중심 픽셀·장착 TF, 통과 |
| `robot_2_calibration.json` | 같은 검사, 선반 로봇, 통과 |
| `arm_sensors.json` | robot_1 epick, A2 관절 유지/상태, 팔 RGB-D 10·15 cm 및 ToF 8·15 cm × 3표본, 통과 |
| `arm_sensors_occluded.json` | 초기 팔 카메라 25 cm 평면 검사의 실패 원본. 중립 중심 광선 약 19.8 cm 전경 관측. 전체 `passed=false`를 보존 |
| `sensor_faults.json` | 합성 ROS 입력의 시각·frame·동기화·무효값·단절·회복 13조건, 통과 |
| `navigation_mock.json` | **모의** 지도/TF/scan/odom/clock, robot_1/2 Nav2 활성화·경로 계산·출력 경로, 통과 |
| `summary.json` | 측정 오차 요약, 자동 검사 217개, 환경과 범위 |
| `manifest.json` | 소스·설정·결과 파일의 SHA-256, 텍스트는 CRLF→LF 정규화 |

## 측정과 판단

- 전방 평면은 Gazebo의 로봇 실제 자세와 장착 설정을 기준으로 만든다.
  로봇의 미세 움직임을 반영해 해당 평면까지 광선 교차 거리를 계산하고 ROS 측정과 비교한다.
  비중심 픽셀을 써서 광학 X 오른쪽·Y 아래·Z 앞도 확인한다.
- 전방 거리 개발 허용값은 15 mm, 팔은 20 mm다. 제조사 정확도·안전 한계·팀 최종 평가값이 아니다.
- 전방 RGB-D 최대 절대 차이: robot_1 약 0.495 mm, robot_2 약 0.00048 mm.
  전방 LiDAR 최대 절대 차이: robot_1 약 1.357 mm, robot_2 약 0.0105 mm.
  이상적 시뮬레이터와 작은 표본의 일관성 값이므로 제품 성능으로 일반화하지 않는다.
- 팔 검사는 관측 시각 TF와 렌더링의 일관성이다. 팔 TF와 독립된 외부 보정값 추정은 아니다.
- RGB-D 동기화는 시각을 바꾸지 않고 기존 30 ms 범위에서 묶는지를 검사했다.
  실제 전방 검사 표본의 묶음 오차는 0 s였다. 합성 입력에서 40 ms는 거절, 20 ms는 수락했다.
- Nav2는 두 로봇별 3개 lifecycle 노드 active, 올바른 footprint·map·base frame,
  ComputePathToPose 각각 78 pose의 경로 생성, `nav/cmd_vel` TwistStamped와 제어기 직접 발행자 부재를 확인했다.
  구동 목표는 보내지 않았으며 A4가 명령을 차단/허용한다는 실제 검증은 하지 않았다.
- 전체 `python3 -m unittest discover -s tools/tests`: 217개 통과(103.199 s).
  후속 Nav2 관측 갱신 기대 시간 0.5 s 설정 뒤 관련 3개 검사와 두 로봇 Nav2 실행을 재확인했다.
  전체 시험 중 기존 B2 하네스의 미닫힌 로그 핸들 ResourceWarning은 있었으며 실패는 없었다.

## 알려진 한계·후속

팔 카메라 25 cm 시험판의 가림은 제거하지 않았다. 가림을 센서 오차로 보고 보정값을 바꾸지 않았다.
관측 가능한 10·15 cm의 통과와 가려진 25 cm의 실패를 함께 전달한다.
작업 자세·관심 픽셀 선정과 표면 관측 가능성은 C2/C4 통합 때 확인해야 한다.

실물 보정·전체 팔 자세/시야·RGB 단독 화질/검출 성능·IMU 오차 추정·정격률·장시간·4대 부하·실제 절전 복귀는 미검증이다.
Nav2는 개발용 설정이며 실제 지도·위치 신뢰성·A4 경로를 연결한 이동, 목표/UUID 영속화·취소·실패·복구는 후속이다.
robot_1 footprint는 epick·무부하·A1 주행 자세 한정이다. Nav2가 팔 자세를 강제하지 않는다.
타인 담당의 A2/B2/A4/D4 구현은 변경하지 않았다.

## 재현

[B1 안내](../../../src/ax_perception/README.md#w2-개발용-보정동기화-검증--2026-11-09),
[D1 안내](../../../src/ax_navigation/README.md)를 따른다. 각 도구는 기존 실행에 연결한다.
ROS_DOMAIN_ID·GZ_PARTITION을 실제 세션과 격리하고 검사 후 launch를 종료한다.

실행 환경: WSL Ubuntu 24.04, ROS Jazzy, Gazebo Harmonic, Nav2 1.3.13.
8개 패키지 격리 빌드 통과. 원시 로그는 `log/w2/`(Git 제외)에 두고 검토한 JSON만 이곳에 보존한다.
