# 이종수 W3 검증 기록 — 2026-11-23

기준은 `main b2816cb`와 로컬 D1 실행 관리·입력 감시 변경이다. WSL Ubuntu 24.04의 ROS 2 Jazzy에서 별도 작업공간, ROS 도메인, Gazebo 파티션을 사용했다.

## D1 주행·정차

- **초기 위치 정정.** 최초 시험에는 Gazebo world 출발 좌표 `(-6.8, 0, 0)`를 B2의 map 좌표 인자에 넣었다. 지도 revision 2의 개발 검증 기준에서 같은 출발점은 map `(0, 0, 0)`이다. 아래 최초 시험의 상태 표본은 보존하며 위치 일치의 근거로 사용하지 않는다.
- **정정 재시험.** 기본 창고 월드에서 `robot_2`의 Gazebo world 좌표가 `(-6.8, 0)`임을 [실측](map-fixed-world-pose.txt)했다. `initial_poses:=robot_2=0,0,0`으로 AMCL을 시작했고, 로그에서 `Setting pose: 0.000 0.000 0.000`, B2 `OK`, 초기 산포 확인용 새 추정 3개를 확인했다. 좌표 재시험에서 Nav2 활성화 서비스 응답은 확인하지 못했고 이동 목표는 보내지 않았다.

정정한 B2 실행 명령(시뮬레이터와 B1을 같은 시험 도메인에서 먼저 실행):

```bash
ros2 launch ax_localization localization.launch.py robot_ids:=robot_2 \
  map:="$PWD/src/ax_localization/maps/ax_warehouse/map.yaml" initial_poses:=robot_2=0,0,0
```

- **Nav2 활성화 재확인.** 별도 Gazebo 실행에서 같은 map 출발점을 사용했다. B2가 `OK`에 도달한 뒤 Nav2를 시작하고 lifecycle 관리 서비스에 STARTUP을 요청했다. [서비스 응답](map-fixed-nav-start.txt)은 `success=True`였고, manager 로그에서 controller·planner·BT navigator가 모두 활성화됐다. 초기 TF 조회의 시각 차이는 이후 해소됐다. 별도 CLI lifecycle 조회는 `Node not found`를 반환했다.
- 같은 실행에서 [A4 상태](map-fixed-a4.txt)는 유효한 출발점 `STOPPED`와 `drive_allowed=false`였다. `SAFETY_STOP`, `TRAVEL_POSE_UNVERIFIED`, `HANDOFF_LOCK`, `SIMULATION_DRIVE_DISABLED`가 남아 이동 목표는 보내지 않았다.

- 관련 패키지 9개를 빌드했다. 실행 원장·입력 감시 단위 시험 15개와 모의 Nav2 통신 검사를 통과했다. 통신 검사에서 목표 UUID 일치, 중복 전송 방지, 수락 거절, 위치 손실 및 위치·전후방 스캔 만료 시 취소, 입력 회복 후 보류 유지를 확인했다.
- 초기 위치 정정 전 Gazebo 시험에서 기본 창고 월드와 `robot_2`를 실행하고 AX_2 정지 판정 역할 설정, B1 전처리, B2 AMCL·위치 감독기, Nav2를 연결했다. 지도 서버·AMCL·Nav2가 활성화됐고, B2 위치 상태가 `valid=true`, `OK`에 도달했다.
- A4 **Schema 2**는 실제 odom을 바탕으로 `valid=true`, `STOPPED`를 발행했다. 이 표본의 `drive_allowed=false`와 차단 사유는 `SAFETY_STOP`, `TRAVEL_POSE_UNVERIFIED`, `HANDOFF_LOCK`, `SIMULATION_DRIVE_DISABLED`였다.
- 같은 시험에서 전·후방 필터링 스캔을 수신했다. B4 표본은 위치와 두 센서를 유효로 표시했지만 전체 판단은 `SENSOR_STALE`로 이동을 차단했다.
- 남은 W3 통합 시험은 B4 정상 입력과 승인된 주행 자세·D2/D3 허가가 모두 갖춰진 상태에서 Nav2 목표를 보내고, 목표 도달 뒤 새 A4 정지 근거를 확인하는 것이다. 이번 시험에서 관측한 A4 정지는 출발점 정지다.

## C1 영상 수집 착수

- 기본 창고 월드의 격리된 실행에 0.2 m 시험 상자 `test_box`를 추가하고, 기존 수집기로 임시 Gazebo RGB-D 카메라를 사용했다. 수집기는 깊이로 보이는 상자 영역을 YOLO 라벨로 저장하고 종료 때 카메라를 제거했다.
- 결과는 작업공간의 `vision_data/box_w3_20261123/`에 있다. 학습 10장, 검증 5장, 평가 5장으로 나눴고, 상자 없는 표본은 각각 2장, 1장, 1장이다. 이미지와 라벨의 파일명 대응 및 양성 라벨의 클래스·정규화 값 범위를 확인했다.
- 초기 수집 범위는 단일 시험 상자와 가상 카메라다. 로봇 B1 카메라 영상, 다른 물품·실물 환경 수집, YOLO 학습은 별도 후속 작업이다.
