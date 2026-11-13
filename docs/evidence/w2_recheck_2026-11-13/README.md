# 이종수 W2 최신 모델 재검증 — 2026-11-13

기준 커밋 `06f6308`의 소스와 최신 AX_2 모델로 B1 센서 일관성·오류 처리와 D1 Nav2 기본 구성을 재검증했다.
현재 AX_2 footprint는 x −0.63~0.67 m, y −0.53~0.53 m(1.30 × 1.06 m)다.
D1 README의 이전 폭 2.06 m 문구를 정정했으며 런타임 설정은 변경하지 않았다.

## 결과

| 검사 | 결과 |
|---|---|
| 빌드 | `ax_navigation ax_perception ax_control ax_localization`까지 의존성 포함 9개 패키지 성공 |
| 자산·물리 생성물 | `generate_asset_v2.py --check`, `generate_physical_properties.py --check` 통과 |
| 단위 검사 | A1 자산 8, W1 모델 12, AX_2 개정 7, B1 처리 14, Nav2 설정 3, DDS 프로필 5: 총 49개 통과 |
| `robot_1_calibration.json` | 전방 RGB-D·점군·LiDAR·장착 TF, 1·2 m 각 3표본 통과 |
| `robot_2_calibration.json` | 최신 AX_2로 같은 6표본 검사 통과 |
| `arm_sensors.json` | robot_1 epick, RGB-D 10·15 cm 및 ToF 8·15 cm 각 3표본: 12표본 통과 |
| `sensor_faults.json` | 합성 ROS 입력의 시각·frame·동기화·무효값·단절·회복 13조건 통과 |
| `navigation_mock.json` | 두 로봇별 lifecycle 3개 active, 각각 78 pose의 경로 계산, 현재 footprint 적용 통과 |

전방 깊이 최대 절대 오차는 robot_1 약 0.831 mm, robot_2 약 0.00048 mm,
LiDAR는 각각 약 1.357 mm, 0.0105 mm였다. 이상적 시뮬레이션의 작은 표본이며 제품 정확도가 아니다.
전방 개발 허용값 15 mm, 팔 20 mm는 변경하지 않았다.

## 실행 조건·재현

- WSL Ubuntu 24.04 / ROS Jazzy / Gazebo Harmonic. 공백 없는 WSL 내부 소스 사본에서 빌드했다.
- `colcon build --packages-up-to ax_navigation ax_perception ax_control ax_localization --parallel-workers 2`.
- `/opt/ros/jazzy/setup.bash`와 해당 사본의 `install/setup.bash`를 source했다.
- 센서 거리 검사는 `tools/empty_test_fixture.py`의 `write_empty_world()`로 생성한 빈 바닥을
  launch의 `test_fixture_world`에 지정했다. 실제 창고에서의 가림·주행 검증은 아니다.
- 단계별 ROS domain은 171(합성 오류), 172(Nav2 모의), 173(robot_1 전방), 174(robot_2 전방), 175(팔),
  Gazebo partition은 각 단계에 고유하게 지정했다. 단계 종료 후 실행한 launch를 종료했다.
- 전방: `ax_description robot_test.launch.py gui:=false robot_ids:=<robot> tool:=epick`와
  `ax_perception sensors.launch.py robot_ids:=<robot> tool:=epick` 실행 후
  `python3 tools/check_w2_calibration.py --robot-id <robot> --output <새 결과 경로>`.
- 팔: `ax_control simulation.launch.py gui:=false robot_ids:=robot_1 tool:=epick`에 같은 시험 치구 지정,
  robot_1 B1 실행 후 `python3 tools/check_w2_arm_sensors.py --output <새 결과 경로>`.
  A2는 관절 유지·상태 제공에 사용하며 팔·바퀴 작업 명령은 보내지 않았다.
- 합성 오류: Gazebo 없이 robot_2 B1을 실행하고 `check_w2_sensor_faults.py` 사용.
- Nav2: `ax_navigation navigation.launch.py`와 `check_w2_navigation.py` 사용.
  모의 지도·TF·scan·odom·clock으로 ComputePathToPose만 요청했다.
- 각 도구의 전체 인자와 별도 터미널 절차는 B1/D1 README를 따른다. 재실행할 때 새 출력 경로를 사용한다.

원본 콘솔·launch·빌드 로그와 실행 스크립트는 `log/w2_recheck_2026-11-13/`에 보존했다(Git 제외).
`summary.json`은 단계별 종료 코드, `static_checks.json`은 생성물·단위 검사 종료 코드다.
`manifest.json`은 현재 소스·설정·모델 Xacro와 결과 JSON의 SHA-256을 기록한다(CRLF→LF 정규화).

## 과거 기록과의 관계·남은 범위

2026-11-09 W2 manifest의 추적 소스 중 현재와 해시가 다른 파일은 `a1_validation.yaml`이었다.
이번 Nav2 결과에서 최신 AX_2 footprint 적용을 직접 확인했다. 과거 결과와 당시 manifest는 그대로 보존한다.

기존 팔 25 cm 시험판 가림 실패를 이번 검사로 해결한 것은 아니다. 관측 가능한 근거리의 일관성만 재확인했다.
전체 팔 시야·작업 자세, 실물 센서 내/외부 보정, IMU 오차·4대 장시간 부하·실제 절전 복귀는 검증하지 않았다.
Nav2 출력은 TwistStamped이고 제어기 직접 발행자는 없었다. 실제 B2/A4 통합 주행·정차,
운영 lease·통로 허가, W3/W4·G1 통과는 이번 결과에 포함하지 않는다.
