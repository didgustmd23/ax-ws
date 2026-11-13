# 이종수 W2 보완 완료 — main 48a0ea6 / 2026-11-13

원격 main의 17개 추가 커밋을 반영하고 이전 W2 정정·증거를 보존했다.
W2의 B1 시뮬레이션 보정·동기화 개발 검증과 D1 Nav2 기본 구성에 대해
수신 품질, 장애물·입력 부족, 대표 팔 자세의 시야 조사와 인계 자료를 보완했다.
실물 보정, 실제 주행·정차(W3), 복구(W4), G1 완료를 뜻하지 않는다.

## 결과

| 구분 | 실행·결과 |
|---|---|
| 의존성·빌드 | rosdep 검사 충족, B1/D1·A2/B2·A3/B3 의존성까지 11개 패키지 빌드 성공. 기존 A4 deprecated 경고 있음 |
| 생성물 | 자산·물리 생성물 `--check` 2개 통과 |
| 단위 검사 | A1 8 + W1 모델 12 + AX_2 7 + B1 14 + Nav2 3 + DDS 5 + W2 측정 도구 4 + domain 예약 4 = 57개 통과 |
| 전방 거리·좌표 | 최신 robot_1/2, 1·2 m 각 3표본, 장착 TF·점군·비중심 픽셀·LiDAR 검사 통과 |
| 팔 RGB-D·ToF | A2가 관절을 유지하는 robot_1, RGB-D 10·15 cm 및 ToF 8·15 cm 각 3표본 통과 |
| 센서 오류·회복 | 합성 입력 13조건 통과 |
| 수신 품질 | 로봇별 15 sim s, 카메라 약 15.15 Hz·LiDAR 10 Hz·IMU 100 Hz, robot_1 ToF 약 30.30 Hz |
| 동기화·대응 | RGB-D 최근접 시각 차 최대 0 s, 관측된 입력/출력 stamp 불일치 0건. 중복·역행·로봇 frame 혼선 없음 |
| IMU | 정지 시뮬레이션 관측의 비유한 표본 0, 평균 Z 가속도 약 9.80665 m/s². 하드웨어 bias/noise 보정 아님 |
| Nav2 정상 | 두 로봇 각각 78 pose의 경로, 현재 footprint·TwistStamped·제어기 직접 발행자 부재 확인 |
| Nav2 장애물 | 두 로봇의 단거리 벽 우회 성공, 지도 전체를 가로막는 벽에서는 빈 경로·실패 확인 |
| Nav2 입력 부족 | map 미수신: 활성화될 수 있지만 경로는 실패. TF 미수신: 8 s 관측 동안 기동 성공하지 않음. scan 미수신: 경로 성공하지 않음 |
| Nav2 입력 만료 | 정상 경로 성공을 먼저 확인한 뒤 TF 또는 scan 발행 중단. 새 요청은 ABORTED·빈 경로 |
| 팔 시야 | neutral/reach_left/reach_right의 고정 기하 모델 × 좌/중앙/우 × 10/15/25 cm = 27조건, 조건당 3표본. 18조건 관측·9조건 가림 |

최종 런타임 검사 17개가 성공했다(`summary.json`). 시야 검사의 성공은 **조사 완료 및 10 cm 대조군 확인**이며
27조건 모두 관측 가능하다는 뜻이 아니다. 세 자세 모두 좌·중앙·우의 10·15 cm 표적은 관측됐고,
25 cm 표적은 전경에 가렸다. 실제 관절을 움직이지 않고 복사한 URDF에 설정 자세를 고정했다.
고정 전후 링크 변환이 원래 URDF 운동학과 같은지는 단위 검사에서 확인했다.
RGB 이미지는 참고용이며 RGB 검출 품질은 평가하지 않았다. 중립 자세 참고 영상에 로봇 전경 형상이
크게 포함됨을 확인했다. 표면·물체를 볼 작업 자세와 관심 픽셀은 C2/C4 연결 시 별도로 선택해야 한다.

수신률은 sim time 기준이다. 수신률·누락률의 승인 기준이 없어 성능 판정은 `not_judged`다.
입력·출력은 독립 구독으로 관측하므로 stamp 불일치가 0이라는 결과를 네트워크 무손실 보장으로 해석하지 않는다.
정지 IMU 관측은 시뮬레이션의 일관성 자료이며 실제 센서 오차 추정이 아니다.

## 발견·수정 및 최초 결과 보존

1. D1 README의 이전 AX_2 폭 2.06 m를 현재 여유 포함 1.30 × 1.06 m로 정정했다.
2. 거리 검사기에서 NaN 깊이가 들어오면 `allow_nan=False` JSON 저장도 실패했다.
   비유한 값은 null로 저장하고 원래 `passed=false`를 유지하도록 수정했다. 회귀 검사 포함.
3. 시험판 교체 후 0.5 sim s 시점에 렌더링 과도 NaN이 1표본 관측됐다.
   이를 성공 표본으로 바꾸거나 허용 오차를 늘리지 않았다. 장면 변경 후 측정 안정화 시간을
   1.0 sim s로 명시하고 별도 회차로 재검증했다. 이후 전방 검사의 모든 표본이 통과했다.
4. 초기 지도 누락 검사는 lifecycle 기동 자체가 실패해야 한다고 가정했다.
   Nav2는 활성화될 수 있으므로 실제 경로 성공 여부로 검사 조건을 바로잡았다. 지도 미수신 경로는 실패했다.

`initial_attempts/`에 최초 검사 기대조건 실패와 무효 깊이 표본 결과를 남겼다.
strict JSON 저장 실패의 traceback과 모든 원본 로그는 `log/w2_complete_2026-11-13/initial_attempts/`에 보존한다.
이전 `docs/evidence/w2/`, `w2_recheck_2026-11-13/`도 그대로 유지한다.

## 재현

공백 없는 WSL 내부 작업공간에서 ROS Jazzy와 빌드 결과를 source한다.
모든 검사는 운영 세션과 분리한 domain·Gazebo partition에서 수행했다.
domain은 공용 `ros_test_domain.reserve_domain(range(90, 101))`으로 예약하고 사용 중 노드를 검사했다.
단계가 끝나면 해당 단계에서 시작한 프로세스만 종료했다.

```bash
colcon build --packages-up-to ax_navigation ax_perception ax_control ax_localization ax_task_executor ax_environment --parallel-workers 2
source install/setup.bash
```

- 기존 B1 전방·팔·합성 입력 검사: `src/ax_perception/README.md` 절차를 따른다.
- 수신 품질: 같은 시험 domain의 Gazebo·B1 실행 뒤
  `python3 tools/check_w2_sensor_quality.py --robot-id robot_1 --duration-s 15 --output <새 결과 경로>`.
  robot_2도 별도 회차에서 반복한다.
- Nav2: 모의 입력만 있는 새 domain에서 `ax_navigation navigation.launch.py`를 시작한 뒤
  `python3 tools/check_w2_navigation.py --scenario <아래 조건> --output <새 결과 경로>`.
  조건은 baseline, obstacle, blocked, missing_map, missing_tf, missing_scan, stale_tf, stale_scan이다.
  각 조건마다 launch를 새로 시작한다. NavigateToPose·FollowPath·구동 명령은 보내지 않는다.
- 팔 시야: `check_w2_arm_visibility.py --pose <자세> --prepare-only <새 치구 폴더>`로 복사 모델·launch를 생성한다.
  출력된 launch를 실행하고 다른 터미널에서 같은 도구에 `--pose <자세> --output <새 결과 경로>`를 준다.
  자세는 neutral/reach_left/reach_right다. 이 launch는 제어기·공구 구동기를 실행하지 않는다.
- `test_w2_measurements.py`는 실패 표본 보존·시각 지표·고정 모델 변환을 검증한다.

전방·품질·팔 검사는 `empty_test_fixture.py`의 빈 바닥에서 수행했다. 실제 창고의 배경·가림과 구분한다.
각 도구의 출력 경로는 재실행마다 새로 지정한다. 원본 launch·빌드·검사 로그는
`log/w2_complete_2026-11-13/`(Git 제외)에 있다. `static_checks.json`과 `summary.json`에 종료 코드를 기록했다.
`manifest.json`은 관련 소스·설정과 결과 파일의 SHA-256을 기록한다(CRLF→LF 정규화).

## 인계·경계

[HANDOFF.md](HANDOFF.md)에 양현승·김성민·엄승연의 소비 인터페이스와 공동 확인 항목을 정리했다.
이는 저장소의 인계 자료이며 상대 담당자의 확인·실제 통합 완료를 대신하지 않는다.

W2의 시뮬레이션 개발 검증·설정·보완 자료는 준비했다. 실제 B2/A4 연결 주행·정차,
인계/통로 운영 허가, 실제 팔 운동의 충돌·가림, 실물 보정, 전체 시야, 4대 장시간·실제 절전 복귀,
W3/W4·G1 통과는 이번 결과에 포함하지 않는다.
