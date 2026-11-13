# W1 A4·B4 검증 기록

## 기록 범위

- 대상 브랜치: `feature/sy`
- 실행 환경: WSL Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- 기록 기준: 이번 W1 연동 검증에서 실제로 실행하고 확인한 결과
- 이 문서는 원본 콘솔 로그가 아니라 검증 요약이다. 별도로 보존되지 않은 원본 출력은 재구성하지 않는다.
- 기존 `log/` 아래 파일은 삭제하거나 덮어쓰지 않았다.

## 실행한 시험과 결과

### 빌드

실행 명령:

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin
colcon build --symlink-install --packages-select ax_safety ax_control
```

결과: `ax_safety`, `ax_control` 빌드 성공.

초기 빌드에서는 WSL PATH에 Windows Anaconda protobuf 경로가 포함되어 실패했다. ROS 경로 우선 PATH로 다시 실행한 결과 성공했다. 관련 colcon 원본 로그는 다음 위치에 남아 있다.

- `log/build_2026-11-13_00-19-54/ax_safety/`
- `log/build_2026-11-13_00-19-54/ax_control/`

### B4 단위 테스트

```bash
python3 src/ax_safety/tools/tests/test_b4_risk_rules.py
```

결과: `Ran 11 tests` / `OK`.

### A4 C++ 단위 테스트

```bash
g++ -std=c++17 \
  -I src/ax_safety/src/ax_safety/include \
  src/ax_safety/tools/tests/test_a4_safety_gate.cpp \
  -o /tmp/ax_w1_test/test_a4
/tmp/ax_w1_test/test_a4
```

결과: 오류 없이 종료.

### A2·Gazebo·A4·B4 실행

```bash
ros2 launch ax_control simulation.launch.py \
  robot_ids:=robot_1 gui:=false sensors:=false tool:=epick safety:=true
```

결과:

- B4, A4 주행 게이트, A4 팔 보호 서버가 시작됨
- Gazebo `arm_controller`, `diff_drive_controller`가 configure/activate됨
- A4 기본 안전 플래그는 false이므로 기본 상태는 잠금

### 팔 명령: A3 개발 정책 서버 연결

시험 조건:

- A3 `lease_check.py`를 `dev_deny`, `dev_allow` 정책으로 각각 실행
- A4 실제 서비스 경로: `/ax/robot_1/executor/lease/check_command`
- A4가 요구하는 SafetyState와 ActuationState는 일회성 시험용 publisher로 공급
- 시험용 상태 입력은 실제 B1~B3 센서 입력이 아님
- 실제 Gazebo `arm_controller` 액션 경로를 사용

결과:

- `dev_deny`: A4가 `LEASE_EXPIRED`로 차단
- `dev_allow`: A4가 실제 Gazebo `arm_controller`로 궤적을 전달하고 `SUCCEEDED`
- A3 서버 미실행: A4가 `A3_LEASE_SERVER_UNAVAILABLE`로 차단

### 주행 명령 차단

시험 조건:

- A4 기본 잠금 상태
- `/ax/robot_1/nav/cmd_vel`에 0.5 m/s 시험 명령을 3초간 입력
- 실제 B1~B3 센서와 D2/D3 permit은 공급하지 않음

결과:

- A4가 A2 경로에 0을 출력
- Gazebo odom 관측: 최대 선속도 0, 최대 각속도 0, 위치 변화 0
- 시험 중 로봇이 이동하지 않음

## 검증하지 않은 항목

- 실제 B1~B3 센서 입력에서 B4까지 이어지는 전체 연동
- D2/D3 permit을 이용한 주행 허용 경로
- 운영 lease의 실제 발급·세대·활성 Goal·갱신·철회·배정 권한 검증
- 장시간 Gazebo 궤적 실행
- 실물 하드웨어 연결

`dev_allow` 결과는 개발 정책 서버 연결 확인이며, 운영 권한 검증 완료를 의미하지 않는다. 위 시험용 publisher는 실제 센서 또는 운영 권한 시스템을 대체하지 않는다.

## 원본 로그 보존 상태

- 기존 colcon 로그: 보존됨. 위 경로를 참조한다.
- B4/A4 단위 테스트 및 A3·Gazebo 런타임 시험: 원본 콘솔 출력 파일은 별도로 남아 있지 않음.
- 따라서 이 문서에는 해당 시험의 관찰 결과만 기록하며, 원본 로그인 것처럼 재구성하지 않음.

## origin/main 통합 후 재검증

통합 내용:

- `origin/feature/sy`에 제가 받지 않은 변경은 없었음
- 최신 `origin/main`을 일반 merge로 `feature/sy`에 통합
- 충돌 없음

통합 후 실행한 검증:

```bash
colcon build --symlink-install \
  --packages-select ax_safety ax_control ax_task_executor
```

결과: `ax_safety`, `ax_control`, `ax_task_executor` 3개 패키지 빌드 성공.
`protected_arm_server.cpp`의 ROS 2 deprecated 경고 1건은 있었지만 빌드는 성공했다.

```bash
python3 tools/tests/test_a3_lease_check.py
python3 tools/tests/test_a3_local_task_executor.py
```

- A3 lease check: 16개 테스트 통과
- A3 local task executor: 9개 중 8개 통과, 1개 실패
- 실패 내용: `test_lease_latency_under_action_load`에서 `p99_s=0.11641092099944217`로 기준 `0.1s` 초과
- 서버 측 측정값은 `server_p99_s=0.09877652800059877`였음
- 성능 기준이나 안전 조건은 완화하지 않았음

전체 `--packages-up-to ax_control ax_safety ax_task_executor` 빌드는 `ax_interfaces`
CMake 의존성 검사 단계가 `/mnt/d` 파일 시스템에서 진행 없이 정체되어 중단했다.
이후 통합 영향 패키지 3개를 `--packages-select`로 별도 빌드해 성공을 확인했다.
