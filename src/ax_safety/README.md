# A4·B4 안전 패키지

ROS 패키지 루트는 `src/ax_safety/`다. A4 C++ 주행 게이트·보호 팔 서버와
B4 Python 위험 판정 노드를 제공한다. 기본 설정은 구동 비허용이다.

## 폴더와 역할

| 경로 (패키지 루트 기준) | 역할 |
|---|---|
| `src/a4/safety_gate.cpp` | Nav2 → A4 → A2 주행 명령 게이트 |
| `src/a4/protected_arm_server.cpp` | 보호 팔 Action·A3 권한 확인·A2 Action 중계 |
| `include/ax_safety/a4/` | A4 헤더 |
| `ax_safety/b4/risk_rules.py` | ROS 독립 위험 판정 규칙 |
| `ax_safety/b4/b4_risk_node.py` | 센서·위치·통로 관측 → SafetyState |
| `launch/safety.launch.py` | B4·A4 주행·A4 팔 노드 실행 |
| `config/safety_limits.yaml` | 개발 기본 설정 |
| `CMakeLists.txt`, `package.xml` | 빌드·설치·의존성 |

Python의 `ax_safety/b4/`와 C++의 `include/ax_safety/`는 모듈·헤더 namespace이므로 유지한다.
과거 ZIP의 `src/ax_safety/src/ax_safety/` 중첩은 제거했다. 과거 검증 기록의 경로는 당시 실행 경로다.

공통 시험 파일은 작업공간의 `tools/tests/`에 있다.
`test_a4_safety_gate.cpp`, `test_b4_risk_rules.py`, `mock_check_command_lease.py`를 사용한다.
마지막 파일은 시험용 서버이며 운영 launch에서 실행하지 않는다.

## 빌드·실행

Ubuntu 24.04 / ROS Jazzy 작업공간 루트에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ax_safety ax_control ax_task_executor
source install/setup.bash
ros2 launch ax_safety safety.launch.py robot_id:=robot_1
```

설치·연결 절차는 [W1 안내](../../docs/guides/W1_A4_B4_SETUP.txt)를 따른다.
A2 시뮬레이션에서 함께 시작하려면 `simulation.launch.py safety:=true`를 사용한다.
시뮬레이션이나 실제 입력 없이 기본 잠금 상태인 것은 정상이다.

## 검사

```bash
python3 -m unittest discover -s tools/tests -p test_b4_risk_rules.py
mkdir -p /tmp/ax_w1_test
g++ -std=c++17 -Wall -Wextra -Wpedantic -I src/ax_safety/include \
  tools/tests/test_a4_safety_gate.cpp -o /tmp/ax_w1_test/test_a4
/tmp/ax_w1_test/test_a4
```

[W1 기록](../../docs/evidence/w1_a4_b4/)에는 당시 빌드·모의 상태·A3 개발 정책·Gazebo 검증 범위가 있다.
경로 정리는 안전 동작 결함 수정이나 운영 승인과 별개다. 실제 입력 유효성, 권한·기한,
최종 속도 제한·취소·중복 실행·실제 정지는 각각 계약에 맞게 검증해야 한다.
