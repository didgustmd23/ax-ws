# W1 A4·B4 — 폴더 구분 버전 (ax-ws 적용용)

## 1. 이 ZIP의 목적
- **이전 실개발 코드 ZIP**을 기반으로 A4·B4 하위 폴더를 분리한 버전입니다.
- 이 ZIP은 **프로젝트 루트 `ax-ws/`에서 풀도록** `src/`와 `tools/` 경로가 포함돼 있습니다.
- `src/ax_d4/` W0 코드 및 다른 팀원 A1·A2·A3 코드는 포함하지 않고 변경하지 않습니다.
- 개인 컴퓨터의 `D:/...` 또는 `/home/...` 경로는 코드에 넣지 않습니다.
- **ROS2 Jazzy 전체 빌드·Gazebo·A2/A3 실제 통합 검증은 아직 수행하지 않았습니다.** 코드 파일이 존재한다는 것이 로봇 안전 기능의 검증 완료를 뜻하지 않습니다.

## 2. 파일 구성과 역할

| 경로 (모두 ax-ws/ 아래) | 목적 |
|---|---|
| `src/ax_safety/src/ax_safety/src/a4/safety_gate.cpp` | Nav2 주행 명령을 A4에서 검사해 A2 제어기로 내보내는 주행 게이트 |
| `src/ax_safety/src/ax_safety/include/ax_safety/a4/safety_gate.hpp` | A4 주행 허용/차단 판단 타입·함수 |
| `src/ax_safety/src/ax_safety/src/a4/protected_arm_server.cpp` | 팔 궤적 검사·A3 권한 확인·제어기 Action 중계 |
| `src/ax_safety/src/ax_safety/include/ax_safety/a4/protected_arm_server.hpp` | A4 팔 보호 서버 클래스 선언 |
| `src/ax_safety/src/ax_safety/ax_safety/b4/risk_rules.py` | B4 위험 수준·방향별 허용 판정 핵심 규칙 |
| `src/ax_safety/src/ax_safety/ax_safety/b4/b4_risk_node.py` | 센서·위치·통로 상태를 수신하고 SafetyState 발행 |
| `src/ax_safety/src/ax_safety/ax_safety/b4/__init__.py` | B4 하위 Python 패키지 지정 |
| `src/ax_safety/src/ax_safety/ax_safety/__init__.py` | ax_safety 상위 Python 패키지 지정 |
| `src/ax_safety/src/ax_safety/config/safety_limits.yaml` | 구동 기본 차단값 및 미검증 개발 임계값 설정 |
| `src/ax_safety/src/ax_safety/launch/safety.launch.py` | 설치된 패키지 위치에서 설정을 찾아 3개 노드 실행 |
| `src/ax_safety/src/ax_safety/CMakeLists.txt` | A4 C++ 파일을 컴파일하고 B4 실행 스크립트 설치 |
| `src/ax_safety/src/ax_safety/package.xml` | ROS2 패키지·의존성 선언 (`maintainer`, `license` 팀 확정 필요) |
| `src/ax_safety/tools/tests/test_a4_safety_gate.cpp` | ROS 없이 A4 게이트 조건 단위 테스트 |
| `src/ax_safety/tools/tests/test_b4_risk_rules.py` | ROS 없이 B4 위험 판단 단위 테스트 |
| `src/ax_safety/tools/tests/mock_check_command_lease.py` | **시험 전용** A3 권한 서비스 대체, 운영에 사용 금지 |

## 3. 폴더 구조
```text
ax-ws/
├── src/
│   └── ax_safety/
│       ├── src/a4/
│       │   ├── safety_gate.cpp
│       │   └── protected_arm_server.cpp
│       ├── include/ax_safety/a4/
│       │   ├── safety_gate.hpp
│       │   └── protected_arm_server.hpp
│       ├── ax_safety/
│       │   ├── __init__.py
│       │   └── b4/
│       │       ├── __init__.py
│       │       ├── risk_rules.py
│       │       └── b4_risk_node.py
│       ├── config/safety_limits.yaml
│       ├── launch/safety.launch.py
│       ├── CMakeLists.txt
│       └── package.xml
├── tools/tests/
│   ├── test_a4_safety_gate.cpp
│   ├── test_b4_risk_rules.py
│   └── mock_check_command_lease.py
└── README_W1_A4_B4_폴더구분_적용가이드.md
```

## 4. 적용법 (VS Code / feature/sy)
1. VS Code에서 **`feature/sy` 브랜치**인지 확인하고 변경 중인 파일이 있으면 먼저 보존합니다.
2. `ax-ws` 프로젝트의 최상위 폴더를 확인합니다. (`src/`와 `tools/`가 보이는 위치)
3. 이 ZIP을 **`ax-ws/` 최상위에서 압축 해제**합니다. `src/ax_d4` 등 W0 경로는 포함되지 않습니다.
4. **예전 W1 ZIP을 이미 프로젝트에 넣었다면** 기존 `src/ax_safety/src/safety_gate.cpp`, `src/ax_safety/src/protected_arm_server.cpp`, `src/ax_safety/include/ax_safety/{safety_gate,protected_arm_server}.hpp`, `src/ax_safety/ax_safety/{risk_rules,b4_risk_node}.py` 파일을 **백업/차이 확인 후 정리**하세요. 같은 이름 파일이 구/신 경로에 공존하지 않게 합니다. W0·다른 팀원 파일은 건드리지 마세요.
5. **개인 PC 경로 수정은 불필요**합니다. CMake/파이썬/launch는 패키지 내부 상대 위치를 사용합니다.

## 5. 코드 단위 시험 (ax-ws 루트에서)
```bash
python3 src/ax_safety/tools/tests/test_b4_risk_rules.py
mkdir -p /tmp/ax_w1_test
g++ -std=c++17 -I src/ax_safety/src/ax_safety/include src/ax_safety/tools/tests/test_a4_safety_gate.cpp -o /tmp/ax_w1_test/test_a4 && /tmp/ax_w1_test/test_a4
```

## 6. ROS2 Jazzy 설치 환경에서 빌드·실행 검증
```bash
source /opt/ros/jazzy/setup.bash
# ax-ws 루트에서
colcon build --symlink-install --packages-up-to ax_safety
source install/setup.bash
ros2 launch ax_safety safety.launch.py robot_id:=robot_1
```
- 위 명령은 **팀 프로젝트의 필수 패키지(특히 `ax_interfaces`)가 정상 빌드 가능한 상태**에서 실행해야 합니다.
- `src/ax_safety/config/safety_limits.yaml`의 기본값은 **주행/팔 비허용**입니다. 안전 검증 없이 허용 플래그를 켜면 안 됩니다.
- A3에는 `/ax/<robot_id>/executor/lease/check_command`를 제공하는 `ax_task_executor/lease_check` 개발 정책 서버가 있습니다. 실제 lease·세대·활성 Goal·배정 판정은 아직 없으며, 기본 `dev_deny`입니다. `tools/tests/mock_check_command_lease.py`는 별도 시험 전용이며 launch에서 함께 실행하지 않습니다.
- B1의 실제 전·후방 입력은 `/ax/<robot_id>/sensors/front_lidar/scan_filtered`와 `/ax/<robot_id>/sensors/rear_lidar/scan_filtered`입니다. `safety.launch.py`가 이 두 경로를 B4에 전달하며, B4는 확인되지 않은 입력을 UNKNOWN으로 처리합니다.
- `docs/evidence/wheel_slip/`는 A4 주행 자세·외곽·정지에 참고하되 검증된 운영 안전 한계는 아닙니다.
- W1 코드 컴파일/단위 테스트와 **실제 A2 제어기 연동/Gazebo 주행**은 서로 다른 검증 단계입니다.

## 7. 팀원이 받을 때
- 패키지명은 `ax_safety`로 유지합니다. `a4/`, `b4/`는 **패키지 내부 분류**이며, 최상위 Git 경로 `src/ax_safety`를 임의로 이동하지 않습니다.
- 팀원이 각기 다른 PC 경로에 `ax-ws`를 저장해도 동일한 패키지 빌드·실행 명령을 사용합니다.
- 팀 main에 병합되면 **기존 팀원 launch에 A4 게이트를 연결**하고 A2 제어기 직행 우회 경로를 제거하는 작업이 남습니다. 이 ZIP은 다른 사람 파일을 자동 수정하지 않습니다.
- 원격 브랜치에 반영하려면 별도로 커밋·푸시해야 합니다. 압축 해제 자체는 GitHub 업로드가 아닙니다.

## 8. 현재 검증 범위
- B4 순수 규칙 Python 단위 시험
- A4 순수 게이트 C++ 단위 시험
- Python 구문 검사와 ZIP 경로 일관성 확인

**미검증:** ROS2 Jazzy colcon 전체 컴파일, A4 팔 Action 통합, A2 컨트롤러/Gazebo 구동, B1~B3 실제 토픽, A3 운영 lease 서버. 현재 확인된 A3는 개발 정책 서버이며 운영 권한 판정 서버가 아닙니다.
