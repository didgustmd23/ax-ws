# ax-ws

**ax-ws는 중앙 운영 AI 에이전트의 상황 판단과 계획 제안을 중심으로, 가상 창고에서 두 로봇의 공동 운반을 조정하는 시스템을 개발하는 프로젝트다.** ROS 2 Jazzy와 Gazebo Harmonic을 기반으로 흡착 팔 로봇 `robot_1`이 물체를 집어 적재함 운송 로봇 `robot_2`에 싣고, 두 로봇이 목적지로 이동한 뒤 `robot_1`이 하역·배치하는 시나리오를 구현한다.

에이전트는 Fleet이 제공하는 로봇·작업·통로 상태와 예측 데이터를 종합해 상황을 요약하고, 작업 우선순위·통로 통과 순서·작업 보류·실패 복구 방안을 제안한다. 초기에는 협업 중인 두 로봇 중 누가 먼저 통로를 통과해야 공동 작업이 빨리 끝나는지 판단하는 데 초점을 둔다. Fleet은 제안의 유효성과 현재 상태를 검증해 운영에 반영하고, 로봇별 Task Executor와 제어부가 실행 및 최종 구동 차단을 담당하도록 설계한다.

개발 범위는 에이전트의 운영 판단을 로봇 모델·제어, 위치 추정·환경 인지, 물체 인식·조작, Fleet 작업 배정·자율주행과 연결하는 것이다. 규칙 기반 공동 운반을 먼저 검증하고, 에이전트 제안을 기록·평가하는 `ADVISORY` 단계를 거쳐 검증된 제안을 반영하는 `ASSISTED` 단계로 확장한다. AI 응답 지연·오류나 제안 검증 실패 시에는 규칙 기반 운영으로 복귀하도록 설계한다. 목표 구성과 역할별 계약은 [통합 상세설계](docs/design/DETAILED_DESIGN.md)에 정리한다.

현재 저장소는 팔이 달린 단일 `ax` 모델과 Gazebo 플러그인 기반 기본 제어를 개발하는 단계다. 중앙 운영 AI 에이전트·두 로봇 협업·흡착 부착/해제·자율주행·ros2_control 전환은 후속 구현 대상이다. 실제 기구가 없는 가상 설계이며, 질량·관성·충돌 형상은 개발용 근삿값이다.

프로젝트·GitHub 저장소 이름: **`ax-ws`**. ROS 모델 이름은 `ax`, 패키지 이름은 기존 값을 유지한다. 로컬 작업공간은 `~/ax_ws`로 사용한다. 저장소를 받을 때 대상 폴더를 `~/ax_ws`로 지정하면 아래 명령을 그대로 사용할 수 있다.

[전체 문서 안내](docs/README.md) · [GitHub 업로드 전 점검](docs/validation/PREUPLOAD_REVIEW.md)

Claude 하네스 파일(`CLAUDE.md`, `.claude/`)은 로컬 작업 환경 전용이며 Git에 포함하지 않는다.

## 전체 시스템 구성도

### 목표 구성

아래는 두 로봇이 협업하는 **목표 설계**다. 각 블록의 현재 구현 범위는 아래 표를 참고한다.

```mermaid
flowchart TD
    User[사용자 요청] --> Context["D: 요청 접수·운영 상태 수집<br/>FleetSnapshot·예측 데이터"]

    subgraph Agent["중앙 운영 AI 에이전트 · 운영 판단의 중심"]
        direction TB
        Summary["상황 요약<br/>로봇·작업·통로 상태 파악"]
        Reason["운영 판단<br/>작업 우선순위·통로 통과 순서 검토"]
        Proposal["계획 제안<br/>우선순위 변경·보류·재시도·복구 방안"]
        Summary --> Reason --> Proposal
    end

    Context -->|상태·예측 데이터| Summary
    Proposal -->|운영 계획 제안| Fleet["D: Fleet Manager<br/>제안 검증·그룹 작업 배정·합류·인계 관리"]
    Fleet -.->|운영 상태 갱신| Context
    Fleet -->|검증 후 로봇별 작업 배정| Executor["A: 로봇별 Task Executor"]
    Fleet --> Corridor["D: 통로 허가 관리"]
    Corridor -->|진입 허가| Nav["D: Nav2 주행"]
    Executor -->|이동 요청| Nav
    Executor -->|집기·놓기 요청| Manip["C: 물체 인식·조작"]
    Perception["B: 위치 추정·환경 인지·안전 판단"] -->|위치·통로 관측| Context
    Perception -->|위치·환경 정보| Nav
    Perception -->|안전 판단| Executor
    Executor -->|실행 허용·정지 관리| Gate["A: 최종 구동 차단"]
    Nav -->|주행 명령| Gate
    Manip -->|팔·흡착 실행 요청| Gate

    subgraph Simulation["Gazebo 시뮬레이션 · ros_gz 센서·상태 브리지"]
        Control["ros2_control 바퀴·팔 제어<br/>흡착 실행은 별도 구현"]
        R1["robot_1: 흡착 팔 로봇<br/>집기·적재·하역"]
        R2["robot_2: 적재함 운송 로봇<br/>상자 지지·운송"]
        Control --> R1
        Control --> R2
    end

    Gate -->|허용된 구동 요청| Control
    Simulation -->|센서·상태| Perception
    Simulation -->|카메라·공구 보유 상태| Manip
    Simulation -->|관절·구동·적재 상태| Executor
    Executor -->|작업 진행·로봇 상태| Context

    classDef agentStep fill:#ede9fe,stroke:#6d28d9,stroke-width:2px,color:#1e1b4b
    class Summary,Reason,Proposal agentStep
    style Agent fill:#f5f3ff,stroke:#6d28d9,stroke-width:3px,color:#1e1b4b
```

중앙 운영 AI 에이전트는 D가 수집한 상태를 바탕으로 **상황 요약 → 운영 판단 → 계획 제안**을 수행한다. D는 제안을 검증해 작업 배정·통로 허가에 반영한다. 제안 검증 실패·AI 지연·오류 시에는 규칙 기반 운영으로 복귀한다. A는 각 로봇의 최종 구동 경로에서 실행 허용과 정지를 강제한다. B는 위치·환경·안전 판단을 제공하고 C는 `robot_1`의 인식·조작을 담당한다. 상세 계약과 미정 수치는 [통합 상세설계](docs/design/DETAILED_DESIGN.md)를 따른다.

### 공동 운반 흐름

```mermaid
flowchart LR
    Source["station_src<br/>두 로봇 정차"] --> Pick["robot_1 집기<br/>robot_2 적재함에 적재"]
    Pick --> Check["적재 확인<br/>흡착 해제·팔 후퇴"]
    Check --> Move["두 로봇 이동<br/>상자는 robot_2가 지지"]
    Move --> Ready["station_dst 정차<br/>하역 준비 확인"]
    Ready --> Place["robot_1 하역<br/>작업대에 배치"]
    Place --> Complete["흡착 해제·팔 후퇴<br/>적재함 EMPTY 확인·완료"]
```

팔 동작 중 `robot_1` 주행과 적재·하역 중 `robot_2` 주행을 차단하는 것이 목표다. 두 로봇의 대형 유지 이동 여부는 미정이다.

### 현재 구현 상태

| 영역 | 현재 코드 | 목표·후속 구현 |
|---|---|---|
| 로봇 모델 | 팔이 달린 단일 `ax`, RViz 집게 4종 선택 | `robot_1`·`robot_2` 분리 및 로봇별 물리 특성 산정 |
| 기본 제어 | Gazebo 플러그인, 팔·집게 Float64 명령, `/cmd_vel` 입력 브리지 | ros2_control 전환, 로봇별 제어·최종 구동 차단 |
| 센서·상태 | `/scan`, `/camera/image_raw`, `/camera/camera_info` 브리지 구성 | `/odom`·이동 TF·Gazebo `/joint_states` 피드백·`/clock`·`use_sim_time` 연결 |
| 공구·조작 | Gazebo 기본 `primitive`, RViz 기본 비교용 `suction` 외형 | EPick 도입, 인식·집기 계획·실제 부착/해제·보유 판정 |
| 작업·주행 운영 | 두 로봇 협업 실행은 미구현 | Task Executor·Fleet·Nav2·통로 허가·중앙 운영 AI |

센서 브리지 구성은 수신·성능 검증 완료를 뜻하지 않으며, RViz GUI 관절값은 Gazebo 물리 상태가 아니다. 2 kg 상자 운반은 검증 목표다. 현재 정적 COM 계산에는 지지 영역을 벗어나는 자세가 있으며, 충돌 없는 작업 범위·동적 안정성·실제 파지는 검증하지 않는다. [현재 인터페이스](docs/guides/INTERFACES.md)와 [물리 검사](docs/validation/PHYSICAL_PROPERTIES_REVIEW.md)에서 제공 범위와 계산 조건을 확인한다.

## 구성

| 경로 | 용도 |
|---|---|
| `src/ax_description/urdf/ax.urdf.xacro` | 차체·바퀴·팔·집게·라이다, v0.1 팔 질량·관절 속도 설정 |
| `src/ax_description/config/arm_joint_limits.yaml` | 5축 속도·가속도 계획값. 현재 브리지의 가속도 제한 기능은 아님 |
| `src/ax_description/urdf/structure_demo.urdf` | 몸체와 팔만 있는 별도 연습 모델 |
| `src/ax_description/launch/display.launch.py` | RViz 수동 관절 표시 |
| `src/ax_control/launch/simulation.launch.py` | Gazebo 생성, 명령·센서 브리지, robot_state_publisher |
| `src/ax_control/ax_control/gripper_controller.py` | 집게 공통 명령을 양쪽 손가락으로 분배 |
| `tools/generate_physical_properties.py` | 질량·COM·관성·충돌 상자의 원본 계산 및 Xacro·JSON 생성 |
| `tools/check_static_stability.py` | 자세·공구·하중 가정에 따른 정적 COM 바닥 투영·지지 여유 계산 |
| `maps/warehouse_map.yaml`, `.pgm` | 기존 지도. 대응하는 창고 월드는 포함하지 않음 |
| [docs/guides/INTERFACES.md](docs/guides/INTERFACES.md) | 팀 연결용 링크·관절·토픽 표 |
| [docs/guides/GRIPPER_COMPARISON.md](docs/guides/GRIPPER_COMPARISON.md) | 집게 4종 선택·개폐 조작·시험 범위 |
| [docs/design/AX_5DOF_ARM_SPEC.md](docs/design/AX_5DOF_ARM_SPEC.md) | 실제 5축 로봇을 참고한 AX 자체 사양안·하중 목표·검증 기준 |
| [docs/validation/PHYSICAL_PROPERTIES_REVIEW.md](docs/validation/PHYSICAL_PROPERTIES_REVIEW.md) | 충돌 형상·가상 질량 분포 보완과 정적 지지 여유 계산 결과 |
| [docs/management/GIT_UPLOAD_POLICY.md](docs/management/GIT_UPLOAD_POLICY.md) | Git 업로드 제외·조건부 보관·필수 포함 파일 분류 |

## 설치와 빌드

대상: Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic. GUI와 GPU LiDAR에는 디스플레이·렌더링 지원이 필요하며 WSL에서는 WSLg를 사용한다.

수신 팀원 PC에서 실행한다. rosdep 초기화가 되어 있다면 `sudo rosdep init`은 생략한다.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep liburdfdom-tools \
  python3-numpy python3-scipy python3-matplotlib
# rosdep 최초 사용 시에만: sudo rosdep init
rosdep update
cd ~/ax_ws
mkdir -p src/ax_description/config src/ax_description/meshes
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to ax_description ax_control
source install/setup.bash
```

실행 의존성은 각 `package.xml`에 선언한다. Gazebo 시스템 플러그인은 Jazzy vendor 환경을 사용하므로 ROS setup을 먼저 적용한다.

ros2_control 전환은 아직 적용 전이므로 현재 빌드·실행에는 아래 패키지가 필요 없다. 전환 작업에서는 `gz_ros2_control`, `controller_manager`, `joint_trajectory_controller`, `joint_state_broadcaster`, `diff_drive_controller` 등을 `package.xml`에 선언하고 이 절의 설치 명령을 함께 갱신한다. 2026-10-22 작업 PC 확인 기준으로 `diff_drive_controller`, `joint_state_broadcaster`, `hardware_interface`는 설치되어 있고 `gz_ros2_control`, `controller_manager`, `joint_trajectory_controller`는 설치되어 있지 않다. 다른 PC에서는 `ros2 pkg prefix <패키지>`로 다시 확인한다. 설치 오류는 해결한 뒤 다음 단계로 진행한다.

## 실행

새 터미널마다 환경을 적용한다.

```bash
cd ~/ax_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ax_control simulation.launch.py
```

월드 `empty`, 모델 `ax`. 설치된 패키지의 Xacro를 `/tmp/ax.urdf`로 변환한다. 이 파일은 동시 실행 시 공유되므로 병렬 실행하지 않는다. Ctrl+C로 종료한다.

RViz 구조만 별도로 확인할 때:

```bash
ros2 launch ax_description display.launch.py
```

설계 기준 공구는 Robotiq EPick(컵 구성 미정)이며 아직 도입하지 않았다. RViz 기본 공구는 비교용 가상 4컵 흡착 헤드(`suction`)다. 표시 기본값일 뿐 흡착 부착·해제는 구현되지 않았다. 오픈소스 Robotiq 2F-85는 `gripper_model:=robotiq`(GUI의 `robotiq_85_left_knuckle_joint` 하나로 연동 개폐), 기존 임시 집게는 `gripper_model:=primitive`, 최대 450 mm 가상 평행 집게는 `gripper_model:=parallel_wide`로 선택한다. 실행을 종료한 뒤 다른 인자로 다시 시작한다. [집게 비교 안내](docs/guides/GRIPPER_COMPARISON.md)를 참고한다. Gazebo 실행은 기존 임시 집게(`primitive`)를 사용하며 공구 선택·흡착·Robotiq 물리 제어 통합은 별도 작업이다. Robotiq 원본·라이선스는 [패키지 안내](src/robotiq_description/README.md)를 참고한다.

Fixed Frame을 `base_link`로 설정하고 RobotModel을 추가한다. 이 GUI 관절값은 Gazebo 실측값이 아니다. 시뮬레이션 상태 확인을 위해 두 launch를 동시에 실행하지 않는다.

## 검사와 명령 예

```bash
xacro "$(ros2 pkg prefix --share ax_description)/urdf/ax.urdf.xacro" -o /tmp/ax_check.urdf
check_urdf /tmp/ax_check.urdf
ros2 topic list -t
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 '{data: 0.015}'
ros2 topic pub --once /column_yaw/command std_msgs/msg/Float64 '{data: 0.3}'
```

명령 단위·범위는 [인터페이스 표](docs/guides/INTERFACES.md)를 따른다. 현재 명령 노드는 입력 범위를 검사하지 않는다.

## 팀별 후속 연결

- B: `/scan`, `lidar_link`, `base_link` 사용. `/clock`, 시뮬레이션 시간, 이동 관절 `/joint_states` 피드백을 후속 연결한다.
- C: 팔·집게 Float64 명령 사용. 자세 피드백·역기구학·인식·집기 성공은 별도 구현 및 검증 대상이다.
- D: ROS `/cmd_vel` → Gazebo 단방향 브리지는 있으나 구동 차단 게이트는 없다. `/odom`, `odom → base_link`는 현재 제공되지 않는다. 로봇별 이름·프레임 규칙을 먼저 확정한다.
- A: 실제 OBJ·단위·원점·관절 축표 확보 후 모델을 교체하고 재검증한다. 질량·관성·제어값·관절 제한은 실측 검증 전이다.

## 전달 파일

`src/`, `docs/`, `tools/`, README를 전달한다. 지도 사용 시 YAML과 PGM을 함께 전달한다. `build/`, `install/`, `log/`, `legacy_before_rename/`, `__pycache__/`, `.bak`, `Zone.Identifier` 부가 파일은 제외한다. CMake에서 참조하는 빈 `config/`, `meshes/`는 유지하거나 빌드 전에 위 mkdir 명령으로 만든다.

다른 팀원 PC에서 새로 빌드하고 모델 생성·명령 전달·센서 수신을 재현해야 인계 재현 확인이 완료된다. 현재 PC 검증과 다른 PC 재현은 구분한다.
