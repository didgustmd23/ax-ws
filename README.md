# ax-ws

가상 AX 로봇의 개발용 모델이다. 차체·바퀴·팔은 분리 OBJ를 사용하고, 집게는 네 종류를 선택할 수 있다. 관절축·질량·충돌 형상은 개발용 근삿값이며 JSON 생성기 대신 Xacro를 사용한다.

프로젝트·GitHub 저장소 이름: **`ax-ws`**. ROS 모델 이름은 `ax`, 패키지 이름은 기존 값을 유지한다. 로컬 작업공간은 `~/ax_ws`로 사용한다. 저장소를 받을 때 대상 폴더를 `~/ax_ws`로 지정하면 아래 명령을 그대로 사용할 수 있다.

[전체 문서 안내](docs/README.md) · [GitHub 업로드 전 점검](docs/validation/PREUPLOAD_REVIEW.md)

## 구성

| 경로 | 용도 |
|---|---|
| `src/ax_description/urdf/ax.urdf.xacro` | 차체·바퀴·팔·집게·라이다, v0.1 팔 질량·관절 속도 설정 |
| `src/ax_description/config/arm_joint_limits.yaml` | 5축 속도·가속도 계획값. 현재 브리지의 가속도 제한 기능은 아님 |
| `src/ax_description/urdf/structure_demo.urdf` | 몸체와 팔만 있는 별도 연습 모델 |
| `src/ax_description/launch/display.launch.py` | RViz 수동 관절 표시 |
| `src/ax_control/launch/simulation.launch.py` | Gazebo 생성, 명령·센서 브리지, robot_state_publisher |
| `src/ax_control/ax_control/gripper_controller.py` | 집게 공통 명령을 양쪽 손가락으로 분배 |
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

실행 의존성은 각 `package.xml`에 선언한다. Gazebo 시스템 플러그인은 Jazzy vendor 환경을 사용하므로 ROS setup을 먼저 적용한다. 설치 오류는 해결한 뒤 다음 단계로 진행한다.

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

RViz 기본 집게는 오픈소스 Robotiq 2F-85다. GUI의 `robotiq_85_left_knuckle_joint` 하나로 연동 개폐를 확인한다. 기존 임시 집게는 `gripper_model:=primitive`, 최대 450 mm 가상 평행 집게는 `gripper_model:=parallel_wide`, 4컵 흡착 헤드는 `gripper_model:=suction`으로 선택한다. 실행을 종료한 뒤 다른 인자로 다시 시작한다. [집게 비교 안내](docs/guides/GRIPPER_COMPARISON.md)를 참고한다. Gazebo 실행은 기존 임시 집게를 사용하며 Robotiq 물리 제어·파지 통합은 별도다. Robotiq 원본·라이선스는 [패키지 안내](src/robotiq_description/README.md)를 참고한다.

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
- D: ROS `/cmd_vel`, `/odom`, `odom → base_link` 연결은 현재 제공되지 않는다. 브리지와 프레임 규칙을 먼저 확정한다.
- A: 실제 OBJ·단위·원점·관절 축표 확보 후 모델을 교체하고 재검증한다. 질량·관성·제어값·관절 제한은 실측 검증 전이다.

## 전달 파일

`src/`, `docs/`, `tools/`, README를 전달한다. 지도 사용 시 YAML과 PGM을 함께 전달한다. `build/`, `install/`, `log/`, `legacy_before_rename/`, `__pycache__/`, `.bak`, `Zone.Identifier` 부가 파일은 제외한다. CMake에서 참조하는 빈 `config/`, `meshes/`는 유지하거나 빌드 전에 위 mkdir 명령으로 만든다.

다른 팀원 PC에서 새로 빌드하고 모델 생성·명령 전달·센서 수신을 재현해야 인계 재현 확인이 완료된다. 현재 PC 검증과 다른 PC 재현은 구분한다.
