# 다른 PC에서 AX 시험 모델 실행

대상 환경은 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic이다.
[공식 ROS/Gazebo 조합](https://gazebosim.org/docs/harmonic/ros_installation/)을 사용한다.
먼저 [ROS 2 Jazzy 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)에 따라 ROS를 설치한다.
Windows에서는 Ubuntu 24.04 WSL2와 WSLg가 필요하다. GUI에는 작동하는 그래픽 드라이버와 디스플레이가 필요하다.
다운로드만으로 실행되는 바이너리는 아니며 최초 한 번 의존성 설치와 빌드가 필요하다.

```bash
git clone --branch main --single-branch https://github.com/didgustmd23/ax-ws.git
cd ax-ws
bash tools/setup_jazzy.sh
source install/setup.bash
ros2 launch ax_control simulation.launch.py robots:=2 tool:=robotiq
```

설치 스크립트는 apt/rosdep에 sudo 권한과 네트워크를 사용한다. 이미 ROS가 설치된 환경에서
의존성을 설치하고 모델·인터페이스·제어(`ax_control`) 패키지를 빌드한다. 사용자 이름이나 다운로드 디렉터리를 코드에 넣지 않는다.
새 터미널에서는 workspace로 이동한 뒤 `source /opt/ros/jazzy/setup.bash`와
`source install/setup.bash`를 실행한다. `build/`, `install/`, `log/`는 복사하지 않고 해당 PC에서 생성한다.

## 실행 월드

선택 가능한 월드와 로봇 출발 위치는
[`worlds.yaml`](../../src/ax_description/config/worlds.yaml)을 따른다.
기본은 `planned/ax_warehouse.world`, 단색 보존판은 `planned/ax_warehouse_plain.world`다.
두 월드는 재질 외 구조가 같고, launch가 각 로봇을 통로 밖의 지정 출발 위치에 생성한다.
실행 가능한 `ax_test.world` 파일은 제거됐다. 빈 바닥 검사는 도구가 생성한 시험 치구를 사용한다.
월드 실행만으로 자동 집기·내비게이션 작업이 시작되는 것은 아니다.

```bash
ros2 launch ax_control simulation.launch.py robots:=2 tool:=robotiq
# 기본 모델 공구: 다중 흡착판 · 12컵(epick은 호환 실행값)
ros2 launch ax_control simulation.launch.py robots:=2 tool:=epick
# GUI 없이 실행 (센서 렌더링에는 EGL 지원 필요)
ros2 launch ax_control simulation.launch.py gui:=false
# 렌더링 없이 제어만 확인
ros2 launch ax_control simulation.launch.py gui:=false sensors:=false
```

`ax_arm_test.launch.py`는 robot_1, `ax_2_test.launch.py`는 robot_2,
`ax_test_both.launch.py`는 두 로봇을 같은 월드에 생성하는 호환 진입점이다.
이 호환 진입점과 `robot_test.launch.py`는 제어기 없이 모델·센서만 띄운다. 조종·시험은 `ax_control` launch를 쓴다.
종전 `drive_test.world`/`ax_arm_test.world`/`ax_test.world` 직접 지정은
`world:=planned/ax_warehouse.world`로 바꾼다(현재 기본값이므로 생략 가능).

## 경로와 검증

메시: `package://패키지/상대경로`, Xacro include: 인접 파일 또는 `$(find 패키지)`.
런치는 ament로 설치된 share 디렉터리를 찾는다. 실행 시 만들어지는 임시 제어기 설정의 절대경로와
ROS 설치 경로 `/opt/ros/jazzy`는 정상이며 특정 사용자의 PC 경로가 아니다.
지도 이미지는 YAML 파일 기준 상대경로다. 외부 EPick/Robotiq 메시도 저장소에 포함되어 있다.

```bash
python3 tools/check_portability.py
# 두 로봇 실행 중, 별도 터미널에서
python3 tools/verify_unified_runtime.py
```

별도 디렉터리에 소스를 복사하고 기존 workspace overlay 없이 빌드·모델 생성·센서 수신을 확인한다.
이 검증은 현재 Ubuntu/WSLg 환경에서 수행하며 모든 PC/GPU에서의 실행 보장을 뜻하지 않는다.
Robotiq 손가락 제어와 TCP 관련 기존 제한은 월드 통합으로 해결되는 항목이 아니다.

중복 정리: 사용되지 않는 URDF 과거 백업 5개는 제거했다. Git 이력에서 조회할 수 있다.
기존 런치 파일은 공통 런치에 위임하는 호환 진입점으로 유지한다.
