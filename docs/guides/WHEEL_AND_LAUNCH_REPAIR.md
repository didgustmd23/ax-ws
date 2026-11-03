# 바퀴 방향·시험 월드·런치 정리

최신 기본 모델은 [A1 V2 메시·EPick 안내](A1_MODEL_ASSETS.md)를 따른다.
아래 58개 collision·4컵 공구·27.82 kg 수치는 초기 수정 시점의 비교 기록이며,
V2/EPick의 현재 수치와 검증 결과는 A1 안내와 증거에 구분해 기록한다.

작업일: 2026-11-03. 저장소를 내려받은 작업공간에서 빌드·실행한다.
다른 PC의 준비 절차는 [실행 안내](RUN_ON_ANOTHER_PC.md)를 따른다.
기존 모델·설정은 `log/repair/before_drive_repair_*.tar.gz`에 백업했다.

## 바로 실행

Ubuntu 터미널에서:

```bash
cd ~/ax-ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ax_control simulation.launch.py
```

기본값은 AX_1 조작 로봇 1대와 AX_2 운송 로봇 1대, 빈 주행 구역과 테이블·상자를 갖춘 통합 시험 월드, GUI 켜짐이다.

```bash
# 조작 로봇만 / 운송 로봇만
ros2 launch ax_control simulation.launch.py robot_ids:=robot_1
ros2 launch ax_control simulation.launch.py robot_ids:=robot_2

# 작업대 2개·상자 1개가 있는 팔 시험 월드
ros2 launch ax_control simulation.launch.py world:=ax_test.world

# 화면 없는 시험 / 4대 활성
ros2 launch ax_control simulation.launch.py gui:=false
ros2 launch ax_control simulation.launch.py robots:=4
```

`ax_control simulation.launch.py`는 `ax_description robot_test.launch.py`(월드·모델·센서)를 포함하고
`controller_config_dir`로 `ax_control/config`의 제어기 YAML을 넘긴 뒤 로봇별 제어기 spawner와 공구 노드를 띄운다.
`ax_arm_test.launch.py`, `ax_2_test.launch.py`, `ax_test_both.launch.py`, `w1_sim.launch.py`는
`robot_test.launch.py`에 연결되는 호환 진입점이며 제어기 없이 모델·센서만 띄운다. Gazebo 월드·clock은 한 번만 생성한다.
로봇별 `robot_description`, `joint_states`, 제어기, 센서 Topic과 TF frame을 분리한다.
spawner는 controller_manager 서비스가 준비될 때까지 기다린다(최대 120 s).

## 좌표·바퀴 이름

로봇이 바라보는 방향이 **+X(앞)**, 왼쪽이 **+Y**, 위쪽이 **+Z**다.
화면에서 바라보는 사람의 좌우와 혼동하지 않는다.

| 위치 | 링크 | 관절 | 원본 메시 파일 |
|---|---|---|---|
| 왼쪽 앞 | `front_left_wheel` | `front_left_wheel_joint` | `ax_tire_left_front.obj` |
| 오른쪽 앞 | `front_right_wheel` | `front_right_wheel_joint` | `ax_tire_right_front.obj` |
| 왼쪽 뒤 | `rear_left_wheel` | `rear_left_wheel_joint` | `ax_tire_left_rear.obj` |
| 오른쪽 뒤 | `rear_right_wheel` | `rear_right_wheel_joint` | `ax_tire_right_rear.obj` |

두 로봇 모두 같은 링크·관절 규칙을 사용하고 namespace로 분리한다.
왼쪽 제어에는 왼쪽 앞·뒤만, 오른쪽 제어에는 오른쪽 앞·뒤만 연결한다.
네 관절의 회전축은 차체 좌표에서 +Y다. 양의 회전이 전진 방향이다.
`wheel_01` 같은 번호로 좌우를 추측하지 않는다. 기존 OBJ 자산 파일명은 보존한다.

4개의 고정 바퀴로 회전하려면 횡방향 미끄럼이 필요하다. AX_1은 기본 등방 마찰에서
바퀴만 돌아가고 차체 yaw가 거의 변하지 않았다. 공통 `ax_wheel_contact`에서
바퀴 로컬 Z(차축)를 주 마찰 방향으로 정하고 차축 방향 `mu1=0.1`, 굴림 방향 `mu2=1.0`을 적용한다.
이는 시뮬레이션용 가정이며 실물 타이어 사양이 아니다. 회전 중 횡방향 이동이 남으므로
정밀 제자리 회전·Nav2 경로 추종 성능 완료를 뜻하지 않는다.

## 이름·설정 정리

- 공통 진입점: 제어 포함 `ax_control/launch/simulation.launch.py`, 모델·센서만 `launch/robot_test.launch.py`.
- 제어기 설정: A2 `ax_control/config`(`common_controllers.yaml`, 모델별 `ax1_controllers.yaml`·`ax2_controllers.yaml`).
- 주행·작업대·팔 공통 월드: `worlds/ax_test.world` / Gazebo world 이름 `ax_test`.
- 런치 변수: `robot_namespace`, `robot_description`, `controller_config_dir`, `model_mappings`,
  `spawn_x`, `spawn_y`, `ROBOT_ROLES`처럼 의미를 드러낸다.
- 생성 치수: `ax2_front_left_wheel_pivot` 등으로 정리하고
  `ax2_ax_tire_left_front_pivot` 같은 중복 이름을 제거했다.
- 권위 모델은 `ax.urdf.xacro`, `ax_2.urdf.xacro`다.
  `ax_1.urdf`는 수정 모델에서 다시 생성한 호환용 export이며 런치는 Xacro를 사용한다.
- Linux 스크립트 실행 권한과 LF 줄바꿈을 반영한다.

## 함께 바로잡은 모델·센서

기존 Ubuntu 모델의 0 원점·균일 관성 대신 링크별 COM·관성을 생성한다.
차체 collision은 AX_1 58개, AX_2 27개이고, 독립 검사에서 차체 메시 정점의 미포함 수를 확인한다.
팔은 원점·회전축·limit이 있는 5축 체인이며 SDF 변환 후에도 5개 관절을 보존한다.
가상 질량 예산 기준 AX_1 전체 질량은 27.82 kg, 중립 자세 COM은 base 기준 z≈0.539 m다.
바닥 기준으로는 중립 주행 높이 약 0.389 m를 더한 약 0.928 m다. 실측 질량이나 전도 시험 결과가 아니다.

전후 LiDAR를 차체 외곽에 두어 각 180°를 관측한다. Harmonic/Ogre2 소프트웨어 렌더링에서
181개 샘플은 정확히 45°에 단일 근거리 반환, 후방에 최소거리 미만 반환이 생겼다.
360개 샘플(간격 약 0.5014°)로 큐브맵 경계와 정확히 겹치는 빔을 피한다.
고정 각도·근거리 데이터를 임의로 삭제하는 필터는 사용하지 않는다.
장애물 없는 근거리 검사와 실제 표적의 거리 검사를 함께 수행한다.

## 재현·증거

```bash
cd ~/ax-ws
bash tools/build_ubuntu_repair.sh
bash tools/run_ubuntu_drive_check.sh --arm-and-lidar
python3 tools/audit_robot_geometry.py
```

`audit_robot_geometry.py`를 별도 실행할 때도 위 ROS/작업공간 setup을 먼저 source한다.

- `log/repair/unit_tests.log`: 물리·트리·프레임·바퀴 위치/축/제어 연결·프로세스 소유권 검사.
- `log/repair/drive_directions.json`: Gazebo 원시 model pose와 joint_states로 전진·후진·좌우 회전 판정.
  odom만으로 성공을 판정하지 않는다.
- 같은 JSON의 `arm_trials`: 실제 팔 5축 목표 추종·Gazebo 자식 링크 회전 확인 후 중립 복귀.
- `lidar_snapshots`: 중립 및 팔 동작 후 네 센서의 0.8m 미만 반환·음의 무한대·NaN 검사.
  다른 로봇은 이 근거리 범위 밖에 배치한다.
- `lidar_obstacle_checks`: 센서 원점에서 2m 앞의 반지름 0.15m 구형 표적 표면(기대 1.85m) 검출.
- `log/repair/model_geometry.json`: COM·차체 메시 포함 검사·SDF 관절 보존.
- 선별한 최종 증거는 `docs/evidence/drive_repair/`에 보관한다.

각 실행은 별도 ROS domain/Gazebo partition을 사용하고 해당 시험 태그의 프로세스만 정리한다.
이 결과는 기본 방향·기구·센서 회귀 확인이다. 전체 관절 범위·하중·안전 게이트·정밀 주행 성능은 별도 검증한다.
