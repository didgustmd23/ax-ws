# D4 작업 상태 기록·검증 (W0)

D4 W0는 A3가 발행하는 `RobotTaskStatus`를 수신해 공통 계약을 검증하고,
유효한 상태를 SQLite 원장에 기록하는 기반 작업입니다.

## 현재 구현 상태

- `input_source.py`: `/ax/robot_1/executor/status`와
  `/ax/robot_2/executor/status`를 구독합니다.
- `common_contract.py`: 메시지 schema, `Header`·`Time`, robot ID,
  execution ID, 결과 형식과 revision/session 규칙을 검증합니다.
- `ledger.py`: 유효성·시각 만료·중복·이전 revision·이전 session을 판정하고
  SQLite의 `latest`, `sessions`, `history`에 기록합니다.
- `d4_main.py`: 수신 → 검증 → 원장 기록 → 최신 상태 조회를 연결합니다.
- `requirements.txt`와 D4 `setup.py`에 `rfc8785`가 선언되어 있습니다.
- `use_sim_time=true`와 A3 STATE publisher에 맞는
  `RELIABLE + TRANSIENT_LOCAL` QoS를 사용합니다.

W0 기본 시험은 실제 ROS2 노드 없이 실행되며, 정상 입력·잘못된 입력·중복
revision·이전 revision·session 변경·만료 상태를 확인합니다.

## 로컬 시험

프로젝트 루트에서 다음을 실행합니다.

```bash
python3 -m pytest -q tools/tests/test_d4_w0.py
python3 tools/tests/test_d4_w0.py
```

실제 ROS2 시험은 Ubuntu 24.04 ROS2 환경에서 실행해야 합니다. 현재 Windows
환경에서는 `rclpy`, `ament_index_python`, ROS 메시지 생성물을 사용할 수 없으므로
실제 ROS2 종단 시험은 완료로 표시하지 않습니다.

## Ubuntu 24.04 설치

PEP 668 시스템 Python 제한을 피하기 위해 workspace 전용 가상환경을 사용합니다.

```bash
cd ax-ws
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

ROS2와 `ax_interfaces`를 먼저 source한 뒤 D4 패키지를 workspace에서 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ax_d4
source install/setup.bash
```

## A3 → D4 연결 시험

터미널을 분리하고 같은 `ROS_DOMAIN_ID`와 같은 `/clock` 환경을 사용합니다.

터미널 1: ROS2와 workspace를 source하고 A3 시험 publisher를 시작합니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ax_task_executor team_handoff.launch.py \
  robot_ids:=robot_1,robot_2 robot_models:=AX_1,AX_2 \
  regions_file:=src/ax_environment/config/ax_warehouse_regions.json \
  map_revision:=1
```

터미널 2: D4를 시작하고 원장 위치를 별도로 지정합니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p /tmp/ax_d4_w0
AX_D4_LEDGER_PATH=/tmp/ax_d4_w0/ledger.sqlite3 ros2 run ax_d4 d4_main
```

터미널 3: 실제 Topic과 원장 파일을 확인합니다.

```bash
ros2 topic info /ax/robot_1/executor/status -v
ros2 topic info /ax/robot_2/executor/status -v
ros2 topic echo --once /ax/robot_1/executor/status
sqlite3 /tmp/ax_d4_w0/ledger.sqlite3 \
  'select robot,source,session,revision from latest;'
```

A3 mock publisher의 상태는 `meta.valid=false`, `MOCK_ONLY`이므로 D4가
메시지를 수신하더라도 운영 원장에 정상 상태로 기록하지 않고 `UNKNOWN`으로
처리하는 것이 정상입니다. 이 시험은 “A3 Topic 수신 및 D4 검증 경로”를
확인하는 시험이며, 실제 작업 성공 권한을 검증하는 시험은 아닙니다.

## Ubuntu에서 반드시 직접 확인할 항목

1. `colcon build --symlink-install --packages-up-to ax_d4` 성공
2. `ros2 run ax_d4 d4_main` 기동 성공
3. 두 A3 상태 Topic의 실제 publisher QoS가 `RELIABLE + TRANSIENT_LOCAL`인지 확인
4. D4가 두 Topic을 실제로 수신하는지 로그와 `ros2 topic echo`로 확인
5. `AX_D4_LEDGER_PATH`에 SQLite 파일이 생성되는지 확인
6. `MOCK_ONLY` 상태가 `UNKNOWN`으로 처리되는지 확인
7. `/clock`이 없는 환경에서 `use_sim_time=true`일 때 timestamp 검증이 정상적으로 보류되는지 확인
8. 실제 A3 상태가 `meta.valid=true`인 별도 시험 입력으로 원장 기록까지 확인

실제 Ubuntu ROS2 시험을 통과하기 전까지는 D4 W0를 “코드·오프라인 시험 완료,
ROS2 종단 검증 대기” 상태로 봅니다.
