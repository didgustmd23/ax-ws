# B1 센서 구성 — W1

이종수 담당 W1은 모델의 센서 장착·브리지·로봇별 입력 구성을 확인한다.
현재 전처리 코드의 보정·동기화 기능은 W2와 이어지는 구현이며, 이 문서로 실물 보정이나 W2 완료를 선언하지 않는다.

## 구성과 경계

- 장착 위치·방향: `../ax_description/config/sensor_mounts.yaml`.
- 토픽·타입·단위·주기: `../ax_description/config/sensor_contract.yaml`.
- 모델·센서 브리지: `../ax_description/launch/robot_test.launch.py`.
- B1 전처리: `scripts/sensor_processing.py`, `launch/sensors.launch.py`.
- robot_1/3은 팔 로봇, robot_2/4는 운송 로봇이다. 입력 토픽은 `/ax/robot_<n>/sensors/`, TF는 `robot_<n>/`로 분리한다.
- 카메라 영상·깊이·CameraInfo·점군, 전후 LiDAR·IMU, 팔 로봇의 공구 ToF, 운송 로봇의 적재판 접촉을 연결한다.
- 접촉 신호는 A3 적재 판정이 아니며 ToF는 흡착 성공 신호가 아니다. A2 제어기와 A4 위험·구동 허용 조건은 변경하지 않는다.

## W1 실행 확인

저장소 루트에서 실행한다. 기본 모델 시험에는 제어기가 필요하지 않다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ax_description ax_perception
source install/setup.bash
ros2 launch ax_description robot_test.launch.py gui:=false sensors:=true tool:=epick
```

같은 환경을 source한 다른 터미널에서 다음을 실행한다.

```bash
python3 tools/check_w1_sensors.py --robot-ids robot_1,robot_2 --tool epick
```

검사기는 구동 명령 없이 센서 토픽을 구독하고 각 입력의 2회 이상 수신·stamp 진행·로봇별 frame 접두사를 확인한다.
접촉 메시지의 빈 frame은 계약에 따라 허용한다. 결과는 `log/w1/sensors.json`에 저장한다.
반복 수신 확인은 정격 주기·보정 정확도·시야·장시간 부하 시험을 대신하지 않는다.
다른 공구나 로봇 조합은 실행한 모델과 동일한 `--tool`, `--robot-ids`로 지정한다.

B1 전처리 연결은 별도 터미널에서 실행한다. 월드·제어기를 새로 띄우지 않는다.

```bash
ros2 launch ax_perception sensors.launch.py robot_ids:=robot_1,robot_2 tool:=epick
```

## 모델·센서 코드 검사

B1 수신 누락·ToF TF 대기는 `CLOCK_BOOTTIME`으로 경과 시간을 잰다.
측정 stamp·동기화·TF 조회 시각은 기존 ROS sim time을 유지한다.
절전 시 BOOTTIME만 진행하는 모의 조건에서 수신 진단이 STALE로 바뀌는 회귀 검사를 포함한다.
WSL 호스트의 실제 절전·복귀에서 시계가 어떻게 진행하는지는 별도 실측 대상이다.

```bash
python3 tools/generate_asset_v2.py --check
python3 tools/generate_physical_properties.py --check
python3 -m unittest discover -s tools/tests -p test_a1_assets.py
python3 -m unittest discover -s tools/tests -p test_w1_models.py
python3 -m unittest discover -s tools/tests -p test_sensor_processing.py
```

실행 전에 ROS와 빌드 결과를 source한다.
생성물 비교와 위 33개 검사는 2026-11-09 작업트리에서 통과했다.
Gazebo 실행 결과는 JSON의 `passed`와 각 토픽 기록으로 확인한다. G1 통과 근거로 쓰지 않는다.

2026-11-09 제어기 없는 두 로봇·epick·headless 월드에서 27개 중 26개 입력의 반복 수신과
stamp 진행·frame 분리를 확인했다. `robot_2/sensors/cargo_contact`는 무적재 조건에서 수신이 없어
당시 전체 검사 결과는 `passed=false`로 보존한다. 이후 별도 robot_2 월드에서 0.5 kg 상자를 올려
서로 다른 stamp의 ROS 접촉 메시지에 적재판·시험 상자의 collision이 함께 나타나는 것을 확인했다.
실제 발행 확인까지 W1 구성 검증을 마무리했다. [검증 자료](../../docs/evidence/w1/)를 참조한다.

접촉 검사 재현: 위 모델 launch에 `robot_ids:=robot_2`를 지정한 뒤 같은 환경의 다른 터미널에서 실행한다.

```bash
python3 tools/check_w1_cargo_contact.py
```

이 명령은 실행 중인 `ax_test`에 0.5 kg 시험 상자를 생성하고 검사 후 삭제한다.
구동 명령·A3 적재 판정은 수행하지 않는다. 실행 중인 월드와 ROS domain·Gazebo partition을 맞춰야 한다.
