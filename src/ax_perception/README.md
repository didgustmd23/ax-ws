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
생성물 비교와 위 34개 검사는 2026-11-09 작업트리에서 통과했다.
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

## W2 개발용 보정·동기화 검증 — 2026-11-09

기존 전처리 구현을 실제 ROS·Gazebo 입력으로 확인했다. 실물 센서 보정 계수를 추정하거나
실물 정확도를 인증한 작업이 아니다. 이상적인 시뮬레이션의 장착·투영·거리·시각 일관성을 검증한다.

- robot_1/2 전방 RGB-D: 알려진 평면 1·2 m, 조건당 서로 다른 stamp 3개.
  비중심 픽셀의 깊이·점군 X/Y/Z, CameraInfo 초점거리, 전방 LiDAR 광선 거리,
  전후 LiDAR·전방 RGB/RGB-D 장착 TF를 설정과 비교했다. 15 mm 개발 허용값에서 통과했다.
- robot_1 epick 팔: 팔 RGB-D 10·15 cm, ToF 8·15 cm에서 조건당 3표본.
  관측 시각 TF·Gazebo 로봇 자세로 평면의 예상 거리를 계산해 20 mm 개발 허용값에서 통과했다.
  팔 관측은 TF와 렌더링의 일관성 시험이며, TF와 독립된 팔 외부 보정은 아니다.
- ROS 모의 입력 13조건: 정상 stamp 보존, 중복·역행·오래된 값·미래 값·다른 로봇 frame 거절,
  RGB-D 40 ms 오차 거절/20 ms 회복, 전부 무효인 깊이의 점군·오류 진단,
  LiDAR ±inf/NaN, 시계 역행, sim time 정지 중 수신 만료와 새 입력 회복을 확인했다.

### 확인한 한계

팔 카메라 중립 자세의 중심 광선에서 25 cm 시험판 대신 약 19.8 cm 앞의 형상이 관측됐다.
가려진 시험판의 거리를 보정 오차로 해석하지 않는다. 해당 초기 실패 기록도 보존했다.
10·15 cm 시험 통과는 모든 자세·전체 시야에서 물체 표면을 볼 수 있다는 뜻이 아니다.
표면 관측이 필요한 C2/C4 연결 전에 작업 자세·관심 픽셀의 가림을 추가 확인해야 한다.
팔 RGB 단독 영상의 가림·화질·C1 검출 성능, 하드웨어 내/외부 보정, IMU 오차 추정,
센서 정격률·장시간·4대 부하·WSL 실제 절전 복귀는 이번 검증에 포함하지 않았다.

### 재현

아래 명령은 저장소 루트에서 ROS·빌드 결과를 source한 뒤 실행한다.
실제 운용과 다른 ROS_DOMAIN_ID·GZ_PARTITION을 사용한다.

```bash
# 전방 센서: robot_1 또는 robot_2를 한 대씩 실행
ros2 launch ax_description robot_test.launch.py gui:=false robot_ids:=robot_2 tool:=epick
ros2 launch ax_perception sensors.launch.py robot_ids:=robot_2 tool:=epick
python3 tools/check_w2_calibration.py --robot-id robot_2 --output log/w2/robot_2_calibration.json

# 팔 센서: 위 실행을 종료하고 별도 격리 실행. A2는 관절 상태/유지에 사용하며 명령을 보내지 않는다.
ros2 launch ax_control simulation.launch.py gui:=false robot_ids:=robot_1 tool:=epick
ros2 launch ax_perception sensors.launch.py robot_ids:=robot_1 tool:=epick
python3 tools/check_w2_arm_sensors.py

# 오류 시험: Gazebo·다른 clock 발행자가 없는 별도 domain에서 B1 robot_2만 실행
ros2 launch ax_perception sensors.launch.py robot_ids:=robot_2
python3 tools/check_w2_sensor_faults.py
```

launch·검사기는 각각 별도 터미널에서 실행한다. 평면 검사기는 자기 이름의 시험판만 생성·삭제한다.
오류 검사기는 합성 clock·센서를 발행한다. 결과와 초기 실패는 [W2 증거](../../docs/evidence/w2/)를 따른다.
