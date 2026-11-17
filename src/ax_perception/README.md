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

B1 노드와 센서 브리지는 큰 영상·점군 누락을 막기 위해 `ax_description/config/fastdds_large_data.xml`을 `FASTRTPS_DEFAULT_PROFILES_FILE`로 받는다(launch가 비어 있을 때만 지정). B1을 `ros2 run`으로 띄우거나 bag을 재생할 때는 `export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ax_description)/share/ax_description/config/fastdds_large_data.xml`을 먼저 실행한다.

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

2026-11-13 최신 소스·AX_2 모델로 재검증했다. 빈 바닥 시험 치구에서 robot_1/2 전방 1·2 m 각 3표본,
robot_1 팔 RGB-D 10·15 cm 및 ToF 8·15 cm 각 3표본, 격리 domain의 합성 오류·회복 13조건이 통과했다.
기존 2026-11-09 기록과 팔 25 cm 가림 실패는 유지한다. 이번에도 전체 팔 시야·실물 보정·실제 절전 복귀는
검증하지 않았다. [새 결과와 재현 조건](../../docs/evidence/w2_recheck_2026-11-13/)을 참조한다.

### W2 수신 품질·시야 보완 — main 48a0ea6

- Gazebo/B1과 같은 격리 domain에서 `check_w2_sensor_quality.py --robot-id <robot_1 또는 robot_2>
  --duration-s 15 --output <새 결과 경로>`로 수신률·시각 간격·입출력 stamp 대응·RGB-D 시각 차·정지 IMU를 기록한다.
  수신률은 sim time 기준이며 승인된 성능 기준이 없어 성능 판정은 not_judged다.
- `check_w2_arm_visibility.py --pose <neutral 또는 reach_left 또는 reach_right> --prepare-only <새 폴더>`로
  고정 자세 치구 launch를 생성한다. 이 launch 실행 뒤 같은 도구에 `--pose`와 `--output`을 주어 조사한다.
  실제 팔 명령을 보내지 않는다. 세 자세 모두 좌/중앙/우의 10·15 cm 표적은 보이고 25 cm는 가렸다.
- 거리 검사기는 장면 교체 후 1.0 sim s 안정화한다. NaN·inf가 있으면 null과 실패 상태로 저장해
  검사 결과 파일 자체가 사라지지 않게 했다. 최초 과도 표본·검사기 오류는 별도 보존했다.

[전체 결과·실행 조건](../../docs/evidence/w2_complete_2026-11-13/)과
[B2/B3/B4·A4·C2/C4 인계 자료](../../docs/evidence/w2_complete_2026-11-13/HANDOFF.md)를 참조한다.

## C2 표면 관측 입력 노드(개발 중)

`scripts/c2_surface_node.py`는 같은 RGB-D 카메라의 B1 출력
`sensors/<camera>/processed/{image,depth_image,camera_info}`와 명시한 검출 토픽(`vision_msgs/Detection2DArray`)을
SENSOR QoS로 구독해, 정확히 같은 stamp의 깊이로 bbox 안 보이는 평면 1개를 계산한다.
계산은 `surface_estimation.py`·`c2_detection_link.py`를 쓰며 두 모듈은 실행 스크립트와 같은 `lib/ax_perception`에 설치된다.
`c2_surface_adapter.py`(C3 Surface 변환)는 노드가 쓰지 않아 설치하지 않는다(설치 미지원, C3 소스 경로 import).

- 출력은 `record_path`의 JSON Lines 로컬 기록뿐이다. 공개 Surface 메시지·토픽은 확정하지 않았다.
- 기록의 `VALID`/`LINKED`는 plane_fit 유효일 뿐 목표 물체 일치가 아니다. object_id·대상 대응·confidence·질량·흡착 적합성을 만들지 않는다.
- 시각: 권위 시각은 header 정수 `stamp_ns`(sec×10^9 + nanosec)다. 관측 ID·변환 계보·대응 판정(`TargetAssociation.observation_stamp_ns`)·
  조립·버퍼·만료·중복·순서·TF 조회와 응답 대조를 정수로 한다. `stamp_s`는 표시·기하·C3 float API 전달용 파생값이며,
  C3 `Surface.stamp_s` 이후로는 ns 정밀성이 보존되지 않는다.
  float 초만 남은 기록은 `legacy_stamp_ns()`의 명시적 비정확 경로(`stamp_origin='legacy_float'`, 관측 ID 접두어 `obsL-`)로만 다루며
  원래 ns라고 주장하지 않는다. 비정확 관측은 정확 관측으로 바꿀 수 없고, 어댑터는 MATCH가 있어도 `STAMP_NOT_EXACT`로 보류한다.
- 최신 깊이로 대체하거나 stamp를 다시 쓰지 않는다. 다른 frame(예: `front_rgb`) 검출·부품은 받지 않고 정상 stamp 조립을 바꾸지 않는다.
  부품 도착 순서는 상관없고, 미완성 부품·검출 선도착·TF 대기는 감독 시계(CLOCK_BOOTTIME) 상한 뒤 사유와 함께 거부한다.
  sim 시각 0이면 입력을 받지 않고, 역행하면 버퍼를 비운다. 변환은 `target_frame`을 줄 때만 관측 stamp_ns TF로 한다.
- 입력 경계(보존·복사 전 검사): 영상 해상도(`expected_width/height`)·정확한 step(행 패딩 미지원)·data 길이·`max_image_bytes`,
  CameraInfo `d` 길이(`max_distortion_len`)·문자열 길이(`max_text_len`), 검출 수(`max_detections_per_array`)·가설 수·class_id 길이.
  검출 수 등 크기 상한은 frame·stamp·미래·만료·중복·역행 검사를 통과한 배열에만 적용하고 배열 전체를 거부한다(결과 0개).
  정확 stamp 묶음이 있을 때만 그 배열을 처리 완료로 표시하고 묶음을 해제한다.
- 보존 상한: 미완성 부품(`part_capacity`)·묶음(`bundle_capacity`)·검출 대기(`detection_capacity`)·TF 대기(`tf_pending_capacity`,
  배열 결과가 다 들어가지 않으면 그 배열 결과 전부 `TF_QUEUE_FULL`)·조립 결과 이력(`outcome_history`)·사유별 상세 진단 행
  (`diagnostic_rows_per_reason`, 이후는 카운터만)·기록 파일(`max_record_bytes`). 큐별 크기·최대치(high-water)·축출·거부를 state 기록에 남긴다.
- 늦게 온 부품(이미 조립·사용된 stamp)은 내용 지문으로 동일·상충·이력 축출(판단 불가)·거부 뒤 도착을 나눠 센다.
  이미 낸 결과를 철회·재계산하지 않으며 상충은 진단일 뿐이다.
- 모든 설정에 기본 운영값이 없다. 빠지거나 범위·상호관계(예: `tf_pending_capacity ≥ max_detections_per_array`)가 맞지 않으면
  시작 시 `CONFIG_INVALID`로 종료(코드 2)한다. `record_path`는 절대 경로이며 이미 있으면 덮어쓰지 않고 시작을 거부한다.
- 종료 코드: 0 정상, 2 시작 거부, 3 기록 쓰기 실패, 4 기록 바이트 상한 도달(표식 1행을 남기고 멈춤, 정상 완료 아님). 같은 실패를 반복 기록하지 않는다.
- C1 기본 구성(`front_rgb` 입력, `perception/detections` 출력)은 이 노드 입력이 아니다. 실제 C1 연결에는 RGB-D 컬러용 가중치·추론 환경·
  `front_rgbd` 입력/출력 토픽 구성·stamp 보존 확인이 먼저 필요하다. 검출이 없으면 기록에 결과가 없을 뿐 성공으로 처리하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run ax_perception c2_surface_node.py --ros-args -r __ns:=/ax/robot_1 --params-file <설정 YAML>
```

검사: `tools/tests/test_c2_surface_node.py`(실제 ROS 메시지·콜백 직접 호출),
`AX_C2_NODE_PREFIX=<격리 install> tools/tests/test_c2_surface_node_process.py`(설치본 ros2 run·DDS·종료 코드·정리).
두 검사의 설정값은 시험값이며 운영 승인값이 아니다.

### C2 검증 범위와 한계 — 2026-11-17

| 구분 | 방식 | 결과 |
|---|---|---|
| 계산·연결 라이브러리 | 합성 깊이·합성 검출, 순수 함수 시험(`test_c2_surface.py`, `test_c2_detection_link.py`) | 통과 |
| 입력 노드 | 실제 ROS 메시지 객체로 구독 콜백을 직접 호출(`test_c2_surface_node.py`, 같은 프로세스, DDS 아님). 무작위 1000회 입력의 큐 상한 확인 포함 | 통과 |
| 설치본 실행 | 격리 install을 `ros2 run`으로 실행, 격리 domain의 실제 DDS로 합성 입력(`test_c2_surface_node_process.py`) | 통과. `AX_C2_NODE_PREFIX`가 없으면 생략(skip)되며, 생략은 통과가 아니다 |
| 기록 fixture 재생(1회, 정수 ns 보완 전 코드) | 고정 RGB-D 기록의 깊이·CameraInfo로 재생. 컬러는 대표 프레임 1장을 반복, 검출은 사람이 고정한 ROI로 만든 fixture bbox | 발행 30프레임·검출 90건 중 첫 프레임은 4개 토픽 모두 미수신. 노드가 받은 29프레임·검출 87건만 오프라인 계산과 상태·사유·관측 ID가 일치 |

재생에서 수신분 87건이 일치했다고 해서 입력 30/90 전체가 성공한 것은 아니다.
실제 C1 검출도, 프레임별 원 컬러와 깊이의 동기화도 검증하지 않았다(프레임별 원 컬러는 보존돼 있지 않다).
정지 장면 프레임은 독립 반복이 아니며 정확도·성공률 판정은 하지 않았다.
재생 결과는 보완 전 코드의 결과다. 관측 ID 규칙이 정수 ns로 바뀌었으므로 그 기록의 ID를 새 ID와 비교하지 않으며, 재생은 반복하지 않았다.

남은 한계(정책 미정, 이번에 임의로 정하지 않음)

- 서로 다른 stamp의 묶음이 역순으로 완성되면 앞 stamp는 연결기 정책에 따라 `OUT_OF_ORDER_BUNDLE`로 버려진다.
- rclpy가 콜백 전에 메시지 하나를 역직렬화하는 메모리와 DDS 계층 메시지 크기는 이 노드가 제한하지 못한다. 상한은 그 뒤의 보존·복사·계산에 적용된다.
- tf2 캐시는 `tf_cache_s` 시간 상한만 있고 다른 발행자의 frame 수·발행률에 따른 개수 상한은 없다.
- 기록 파일 상한·쓰기 오류는 모의(쓰기 실패 주입·작은 상한)로만 시험했다. 실제 디스크 가득 참·네트워크 파일시스템 지연은 시험하지 않았다.
  상한 도달 뒤 산출물이 끊기는 동작(종료 코드 4)을 운영에서 어떻게 다룰지는 정하지 않았다.
- `max_image_bytes` 검사는 해상도·step 정확 일치와 `max_image_bytes ≥ 4×width×height` 설정 조건 아래에서는 실제로 걸리지 않는 방어 검사다.
- 어댑터 `accepted_frames` 목록 개수는 상한을 두지 않았다(호출자 설정 값). 각 문자열 길이만 본다.
- C2 `VALID`는 plane_fit 유효일 뿐이다. C3 Surface 어댑터는 외부 대상 대응 판정(TargetAssociation MATCH)이 없으면 Surface를 보류한다.
  외부 판정의 정확성은 검증하지 않았다(판정 주체 C4 미구현).
- 시험 설정(`TEST_SETTINGS`, 용량·대기·만료·개수·길이 상한)은 시험값이며 운영 기본값이 아니다.
