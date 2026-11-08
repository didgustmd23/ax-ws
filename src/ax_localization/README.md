# ax_localization (B2)

B2 지도 작성·위치 추정 패키지다. 이 패키지가 제공하는 것은 다음 두 가지다.

- **지도 작성:** `mapping.launch.py`와 지도 작성·검증 도구. 사용법은 [지도 안내](maps/aws_warehouse/README.md)에 있다.
- **위치 추정:** 이 문서. 공통 `map_server` 1개와 로봇별 AMCL로 구성한다.

설계 기준은 [상세설계 6.1절](../../docs/design/DETAILED_DESIGN.md#61-기준-지도-작성검증전달-b2)과 [공통 인터페이스 계약](../../docs/design/INTERFACE_CONTRACT.md) 2.2·4.2절이다.

## 위치 추정(AMCL) — 개발용

- **현재 상태:** 채택한 운영 지도가 없어 개발용 지도로만 실행·검증했다. A1이 운영 모델·기준 월드를 확정하면 지도를 다시 만들고 위치 추정도 다시 검증한다.
- **정확도 판정:** 기준이 미확정이므로 수치만 기록하고 판정은 `not_judged`다.
- **범위 밖(후속 작업):** 위치 추정 상태(`LocalizationHealth`), 재위치 서비스(`localization/reinitialize`), Nav2 연결, 운영 지도 채택.

### Topic·frame

| 항목 | 이름 · 타입 | QoS | 생산자 |
|---|---|---|---|
| 지도 | `/map` · `nav_msgs/OccupancyGrid`, frame `map` | reliable · transient_local | 공통 `/map_server` 1개 |
| LiDAR 입력 | `/ax/<robot_id>/sensors/front_lidar/scan_filtered` · `LaserScan`, frame `<robot_id>/front_lidar_link` | SENSOR(best_effort) | B1 |
| 초기 위치 입력 | `/ax/<robot_id>/initialpose` · `geometry_msgs/PoseWithCovarianceStamped`, frame `map` | Nav2 AMCL 구독 기본값(확인하지 않음) | 시험자·후속 재위치 |
| 추정 결과 | `/ax/<robot_id>/amcl_pose` · `PoseWithCovarianceStamped`, frame `map` | reliable · transient_local | `/ax/<robot_id>/amcl` |
| TF | `map → <robot_id>/odom` | TF 기본 | `/ax/<robot_id>/amcl`(계약상 B2 소유) |
| TF | `<robot_id>/odom → <robot_id>/base_link` | TF 기본 | A2 `diff_drive_controller` |

- AMCL TF의 stamp는 처리한 스캔 시각에 `transform_tolerance`(1.0 s)를 더한 값이다(Nav2 동작).
- `/map`은 [공통 인터페이스 계약 4.2절](../../docs/design/INTERFACE_CONTRACT.md#42-로봇-안전위치-ab)에 등록돼 있다(초안). 지도 작성 중 slam_toolbox가 내는 `/ax/<robot_id>/map`은 저장 도구용이며 위치 추정에 쓰지 않는다.
- 설정은 [`config/amcl.yaml`](config/amcl.yaml)에 있다. 거리 범위는 B1 출력(0.12~10 m)에 맞췄고 나머지는 Nav2 기본값에서 출발한 개발값이다.

### 실행 순서

모든 터미널에서 같은 `ROS_DOMAIN_ID`·`GZ_PARTITION`을 쓴다. 검증 실행은 사용자 세션과 다른 값으로 격리한다.

```bash
source install/setup.bash
export ROS_DOMAIN_ID=<번호> GZ_PARTITION=<이름> ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# 1. 시뮬레이션(A1·A2)과 B1 LiDAR 전처리
ros2 launch ax_control simulation.launch.py world:=aws_warehouse.world robot_ids:=robot_1,robot_2 gui:=false
ros2 launch ax_perception sensors.launch.py robot_ids:=robot_1,robot_2

# 2. 위치 추정: 공통 지도 서버 + 로봇별 AMCL
ros2 launch ax_localization localization.launch.py \
    map:=log/b2_mapping/<KST 시각>/run/slam_map.yaml robot_ids:=robot_1,robot_2 \
    initial_poses:="robot_2=0,0,0;robot_1=-2.5,1.5,0"

# 3. 확인
ros2 lifecycle get /map_server
ros2 lifecycle get /ax/robot_2/amcl
ros2 run tf2_ros tf2_echo map robot_2/base_link
```

- launch는 시뮬레이션 로봇(`/ax/<robot_id>/robot_state_publisher`)이 보이지 않으면 시작하지 않는다.
- 다음이 이미 있으면 시작하지 않는다. 노드 이름, `/map` 발행자, `/tf`의 실제 변환으로 확인한다.
  - 같은 로봇의 `slam_toolbox`·`amcl`
  - 다른 `map_server`
  - `/tf`에 이미 들어오는 `map → <robot_id>/odom`
- AMCL이 실행 중이면 `mapping.launch.py`도 같은 로봇에 SLAM을 시작하지 않는다.
- 두 launch를 동시에 띄우면 서로를 확인하기 전이라 이 검사를 피할 수 있다. 하나가 활성화된 뒤에 다른 것을 실행한다.
- lifecycle은 launch가 직접 configure → activate한다. Nav2 `lifecycle_manager`는 쓰지 않는다. 시험에서 문제가 둘 확인됐기 때문이다.
  - DDS 발견 경합으로 configure 응답이 유실되자 시간 제한 없이 멈췄다.
  - bond가 끊긴 AMCL에 다시 연결하면 launch 초기 위치로 조용히 재초기화됐다.

### 시작 실패 처리

- `map_server`와 모든 AMCL이 `startup_timeout`(기본 60 s, 벽시계) 안에 모두 active가 되어야 시작 성공이다.
- **시작 중**은 모든 노드가 active가 되기 전이고, **활성화 완료 후**는 그 뒤다. 두 구간에서 노드가 끝났을 때의 처리가 다르다.

| 구간 | 사건 | 처리 | 종료 코드 |
|---|---|---|---|
| 시작 중 | configure·activate 거절(전이가 `unconfigured`·`inactive`로 되돌아가거나 `errorprocessing`·`finalized`로 감) | **실패 처리 → 정리** | 1 |
| 시작 중 | 무응답(제한시간 안에 active가 되지 않음) | **실패 처리 → 정리** | 1 |
| 시작 중 | 노드 프로세스 종료 | **실패 처리 → 정리** | 1 |
| 활성화 완료 후 | 노드 프로세스 종료 | **기록만 수행.** `startup_report`에 그 노드의 `exit_code`를 남기고 오류 로그(`시작 뒤 … 프로세스 종료`)를 낸다. `status: active`·`failure: null`은 그대로이고 launch와 다른 노드는 계속 실행된다. 종료된 로봇의 `map → odom`만 멈추며 자동 재시작하지 않는다 | 영향 없음(나중에 끝낼 때의 코드는 정상 종료 기준) |
| 정리 중(Ctrl+C·실패 처리 중) | 노드 프로세스 종료 | `exit_code`만 기록하고 새 실패로 만들지 않는다 | — |

- 실패 처리는 다음 순서다.
  1. 실패 노드·단계(`configure`·`activate`·`timeout`)·사유를 로그(`시작 실패: ...`)에 남긴다. `startup_report:=<경로.json>`을 주면 노드별 상태·전이 기록도 JSON으로 남긴다.
  2. 이번 launch가 실행한 노드만 정리한다. 이미 실행 중인 시뮬레이션·B1·다른 노드는 종료하지 않는다.
  3. 종료 코드 1로 끝난다.
- 활성화 완료 후 종료를 다른 기능에 알리는 것은 이 launch가 하지 않는다. 위치 신뢰성 상태(`LocalizationHealth`, 후속)로 다룬다.

```bash
ros2 launch ax_localization localization.launch.py map:=<지도.yaml> robot_ids:=robot_2 \
    startup_timeout:=60 startup_report:=/tmp/<폴더>/startup.json
```

### 종료 순서

1. 위치 추정 launch를 Ctrl+C로 끝낸다. `map → <robot_id>/odom` 발행이 멈춘다.
2. B1 launch를 끝낸다.
3. 시뮬레이션을 끝낸다.

로봇 하나만 멈추려면 `ros2 lifecycle set /ax/<robot_id>/amcl deactivate`를 쓴다. 자동으로 다시 활성화되지 않으며 그 로봇의 `map → odom`만 멈춘다. 같은 로봇에서 SLAM을 쓰려면 위치 추정 launch를 먼저 끝낸다.

### 초기 위치 지정

- **launch 인자:** `initial_poses:="robot_id=x,y,yaw;..."`(`map` 기준 m·rad). 지정하지 않은 로봇은 `initialpose` 입력을 기다리며 TF를 내지 않는다.
- **개발용 지도의 시험값:** 지도 작성 회차의 `map` 원점은 robot_2의 출발 위치다.
  - robot_2 출발 위치는 `(0, 0, 0)`이다.
  - robot_1은 launch 출발 위치가 world (0, 0)이므로 `map`에서 약 `(-2.5, 1.5, 0)`이다(`robot_test.launch.py` 출발 위치 기준).
  - 다른 지도에서는 이 값이 맞지 않는다.
- **실행 중 변경:** 해당 로봇의 Topic에만 보낸다.

  ```bash
  ros2 topic pub --once -w 2 /ax/robot_1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: -2.5, y: 1.5}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
  ```

  `-w`는 기다릴 구독자 수다. 다른 구독자(기록 도구 등)가 있으면 그 수만큼 늘려야 AMCL이 받는다. 시험에서 `-w 1`은 기록 도구에만 전달되고 끝났다.
- **정지 상태의 한계:** 로봇이 멈춰 있으면 AMCL은 틀린 초기 위치를 고치지 않는다(`update_min_d` 0.25 m·`update_min_a` 0.2 rad). 시험에서 0.5 m 틀린 입력이 그대로 유지됐다.

### 평가 도구

`scripts/check_localization.py`가 시뮬레이션·B1·위치 추정에 붙어 로봇별로 다음을 기록한다.

- 입력·출력: `map→odom` TF, odom, `amcl_pose`, `initialpose`, 스캔 수신
- 검증 전용: Gazebo 실제 자세
- 연결: `/tf` 발행 노드, TF 연결 조회 결과

같은 시각의 AMCL 추정 자세와 실제 자세의 오차를 `localization_result.json`에 쓴다.

```bash
python3 src/ax_localization/scripts/check_localization.py --robot-ids robot_1,robot_2 --duration 40 \
    --map-accuracy log/b2_mapping/<KST 시각>/run/accuracy.json --output <결과 폴더>
```

`--map-accuracy`의 `T_world_map`은 지도 검증 기록의 값이다. 실제 자세를 `map`으로 옮기는 데만 쓰며 위치 추정 입력으로 쓰지 않는다.

결과의 `status`는 검증 가능 여부이고, `judgment`는 정확도 판정이다. 둘은 따로 본다.

| 값 | 의미 |
|---|---|
| `status: completed` | 요청한 기록 시간을 채웠고 요청한 **모든** 로봇이 검증 가능하다. 정확도가 좋다는 뜻은 아니다 |
| `status: incomplete` | 기록 시간이 부족하거나 한 로봇이라도 필수 입력·연결이 없다. `problems`와 로봇별 `problems`에 사유를 남기고 종료 코드 1로 끝난다. 확보한 기록·계산 결과는 그대로 남는다 |
| `judgment: not_judged` | 위치 추정 정확도 기준이 미확정이라 판정하지 않았다. 수치만 기록한다 |

로봇별 필수 항목은 다음과 같다. 하나라도 없으면 그 로봇은 `verifiable: false`다.
- 스캔·odom·`map → odom` 수신
- Gazebo 실제 자세 표본과 오차 계산 표본
- `map → odom → base_link` TF 연결

AMCL을 실행하기 전이나 비활성화한 뒤에 기록하면, `map → odom`이 없어 `incomplete`가 나오는 것이 정상이다. TF가 없다는 것을 확인할 때 이 결과를 쓴다.

### 개발용 지도 제한

- **시험 지도:** `log/b2_mapping/20261108_131510/run/slam_map.yaml`(Git 제외).
  - 단순화 월드(`light_collision`)와 바퀴 `slip1=0.05` 임시 사본에서 만든 개발용 지도이며 채택하지 않았다([S005 자료](../../docs/evidence/wheel_slip/)).
  - launch는 `map_info.yaml` 채택 기록이 없는 지도를 "개발용 지도"로 표시한다.
- **월드 조건:** 위치 추정 시험도 같은 단순화 월드에서 실행했다(바퀴는 원본 설정). 원본 월드와 운영 모델에서는 확인하지 않았다.
- **보장 범위:** 지도는 앞 LiDAR 높이의 관측만 담는다. 미관측 영역과 다른 높이의 장애물은 위치 추정 근거가 되지 않는다.

### 개발 시험 결과 (2026-11-08, `not_judged`)

시험 조건은 다음과 같다.
- 격리 headless Gazebo, 단순화 월드, 원본 바퀴, B1 `scan_filtered`
- 위의 개발용 지도(sha256 `6ab83ae7…`), `T_world_map`은 그 지도의 `optimal_T_world_map`
- 오차는 0.5 s 간격 표본의 AMCL 추정(`map→odom · odom`)과 실제 자세의 차이

| 시험 | 결과 |
|---|---|
| robot_2 정지 | 스캔·`map→odom` 약 10 Hz. `map→odom→base_link` 연결됨. 위치 오차 0.008 m(지도 정렬 보정 수준), 방향 0.0003 rad |
| TF 발행 소유 | 실행 전 `map→robot_2/odom` 0건. `/tf` 발행 노드는 AMCL·`diff_drive_controller`·`robot_state_publisher`뿐. AMCL 비활성화 뒤 0건. 중복 AMCL·SLAM launch 거절 |
| robot_2 이동(직진 1.5 m → 90° 회전 → 직진 6.8 m → 정지) | 위치 오차 평균 0.198·p95 0.407·최대 0.487 m, 방향 오차 최대 0.092 rad, 정지 후 0.199 m. 국면별 최대: 직진 0.456 m, 회전 0.487 m. 같은 주행의 odom 단독 오차는 최대 0.484·마지막 0.468 m |
| robot_1·robot_2 동시(정지) | 스캔 frame·`initialpose`·`amcl_pose`·TF가 로봇별로 분리됨. robot_1 초기 위치를 0.5 m 바꾸자 robot_1 `map→odom`만 0.507 m 바뀌고 robot_2는 변화 0·새 추정 0건. robot_2 AMCL을 비활성화해도 robot_1 발행 유지 |

**알려진 제한**
- **미완료:** 운영 조건(원본 월드·운영 모델) 검증, 위치 신뢰성(`LocalizationHealth`), 재위치 서비스(`localization/reinitialize`). 정지 상태에서는 틀린 초기 위치가 보정되지 않는다.
- **Ctrl+C 시점의 보고 상태:** 모든 노드가 active가 되기 전에 사용자가 Ctrl+C로 끝내면 노드는 정리되지만, `startup_report`의 `status`는 마지막 값인 `starting`으로 남는다. 이 경우는 따로 시험하지 않았다.
- **`run_mapping_session.sh --light` 기존 결함:** 임시 사본에 `lib/ax_description`이 없어 robot_1을 띄울 수 없다. robot_2 지도 작성에는 영향이 없다. 위 위치 추정 시험은 이 문제를 보완한 별도 하네스(저장소 밖)로 실행했다.

**확인하지 못한 것**
- 두 로봇이 동시에 이동하는 경우
- 전체 지도 경로
- 원본 월드와 운영 모델
- 긴 시간 실행
- 전역 위치 추정(초기 위치 없이 시작)
- 시뮬레이션이 매우 느릴 때 AMCL TF 시각 처리
