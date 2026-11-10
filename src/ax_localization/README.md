# ax_localization (B2)

B2 지도 작성·위치 추정 패키지다. 이 패키지가 제공하는 것은 다음 두 가지다.

- **지도 작성:** `mapping.launch.py`와 지도 작성·검증 도구. 사용법은 [지도 안내](maps/aws_warehouse/README.md)에 있다.
- **위치 추정:** 이 문서. 공통 `map_server` 1개와 로봇별 AMCL로 구성한다.

설계 기준은 [상세설계 6.1절](../../docs/design/DETAILED_DESIGN.md#61-기준-지도-작성검증전달-b2)과 [공통 인터페이스 계약](../../docs/design/INTERFACE_CONTRACT.md) 2.2·4.2절이다.

## 위치 추정(AMCL) — 개발용

- **현재 상태:** 채택한 운영 지도가 없어 개발용 지도로만 실행·검증했다. A1이 운영 모델·기준 월드를 확정하면 지도를 다시 만들고 위치 추정도 다시 검증한다.
- **정확도 판정:** 기준이 미확정이므로 수치만 기록하고 판정은 `not_judged`다.
- **구현 범위:** 공통 지도 서버·로봇별 AMCL, 위치 신뢰성 발행(`LocalizationHealth`), 고정 출발 위치 최초 초기화, 같은 세션의 마지막 신뢰 위치 기반 제한 재위치(`localization/reinitialize`).
- **범위 밖:** 전역 재위치, 표식 인식, 복구 이동, 예비 로봇 배정, 안전 해제, Nav2 연결, 운영 지도 채택, 위치 이상 탐지(Isolation Forest, `anomaly_model_available=false`).
- **A4·D:** 실제 A4(`ActuationState`)와 D(재위치 요청·작업 보류)는 아직 없다. 시험은 스텁으로만 했으며 실제 통합은 확인하지 않았다.

### Topic·frame

| 항목 | 이름 · 타입 | QoS | 생산자 |
|---|---|---|---|
| 지도 | `/map` · `nav_msgs/OccupancyGrid`, frame `map` | reliable · transient_local | 공통 `/map_server` 1개 |
| LiDAR 입력 | `/ax/<robot_id>/sensors/front_lidar/scan_filtered` · `LaserScan`, frame `<robot_id>/front_lidar_link` | SENSOR(best_effort) | B1 |
| 초기 위치 입력 | `/ax/<robot_id>/initialpose` · `geometry_msgs/PoseWithCovarianceStamped`, frame `map` | Nav2 AMCL 구독 기본값(확인하지 않음) | 시험자·후속 재위치 |
| 추정 결과 | `/ax/<robot_id>/amcl_pose` · `PoseWithCovarianceStamped`, frame `map` | reliable · transient_local | `/ax/<robot_id>/amcl` |
| TF | `map → <robot_id>/odom` | TF 기본 | `/ax/<robot_id>/amcl`(계약상 B2 소유) |
| TF | `<robot_id>/odom → <robot_id>/base_link` | TF 기본 | A2 `diff_drive_controller` |
| 위치 신뢰성 | `/ax/<robot_id>/localization/health` · `ax_interfaces/LocalizationHealth` | STATE(reliable · transient_local · 1) · 5 Hz 재발행 | `/ax/<robot_id>/localization_supervisor` |
| 재위치 요청 | `/ax/<robot_id>/localization/reinitialize` · `std_srvs/Trigger` | Service | D → 감독 노드 |
| 정지 상태 입력 | `/ax/<robot_id>/safety/actuation_state` · `ax_interfaces/ActuationState` | STATE | A4(현재는 시험 스텁) |

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

# 2. 위치 추정: 공통 지도 서버 + 로봇별 AMCL·위치 신뢰성 감독, 고정 출발 위치로 최초 초기화
ros2 launch ax_localization localization.launch.py \
    map:=log/b2_mapping/20261109_085358/run/slam_map.yaml robot_ids:=robot_1,robot_2 \
    start_poses:=src/ax_localization/config/start_poses/aws_warehouse_dev.yaml

# 3. 확인
ros2 lifecycle get /map_server
ros2 lifecycle get /ax/robot_2/amcl
ros2 run tf2_ros tf2_echo map robot_2/base_link
ros2 topic echo /ax/robot_2/localization/health --qos-durability transient_local --qos-reliability reliable
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
- 활성화 완료 후 AMCL이 끝나면 그 로봇의 `localization/health`가 `LOST`(`TF_MISSING`)가 된다. 다른 기능은 이것으로 안다. launch 자체는 알리지 않는다.

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
- **개발용 지도의 시험값:** `config/start_poses/aws_warehouse_dev.yaml`이 지도 `20261109_085358`(이미지 sha256 `4a8f0475…`)에 묶여 있다.
  - 값 = 그 지도의 검증 변환 `T_world_map`(`optimal`)의 역변환 · AWS 월드 launch 출발 위치(`robot_test.launch.py` `WORLD_SPAWN_POSITIONS`: robot_1 world (0, 0), robot_2 world (-1.0, -2.3)). robot_1 `map` (1.0045, 2.2850), robot_2 (0.0050, -0.0152).
  - robot_2 기본 출발 world (2.5, -1.5)는 원본 월드에서 선반 충돌과 겹쳐 2026-11-09에 바꿨다. 그 전 지도(`20261108_131510`) 기준 값은 쓰지 않으며, 그 지도와 이 설정을 함께 주면 이미지 해시가 달라 launch가 거절한다.
  - 다른 지도에서는 이 값이 맞지 않는다.
- **실행 중 변경:** 해당 로봇의 Topic에만 보낸다. 아래 값은 위 개발용 지도·AWS 출발 조건의 robot_1 시험값(`aws_warehouse_dev.yaml` x 1.0045, y 2.2850, yaw 0.0002 rad → quaternion z = sin(yaw/2), w = cos(yaw/2))이다. 다른 지도나 로봇이 다른 위치에 있을 때는 쓰지 않는다.

  ```bash
  ros2 topic pub --once -w 2 /ax/robot_1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: 1.0045, y: 2.2850}, orientation: {z: 0.0001, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
  ```

  `-w`는 기다릴 구독자 수다. 다른 구독자(기록 도구 등)가 있으면 그 수만큼 늘려야 AMCL이 받는다. 시험에서 `-w 1`은 기록 도구에만 전달되고 끝났다.
- **정지 상태의 한계:** 로봇이 멈춰 있으면 AMCL은 틀린 초기 위치를 고치지 않는다(`update_min_d` 0.25 m·`update_min_a` 0.2 rad). 시험에서 0.5 m 틀린 입력이 그대로 유지됐다.

### 위치 신뢰성 (`LocalizationHealth`)

`localization_supervisor.py`가 로봇마다 다음 입력을 보고 5 Hz로 상태를 낸다.
- 입력: AMCL 추정(`amcl_pose`)과 공분산, `map → odom` TF와 `map → base_link` 연결, 스캔, odom, sim time 진행
- 판정 로직: `scripts/localization_health.py`(ROS 없이 시험)
- 판단 값: [`config/localization_health.yaml`](config/localization_health.yaml). 모두 **개발용 출발값**이다.

| 상태 | `valid` | 뜻 | 사유 코드(`meta.validity_reason`, 계약 D-19) |
|---|---|---|---|
| `UNKNOWN` | false | 판정 불가 | `NOT_INITIALIZED`(초기 위치 없음), `CLOCK_INVALID`(sim time 역행·벽시계 2 s 정지) |
| `OK` | true | 입력·연결이 최신이고 표준편차가 기준(위치 0.25 m·방향 0.15 rad) 이하 | 없음 |
| `DEGRADED` | true | 신뢰도 저하 | `TF_STALE`·`SCAN_STALE`·`ODOM_STALE`·`ESTIMATE_STALE`·`COVARIANCE_HIGH` |
| `LOST` | true | 위치 상실 | `ESTIMATE_DISCONTINUOUS`(추정 도약 latch)·`TF_MISSING`·`ESTIMATE_INVALID`(NaN·음수 분산(반올림 경계 아래)·현재 자세 계산 불가)·`ODOM_INVALID`(odom 비유한 값)·`ODOM_DISCONTINUOUS`(odom 끊김 뒤 AMCL 새 추정 3개 전)·`INPUT_LOST`·`COVARIANCE_LOST`, 재위치 진행·실패 코드 |

- **정지 중 AMCL 특성 반영:** AMCL은 정지 중 추정을 새로 내지 않는다. 그래서 갱신 정체(`ESTIMATE_STALE`)는 시간이 아니라 마지막 추정 뒤 odom 이동량(0.5 m·0.4 rad)으로 판단한다. 기준 odom은 추정을 받을 때의 odom이다. 추정이 odom보다 먼저 왔으면 그 뒤 처음 받은 유효한(끊김 없는) odom을 기준으로 삼고, 다음 추정 전까지 바꾸지 않는다. 기준·현재 자세 중 하나라도 비유한 값이면 이동량을 알 수 없으므로 `ESTIMATE_STALE`로 본다.
- **공분산 반올림 경계:** 판단에 쓰는 대각 분산(xx·yy m², yaw rad²)이 −1e-12 이상 0 미만이면 반올림 오차로 보고 판정 내부에서만 0으로 쓴다(`ROUNDOFF_VARIANCE`). `initial_poses`(AMCL `set_initial_pose`)는 초기 공분산 0으로 필터를 시작해 yaw 분산이 −1.09e-13처럼 나온다(Jazzy pf 통계 계산의 반올림으로 본다. 입자 2000개 기준 설명용 오차 규모 추정은 약 1e-12 이하이며, 설치본 동일 커밋·모든 연산 오차를 증명한 엄밀 상한은 아니다). −1e-12 미만인 음수·비유한 값·길이 오류는 계속 `ESTIMATE_INVALID`다. 원시 메시지는 바꾸지 않고, 진단(`covariance_roundoff`)에 0으로 본 항목·원래 값·경계를 남긴다. 작거나 0인 공분산은 정확도·회복 근거가 아니며 도약 latch·초기화 확인·신뢰 기록·재위치 조건은 그대로 적용된다. 선형 분산의 상쇄 오차는 위치 값이 클수록 커질 수 있다. 원점에서 먼 위치라도 오차가 경계 안이면 0으로 보고, **오차가 −1e-12를 넘어선(−1e-12 미만) 표본만** 미지원으로 두어 무효(`ESTIMATE_INVALID`)로 거절한다(경계 확대·위치 비례 경계 없음).
- **`OK`의 한계:** `OK`는 정확도 검증이 아니다. 공분산은 상한 검사에만 쓰고 정확도를 확정하지 않는다. 정확도는 평가 도구로 따로 보며 판정은 `not_judged`다.
- **odom 이상:** odom 비유한 값과 연속성 상실(시각 역행·속도로 설명되지 않는 도약)은 판정에 바로 `LOST`로 들어간다. 끊김 뒤에는 AMCL 새 추정이 3개 쌓이기 전까지 `OK`가 되지 않고, 마지막 신뢰 위치도 지운다. **이번 범위에서 자동 복구 경로 없음.** 정지 상태에서는 `LOST`가 유지되고 재위치는 거절된다(`REJECTED_ODOM_DISCONTINUOUS`).
- **입력 stamp:** `amcl_pose`·`map → odom` TF는 앞 stamp보다 늦은 것만 받아들인다. 같은 stamp는 값이 같으면 중복, 다르면 충돌로 나눈다. 받아들이지 않은 입력은 AMCL 갱신 수, odom 끊김 뒤 회복 수, 재위치 회복 수에 넣지 않고, 오래된 TF가 최신 추정을 덮지 않는다. 충돌이면 처음 값을 유지하고, 그 입력의 다음 새 값이 올 때까지 마지막 신뢰 위치를 갱신하지 않는다. 판정은 감독 진단의 `input` 줄에 남는다.
- **추정 도약(`ESTIMATE_DISCONTINUOUS`):** 직전 비교 기준 `map → odom`과 새 추정을 **같은 odom 자세**(추정 시각 ±0.1 s의 가장 가까운 odom 표본)에서 비교한다. `map → odom` 평행이동 크기로 대신하지 않는다.
  - 기준: 0.3 m·0.3 rad(`estimate_jump`, 재위치 `consistency`와 별도 키). 단일 회차 정상 갱신 최대 0.027 m·0.032 rad와 lifecycle 재설정 도약 5.94 m·1.59 rad에서 정한 **개발 출발값**이며 운영 확정값이 아니다.
  - 비교 대상: TF와 `amcl_pose` 둘 다. `amcl_pose`가 TF보다 먼저 와도 기준과 그 시각 odom으로 비교한다.
  - 비교 기준: 마지막으로 검증한 `map → odom`이다. TF 공백 동안에도 유지하며 마지막 신뢰 위치가 없어도 감시한다. 최초 TF는 비교 기준일 뿐 초기화가 맞다는 근거가 아니다.
  - 도약이면 `LOST`·`ESTIMATE_DISCONTINUOUS`로 고정(latch)하고 마지막 신뢰 위치를 덮어쓰지 않는다. 공분산이 작아도, 시간이 지나도, 같은 추정이 다시 와도 풀리지 않는다. **해제는 허용된 재위치가 성공했을 때만**이며(아래 재위치), 그때 비교 기준을 검증된 새 추정으로 바꾼다.
  - 근거 부족: 추정 시각의 odom 표본이 없거나 0.1 s보다 멀면 도약 여부를 확정하지 않는다. 비교 기준을 유지하고 `DEGRADED`(`ODOM_STALE`)로 내며 마지막 신뢰 위치를 갱신하지 않는다. 판정마다 다시 비교한다.
  - 감독 노드가 보낸 초기 위치(`START_POSE`)·재위치 후보(`REINIT`)는 송신 뒤 stamp의 새 추정이 후보와 0.5 m·0.3 rad 안이면 한 번만 허용한다(sim 15 s·벽시계 60 s 제한, 시계 무효면 닫음). 그 밖의 도약은 이 기간에도 latch한다. 외부 `initialpose`는 허용 근거가 아니다.
  - 우선순위: 시계 무효(`UNKNOWN`·`valid=false`)는 덮지 않고 사유만 덧붙인다(예: `CLOCK_INVALID,ESTIMATE_DISCONTINUOUS`).
- **sim time 역행:** 한 번 역행하면 감독 노드를 다시 시작하기 전까지 `UNKNOWN`(`CLOCK_INVALID`)이 유지된다. 시뮬레이션을 다시 시작하거나 시간을 되돌렸다면 위치 추정 launch도 다시 시작한다. 벽시계 2 s 정지로 생긴 `CLOCK_INVALID`는 sim time이 다시 흐르면 풀린다.
- **판정 불가 유지:** 시계가 무효면 재위치 중이거나 실패 뒤여도 `UNKNOWN`·`valid=false`를 유지하고, 재위치 코드를 뒤에 덧붙인다(예: `CLOCK_INVALID,REINIT_IN_PROGRESS`).
- **이상 탐지 없음:** Isolation Forest가 없으므로 `anomaly_model_available=false`, `anomaly_score=0`이다.
- **메타 정보:** `meta.source_session_id`는 감독 노드가 시작할 때마다 새로 만들고, 상태·사유가 바뀔 때마다 `revision`을 올린다.

### 고정 출발 위치 (`start_poses`)

- 로봇별 `map` 기준 출발 pose·표준편차와 대상 지도 revision·이미지 sha256을 yaml로 준다. 예: [`config/start_poses/aws_warehouse_dev.yaml`](config/start_poses/aws_warehouse_dev.yaml), 개발 지도 전용.
- launch가 지도와 대조하고, 다음이면 시작을 거절한다(노드를 띄우지 않음).
  - 설정 누락: 로봇·키
  - 비정상 값: 비유한, 표준편차 범위 밖
  - 지도 불일치: revision 또는 이미지 해시
  - `initial_poses`와 같은 로봇에 겹침
- 지도 revision은 채택 지도면 `map_info.yaml`의 `map_revision`, 개발 지도면 0이다.
- **사용 범위:** 그 지점에서 시작함이 보장되는 개발 시뮬레이션의 **최초 초기화에만** 쓴다.
  - 감독 노드가 AMCL이 active가 되고 sim time이 유효해진 뒤 이 세션에서 한 번만 `initialpose`로 보낸다.
  - 송신 뒤 새 추정이 출발 위치와 맞아야 초기화를 확인한 것으로 보고, 그 뒤에만 마지막 신뢰 위치를 기록한다(아래 방식별 표).
  - 실행 중 위치를 잃어도 다시 보내지 않는다.
- **값의 출처:** 출발 위치는 launch 출발 좌표와 지도 원점으로 정한 값이다. Gazebo 실제 자세는 읽지 않으며 평가에만 쓴다.
- **`initial_poses`:** AMCL 매개변수로 넣는 수동 시험용이다. 감독 노드는 `initialpose`를 보내지 않는다. launch가 검사한 같은 pose를 감독 노드에 최초 초기화 확인 후보(`expected_initial_*`)로만 넘긴다.

**초기화 방식별 최초 확인** (확인 전에는 마지막 신뢰 위치를 기록하지 않는다. 도약 감시는 모든 방식에서 한다)

| 방식 | 후보 | 확인 조건 | 확인 못 하면 |
|---|---|---|---|
| `start_poses` | 감독 노드가 보낸 `START_POSE` | 기준·제한시간 시작 = 송신 시각. 그 뒤 stamp의 새 TF(이미 받은 TF와 같은 값이어도)가 같은 odom 자세에서 후보와 0.5 m·0.3 rad 안 | 신뢰 기록 없음 |
| `initial_poses` | launch가 AMCL 매개변수와 감독 노드 매개변수(`expected_initial_*`)로 같은 pose를 전달(`LAUNCH_INITIAL_POSE`). 감독 노드 송신 없음 | 기준 = 감독 노드 sim time이 유효해진 시각(그 뒤 stamp의 입력만 씀). 제한시간 시작 = AMCL active를 처음 확인한 시각(그 전에는 만료 안 함). 확인 = 기준 뒤 stamp의 **새 `amcl_pose`**가 후보와 맞고, 그 뒤 TF도 후보와 맞음. AMCL 첫 추정이 기준보다 먼저 와서 배제되고 로봇이 정지해 있어도 확인할 수 있도록, AMCL active 확인 뒤 확인·만료 전까지 0.5 s마다 `request_nomotion_update`를 부른다(`initialpose` 아님). 재발행 TF나 기준 전 추정(transient local로 다시 받은 이전 `amcl_pose`)만으로는 확인하지 않음 | 신뢰 기록 없음 |
| 후보 없음(외부 `initialpose`·AMCL 자체 초기화) | 없음 | 확인하지 않음. 첫 TF는 비교 기준일 뿐이다 | 신뢰 기록 없음 |

- 제한시간: 위 시작 시각부터 sim 15 s·벽시계 60 s(`estimate_jump.intent_timeout_*`). 시계가 무효해지면 닫는다. `initial_poses`에서 AMCL 활성화가 늦으면 active를 확인할 때까지 제한시간을 시작하지 않는다(그동안 신뢰 기록 없음). 활성화 뒤 15 s 안에 맞는 새 추정이 없으면 의도를 닫고 그 세션은 신뢰 기록 없이 남는다.
- 확인 못 한 세션의 복구 제한: 마지막 신뢰 위치가 없으므로 재위치는 `REJECTED_NO_TRUSTED_POSE`로 거절된다. **이전과 달라진 점:** 후보 없음 모드는 예전에는 `OK`·표준편차 조건만으로 신뢰 위치를 기록했다. 이제는 기록하지 않는다. 외부 `initialpose`로 확인 권한을 주는 경로는 만들지 않았다.
- 감독 노드 재시작 한계: 재시작한 감독 노드는 과거 추정을 초기화 근거로 쓰지 않는다. 다만 `initial_poses` 모드에서 재시작 뒤 제한시간 안에 AMCL이 launch pose 근처의 새 추정을 내면(예: 로봇이 출발 위치에 그대로 있음) 그것이 초기화 직후인지 구별하지 못한다. launch는 감독 노드를 자동 재시작하지 않는다.

### 재위치 (같은 세션의 마지막 신뢰 위치 기반)

- **마지막 신뢰 위치:**
  - 기록 조건: `OK`이고 표준편차가 위치 0.15 m·방향 0.10 rad 이하. 그리고 추정 쪽 조건을 모두 만족한다(진단 `trust_rejection` 코드).
    - 도약 latch 없음(`ESTIMATE_DISCONTINUOUS`), 같은 stamp 충돌 없음(`INPUT_CONFLICT`), 근거 부족 없음(`ESTIMATE_UNVERIFIED`)
    - 초기화 확인(`NOT_ESTABLISHED`가 아님): 위 초기화 방식별 확인 또는 재위치 성공
    - 최신 `amcl_pose`와 현재 TF가 같은 갱신(`ESTIMATE_UNPAIRED`가 아님): 같은 odom 자세에서 0.02 m·0.02 rad 안. stamp 일대일 대응을 가정하지 않으며, 한쪽만 새로 오면 다른 쪽이 올 때까지 보류한다.
  - 기록 내용: 그때의 `map → odom`·표준편차·시각·지도 식별값과 odom 자세. 기록 뒤에는 odom 표본마다 **누적** 이동 거리·회전량을 더한다(왕복·반복 회전도 모두 셈). 로봇마다 따로 메모리에만 두며, 프로세스를 다시 시작하면 사라진다.
  - 갱신하지 않을 때: 재위치 중이거나 입력이 무효일 때.
  - 무효로 할 때: 지도 변경, sim time 역행, odom 끊김(시각 역행·속도로 설명되지 않는 도약·비유한 값). 무효가 된 기록은 복구 근거로 쓰지 않는다.
- **요청 처리** (`ros2 service call /ax/<robot_id>/localization/reinitialize std_srvs/srv/Trigger`):
  1. 접수 조건을 확인한다.
     - A4 `ActuationState`가 최신(0.3 s)·유효·`STOPPED`
     - 마지막 신뢰 위치가 있고, 그 뒤 60 s 이내·odom **누적** 이동 1.0 m·0.8 rad 이하(순변위가 아님)
     - 이미 `OK`가 아님
  2. 후보 = 마지막 신뢰 `map → odom` · 현재 odom. 표준편차는 누적 이동량에 따라 키운다.
  3. 접수하면 `LOST`·`REINIT_IN_PROGRESS`를 먼저 발행하고, `success=true`·`REINIT_ACCEPTED`로 응답한 뒤 후보를 `initialpose`로 보낸다.
  4. 0.5 s마다 AMCL `request_nomotion_update`를 불러 정지 상태에서도 새 추정을 받는다.
  5. **완료**(`OK`) 조건은 모두 만족해야 한다. 입력 pose가 그대로 돌아온 것만으로는 완료가 아니다.
     - 전달 뒤 새 AMCL 추정 3개 이상
     - 마지막 추정의 표준편차가 0.25 m·0.15 rad 이하이고 후보 표준편차의 0.9배 이하
     - 후보와 0.5 m·0.3 rad 이내
     - 전달 뒤의 `map → odom` TF가 최신, 스캔이 최신
     - 현재 `map → odom` · odom 자세도 후보와 0.5 m·0.3 rad 이내
     - 성공하면 같은 판정 주기에 추정 도약 latch를 풀고 비교 기준을 현재 TF로 바꾼다. 재위치 진행 판정은 latch를 반영하기 전 입력으로 하므로 latch가 성공 판정을 막지 않는다.
  6. **실패**(`LOST`·`REINIT_FAILED_*`)는 다음 요청이나 AMCL 회복 근거가 생길 때까지 유지한다.
     - `NOT_STOPPED`: 진행 중 정지 상태 상실
     - `CLOCK_INVALID`: 진행 중 sim time 역행·정지(상태는 `UNKNOWN`·`valid=false`)
     - `AMCL_UNAVAILABLE`: 15 s(sim time) 또는 60 s(벽시계) 동안 새 추정 없음
     - `TIMEOUT`: 추정은 왔으나 조건 미충족
     - `NOT_RECOVERED`: 후보에서 1.0 m·0.6 rad 넘게 벗어남
- **latch와 실패:** 재위치가 실패하거나 거절되면(후보 없음·만료·이동 초과) 추정 도약 latch는 그대로 `LOST`다. AMCL 회복 근거로 `REINIT_FAILED_*`가 지워져도 latch는 남는다. odom 끊김·지도 변경으로 지운 신뢰 위치는 latch 해제 뒤에도 되살리지 않는다.
- **응답 코드:** 진행 중 재요청은 `success=true`·`REINIT_IN_PROGRESS`이며 새 재위치를 만들지 않는다. 거절은 `success=false`와 `REINIT_REJECTED_*`이다(`NOT_STOPPED`·`ACTUATION_STALE`·`NO_TRUSTED_POSE`·`TRUSTED_POSE_EXPIRED`·`MOTION_TOO_LARGE`·`ODOM_DISCONTINUOUS`·`MAP_CHANGED`·`CLOCK_INVALID`·`NOT_NEEDED`).
- **책임 범위:** 감독 노드는 로봇을 움직이지 않는다. 재위치 중 작업·이동 보류는 D, 실제 정지·구동은 A의 책임이다.
- **절차 완료와 정확도:** 절차 완료(`OK`)와 정확도 판정은 다르다. 정확도는 평가 도구로 기록하며 `not_judged`다.

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

#### 원시 기록·감독 진단·분석 (선택, 시험 기록 전용)

```bash
# 위치 추정 launch보다 먼저 띄워 최초 초기화부터 받는다. stop 파일이 생기면 정상 종료한다.
python3 src/ax_localization/scripts/check_localization.py --robot-ids robot_2 --map-accuracy <accuracy.json> \
    --duration 3000 --raw-dir log/b2_health/<KST 시각>_<label> --stop-file log/b2_health/<KST 시각>_<label>/stop \
    --map <지도.yaml> --start-poses <start_poses.yaml> [--world-file <월드>] [--hash-file NAME=PATH ...]
ros2 launch ax_localization localization.launch.py ... health_diagnostics:=log/b2_health/<KST 시각>_<label>/supervisor
# 끝난 뒤 ROS 없이 분석
python3 src/ax_localization/scripts/check_localization.py --analyze log/b2_health/<KST 시각>_<label> --map-accuracy <accuracy.json>
```

- **보관 위치:** `log/b2_health/`(Git 제외). 일회성 임시 폴더에 두지 않는다.
- **`conditions.json`:** 수집을 시작할 때 쓴다. 담는 값은 다음과 같다.
  - git commit·미커밋 파일
  - 도구·판단 값·AMCL 설정·launch·start_poses·지도·월드 sha256
  - `ROS_DOMAIN_ID`·`GZ_PARTITION`, 수집 옵션
  - 같은 순간의 UTC·단조 시각
  - 실행 중 알게 된 URDF 해시·바퀴 접촉·감독 세션은 `runtime`에 갱신한다.
- **`raw/<robot_id>/*.jsonl`:** 받는 즉시 줄 단위로 쓰고 flush한다. 중단돼도 받은 만큼 남는다.
  - 대상 자료: `tf_map_odom`, `amcl_pose`, `initialpose`, `odom`, `scan`, `health`, `actuation`, `amcl_lifecycle`, `tf_chain`, `publishers`
    - `tf_map_odom`: 원래 stamp와 유효 시각을 함께 남긴다.
    - `amcl_pose`·`initialpose`: 공분산 36개를 모두 남긴다.
    - `tf_chain`: TF 연결 조회 결과(1 Hz)
    - `publishers`: `amcl_pose` 발행자·GID
  - 줄마다 원래 stamp와 수신 `recv_sim`·`recv_mono`를 구분해 둔다.
  - `truth.jsonl`은 평가 전용이다.
  - 자료 파일이 아예 없으면 결과의 `raw.missing`에 남고 `status`가 `incomplete`가 된다.
- **`status`와 기록 품질은 다르다:**
  - `completed`는 기존 의미 그대로다. 요청 시간과 필수 입력·연결을 충족했다는 뜻이며, 모든 메시지를 받았다는 뜻이 아니다.
  - 수집량은 `record_quality`(결과 파일과 `--analyze`)에 따로 남는다.
    - 수집 구간 `collection_window_mono`: 시작은 기록 시작 단조 시각, 끝은 기록 종료 또는 마지막 수신 시각
    - 자료별 값: 출처, 받은 줄 수·고유 stamp·중복, 수신 최대 공백, `subscriber_ready`
      - `subscriber_ready`는 구독 준비를 증명할 자료가 없어 `unknown`이다.
  - 대조 기준과 범위:
    - health: 감독 진단 tick과 stamp 집합으로 대조한다. 중복이 누락을 상쇄하지 않는다.
      - `all_ticks`: 진단 전체(기간 제한 없음)
      - `collection_window`: 수집 구간 안
      - `observed_interval`: 원시 기록이 처음·마지막으로 받은 사이
      - 구간 밖 기준 사건은 `reference_out_of_window`로 따로 센다.
    - initialpose: 감독이 보낸 개수와만 비교한다(`coverage: count_only`, `unmatched_count`).
      - 개수가 같으면 `count_equal_unproven`이다. 완전 수신을 뜻하지 않는다.
  - 손실 판정의 뜻:
    - `loss: confirmed`: 기준 사건 중 원시 기록에 없는 것이 있다는 뜻이다. 전송·DDS 원인을 확정한 것은 아니다.
      - 구독 준비 전에 보낸 메시지였을 수도 있다. 그런 미수신과 전송 손실은 구별하지 못한다.
    - `none_in_reference`: 비교한 범위의 기준 사건이 모두 있다는 뜻이다.
    - `unverified`: 기준이 없다. 받은 줄 수만으로 전량 수집을 주장하지 않는다.
- **감독 진단:** `supervisor/<robot_id>_supervisor.jsonl`에 남는다.
  - 판정 tick마다: 상태·표준편차, 신뢰 기록 판단(`trust_stored`·`trust_rejection`), 기록 나이·누적 이동, 추정 감시 요약(`estimate`: latch·초기화 확인·근거 부족·의도·비교 기준·갱신 대응)과 입력 판정 개수(`gates`)
  - 입력 판정(`input`): `amcl_pose`·TF의 stamp·수용 여부(`ACCEPTED`·`DUPLICATE`·`CONFLICT`·`REVERSED`·`INVALID_STAMP`)와 도약 판단. 값이 같은 TF 재발행은 남기지 않는다.
  - 추정 사건(`estimate_events`): `BASELINE`·`INTENT`·`INTENDED`·`UNVERIFIED`·`LATCHED`·`CONFLICT`·`RELEASED`·`INTENT_CLOSED`
  - 재위치 요청 응답, 감독 노드가 보낸 initialpose(`START_POSE`·`REINIT`)
  - 판정 동작은 바꾸지 않으며 공유 토픽으로 내지 않는다.
- **`--analyze`:** 회차 폴더에 `analysis.json`을 쓴다. 정확도는 `not_judged`다.
  - map→odom 갱신량: 재발행(값 같음)과 실제 갱신을 나눈다.
    - stamp 정리: 같은 stamp에 같은 값은 중복, 다른 값은 `conflicting_stamp`로 따로 센다. 역행 stamp도 센다.
    - 같은 odom에서 본 추정 변화: 쓴 odom의 나이를 남기고, 상한(기본 0.1 s)을 넘으면 대응 불가로 센다.
  - TF·`amcl_pose` 대응: 대응 수·무대응 수만 기록한다. 무대응 원인은 미확정으로 둔다.
  - health 상태·사유별 시간, 신뢰 기록 판단 빈도
  - 실제 오차와 그 시각 공분산

### 개발용 지도 제한

- **현재 시험 지도:** `log/b2_mapping/20261109_085358/run/slam_map.yaml`(Git 제외, 미채택).
  - 원본 월드·S005 반영 개발 모델(바퀴 `slip1 0.05`)·robot_2 선반 포함 외곽 경로로 만들었다. 정확도는 `not_judged`이며 운영 지도로 채택하지 않았다([지도 안내](maps/aws_warehouse/README.md)).
  - launch는 `map_info.yaml` 채택 기록이 없는 지도를 "개발용 지도"로 표시한다.
- **이전 시험 지도:** `log/b2_mapping/20261108_131510`은 단순화 월드(`light_collision`)·바퀴 임시 사본에서 만든 개발용 결과다([S005 자료](../../docs/evidence/wheel_slip/)). 아래 2026-11-08 결과는 그 지도·단순화 월드·이전 출발 위치 조건이다.
- **보장 범위:** 지도는 앞 LiDAR 높이의 관측만 담는다. 미관측 영역과 다른 높이의 장애물은 위치 추정 근거가 되지 않는다.

### 개발 시험 결과 — 원본 월드·S005 모델 (2026-11-09, `not_judged`)

시험 조건은 다음과 같다.
- 격리 headless Gazebo, 원본 AWS 월드, S005 반영 개발 모델, B1 `scan_filtered`
- 현재 시험 지도(이미지 sha256 `4a8f0475…`), `T_world_map`은 그 지도의 `optimal_T_world_map`, `start_poses`로 최초 초기화
- A4는 odom 기반 시험 스텁, D 요청은 시험자 직접 호출(실제 통합 아님)

| 시험 | 결과 |
|---|---|
| robot_2 출발·정지 | 최초 1회 초기화 → `OK`, 위치 오차 0.019 m |
| robot_2 위치 상실·재위치 | AMCL 전체 재시작 → `DEGRADED`(`COVARIANCE_HIGH`). 후진 0.26 m 중 요청 거절(`REJECTED_NOT_STOPPED`), 정지 뒤 접수, 중복 요청 `REINIT_IN_PROGRESS`, 약 1 s 뒤 `OK`, 오차 0.044 m |
| robot_2 이동(동 2 m → 남 5.8 m) | 위치 오차 평균 0.097·최대 0.143 m, 방향 오차 최대 0.040 rad |
| 두 로봇 동시 | robot_2가 같은 경로를 주행하는 동안 robot_1 AMCL 재시작(지도 원점으로 재초기화되어 최대 2.46 m 틀림, `DEGRADED`) → 후진 0.20 m 중 거절 → 정지 뒤 재위치 → `OK`, 오차 0.042 m. robot_2는 재위치 코드 없이 주행 완료(오차 평균 0.095·최대 0.126 m). 두 감독 노드 세션·revision·스캔·`map→odom`이 로봇별로 분리 |

이 시험에서 확인한 판단 값의 한계(개발용 출발값, 결과에 맞춰 바꾸지 않았다):
- 이동 중 AMCL 표준편차가 0.16~0.23 m·방향 0.15 rad 이상으로 커져 정상 주행 대부분이 `DEGRADED`(`COVARIANCE_HIGH`)였다. 같은 구간의 실제 오차는 0.1 m 수준이었다.
- 그래서 이동 중에는 마지막 신뢰 위치(표준편차 0.15 m·0.10 rad 이하)가 갱신되지 않았다. 주행 뒤(마지막 신뢰 기록 60 s 초과) 재위치 요청은 `REINIT_REJECTED_TRUSTED_POSE_EXPIRED`로 거절됐다.
- AMCL을 다시 시작하면 마지막 위치가 아니라 지도 원점 근처로 다시 초기화되며, 공분산(약 0.5 m)이 `LOST` 기준(1.0 m)보다 작아 틀린 위치가 `LOST`가 아닌 `DEGRADED`로 나온다. 2026-11-08 단순화 월드 시험에서 재시작 뒤 회복된 것처럼 보인 것은 로봇이 지도 원점 근처에 있었기 때문이다.

#### 위치 신뢰성 기준선 기록 (2026-11-09, 단일 회차, `not_judged`)

**회차**
- 기록: `log/b2_health/20261109_195316_robot2_normal_reset/`(Git 제외)
  - `conditions.json`·`raw/`·`supervisor/`·`analysis.json`·`segments.json`
  - 하네스 사본은 `session/harness.sh`
- 조건: 위 원본 월드·S005·현재 시험 지도, robot_2 1대
- 방법: A4 스텁·수동 하네스로 다음 순서를 기록했다.
  1. 정지
  2. 기존 경로 앞 2구간(동 2 m → 회전 → 남 5.8 m)
  3. 정지
  4. AMCL lifecycle 재설정(deactivate → cleanup → configure → activate)
- 판단 값은 바꾸지 않았다. 모든 수치는 개발용 출발값 제안의 근거일 뿐이다.

**구간별 결과** (health는 감독 진단 tick 기준)

| 구간 | 결과 |
|---|---|
| 정상 주행 34.6 s | health `DEGRADED`(`COVARIANCE_HIGH`) 27.2 s(79 %). `amcl_pose` 35건 중 yaw 표준편차 > 0.15 rad 29건, 위치 > 0.25 m 8건(8건 모두 yaw도 초과). 실제 오차 위치 평균 0.076·최대 0.124 m, yaw 최대 0.035 rad. 같은 시각으로 묶은 표본 62개에서 오차/표준편차 중앙값은 위치 0.38·yaw 0.07 |
| 정상 갱신당 추정 변화(같은 odom 기준, 35건) | 위치 p95 0.026·최대 0.027 m, yaw p95 0.026·최대 0.032 rad. 쓴 odom 나이 최대 0.010 s(상한 0.1 s), 대응 불가 0건 |
| 주행 뒤 정지 | 정지 중에는 AMCL이 갱신하지 않아 `DEGRADED`가 유지됐다. 신뢰 기록은 주행 시작 약 3 s 뒤부터 갱신되지 않았다 |
| lifecycle 재설정 | map→odom 공백: 수신 sim 64.47 → 69.95 s(5.48 s), 원래 stamp 65.4 → 69.7 s(4.3 s, 유효 시각 64.4 → 68.7 s). 공백 전 값은 stamp 50.1 s 뒤 바뀌지 않음. `TF_STALE` → `LOST`(`TF_MISSING`) 2.2 s 뒤 새 추정(원점 근처, 표준편차 0.52 m·0.27 rad) → `DEGRADED`. 추정 도약 5.94 m·1.59 rad, 실제 오차 6.00 m가 계속 `DEGRADED`로 남음. 재위치 요청은 `REINIT_REJECTED_TRUSTED_POSE_EXPIRED`(기록 나이 68.8 s, 누적 7.29 m) |
| TF·`amcl_pose` 대응 | `amcl_pose` 38건 모두 TF 유효 시각과 0 s 차이로 대응. 짝 없는 TF 778건은 모두 직전 값과 같았다(원인 미확정). 같은 stamp 중복 TF 57건(값 같음, 원인 미확정) |

**기록 한계** (`review_quality.json` 보완. 원본 raw·result는 고치지 않았다)
- **`status`:** 결과 `status`는 `completed`이고 `raw.missing`은 없다. 그래도 전량 수집은 아니다.
- **health:** `raw/<robot>/health.jsonl`은 611건이다. 감독 진단 tick과 대조하면 빠진 수가 범위마다 다르다(`loss: confirmed`).
  - 진단 전체(고유 1029개): 418건
  - 수집 구간 안(1008개, 구간 밖 tick 21건 제외): 397건
  - 원인: 구독 깊이 1. 같은 시험을 스텁으로 다시 해 보면 깊이 1에서는 버스트 60건 중 59건을 잃었다.
  - 기록 도구 구독 깊이를 100으로 고쳤다. 스텁으로 버스트 60건과 odom 부하 아래 지속 5 Hz 15건을 모두 받는 것을 확인했다.
  - Gazebo로 다시 확인하지는 않았다.
- **initialpose:** 감독이 보낸 1건은 수집 시작 6 s 뒤라 수집 구간 안이다. 기록 도구가 받은 것은 0건이다.
  - 개수 대조로 `unmatched_count 1`이다.
  - 구독 준비 여부가 `unknown`이라, 미수신인지 전송 손실인지는 구별하지 못한다.
- **같은 stamp TF:** 57건은 모두 같은 값이었다. 충돌은 0건이다.

#### 추정 도약 감지·공분산 반올림 확인 (2026-11-09~10, 단일 회차, `not_judged`)

위 기준선은 도약 감지 **전** 소스 기록이다. 그 재설정 구간의 `DEGRADED`(실제 오차 6.00 m)는 현재 소스의 동작이 아니다. 아래는 현재 소스 기준이며, 판단 값·임계값은 바꾸지 않았다.

| 회차(`log/b2_health/`) | 소스 | 결과 |
|---|---|---|
| `20261109_231033_robot2_jump_latch` (`start_poses`) | 도약 감지 확정 소스(공분산 반올림 보완 전) | C: 0.58 m 직진 뒤 AMCL lifecycle 재설정 → 0.53 m 도약 → `LOST`(`ESTIMATE_DISCONTINUOUS`), 고정 중 신뢰 기록 덮어쓰기 0 → 재위치 접수(기록 나이 10.8 s·누적 0.355 m) → 해제·`OK`. A: 정상 2구간 주행에서 도약 오검출 0(같은 odom 기준 갱신 최대 0.034 m·0.035 rad). B: 경로 끝 재설정 → 5.68 m·1.67 rad 도약 → `LOST` 유지, 재위치 `REJECTED_TRUSTED_POSE_EXPIRED`(61.6 s·6.68 m) |
| `20261109_231523_robot2_initial_poses_smoke` | 같음 | 실패. 초기화 확인·`initialpose` 송신 0은 동작했으나 yaw 분산 −1.09e-13 때문에 `ESTIMATE_INVALID`가 유지돼 신뢰 기록 없음 |
| `20261110_085514_robot2_initial_poses_roundoff` | 공분산 반올림 보완 뒤(최종 소스) | 같은 조건 정지 30 s: launch 후보 확인 → `OK` 179 tick·신뢰 기록 179, `initialpose` 송신 0. 진단에 y·yaw 정규화(원래 값)·경계 1e-12, 원시 음수 보존 |

- **재사용 근거:** 반올림 보완은 대각 분산 [−1e-12, 0) 구간만 바꾼다. `jump_latch` 회차 `amcl_pose` 41건에는 1e-6 미만 대각이 없어 판정 경로가 같으므로 C/A/B를 다시 실행하지 않았다. 도약·신뢰 기록·재위치 경로는 단위·ROS 스텁 회귀로 다시 확인했다.
- **한계:** 단일 회차, `robot_2` 하나, 정지·짧은 경로만. `roundoff` 회차에서는 AMCL 첫 추정이 기준 시각보다 늦게 와서 무이동 갱신 요청 경로가 실행되지 않았다(이전 실패 회차·ROS 스텁으로만 확인).
- **기록 누락:** health 기록이 진단 tick보다 적다(`jump_latch` 수집 구간 1261 중 77, `roundoff` 169 중 28, 원인 미확정). 감독 노드가 보낸 `initialpose`는 `jump_latch`에서 2건 중 1건만 원시 기록에 있다(원인 미확정).
- **정확도:** `roundoff` 회차 평가 출력의 위치 오차 0.000 m는 원인(출발 위치 계산·반올림·해상도)을 확인하지 않았으므로 성공 근거로 쓰지 않는다.

### 개발 시험 결과 — 단순화 월드·이전 지도 (2026-11-08, `not_judged`)

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
| 고정 출발 위치·재위치 robot_2 | `start_poses`로 최초 1회 초기화 → `OK`, 위치 오차 0.031 m. AMCL 전체 재시작 → `LOST`(`TF_MISSING`). 후진 0.36 m 중 요청 거절(`REJECTED_NOT_STOPPED`). 정지 뒤 접수, 중복 요청은 `REINIT_IN_PROGRESS`, 0.8 s 뒤 `OK`. 재위치 뒤 위치 오차 0.094 m. 출발 위치 재전송 없음 |
| 재위치 robot_1·robot_2(검토 수정 전 실행) | robot_1 AMCL 재시작·후진 0.28 m·재위치(접수 뒤 1.8 s에 `OK`, 오차 0.096 m) 동안 robot_2는 `OK` 유지(오차 0.031 m 그대로) |
| 진행 중 정지 상실 | 정지 상태에서 접수한 뒤 로봇이 움직이자 `REINIT_FAILED_NOT_STOPPED` |

재위치 시험은 다음 조건에서 했다.
- A4: odom 속도로 `STOPPED`를 정하는 시험 스텁(`tools/tests/stub_actuation_state.py`)
- D: 시험자가 서비스를 직접 호출
- 위치 상실: AMCL lifecycle을 deactivate → cleanup → configure → activate로 다시 시작해 만들었다. 재시작 뒤 AMCL은 지도 원점 근처에서 큰 공분산(약 0.46 m)으로 다시 시작해 `LOST` 대신 `DEGRADED`(`COVARIANCE_HIGH`)가 됐다(두 로봇 시험). 당시 로봇이 원점 근처라 위치가 크게 틀리지 않았다(2026-11-09 시험 참조).

**알려진 제한**
- **미완료:**
  - 운영 모델·기준 월드 확정 뒤 재검증(현재 원본 월드·S005 개발 모델까지 확인), 운영 지도 채택
  - 판단 값 재검토: 이동 중 대부분 `DEGRADED`, 이동 뒤 신뢰 기록 만료로 재위치 거절, AMCL 재시작의 틀린 위치가 `DEGRADED`로 나옴(위 2026-11-09 결과)
  - 실제 A4·D와의 통합
  - 위치 이상 탐지
  - 판단 값·회복 조건 확정(모두 개발용 출발값)
  - 프로세스 재시작 뒤 신뢰 위치 재사용(이번 범위 제외)
  - 정지 상태에서 틀린 위치를 스스로 고치는 일. AMCL은 고치지 않으며, 재위치는 마지막 신뢰 위치에 기대므로 그 기록이 틀리면 틀린 채로 복구한다.
- **초기 위치 stamp:** 감독 노드가 보내는 `initialpose`에 대해 AMCL이 "extrapolation into the future" 경고를 내고 이동 보정 없이 적용한다. AMCL의 TF 버퍼가 자기 시계보다 수십 ms 늦기 때문이다. 정지 상태에서만 보내므로 자세 차이는 없지만 경고는 남는다.
- **Ctrl+C 시점의 보고 상태:** 모든 노드가 active가 되기 전에 사용자가 Ctrl+C로 끝내면 노드는 정리되지만, `startup_report`의 `status`는 마지막 값인 `starting`으로 남는다. 이 경우는 따로 시험하지 않았다.
- **`run_mapping_session.sh --light` 기존 결함:** 임시 사본에 `lib/ax_description`이 없어 robot_1을 띄울 수 없다. robot_2 지도 작성에는 영향이 없다. 위 위치 추정 시험은 이 문제를 보완한 별도 하네스(저장소 밖)로 실행했다.

**확인하지 못한 것**
- 두 로봇이 함께 긴 경로를 이동하는 경우(2026-11-09 시험은 robot_2 경로 주행 + robot_1 짧은 후진)
- 전체 지도 경로를 따라 AMCL로 이동(지도 작성은 전체 경로, 위치 추정 이동은 앞 2구간)
- 운영 모델·기준 월드 확정 뒤 조건
- 긴 시간 실행
- 전역 위치 추정(초기 위치 없이 시작)
- 시뮬레이션이 매우 느릴 때 AMCL TF 시각 처리
