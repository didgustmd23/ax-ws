# B3 통로·작업대 영역과 통로 관측

B3 김성민 담당의 통로·접근·작업대·대기 영역 장부와 통로 관측 발행 노드다. 다른 담당자의 지도·제어·허가 코드는 바꾸지 않는다.
통로 허가·해제·정지 명령은 하지 않는다(D3·A4 소유). 이 패키지의 관측은 그 판단의 입력일 뿐이다.

## 파일과 의미

- `config/ax_warehouse_regions.json`: 현재 기준 월드 `planned/ax_warehouse.world`(단색 보존 월드 `ax_warehouse_plain.world`와 구조 동일)의
  개발용 영역이다. `frame_id` map, `map_revision` 1, **`approved=false`**(팀 확인 전).
  - `tools/generate_b3_regions.py`가 world SDF 형상에서 계산해 만든다(손으로 고치지 않는다. `--check`로 일치 확인).
  - 통로 `corridor_1~3`: SDF `corridor_<n>_wall_north/south` 안쪽 면(y)과 문 벽 `wall_*_door_section` 바깥 면(x ±4.2) 사이.
  - 접근 구역(`kind: approach`): 각 통로 양 입구 바깥, 통로 폭, 깊이 1.5 m(개발값, AX_1 이동 외곽 길이 수준).
    `corridor_id`(소속 통로)와 `entry_heading_rad`(진입 방향: 서쪽 입구 0, 동쪽 입구 π)를 가진다.
  - 작업대 `station_src`·`station_dst`: SDF `source_pickup_zone`·`destination_unload_zone` 바닥 표시 상자.
  - 대기 지점 `wait_src`·`wait_dst`: B3 개발 제안 빈 바닥(접근 구역에서 0.8 m 이상). 안전 주차 승인이 아니다.
  - world→map: 채택 지도 revision 1 README의 `T_world_map` 초기값 (−6.8, 0, 0)(map 원점 = robot_2 출발 자세).
    `bounds`는 map, `world_bounds`는 world 좌표다. 참조 지도 최적 정합과의 약 0.02 m 차는 반영하지 않았다.
- `config/ax_test_regions.json`: 빈 시험 월드의 논리 영역(실제 작업대 없음).
- `config/aws_warehouse_regions.json`: 이전 AWS 월드용. 월드·채택 지도가 저장소에 없어 좌표가 비어 있고 미승인이다.
  판정은 항상 UNKNOWN이다. 시험(`test_unadopted_aws_cannot_be_used_for_release`)만 이 성질을 확인하며 파일은 보존한다.
  더 쓰지 않는다면 팀 확인 뒤 이력용으로 옮기거나 지우는 것을 제안한다.
- `ax_environment/regions.py`: 파일 형식·경계 검사, 축 정렬 판정(`observe_corridor`, 이전 방식), 회전 외곽 판정(`observe_robot`).
- `ax_environment/outline.py`: 로봇 모델별 외곽(아래 "외곽").
- `ax_environment/corridor_observer.py`: 통로 관측 발행 노드.

## 사용

```bash
ros2 run ax_environment check_regions                       # 기본: ax_warehouse_regions.json
ros2 run ax_environment check_regions --config src/ax_environment/config/ax_test_regions.json
python3 tools/generate_b3_regions.py --check                 # 영역 파일이 SDF 계산과 같은지

ros2 launch ax_environment corridor_observer.launch.py robot_id:=robot_2 robot_model:=AX_2
# 또는 ros2 run ax_environment corridor_observer --ros-args -r __ns:=/ax/robot_2 \
#   -p robot_id:=robot_2 -p robot_model:=AX_2 -p use_sim_time:=true
```

`robot_model`(AX_1 조작·AX_2 운송)은 robot_id에서 추론하지 않으므로 반드시 지정한다. 비우면 OUTLINE_UNDEFINED로 UNKNOWN만 낸다.

## 통로 관측 노드(corridor_observer)

| 구분 | 이름 | 내용 |
|---|---|---|
| 입력 | TF `map→<robot_id>/base_link` | B2 map→odom + A2 odom→base_link. 표본 나이 `max_pose_age_s` 1.0 s(sim time), 미래 stamp는 `future_tolerance_s` 1.0 s(AMCL transform_tolerance)까지 허용 |
| 입력 | `localization/health` | `LocalizationHealth`, STATE QoS. OK 또는 DEGRADED·valid·revision>0이고 나이 1.0 s 안(sim time·벽시계 무수신 둘 다)일 때만 위치 판정(DEGRADED는 아래 완화 규칙) |
| 출력 | `perception/corridor_observation` | `CorridorObservationList`, STATE QoS(reliable·transient_local·depth 1, B2 health와 같음), 5 Hz 재발행, 내용·유효성·사유가 바뀔 때 revision+1, 모든 통로 포함 |

- 판정: 외곽이 통로와 겹치면(닿아도) INSIDE. 아니면 그 통로의 접근 구역과 겹치고 차체 +X가 진입 방향에서 60°(개발값) 안이면
  APPROACHING. 아니면 OUTSIDE. 판정 불가는 모든 통로 UNKNOWN·`position_valid=false`·`meta.valid=false`와 사유
  (`REGIONS_NOT_APPROVED`, `REGIONS_INVALID`, `CLOCK_INVALID`, `OUTLINE_UNDEFINED`, `HEALTH_UNAVAILABLE`, `HEALTH_INVALID`,
  `HEALTH_STD_INVALID`, `HEALTH_LOST`, `HEALTH_UNKNOWN`, `HEALTH_STALE`, `POSE_UNAVAILABLE`, `POSE_STALE`, `FRAME_OR_MAP_MISMATCH`).
- health DEGRADED(valid·나이 OK)는 완화 판정한다: 불확실성 = `max(min_uncertainty_m, k·position_std_m) + degraded_margin_m`
  (기본 0.5 m, B2 `estimate_travel_m`과 같은 개발값), yaw는 `k·yaw_std_rad`. 이렇게 넓힌 외곽이 그 통로와 그 통로의 접근 구역
  어디와도 겹치지 않으면 OUTSIDE(`position_valid=true`), 겹치면 UNKNOWN(`position_valid=false`)이다. INSIDE·APPROACHING은 확정하지 않는다.
  목록 `meta.valid=true`, `validity_reason`은 비운다(계약상 valid=false용). DEGRADED 시작·종료는 노드 로그에 남는다.
  std가 비유한·0 이하·상한(`max_position_std_m` 1.0 m, `max_yaw_std_rad` 0.5 rad = B2 LOST 기준) 초과면 LOST와 같게 모든 통로 UNKNOWN
  (`HEALTH_STD_INVALID`). 결과: 위치가 다소 불확실해도 통로에서 충분히 먼 로봇은 막히지 않고, 통로 근처는 보수적으로 막힌다.
- 영역 파일이 미승인이면 항상 `REGIONS_NOT_APPROVED`다. 소비자가 해제 근거로 쓰지 못하게 하기 위함이다.
  지금 `ax_warehouse_regions.json`은 미승인이므로 실제 실행에서는 UNKNOWN만 나온다. 판정 시험은 승인 표시만 바꾼 임시 사본으로 한다.
- 외곽은 yaw로 회전한 다각형으로 판정한다(축 정렬 포위 사각형으로 넓히지 않음). 위치 불확실성만큼 보수적으로 넓힌다:
  `max(min_uncertainty_m, position_sigma_k × position_std_m) + position_sigma_k × yaw_std_rad × 외곽 최대 반지름`(k 기본 2.0, 개발값).
  방향 불확실성은 진입 방향 허용 각에도 더한다.

### 외곽(`outline.py`)

| 모델 | base_link 기준 [xmin, ymin, xmax, ymax] m | 근거 |
|---|---|---|
| AX_1 | [−0.78, −0.42, 0.82, 0.613] | `ax1_spec.json` visual_bounds_base_m(관절 0 자세, epick) ∪ `a1_validation.yaml` robot_1 footprint(이동 자세, 센서 포함 collision + 약 20 mm) |
| AX_2 | [−0.63, −0.53, 0.67, 0.53] | `ax2_spec.json` visual_bounds_base_m ∪ robot_2 footprint(적재함·센서 + 약 30 mm) ∪ 적재물 허용 범위 ±0.34 m |

- 가정: 이동 중 AX_1 팔은 이동 자세 또는 0 자세, AX_2 적재물은 갑판 허용 범위 안. 팔을 뻗은 작업 자세나 갑판 밖으로 나온 적재물은 포함하지 않는다.
  필요하면 `outline_margin_m`으로 넓힌다.

## 한계

- 팀 B4의 실제 `b4_risk_node.py`를 수정 없이 사용해 B3 Topic의 정상 수신·미승인 UNKNOWN·TF 만료·회복을
  `tools/tests/test_b3_b4_handoff_ros.py`에서 확인한다. 위치·거리 입력과 승인 영역은 시험 전용이며
  실제 B2 AMCL·Gazebo 주행 검증은 아니다. 저장소 영역의 `approved=false`는 유지한다.
- 위 B4 연결 시험은 robot_1/AX_1과 robot_2/AX_2 양쪽에서 같은 조건을 확인한다.
  함께 실행하려면 A3 패키지의 `ros2 launch ax_task_executor team_handoff.launch.py`를 사용할 수 있다.
  이 명령은 A3·B3만 실행하며 다른 담당자의 센서·위치 추정·안전 노드는 별도로 실행한다.
- B4가 빈 목록을 정상으로 오인하지 않도록 통로가 하나도 없는 B3 설정은
  `meta.valid=false`, `REGIONS_INVALID`로 발행한다. 비유한 시각도 `CLOCK_INVALID`로 처리한다.
- B2 세션 ID 누락·revision 0·무효인 새 세션은 기존 정상 세션을 폐기하지 않는다.
  받은 무효 입력은 관측에 반영해 UNKNOWN을 내되, 정상 세션의 다음 메시지로 복구할 수 있다.
  정상 새 세션으로 교체된 뒤의 이전 세션 메시지는 계속 무시한다.
- 새 세션으로 교체하기 전에 ROS 시각과 측정 stamp의 최신성도 확인한다.
  stamp 0·너무 오래된 값·미래 허용 범위를 넘는 값·유효하지 않은 현재 시각이면
  기존 세션을 폐기하지 않는다. 관측은 무효 처리하고 정상 세션의 새 메시지로 복구한다.

```bash
python3 tools/tests/test_b3_b4_handoff_ros.py
python3 tools/tests/test_b3_review_regressions.py
```

- 영역·접근 깊이·진입 각도·불확실성 배수·대기 지점은 개발값이며 운영 승인이 아니다.
- 접근 구역에서 통로를 등지고 후진 진입하면 APPROACHING이 아니라 OUTSIDE다(겹치기 시작하면 INSIDE).
- 통로 판정은 외곽과 통로 사각형의 겹침만 본다. 센서로 통로 안 물체를 관측하지 않는다.
- 노드는 Gazebo 없이 가짜 시계·TF·health로만 시험했다(`tools/tests/test_b3_corridor_observer_ros.py`). B2 AMCL과의 실주행 통합은 아직 확인하지 않았다.

## 운영 연결 전에 필요한 공동 확인

1. B2: 채택 지도·revision과 map↔odom 변환.
2. A1/A2/A4: robot_1 접힌 팔과 robot_2 적재물까지 포함한 외곽·위치 여유.
3. B3/D1/D3/A4: 실제 통로 경계·진입선·접근 영역·대기 지점·완전 이탈 기준 합의.
4. D2: 작업대·등록 지점 ID와 용량·접근 조건 확인.

승인은 파일의 bool을 바꾸는 것만으로 성립하지 않는다. 위 확인과 관측 시험 근거가 필요하다.
