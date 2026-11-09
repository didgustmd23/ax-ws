# AWS 창고 지도 (B2)

기준 월드 `aws_warehouse.world`의 점유 지도를 만들고 검증해 팀원에게 제공하는 위치다. 설계 기준은 [상세설계 6.1절](../../../../docs/design/DETAILED_DESIGN.md#61-기준-지도-작성검증전달-b2)이다.

## 현재 상태

- **채택한 운영 지도는 없다.** 이 폴더에는 아직 `map.yaml`·`map.pgm`이 없다.
- 지금까지의 결과는 모두 **개발용**이며 운영 지도로 채택하지 않았다.
  - 2026-11-08 이전 결과: 바퀴 마찰 후보(S005, `slip1=0.05`)를 넣은 임시 사본과 충돌을 단순화한 월드(`light_collision`)에서 만들었다. 근거는 [S005 바퀴 미끄럼 검증·공유 자료](../../../../docs/evidence/wheel_slip/)에 있다.
  - 2026-11-09 결과(`log/b2_mapping/20261109_085358`): 원본 월드·S005 반영 개발 모델로 전체 경로를 주행해 새로 작성했다. 메타정보 누락 0, 저장·재로딩 일치, 판정 `not_judged`. 채택 조건 검사는 저장소 밖 임시 폴더에서만 해 보았고 이 폴더에는 넣지 않았다. A1 모델이 개발 단계이므로 운영 지도가 아니다.
- **경로 조건(2026-11-09):** 경로 외곽은 robot_2 선반 포함 외곽(앞 0.66·뒤 0.62·반폭 1.03 m)이다. 이 폭으로 지날 수 없는 서쪽 통로·북쪽 횡단은 경로에서 빼고 LiDAR로 관측한다. AWS 월드의 robot_2 출발 위치는 선반 충돌 겹침 때문에 world (-1.0, -2.3)으로 옮겼다(`robot_test.launch.py`).
- **낮은 충돌물(원인 미특정):** 중앙 동서 통로 서쪽(world x 약 -4.45)에서 앞 LiDAR 평면보다 낮은 충돌물에 차체가 막히거나 타고 올랐다. 무엇인지는 특정하지 못했고 해당 구간을 경로에서 피했다. 해결한 것이 아니다.
- 운영 모델과 기준 월드가 확정되면 아래 순서로 다시 만들고 검증한다. 정확도 기준이 확정되기 전에는 판정 상태가 `not_judged`다.

## 산출물

채택하면 이 폴더에 다음 파일이 생긴다(`check_map_accuracy.py --adopt`).

| 파일 | 내용 |
|---|---|
| `map.yaml`·`map.pgm` | 점유 지도(Nav2 trinary). 해상도·원점은 `map.yaml` |
| `pose_graph.posegraph`·`pose_graph.data` | slam_toolbox pose graph(이어 작성·위치 추정용) |
| `map_info.yaml` | `map_revision`, 지도 요약(원점·해상도·크기·관측/미관측 셀), 월드·모델·센서·바퀴 마찰·SLAM 설정·경로와 각 해시, 코드 버전, odom 기준 정지 확인, 검증 결과·판정, 재로딩 확인, 파일 해시, 누락 항목 |

회차마다 만든 결과는 `log/b2_mapping/<KST 시각>/`에 남는다(Git 제외).

| 경로 | 내용 |
|---|---|
| `run/drive_result.json` | 주행 상태·odom 기준 정지 확인(`stops`, `stop_gate`)·저장 결과·`saved_map_usable`·모델 기록 |
| `run/slam_map.yaml`·`slam_map.pgm`, `run/pose_graph.*` | 이번 회차의 저장 지도 |
| `run/robot_description.urdf` | 실제 spawn에 쓰인 모델(바퀴 마찰·LiDAR 설정 근거) |
| `run/accuracy.json`, `run/overlay.png`, `run/reference_map.*` | 검증 결과·겹침 그림·참조 지도 |
| `run/recording.npz` | 스캔·odom·map→odom·Gazebo 실제 자세 원시 기록 |
| `session/session.json` | 실행 조건(월드·바퀴 조건과 파일 해시, git commit, ROS_DOMAIN_ID·GZ_PARTITION) |

## 좌표계

- 지도 좌표계는 `map`이다. TF는 `map → robot_N/odom → robot_N/base_link`.
- 해상도는 0.05 m/셀(`config/slam_mapping.yaml`의 `resolution`)이다.
- `map.yaml`의 `origin: [x, y, yaw]`는 이미지 왼쪽 아래 셀 모서리의 `map` 좌표다. 회전된 원점(yaw ≠ 0)은 지원하지 않는다.
- `map` 원점은 SLAM을 시작할 때 로봇의 출발 위치(odom 원점)로 정해지며 Gazebo 월드 좌표와 다르다. `map_info.yaml`의 `T_world_map`은 검증 전용이며 운영 위치 추정·주행에 쓰지 않는다.

## 재생성 순서 (시연용)

작업 공간 루트에서 실행한다. 빌드(`colcon build`)와 `source install/setup.bash`가 끝나 있어야 한다.

```bash
# 1~5를 한 번에: 격리 시뮬레이션 → B1 LiDAR 전처리 → SLAM(새 지도) → 지도 작성 주행 → odom 기준 정지 확인 → 저장 → 검증
bash src/ax_localization/scripts/run_mapping_session.sh            # 기본 headless, robot_2
bash src/ax_localization/scripts/run_mapping_session.sh --gui --keep   # 시연: Gazebo 화면, 끝난 뒤 RViz 확인 대기
```

1. **새 지도 작성:** slam_toolbox를 `mode: mapping`으로 띄우고 기존 지도(`map_file_name`·`deserialize_map`)를 읽지 않는다.
2. **지도 생성 주행:** `mapping_drive.py`가 `config/routes/aws_warehouse.yaml` 경유점을 따라간다. 격리 개발 시뮬레이션 전용이며 운영 주행 경로가 아니다(A4 없이 제어기에 직접 명령).
3. **odom 기준 정지 확인:** 경로 완료 뒤와 저장 직전에 0 속도 명령과 odom 속도가 D-06 값 안에 0.5 s 머무는지를 따로 기록한다. 확인하지 못하면 저장하지 않고 `failed`로 끝난다. 이 확인은 Gazebo 실제 자세로 확인한 정지가 아니며, 실제 자세는 `recording.npz`·`trajectory_comparison`에서 따로 비교한다.
4. **저장:** `map_saver_cli`(`--free 0.196`)와 `serialize_map`으로 `run/`에 저장한다. 종료 정지를 확인하지 못하면 저장한 지도를 `saved_map_usable: false`로 표시한다.
5. **검증:** `check_map_accuracy.py`가 참조 지도와 비교해 `run/accuracy.json`을 쓴다.

검증만 다시 하거나 기준을 줄 때:

```bash
python3 src/ax_localization/scripts/check_map_accuracy.py log/b2_mapping/<KST 시각>/run
python3 src/ax_localization/scripts/check_map_accuracy.py log/b2_mapping/<KST 시각>/run --criteria <기준.yaml>
```

`--criteria`가 없으면 `judgment: not_judged`다. 결과에는 `drive_status`·`trajectory_comparison`·`map_comparison` 세 상태, 지표, 지도 요약(`map`), 관측 범위(`coverage`)와 메타정보 문제(`metadata_problems`)가 따로 기록된다.

**실패 시:** 결과 폴더의 `drive_result.json`(`status`, `error`, `stops`)과 `accuracy.json`에 실패 단계와 사유가 남는다. 이전 회차 지도나 이 폴더의 지도를 이번 회차의 새 지도처럼 대신 제공하지 않는다.

## 채택 (운영 모델·기준 월드 확정 뒤)

```bash
python3 src/ax_localization/scripts/check_map_accuracy.py log/b2_mapping/<KST 시각>/run --adopt
```

다음을 모두 만족할 때만 이 폴더의 파일을 바꾸고 `map_revision`을 1 올린다. 하나라도 어긋나면 `adoption: refused`와 사유를 남기고 기존 파일을 건드리지 않는다.

- 전체 경로 주행(부분 경로 `--waypoints` 제외)·궤적 비교 완료, 저장 전·종료 odom 기준 정지 확인, `saved_map_usable: true`
- 메타정보 누락 없음(실행 월드 해시, spawn 모델 기록, `session.json`)
- 월드·바퀴 조건이 `original`(`--light`·`--wheel-slip1` 결과는 채택 불가)
- 채택용 지도를 다시 읽은 격자·원점·해상도·크기가 저장 지도와 같음

## revision·해시 확인

```bash
grep map_revision src/ax_localization/maps/aws_warehouse/map_info.yaml
sha256sum src/ax_localization/maps/aws_warehouse/map.{yaml,pgm} src/ax_localization/maps/aws_warehouse/pose_graph.*
```

`sha256sum` 결과가 `map_info.yaml`의 `files` 값과 같아야 한다. 지도를 쓰는 쪽은 사용한 `map_revision`을 함께 기록한다. **지도가 바뀌면(`map_revision` 변경) 등록 지점 좌표와 revision을 다시 확인해야 한다**([상세설계 7.12절](../../../../docs/design/DETAILED_DESIGN.md#712-입고재고등록-지점위치-예약)).

## 사용 제한

- 판정 기준이 확정되기 전에는 지표가 좋아도 `not_judged`이며 통과 지도가 아니다.
- 미관측 영역(`map_info.yaml`의 `map.cells.unknown`, `verification.metrics.coverage`)은 빈 공간으로 보장하지 않는다.
- 참조 지도와 SLAM이 같은 앞 LiDAR 스캔을 쓰므로, LiDAR 높이 밖 장애물이나 가림은 둘 다 놓칠 수 있다.
- `stop_gate`는 odom 기준이다. odom과 Gazebo 실제 자세가 어긋날 수 있으므로 odom 기준 정지 확인만으로 실제 정지나 지도 품질을 보장하지 않는다. 11-08 원본 월드 짧은 시험(`log/b2_mapping/20261108_164821`)에서 odom은 약 6.8 m 이동을 기록했지만 Gazebo 실제 자세는 거의 움직이지 않았다. 원인은 **추정**이다. 2026-11-09에 같은 출발 위치 world (2.5, -1.5)·원본 월드에서 robot_2가 선반 충돌과 겹쳐 움직이지 않는 것(odom 0.575 m, 실제 0.002 m)을 확인했다. 11-08 실행도 출발 위치·월드가 같았다. 바퀴는 S005 반영 전이고 출발 높이·시험 명령도 달라 같은 원인인지는 대조하지 않았다.
- 재로딩 확인은 지도 파일 수준이며 slam_toolbox pose graph 재로딩은 포함하지 않는다.
- AMCL 위치 추정·`map→odom` 신뢰성, 작업대·통로 등록, Nav2 연결은 이 지도 전달 범위가 아니다. 위치 추정 실행(개발용)은 [패키지 안내](../../README.md#위치-추정amcl--개발용)에 있다.
