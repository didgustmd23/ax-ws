# 두 창고 월드(`planned/ax_warehouse.world`) 지도 — 채택 지도 revision 1 (개발용)

- **상태:** 2026-11-12에 채택했다(`map_revision: 1`, `judgment: PASS`). 판정 기준은 개발용 [`acceptance_criteria_dev.yaml`](acceptance_criteria_dev.yaml)이다(사용자 승인, 운영·실물 기준 아님).
- **AMCL(6단계) 회귀:** 2026-11-12 robot_2 단독 정지·짧은 이동 구간에서 위치 오차 최대 0.036 m였다([AMCL 회귀](#amcl-회귀-2026-11-12) 절). 판정 기준이 없어 not_judged다. 두 로봇·전체 경로는 확인하지 않았다.
- **원 회차:** `log/b2_mapping/20261112_083635/`(Git 제외).
  - robot_2 무부하, SLAM 입력은 B1 앞·뒤 `scan_filtered` 병합 스캔(S2, `slam_mapping.yaml` `4cd2c5bf…`, `scan_merge.py` `bb7754ce…`), slam_toolbox async, 경로 사본 `704b7241…`, 20경유점 completed.
  - 평가와 채택은 같은 바이트의 복사본 `log/b2_map_adopt/20261112_111033/copy/`에서 했다(62개 파일 해시 일치). 그래서 `map_info.yaml`의 `verification.result`는 복사본 경로를 가리킨다.
  - 원 회차 평가 보고: `log/b2_custom_warehouse/20261112_074010/merged_default/REPORT.md`(2회차 d106).

## 파일
| 파일 | SHA-256 | 비고 |
|---|---|---|
| `map.yaml` | `22a284457b1b194dc2eb6e813d2850d9da23925149d42adfc256ca1c90c05ec9` | nav2 map_server 입력, image `map.pgm` |
| `map.pgm` | `e62cac44131c7199d123a999bda8fcb161973ab302f8b15dde1d1eea39bbaa31` | 원 회차 `run/slam_map.pgm`과 바이트 동일 |
| `pose_graph.posegraph` | `6e024528eb0235799938cf81507ad7e0dbf51f2f7c04f3192b8df54879f3d764` | 원 세션 경로·해시에 보존, Git 제외(`log/b2_mapping/20261112_083635/run/`, map_info.yaml은 생성 파일이라 원 기록 유지). slam_toolbox 이어 작성·재로딩용 |
| `pose_graph.data` | `10015da39277115a54faf262f9cb1f9ecb077db22390d2a69704ff4fea43d597` | 〃 |
| `map_info.yaml` | `e19d21dacd5d9277bc56710f323425e0f4c6b4bb0b30923ecb9e17736157ddce` | revision·월드·모델·센서·마찰·설정·병합 노드 해시, 판정·지표 |
| `applicable_worlds.yaml` | (손 관리 파일, 해시 생략) | 이 지도를 적용할 월드·해시·근거. 생성 파일 아님 |

pose graph 두 파일은 용량(약 13 MB) 때문에 저장소에 넣지 않는다(2026-11-12 사용자 결정). `map_info.yaml`의 `pose_graph.files`·`files` 해시는 채택 당시 이 폴더에 둔 파일 기준이며, 원 회차 파일과 같다. 이어 작성·재로딩이 필요하면 원 회차 파일을 해시로 확인한 뒤 쓴다.

`map_info.yaml`의 `world.world_sha256`은 지도를 만든 월드 `9b5bb281…`(session 사본 일치)다. 이 월드는 2026-11-12 기준 월드 교체 뒤 단색 보존 월드 `planned/ax_warehouse_plain.world`로 이름만 바뀌었다(내용 동일).

- 같은 경로 `planned/ax_warehouse.world`에는 이제 질감 월드(`35a8c0ef…`)가 있다. map_info.yaml의 `world_file`은 기록 당시 경로이므로 해시로 구분한다.
- 이 지도를 쓸 수 있는 월드와 근거는 손으로 관리하는 [`applicable_worlds.yaml`](applicable_worlds.yaml)에 있다(단색·질감 두 월드, LiDAR 동일 근거). 지도 이미지가 그대로라 `map_revision`은 1이다.
- 해시 대조 시험: `tools/tests/test_map_applicable_worlds.py`.

## 좌표계
- map 원점은 robot_2 출발 자세다. world 기준 `T_world_map`은 초기 (−6.800, 0, 0)이고, 참조 지도와의 최적 정합은 (−6.822, +0.006, +0.0003 rad)다. 평가 전용 값이며 운영 위치 추정에 쓰지 않는다.
- `map.yaml` origin (−15.231, −8.125, 0)은 이미지 왼쪽 아래 셀 모서리의 map 좌표다. 해상도 0.05 m, 881×324셀(44.05×16.2 m), trinary, occupied 0.65·free 0.196.
- 셀 수: 자유 229,825 / 점유 7,274 / 미지 48,345. 관측 비율 0.83(지도 사각형 기준). 동쪽 창고 관측 비율 0.993(원 평가).

## 판정 (개발용 기준)
**`check_map_accuracy.py` 판정(지도 형상, 참조 = 실제 자세로 누적한 앞 LiDAR 격자)**

| 항목 | 기준 | 값 |
|---|---|---|
| 장애물 검출 비율 | ≥ 0.95 | 0.991 |
| 역방향 일치 비율 | ≥ 0.80 | 0.858 |
| 벽 오차 p95 | ≤ 0.10 m | 0.050 m |
| 비교 가능 면적 | ≥ 500 m² | 588.9 m² |

**별도 검사(도구가 판정하지 않음)**

결과는 `log/b2_map_adopt/20261112_111033/supplementary_checks.json`에 있다.

| 항목 | 기준 | 값 |
|---|---|---|
| 주행·저장 | completed, saved_map_usable, 저장 전 정지 확인 | completed, true, 확인 |
| 큰 도약(같은 odom 기준 >0.1 m 또는 >0.05 rad) | 0 | 0(작은 도약 0.03~0.06 m 3건) |
| SLAM−실제 위치 최대 | ≤ 0.15 m | 0.069 m(RMS 0.036) |
| SLAM−실제 yaw 최대 | ≤ 0.02 rad | 0.0035 rad |
| 전체 큰 오표시(>0.30 m) | 0셀 | 0 |
| 통로 1~3 오표시(>0.15 m) 합 | ≤ 20셀 | 0 / 0 / 0 |
| 동−서 창고 차 \|dx\|, \|dy\| | ≤ 0.05 m | 0.002, 0.020 m |
| 관문 간 map−실제 거리 | \|오차\| ≤ 0.10 m | −0.060 / +0.010 / −0.020 m |
| 재로딩 | map_server 셀 불일치 0, deserialize 성공 | 불일치 0, `deserialize_map` 응답, 노드 163·간선 168 |

- 재로딩은 Gazebo 없이 격리 domain에서 했다. deserialize 뒤 발행된 지도의 셀 수가 파일과 같았다.

## AMCL 회귀 (2026-11-12)
**조건**
- 설치본 당시 기준 월드, 지금의 단색 보존 월드(`planned/ax_warehouse_plain.world`, `9b5bb281…`), robot_2만, headless, RTF 약 0.20(0.200~0.206).
- AMCL 입력은 앞 LiDAR `scan_filtered`만이다. AMCL·health 설정은 설치본 그대로다.
- 지도는 이 폴더의 `map.yaml`(revision 1)을 절대 경로로 지정했다. initial pose는 robot_2 (0, 0, 0)이다(map 원점 = 출발 자세).
- 실제 위치 비교에는 채택 평가의 `T_world_map`을 썼다(검증 전용).
- 평가는 `check_localization.py`로 했고, 지표는 이전 회귀와 같다. 판정 기준이 없어 judgment는 not_judged다.

**회차 1: 정지 + 짧은 이동**

순서는 출발 정지 20 s → 직진 1.5 m → 좌회전 90° → 원위치 회전 → 후진 1.5 m → 최종 정지 10 s였고, 사이사이 5 s씩 정지했다.

| 항목 | 값 |
|---|---|
| 위치 오차 평균 / p95·최대 | 0.021 / 0.036 m |
| yaw 오차 최대 | 0.024 rad |
| 구간별 위치 오차 최대 | 출발 정지 0.023, 직진 0.023, 좌회전 0.017, 원위치 회전 0.030, 후진 0.032, 최종 정지 0.036 m |
| 도약 | 0(정상 갱신당 추정 변화 최대 0.025 m·0.025 rad) |
| health | OK 58 %, DEGRADED(COVARIANCE_HIGH) 40 %, LOST(TF_MISSING) 시작 직후 1.0 s |

- DEGRADED는 이동 중이다. AMCL 표준편차가 0.16~0.27 m·0.12~0.22 rad로 커졌다.
- 신뢰 자세는 정지 중에만 기록됐다. 이동 중에는 POSITION_STD_HIGH로 갱신되지 않았다.
- 이전 AWS 개발 지도 회귀와 비교(이동 거리가 다르므로 직접 비교할 수 없다)
  - 출발 정지 오차: 0.019 → 0.023 m
  - 이동 중 위치 오차: 평균 0.097·최대 0.143 m → 평균 0.021·최대 0.036 m
  - yaw 최대: 0.040 → 0.024 rad
  - 이번 이동은 1.5 m 왕복과 90° 회전이고, 이전은 동쪽 2 m → 남쪽 5.8 m였다.

**회차 2: AMCL 재설정 관찰**

순서는 정지 → 직진 0.6 m → AMCL lifecycle 재설정(deactivate·cleanup·configure·activate) → 재위치 요청이었다.
- 이전 AWS 시나리오(도약 → LOST → 재위치)는 재현되지 않았다.
  - 재설정 뒤 AMCL은 initial pose (0, 0, 0)이 아니라 마지막 저장 자세로 다시 초기화됐다.
  - 저장 자세는 x 0.58 m, 실제 위치는 0.69 m였다.
- 정지 상태라 AMCL이 갱신하지 않았다. 그래서 위치 오차 0.111 m가 끝까지 남았고, 그동안 health는 OK였다.
- 추정 변화 0.12 m는 도약 기준 0.3 m보다 작다. 그래서 재위치 요청은 REINIT_REJECTED_NOT_NEEDED로 거절됐다(현재 설계상 정상 동작).
- 재설정 때 마지막 갱신과 실제 사이의 차이가 OK 상태로 남는 경로다. B2 후속 과제로 다룬다.

**하지 않은 것**
- 두 로봇 동시
- 도약을 일으키는 재위치(초기 자세 설정 변경이 필요해 결정 사항)
- 전체 경로에서 관측 높이 차(병합 지도 vs 앞 스캔 AMCL)의 영향

**근거(작성 PC, Git 제외)**
- `log/b2_health/20261112_112923_robot2_rev1_amcl/`(회차 1, `REPORT.md`)
- `log/b2_health/20261112_113642_robot2_rev1_reinit/`(회차 2)

## 사용 범위와 한계
- **역방향 일치 0.858:** SLAM 점유의 약 14%가 참조(앞 LiDAR만)에 없다. 원인은 뒤 LiDAR 때문으로 추정한다. 뒤 LiDAR(차체 기준 z 0.02 m)가 앞 LiDAR 평면(z 0.14 m) 아래의 선반 다리·하단 띠·낮은 상자를 그린다. 원 평가에서 앞 스캔 지도 대비 새 점유 1,652셀 중 1,650셀이 구조물 0.1 m 안이었다. 실제 구조라 오표시는 아니다.
- **관측 높이 차:** 이 지도에는 뒤 LiDAR(0.02 m)가 본 낮은 물체가 들어 있다. AMCL은 앞 0.14 m 스캔만 쓴다. 짧은 이동 구간 AMCL 회귀에서는 오차가 작았지만(최대 0.036 m), 전체 경로에서의 영향은 확인하지 않았다.
- **2D로 표현할 수 없는 영역:** 선반판 아래와 두 LiDAR 평면 밖의 물체는 지도에서 비어 보여도 차체가 충돌할 수 있다. 별도 collision 제약으로 다룬다.
- **동적 상자:** 지도 작성 중의 상자 배치를 기준으로 한다. 배치가 바뀌면 그 영역은 무효다.
- **회차 간 편차:** 같은 설정의 1회차(d105)는 위치 최대 0.222 m, 통로 오표시 40/48/96셀로 이 회차보다 크게 나빴다. 원인은 확인하지 못했다. 이 지도는 좋은 쪽 회차이며, 재생성하면 같은 품질을 보장하지 않는다.
- **시뮬레이션 조건:** 바퀴 odom이 매우 정확하다(odom 단독 위치 최대 0.095 m). 실물 odom 오차나 마찰 변경 조건에서의 지도 품질은 알 수 없다.
- **질감 월드:** 질감 사본 월드(재질만 다름)를 나중에 기준 월드로 채택해도 LiDAR 스캔이 같음을 확인했으므로(차이 0 m) 지도 내용은 유효하다. 다만 `map_info.yaml`의 월드 해시와 `map_revision`은 갱신해야 한다.
- **메타정보 경로:** `map_info.yaml`의 설정·병합 노드·경로 `source`·`file`은 작성 PC의 절대 경로다. 대조에는 해시를 쓴다.

## 이력
- 2026-11-11 후보(채택 안 함): `log/b2_mapping/20261111_143110/`. 앞 `scan_filtered`만 썼고, 동−서 창고 (+0.17, −0.27) m 어긋남, 통로 1 가짜 점유 88셀, 큰 오표시 462셀, 위치 최대 0.47 m라 쓰지 않았다. 평가: `log/b2_custom_warehouse/20261111_143013/full_map/`.

## 재생성·채택
- 아래 명령의 병합 입력(`--scan merged`)·`--criteria`·`--map-dir` 동작은 후속 커밋(지도 작성 도구·병합 스캔 입력, RTAB-Map 비교 모드)의 도구 기준이다.

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
# 주행·저장(격리 domain). 세션 기본 SLAM 입력은 앞·뒤 병합(--scan merged)
bash src/ax_localization/scripts/run_mapping_session.sh --world planned/ax_warehouse.world \
  --route src/ax_localization/config/routes/ax_warehouse_full.yaml --gate-dir <관문 폴더> --domain <N> [--wait-before-drive]
# 평가: 복사본에서 판정만(--adopt 없이) → judgment PASS 확인
rsync -a --exclude accuracy.json log/b2_mapping/<회차>/run log/b2_mapping/<회차>/session <평가 폴더>/
python3 src/ax_localization/scripts/check_map_accuracy.py <평가 폴더>/run \
  --criteria src/ax_localization/maps/ax_warehouse/acceptance_criteria_dev.yaml
# 별도 검사(도약·통로 오표시·동서 차·관문·재로딩)를 기준 파일의 supplementary 항목대로 확인한 뒤 채택
python3 src/ax_localization/scripts/check_map_accuracy.py <평가 폴더>/run \
  --criteria src/ax_localization/maps/ax_warehouse/acceptance_criteria_dev.yaml \
  --adopt --map-dir src/ax_localization/maps/ax_warehouse
```
- `--adopt`는 pose graph 두 파일도 이 폴더에 복사한다. 커밋 전에 원 회차와 해시를 대조하고 이 폴더에서 빼 둔다(Git 제외).
- `--adopt`는 judgment PASS를 확인하지 않고 주행·정지·메타정보만 본다. 그래서 반드시 판정을 먼저 확인한다(도구 보완은 남은 일).
- `config/routes/ax_warehouse_full.yaml`은 실행 사본(`704b7241…`)과 경로 값이 같고 머리말 주석만 다르다. 구역별·통로·재로딩 평가 스크립트는 평가 회차의 `harness/`에 있다.

## 남은 일
- AMCL 남은 확인: 두 로봇 동시, 도약을 일으키는 재위치, 전체 경로 관측 높이 차 영향, 정확도 판정 기준.
- AMCL 재설정 때 마지막 저장 자세와 실제 위치 차이가 OK 상태로 남는 경로(아래 회차 2)를 B2 후속 과제로 다룬다.
- `check_map_accuracy.py --adopt`가 judgment PASS를 확인하도록 보완하고, 별도 검사 항목을 도구 판정에 넣을지 결정한다.
- 팀 전달(D1 Nav2·B3 등 지도 소비자): revision 1과 위 한계.
