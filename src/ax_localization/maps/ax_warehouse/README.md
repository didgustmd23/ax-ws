# 두 창고 월드(`planned/ax_warehouse.world`) 지도 — 채택 지도 revision 2 (개발용)

- **상태:** 2026-11-15 사용자 채택 지시로 채택했다(`map_revision: 2`). 도구 판정(`check_map_accuracy.py --criteria`, 개발용 [`acceptance_criteria_dev.yaml`](acceptance_criteria_dev.yaml), 운영·실물 기준 아님)은 `judgment: PASS`다.
  - **별도 검사 1건이 기준을 넘었다:** "큰 도약 0"(`max_large_jumps: 0`)에 0.1072 m 보정 1건이 있다. 이 이력을 알고 사용자가 채택을 지시했다. 별도 검사 전체 PASS가 아니며, 기준 값은 바꾸지 않았다(아래 [판정](#판정-개발용-기준)).
  - 채택은 개발용 지도 지정이다. 운영·안전 검증이나 Nav2 실제 주행 승인이 아니다([사용 범위와 한계](#사용-범위와-한계)).
- **월드:** 최신 main(c365b03)의 기준(질감) 월드 `6c78d56c…`.
  - 이동 상자 2개(`sample_fit_medium_01` 0.30×0.25×0.24 m, `sample_reject_wide_01` 0.85×0.85×0.25 m)가 실제로 있다.
  - 두 상자는 두 LiDAR 평면(robot_2 앞 0.409 m·뒤 0.289 m) 아래라 지도에 점유가 없고, 그 자리는 free다. 상자가 없는 월드에서 수집한 지도가 아니다.
- **AMCL 회귀:** revision 2 지도로는 아직 하지 않았다. 아래 revision 1 이력의 회귀 결과를 이 지도에 적용하지 않는다.
- **원 회차:** `log/b2_map_current/20261115_083857/run_d141/session_base_dir/`(Git 제외). 평가 보고는 `log/b2_map_current/20261115_083857/REPORT.md`다.
  - robot_2 무부하, SLAM 입력은 B1 앞·뒤 `scan_filtered` 병합 스캔(`slam_mapping.yaml` `8507bb64…`, `scan_merge.py` `bb7754ce…`), slam_toolbox async.
  - 경로 `config/routes/ax_warehouse_full.yaml`(`f1236f68…`) 20경유점 completed, 관문 6개 자동 통과, RTF 1.00.
  - 채택은 같은 바이트의 복사본 `log/b2_map_adopt/20261115_094242/copy/`에서 했다(원 세션 61개 파일 해시 전후 일치). `map_info.yaml`의 `verification.result`는 그 복사본을 가리킨다.

## 파일
| 파일 | SHA-256 | 비고 |
|---|---|---|
| `map.yaml` | `1e7be7232d3b0ff7cce61da5c7103a7ebb70b1710e3edfdf2008cdc8724b3e34` | nav2 map_server 입력, image `map.pgm`(원 `slam_map.yaml`과 값 같음, 표기만 다시 씀) |
| `map.pgm` | `16a5d72473409f093de9d0eb8ca89fac3ae2362c5c205d7556d92cb9beeb8c62` | 원 회차 `run/slam_map.pgm`과 바이트 동일 |
| `pose_graph.posegraph` | `c39a2d45e6425bb05e133b987a120a1138ab14d8f50cccad7aec61f3e8d86c9c` | Git 제외. 원 회차 `run/`과 `log/b2_map_adopt/20261115_094242/pose_graph_moved/`에 보존(해시 같음). slam_toolbox 이어 작성·재로딩용 |
| `pose_graph.data` | `2de5dbd4e60e051dd7d401348d806f0a5a1a83b0033db1a5803a1d5a38e378c0` | 〃 |
| `map_info.yaml` | `ed29f06b470414a2bdea5672952bc9506ee2820d9c63a866eb95aeece47fecac` | revision·월드·모델·센서·마찰·설정·병합 노드 해시, 판정·지표(생성 파일) |
| `applicable_worlds.yaml` | (손 관리 파일, 해시 생략) | 이 지도를 적용할 월드·해시·근거, 미적용 월드와 사유 |

- pose graph 두 파일은 용량(약 17 MB) 때문에 저장소에 넣지 않는다(2026-11-12 사용자 결정을 따름).
  - `map_info.yaml`의 `pose_graph.files`·`files` 해시는 채택 당시 이 폴더에 둔 파일 기준이며, 위 보존 위치의 파일과 같다.
  - 이어 작성·재로딩이 필요하면 보존 파일을 해시로 확인한 뒤 이 폴더에 둔다.
- **적용 월드**([`applicable_worlds.yaml`](applicable_worlds.yaml)):
  - 적용: 기준(질감) 월드 `planned/ax_warehouse.world` `6c78d56c…`(작성·평가 월드)
  - **미적용:** 단색 보존 월드 `planned/ax_warehouse_plain.world` `6cd3d463…`
    - revision 2와의 LiDAR 관측 동등성을 직접 측정하지 않았다.
    - 간접 근거만 있다: `tools/make_textured_world.py --check` 구조 차이 0, 시설 collision·visual XML이 11-12 LiDAR 동일 실측 쌍과 같다.
    - 실측 뒤에 적용 목록으로 옮긴다.
  - 이전 상자 143개 월드(0e88885)는 등록하지 않는다.
  - 해시 대조 시험: `tools/tests/test_map_applicable_worlds.py`.

## 좌표계
- map 원점은 robot_2 출발 자세다.
  - world 기준 `T_world_map`은 초기 (−6.800, 0, 0)이다(revision 1과 같은 정의·같은 값).
  - 참조 지도와의 최적 정합은 (−6.807, −0.043, −0.0004 rad)다.
  - 평가 전용 값이며 운영 위치 추정에 쓰지 않는다.
- `map.yaml` origin (−15.251, −8.061, 0)은 이미지 왼쪽 아래 셀 모서리의 map 좌표다. 해상도 0.05 m, 883×324셀(44.15×16.2 m), trinary, occupied 0.65·free 0.196.
- 셀 수: 자유 234,057 / 점유 5,656 / 미지 46,379. 관측 비율 0.84(지도 사각형 기준).
- revision 1(881×324, origin (−15.231, −8.125))과 격자 크기·원점이 다르다. 셀 단위 비교는 같은 `T_world_map`으로 world 격자에 재표본해서 한다.

## 판정 (개발용 기준)
**`check_map_accuracy.py` 판정(지도 형상, 참조 = 실제 자세로 누적한 앞 LiDAR 격자): PASS**

| 항목 | 기준 | 값(최적 정합) | 참고: 초기 정합 |
|---|---|---|---|
| 장애물 검출 비율 | ≥ 0.95 | 0.999 | 0.971 |
| 역방향 일치 비율 | ≥ 0.80 | 0.860 | 0.825 |
| 벽 오차 p95 | ≤ 0.10 m | 0.050 m | 0.070 m |
| 비교 가능 면적 | ≥ 500 m² | 596.8 m² | 595.4 m² |

**별도 검사(도구가 판정하지 않음): 9개 중 8개 통과, 1개 불통과**

| 항목 | 기준 | 값 | 결과 |
|---|---|---|---|
| 주행·저장 | completed, saved_map_usable, 저장 전 정지 확인 | completed, true, 확인 | 통과 |
| **큰 도약(같은 odom 기준 >0.1 m 또는 >0.05 rad)** | **0** | **1건: sim 604.0→606.4 s, 차체 0.1072 m·+0.0003 rad** | **불통과** |
| SLAM−실제 위치 최대 | ≤ 0.15 m | 0.110 m(RMS 0.057) | 통과 |
| SLAM−실제 yaw 최대 | ≤ 0.02 rad | 0.0051 rad | 통과 |
| 전체 큰 오표시(>0.30 m) | 0셀 | 0 | 통과 |
| 통로 1~3 오표시(>0.15 m) 합 | ≤ 20셀 | 0 / 0 / 0 | 통과 |
| 동−서 창고 차 \|dx\|, \|dy\| | ≤ 0.05 m | 0.001, 0.009 m | 통과 |
| 관문 간 map−실제 거리 | \|오차\| ≤ 0.10 m | −0.010 / +0.075 / +0.010 m | 통과 |
| 재로딩 | map_server 셀 불일치 0, deserialize 성공 | 불일치 0, `deserialize_map` 응답, 노드 163 | 통과 |

- **큰 도약 1건**(`log/b2_map_current/20261115_083857/REPORT.md` 5·8절):
  - slam_toolbox의 map→odom TF가 약 30.6 s마다 2.3~2.4 s씩 발행되지 않았다(24회). 그중 한 공백 뒤에 0.107 m 보정이 한 번에 반영됐다.
  - 보정 전 SLAM−실제 오차 (−0.087, +0.06) m가 약 28 s 동안 유지되다가, 보정 뒤 (0.001, 0.008) m로 줄었다. odom 표류가 아니라 추정 보정이다.
  - 같은 벽시계 길이의 공백은 revision 1 회차(RTF 0.20)에도 있었고 sim으로 0.6~0.7 s였다. 원인은 확인하지 않았다.
- **재로딩:** 채택 전 같은 바이트의 후보 자료로 2026-11-15에 격리 domain에서 실행한 결과(`log/b2_map_current/20261115_083857/eval/reload/reload.json`)를 재사용한다.
  - 원 세션 해시가 같아 근거로 쓴다. 채택 뒤 다시 실행하지는 않았다.
  - 채택 도구의 파일 재로딩 검사(`map_info.yaml` `reload_check`)는 채택 때 통과했다.
- **잔상:** 최신 월드 collision 격자 대조에서 삭제 상자 잔상 0, 현재 구조에서 0.15 m 넘는 점유 0(`log/b2_map_current/20261115_083857/eval/candidate_map_check/`).

## 사용 범위와 한계
- **낮은 상자 2개:** 지도에서 free지만 물체가 있다. LiDAR 평면 아래라 Nav2 LiDAR 장애물 층에도 보이지 않는다. 지도의 free는 차량 통과 가능을 보장하지 않는다.
- **선반 내부:** 선반판이 앞 LiDAR 평면 밖이라 선반 10개 내부 셀이 대부분 free다(`log/b2_map_baseline_prep/20261115_093443/results/rack_interior_check.json`). 차체(높이 1.05 m)는 선반판에 닿는다. keepout 등 별도 제약 없이 이 지도만으로 경로를 계획하면 선반 아래로 들어갈 수 있다.
- **2D로 표현할 수 없는 영역:** 선반판 아래와 두 LiDAR 평면 밖의 물체는 지도에서 비어 보여도 차체가 충돌할 수 있다. 별도 collision 제약으로 다룬다.
- **관측 높이 차:** 지도에는 뒤 LiDAR(0.289 m)가 본 낮은 구조가 들어 있고, AMCL은 앞 0.409 m 스캔만 쓴다. revision 2에서 그 영향은 확인하지 않았다.
- **배치 물체:** 지도 작성 시점의 상자 배치를 기준으로 한다. 상자를 추가·이동하면 실시간 장애물 층(관측 가능한 높이일 때만)이 반영해야 하며, 지도는 바꾸지 않는다. 시설(벽·기둥·선반) 변경은 지도 갱신 대상이다.
- **미완료(채택과 무관하게 남음):** revision 2 AMCL 회귀, Nav2 keepout, 낮은 물체 관측(RGB-D 장애물 출처), A4 주행 허가 경로, D1 목표 관리, 두 로봇 동시.
- **시뮬레이션 조건:** 바퀴 odom이 매우 정확하다(odom 단독 위치 최대 0.104 m). 실물 odom 오차나 마찰 변경 조건에서의 지도 품질은 알 수 없다.
- **회차 간 편차:** 같은 설정이라도 회차마다 SLAM 결과가 다를 수 있다(revision 1 때 d105 0.222 m). 재생성하면 같은 품질을 보장하지 않는다.
- **소비자:** B3 영역 장부(`src/ax_environment/config/ax_warehouse_regions.json`)는 아직 `map_revision: 1`이다.
  - 좌표 변환은 같은 (−6.8, 0, 0)이지만 revision 대조가 맞지 않는다. 그래서 `tools/check_b3_region_readiness.py`의 `map_frame_revision`이 실패한다.
  - B3 담당이 생성기(`tools/generate_b3_regions.py` `MAP_REVISION`)를 확인해 갱신해야 한다.
- **메타정보 경로:** `map_info.yaml`의 설정·병합 노드·경로 `source`·`file`은 작업공간 기준 상대 경로다. 대조에는 해시를 쓴다. `code.git_dirty`는 true다(다른 작업자 미커밋 변경, diff 해시 기록).

## 이력
- **revision 1(2026-11-12 채택, 2026-11-15 교체):**
  - 단색 보존 월드 `9b5bb281…`(상자 143개)에서 만든 지도다. 도구 PASS, 별도 검사 모두 통과였다.
  - 2026-11-15 팀원의 상자 141개 제거(4406349) 뒤 최신 월드 대조에서 삭제 상자 잔상 783셀, 현재 구조에서 0.30 m 넘는 점유 983셀이 나와 최신 월드에 부적합했다(`log/b2_map_current/20261115_083857/existing_map_check/`). 이 판정은 그대로 둔다.
  - 백업: `log/b2_map_adopt/20261115_094242/backup_rev1_maps/`(지도·메타·적용 목록·README), pose graph `log/b2_map_adopt/20261112_111033/pose_graph_moved/`.
  - 원 회차 `log/b2_mapping/20261112_083635/`, 채택 보고 `log/b2_map_adopt/20261112_111033/REPORT.md`.
  - revision 1의 AMCL 회귀(2026-11-12, robot_2 단독 정지·짧은 이동, 위치 오차 최대 0.036 m, not_judged)는 그 지도·당시 설정의 결과이며 revision 2에 적용하지 않는다.
- **2026-11-11 후보(채택 안 함):** `log/b2_mapping/20261111_143110/`. 앞 `scan_filtered`만 썼고, 동−서 창고 (+0.17, −0.27) m 어긋남, 통로 1 가짜 점유 88셀, 큰 오표시 462셀, 위치 최대 0.47 m라 쓰지 않았다.

## 재생성·채택
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
# 주행·저장(격리 domain). 세션 기본 SLAM 입력은 앞·뒤 병합(--scan merged)
bash src/ax_localization/scripts/run_mapping_session.sh --world planned/ax_warehouse.world \
  --route src/ax_localization/config/routes/ax_warehouse_full.yaml --gate-dir <관문 폴더> --domain <N> [--wait-before-drive]
# 평가: 복사본에서 판정만(--adopt 없이) → judgment PASS 확인
rsync -a --exclude accuracy.json <회차>/run <회차>/session <평가 폴더>/
python3 src/ax_localization/scripts/check_map_accuracy.py <평가 폴더>/run \
  --criteria src/ax_localization/maps/ax_warehouse/acceptance_criteria_dev.yaml
# 별도 검사(도약·통로 오표시·동서 차·관문·재로딩)를 기준 파일의 supplementary 항목대로 확인한 뒤 채택
python3 src/ax_localization/scripts/check_map_accuracy.py <평가 폴더>/run \
  --criteria src/ax_localization/maps/ax_warehouse/acceptance_criteria_dev.yaml \
  --adopt --map-dir src/ax_localization/maps/ax_warehouse
```
- revision 2 회차는 위 세션 스크립트의 HEAD 판 사본(결과 폴더만 회차 안으로 바꿈)으로 실행했다(`log/b2_map_current/20261115_083857/harness/`). 경로·관문 자동 점검·구역 평가 스크립트도 그 폴더에 있다.
- `--adopt`는 pose graph 두 파일도 이 폴더에 복사한다. 커밋 전에 원 회차와 해시를 대조하고 이 폴더에서 빼 둔다(Git 제외).
- `--adopt`는 judgment가 PASS일 때만 채택한다. 별도 검사 항목은 도구가 판정하지 않으므로 채택 전에 따로 확인하고, 불통과가 있으면 채택 여부를 사용자가 정한다(revision 2 사례).

## 남은 일
- revision 2 AMCL 회귀(robot_2 단독부터), 단색 월드 LiDAR 동등성 실측(적용 목록 이동 여부).
- B3 영역 장부 revision 정합(B3 담당).
- slam_toolbox map→odom TF 주기 공백의 원인 확인.
- 별도 검사 항목(도약 등)을 `check_map_accuracy.py` 판정에 넣을지 결정.
