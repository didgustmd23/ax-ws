# 두 창고 운반 시험 월드

실행 가능한 월드는 이 창고의 두 파일이다. 기준(기본)은 질감을 입힌 `ax_warehouse.world`이고,
`ax_warehouse_plain.world`는 같은 구조의 단색 보존본이다(아래 [질감과 단색 보존 월드](#질감과-단색-보존-월드) 절).
빈 월드는 삭제했으며 주행 시험의 빈 바닥은
`tools/empty_test_fixture.py`가 테스트 때만 임시 생성한다. 테스트 파일은 유지한다.

`ax_warehouse.world`는 서쪽 출발 창고와 동쪽 도착 창고를 연결한다.
월드 내부 이름은 기존 launch와 호환되도록 `ax_test`를 유지한다.

| 영역 | 위치·크기 |
|---|---|
| 출발 창고 | X -22~-4m, Y -8~8m |
| 도착 창고 | X 4~22m, Y -8~8m |
| 연결 통로 3개 | 각각 길이 8m, 유효 폭 1.8m, 중심 X=0, Y≈-5.02/0/5.08m |
| 출발 창고 선반 사이 길 | Y 2.2~4.0m, 유효 폭 1.8m |
| 픽업 구역 | 중심 (-8,-3.8)m |
| 도착 하역 구역 | 중심 (9,0)m, 4×4m |
| 빈 적재 구역 | 중심 (15,±3.8)m, 각 5×3m |
| 도착 창고 선반 4개 | 중심 (14,±5.8), (19,±5.8)m |

출발 창고 남쪽에는 18~25cm 동적 상자 **120개**를 40개 묶음으로 3단씩 쌓는다.
각 상자는 개별 충돌 형상과 질량(0.35~0.65kg)을 가지므로 집거나 밀 수 있다.
출발 창고에는 선반 6개, 도착 창고에는 빈 선반 4개가 있다.
선반은 기둥 4개와 선반판 3개로 구성되며 실제 충돌 형상이 있다.
중앙 통로와 하역 구역은 비워두며, 바닥의 색 표시는 충돌 형상이 없다.

로봇 1~4 출발 좌표: (-10,-2.4), (-6.8,0), (-17,-2.4), (-13,-2.4)m.

```bash
ros2 launch ax_description ax_2_test.launch.py world:=planned/ax_warehouse.world
```

예상 흐름: 출발 창고 픽업 → 통로 앞 대기 → 통로 통과 → 도착 하역 → 적재.
통로는 AX_2 한 대 통과용이며 교행을 가정하지 않는다.
자동 집기·운반·적재, 통로 교행 제어, 이 월드의 지도와 경로는 별도 구현이 필요하다.
기존 월드용 지도는 변경된 구조와 일치하지 않는다.

## 크기별 적재 샘플

기존 상자 120개에 샘플 23개를 추가해 총 143개다. 초록색은 빈 적재판의
72×72×50cm 범위에 들어가는 상자, 빨간색은 한 축 이상 초과하는 상자다.
초록색은 무게·집기 가능성까지 보장하는 표시는 아니며, 쌓인 묶음의 전체 높이가
아닌 개별 상자 크기를 기준으로 한다. 자동 인식 결과가 아닌 시험용 정답 표시다.

| 종류 | 가로×세로×높이(cm) | 수량 | 표시 |
|---|---|---:|---|
| 초소형 | 12×12×12 | 3 | 초록 |
| 소형 | 20×18×18 | 2 | 초록 |
| 중형 | 30×25×24 | 2 | 초록 |
| 대형 | 40×35×30 | 1 | 초록 |
| 특대형 | 50×45×35 | 1 | 초록 |
| 큰 바닥면 | 60×60×42 | 1 | 초록 |
| 한 개로 거의 가득 차는 상자 | 68×68×46 | 2 | 초록 |
| 한계에 가까운 상자 | 70×70×49 | 1 | 초록 |
| 바닥면 초과 | 85×85×25 | 2 | 빨강 |
| 길이 초과 | 105×35×25 | 3 | 빨강 |
| 깊이 초과 | 30×95×25 | 2 | 빨강 |
| 높이 초과 | 40×40×65 | 2 | 빨강 |
| 모든 축 초과 | 90×90×65 | 1 | 빨강 |

크기별 샘플은 작은 상자 더미 옆 남쪽 구역에 14개 묶음으로 모았다.
큰 상자가 아래에 오도록 일부 묶음은 2~3단으로 쌓고, 작은 상자 구역과
샘플 구역 사이에는 약 1.5m 접근 공간을 뒀다. 상자끼리 물리적으로 관통하지 않는다.
선반 뒤 북쪽(Y=5.5~7.8m)과 남쪽(Y=-7.8~-6.0m)은 우회로로 비워둔다.
세 연결 통로와 그 입구에는 상자를 놓지 않는다.
모두 동적 물체이며 개별 충돌 형상과 가정한 질량을 가진다.
`box_samples.json`에 모델 이름, 크기, 위치, 크기 적합 여부와 초과 축을 기록했다.

## 질감과 단색 보존 월드

2026-11-12 사용자 결정으로 기준 월드를 질감 버전으로 바꿨다. 채택 근거는 현실성(무늬 없는 단색 면 대신 바닥·벽·선반·상자 질감)이다.
카메라 기반 비교(C)에서는 질감이 성능을 높인다고 수치로 판정하지 못했다. 그래서 이 월드를 카메라 성능 향상의 근거로 쓰지 않는다.

| 파일 | 내용 | sha256 |
|---|---|---|
| `ax_warehouse.world` | 기준(기본). 단색 월드의 재질 240개에 `<pbr><metal><albedo_map>`만 더한 생성 파일 | `35a8c0ef…` |
| `ax_warehouse_plain.world` | 단색 보존본. 교체 전 기준 월드와 바이트 동일 | `9b5bb281…` |

- **구조:** 대상 재질은 바닥 5·벽 22·랙 기둥 40·선반판 30·운반 상자 120·샘플 23개다.
  geometry·pose·collision·inertial·plugin과 월드 이름 `ax_test`는 두 파일이 같다.
- **단색 보존 이유:** 채택 지도 revision 1을 이 단색 월드에서 만들었다(`map_info.yaml` 월드 해시 `9b5bb281…`).
  구조를 바꿀 때 고치는 원본도 이 파일이다. 질감과 무관한 비교·회귀가 필요할 때 단색 월드로 실행한다.

```bash
ros2 launch ax_description robot_test.launch.py                                          # 기준(질감)
ros2 launch ax_description robot_test.launch.py world:=planned/ax_warehouse_plain.world   # 단색 보존본
```

- **질감:** `textures/`의 JPG 33장과 `textures_manifest.json`, 합계 약 14.8 MB.
  - 외부 이미지 없이 `tools/gen_textures.py`가 절차적으로 만든다(seed 20261112, 바닥·벽 220 px/m, 랙 기둥 512 px/m, 선반판 256 px/m, 상자 512 px, 샘플 256 px, JPG 품질 92).
  - 기존 ambient·diffuse 색에 곱해지는 밝기 변조 이미지라 단색 월드보다 바닥이 약 21% 어둡게 보인다(밝기 보정 안 함).
  - 벽에는 구역 표지(AN-01 등), 상자에는 크기별 라벨(AX-180 등)이 있다.
- **경로:** `albedo_map`은 `model://ax_description/worlds/planned/textures/<파일>`이다.
  - `robot_test.launch.py`가 ax_description share의 상위 폴더를 `GZ_SIM_RESOURCE_PATH`에 넣으므로 설치본에서 풀린다.
  - launch 없이 `gz sim`으로 직접 열면 `GZ_SIM_RESOURCE_PATH`에 `install/ax_description/share`를 넣어야 한다. 넣지 않으면 재질마다 오류를 내고 단색으로 동작한다.
- **재생성:** 구조는 단색 월드에서 고치고 기준 월드를 다시 만든다.
  - `--check`는 재생성 결과 일치·구조 차이 0·질감 파일 해시를 확인한다(`tools/tests/test_textured_world.py`).

  ```bash
  python3 tools/make_textured_world.py           # 단색 월드 → 기준(질감) 월드
  python3 tools/make_textured_world.py --check
  # 질감 이미지까지 다시 만들 때(빈 폴더에 생성한 뒤 textures/로 옮긴다)
  python3 tools/gen_textures.py src/ax_description/worlds/planned/ax_warehouse_plain.world <빈 폴더> \
    --ppm 220 --upright-ppm 512 --deck-ppm 256 --box-px 512 --sample-px 256 --format jpg --jpg-quality 92
  ```

  대상 모델 이름이 바뀌거나 늘면 질감·manifest도 다시 만든다. JPG 바이트는 OpenCV·libjpeg 버전에 따라 달라질 수 있다.
- **실행 확인(2026-11-12, headless, robot_2만):** `simulation.launch.py world:=planned/ax_warehouse.world`와 `world:=planned/ax_warehouse_plain.world` 모두 출발 자세 (−6.8, 0)으로 열렸고 질감 오류가 없었다.
  - 같은 5자세에서 두 월드의 앞·뒤 LiDAR 거리는 같았다(차이 0). 카메라 영상 전체 평균 밝기는 질감 월드가 약 4~10 % 낮았다(10장면, 바닥만 보면 약 21 %). RTF는 약 0.22로 같았다.
  - 근거: `log/world_swap_prep/gz_base2`, `log/world_swap_prep/gz_plain`(작성 PC 기록, Git 제외).
- **LiDAR 영향:** 같은 경로 주행에서 두 월드의 LiDAR 거리가 같았다(차이 0 m, RTF 약 0.22로 같음).
  - 근거: `log/b2_textured_world/20261112_074753/runs/20261112_093103/REPORT.md`. C 회차 1·2에서도 같은 자세 86·92쌍의 최대 차이가 0.0·0.055 mm였다(`log/b2_custom_warehouse/20261112_100617/option_c/REPORT.md`, 작성 PC 기록, Git 제외).
  - 같은 조건 S2 지도 작성의 위치 오차 차이(예: C 회차 1 단색 0.225 m, 질감 0.198 m)는 질감 효과가 아니다. 회차 간 SLAM 변동이다.
- **지도:** 채택 지도 `ax_localization/maps/ax_warehouse`(revision 1)는 단색 월드(`9b5bb281…`)에서 만들었다.
  - LiDAR가 같으므로 기준(질감) 월드에서도 지도 내용은 유효하다. 적용 월드 목록은 `ax_localization/maps/ax_warehouse/applicable_worlds.yaml`이며 `map_info.yaml`(생성 파일)과 `map_revision: 1`은 그대로다.
  - B2 지도 작성 도구(`run_mapping_session.sh --world planned/ax_warehouse.world`)는 이제 질감 월드로 실행하고, 그 월드 해시를 기록한다.
