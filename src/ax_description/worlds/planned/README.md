# 두 창고 운반 시험 월드

기본 실행 월드는 질감 버전 `ax_warehouse.world`이며 `ax_warehouse_plain.world`는 같은 구조의 단색 원본이다.
월드 이름은 `ax_test`다. 2026-11-15 요청에 따라 기존 일반 상자 120개와 나머지 샘플 21개를 제거해 **상자 총 2개**만 남겼다.

| 색 | 모델 이름 | 크기(X×Y×Z) | 중심 위치(X,Y,Z), m |
|---|---|---|---|
| 초록 | `sample_fit_medium_01` | 30×25×24 cm | (-11.2, -5.1, 0.122) |
| 빨강 | `sample_reject_wide_01` | 85×85×25 cm | (-11.2, -3.8, 0.127) |

두 상자는 기존 지면 위치의 동적 물체로 개별 충돌 형상·질량을 유지한다.
초록은 빈 적재판 72×72×50cm에 개별 크기가 들어가고, 빨강은 폭·깊이가 초과한다는 시험용 표시다.
무게·집기 성공이나 자동 인식 결과를 뜻하지 않는다. `box_samples.json`에 현재 두 상자만 기록한다.

벽·선반·바닥·3개 통로·로봇 출발 위치는 유지한다. 출발 창고는 X -22~-4m, 도착 창고는 X 4~22m다.
통로는 길이 8m·유효 폭 1.8m, 중심 Y 약 -5.02/0/5.08m다.
로봇 1~4 출발 좌표는 (-10,-2.4), (-6.8,0), (-17,-2.4), (-13,-2.4)m다.

실행 중인 Gazebo에는 파일 수정이 자동 적용되지 않는다. ax_description을 다시 빌드하고 월드를 재시작한다.

```bash
colcon build --packages-select ax_description
source install/setup.bash
ros2 launch ax_control simulation.launch.py
```

채택 지도 revision 1은 이전 상자 배치에서 만든 결과다. 제거한 상자의 점유가 남을 수 있으므로 현재 월드와 동일한 지도라고 간주하지 않는다.
지도 재생성·평가·채택은 B2 절차로 별도 진행하며 기존 지도나 과거 증거는 덮어쓰지 않는다.

## 질감과 단색 보존 월드

2026-11-12 사용자 결정으로 기준 월드를 질감 버전으로 바꿨다. 채택 근거는 현실성(무늬 없는 단색 면 대신 바닥·벽·선반·상자 질감)이다.
카메라 기반 비교(C)에서는 질감이 성능을 높인다고 수치로 판정하지 못했다. 그래서 이 월드를 카메라 성능 향상의 근거로 쓰지 않는다.

| 파일 | 내용 | sha256 |
|---|---|---|
| `ax_warehouse.world` | 기준(기본). 단색 월드의 재질 99개에 `<pbr><metal><albedo_map>`만 더한 생성 파일 | `6c78d56c…` |
| `ax_warehouse_plain.world` | 단색 보존본. 같은 두 상자 배치의 단색 원본 | `9b5bb281…` |

- **구조:** 대상 재질은 바닥 5·벽 22·랙 기둥 40·선반판 30·운반 상자 0·샘플 2개다.
  geometry·pose·collision·inertial·plugin과 월드 이름 `ax_test`는 두 파일이 같다.
- **단색 보존 이유:** 채택 지도 revision 1은 상자를 제거하기 전 단색 월드에서 만들었다(`map_info.yaml` 월드 해시 `9b5bb281…`).
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
