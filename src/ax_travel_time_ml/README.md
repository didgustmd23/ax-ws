# ax_travel_time_ml — D ETA 이동시간 예측(W7 개발, 오프라인 도구)

[통합 상세설계](../../docs/design/DETAILED_DESIGN.md) 4.4절 "D: 이동시간 예측"의 학습·평가 도구다.
D2 원장 `eta_samples` 표를 `tools/export_d2_eta_samples.py`로 내보낸 CSV에서 계층형 기준선과
XGBoost 회귀 모델을 학습·평가한다. ROS 노드와 D2 배정 연동은 아직 없다. 이 도구의 예측을 배정
판단에 쓴다고 간주하지 않는다.

## 현재 상태와 제한

- **LIVE 표본이 없다.** 성능·합격 기준·하이퍼파라미터는 미정이다. 지금까지의 학습·평가는 합성 데이터의
  코드 경로 확인뿐이다. **합성 결과는 AX 주행 성능의 근거가 아니다**(`meta.json`의 `performance_evidence: false`).
- 설계 입력 중 경로 길이·회전 수·통로 대기열·장애물 밀도(`path_length_m`, `turn_count`, `corridor_wait_s`,
  `obstacle_density`)는 원장에서 현재 항상 NULL이다. 학습 세트에서 전부 결측인 열은 자동으로 빠지고
  `features.dropped_all_null`에 기록된다. 값이 생기면 재학습할 때 들어간다.
- 목표값은 `duration_sim_s`(sim 시간, 전송→종료)다. 학습에는 `SUCCEEDED` 표본만 쓴다. 실패·시간초과
  표본(중도절단)을 빼서 생기는 낙관 편향을 어떻게 반영할지는 미정이다.
- "전체 작업 완료시간"은 원장에 조작 단계 시간이 없어 계산하지 않는다. 작업별 주행 소요 합의
  오차(`*_task_sum`)를 대리 지표로 낸다.
- 첫 구간(출발지로 가는 주행)은 원장에 출발 지점·자세가 없다. 그래서 `from_location_id`가
  `UNKNOWN_ORIGIN`이고 직선거리 특징은 결측이다.

## 특징

| 구분 | 특징 |
|---|---|
| 지금 쓸 수 있음 | `role`, `robot_id`, `from_location_id`(NULL은 `UNKNOWN_ORIGIN`), `to_location_id`, `object_state`(범주, one-hot), `first_leg`, `carrying_object`, `attempt_index` |
| 파생(조건부) | `straight_distance_m`: `--catalog` 지점 카탈로그의 역할별 정지 자세 사이 직선거리. 출발지 NULL·지점·revision 불일치면 결측. `prev_nav_duration_s`: 같은 작업·같은 로봇의 앞서 끝난 주행 소요. sim 시각은 세션마다 다시 시작하므로 다른 작업과는 비교하지 않는다 |
| 현재 항상 NULL | `path_length_m`, `turn_count`, `corridor_wait_s`, `obstacle_density` |

모델 사용 전제는 `role`, `to_location_id`, `carrying_object`이다. 하나라도 없으면 기준선으로 복귀한다.

## 기준선("기존 방식")

설계 4.4절의 "예측 오류·입력 결측 시 기존 방식으로 복귀"에서 "기존 방식"을 정의한 곳이 저장소에 없다.
그래서 아래 계층형 기준선 B1~B5를 그 정의로 쓴다. 위에서부터 처음 성공한 단계의 값을 쓴다.

| 단계 | 방식 |
|---|---|
| B1 | (role, from, to, carrying)별 학습 소요 중앙값. 표본 `--min-n`(기본 5) 이상일 때 |
| B2 | (role, to, carrying)별 중앙값. 표본 `--min-n` 이상일 때 |
| B3 | 직선거리 d / v_eff + t0(학습 세트 최소제곱). 표본이 부족하거나 값이 비물리적이면 v=0.67 m/s, t0=1.0 s 하한 근사를 쓰고 `B3_LOWER_BOUND_ONLY`로 표시 |
| B4 | role별 중앙값. 표본 `--min-n` 이상일 때 |
| B5 | UNKNOWN(값 없음, `NO_BASELINE`). 호출자는 ETA 없이 판단한다 |

거리를 알면 모든 예측을 하한 d/0.67 + 1.0 s 이상으로 맞춘다(`CLAMPED_TO_LOWER_BOUND`). 0.67 m/s·0.67 m/s²는
`ax_control` diff_drive 직선 제한이다. 이 하한은 주행 시간 추정이 아니다.

## 모델 사용과 복귀

`EtaPredictor.predict(row, prev_nav_duration_s)`는 `EtaResult(eta_s, source, reasons)`를 돌려준다.
`source`는 `MODEL`, `BASELINE_B1`~`B4`, `UNKNOWN` 중 하나다. 아래 경우에는 모델을 쓰지 않고 사유를 남긴다.

- `MODEL_NOT_LOADED:<원인>`: 모델 없음(`UNAVAILABLE`), xgboost 없음(`XGBOOST_MISSING`), 주버전 불일치,
  파일 손상(`LOAD_FAILED`)
- `MODEL_NOT_ADOPTED`: 보류 세트 MAE가 기준선보다 작지 않았다
- `SYNTHETIC_MODEL`: 합성 데이터로 학습한 모델(`allow_synthetic=True` 시험에서만 사용)
- `CATALOG_MISMATCH`, `MAP_REVISION_UNSEEN`: 학습 때와 다른 지점 카탈로그(파일 SHA-256)·지도 revision
- `MISSING_FEATURE:*`, `UNSEEN_CATEGORY:*`: 필수 특징 결측·학습에 없던 범주
- `MODEL_OUTPUT_INVALID`: NaN·무한·0 이하 출력

## 사용법

```bash
colcon build --packages-select ax_travel_time_ml && source install/setup.bash

# 원장 → CSV(기본: DEVELOPMENT_MIXED·LIVE·SUCCEEDED)
python3 tools/export_d2_eta_samples.py <원장.sqlite> -o /tmp/eta.csv

# 학습(task_id 단위 분할로 평가 포함). 출력 폴더에 meta.json, model.json
ros2 run ax_travel_time_ml ax_eta_train --csv /tmp/eta.csv \
    --catalog src/ax_fleet_manager/config/dev_intake.json --out /tmp/eta_model

# 별도 CSV로 평가(기준선·모델·실제 예측 경로 지표)
ros2 run ax_travel_time_ml ax_eta_evaluate --csv /tmp/eta_new.csv --model-dir /tmp/eta_model \
    --catalog src/ax_fleet_manager/config/dev_intake.json

# 합성 데이터(코드 시험 전용, mode·provenance=SYNTHETIC)
ros2 run ax_travel_time_ml ax_eta_synth --out /tmp/syn.csv --catalog-out /tmp/syn_catalog.json
ros2 run ax_travel_time_ml ax_eta_train --csv /tmp/syn.csv --catalog /tmp/syn_catalog.json \
    --out /tmp/syn_model --synthetic
```

`ax_eta_train`의 종료 코드는 다음과 같다.

| 코드 | 의미 |
|---|---|
| 0 | 모델 학습 |
| 2 | 입력 오류 |
| 3 | 작업 수 10 미만(`INSUFFICIENT_DATA`, 지표와 meta를 만들지 않음) |
| 4 | xgboost 없이 기준선만 저장 |

## 재현성·산출물

- 모델은 XGBoost 네이티브 JSON(`model.json`)으로 저장한다. pickle은 쓰지 않는다. sklearn 래퍼 대신
  `xgboost.train`·`Booster`를 써서 scikit-learn 의존이 없다.
- 매개변수는 `model.py`의 `DEFAULT_PARAMS`로 고정한다(200회, 깊이 4, eta 0.1, subsample 0.8, hist).
  `nthread=1`과 `seed`로 같은 입력이면 `model.json` 바이트가 같다(시험 `test_eta_ml_xgboost.py`).
- `meta.json`에는 다음이 들어간다.
  - 원본 CSV SHA-256과 필터·학습·평가 분할 정규화 SHA-256(행 순서 무관)
  - 필터·제외 사유, seed, 분할(평가 작업 목록 해시)
  - 카탈로그 SHA-256·map_revision, 특징 열 순서·범주
  - 기준선 통계, 매개변수, xgboost 버전, git commit
  - 지표, `adopted`, `performance_evidence`
- 학습 산출물은 실행마다 지정한 폴더에 쓴다. 저장소에 커밋하지 않는다(합성 모델은 특히 커밋하지 않는다).

## 의존성

- xgboost: Ubuntu 24.04 apt `python3-xgboost`(2.0.3)를 쓴다. 설치는 사용자가 직접 한다(`sudo apt install python3-xgboost`).
  rosdep 키가 있는지 확인하지 못해(rosdep 미초기화) `package.xml`에 넣지 않았다. 확인한 뒤 `exec_depend`로 추가한다.
- xgboost가 없어도 기준선 학습·예측은 동작한다(`ax_eta_train` 종료 코드 4).

## 시험

```bash
.venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
    tools/tests/test_eta_ml_dataset.py tools/tests/test_eta_ml_baseline.py \
    tools/tests/test_eta_ml_predict.py tools/tests/test_eta_ml_xgboost.py
```

- 각 시험 파일이 확인하는 것:
  - `test_eta_ml_dataset.py`: 원장 열 계약(`task_store.ETA_SAMPLE_FIELDS`)과 export 스크립트 CSV 왕복을 읽기 전용으로 확인
  - `test_eta_ml_xgboost.py`: xgboost가 없으면 import에서 실패한다(skip하지 않는다)
- 합성 데이터에서 모델이 기준선보다 낫다는 시험은 코드 경로 확인이다. 성능 근거가 아니다.
