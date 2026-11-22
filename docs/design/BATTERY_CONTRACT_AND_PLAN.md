# 배터리 기능 공통 계약안 및 구현 계획

상태: 구현 전 제안안 — 임계값, 충전 중 정책, 배터리 전용 사유 코드는 아직 확정하지 않음

> **W2 후속 구현 상태 (2026-11-22)**
>
> 아래 초안의 신규 BatteryState 메시지·토픽 제안은 아직 공통 계약으로
> 확정하지 않았다. 현재 구현은 종수님 시뮬레이터가 사용하는 표준
> `sensor_msgs/BatteryState`와 상대 토픽 `battery_state`를 재사용한다.
> 종수님 구현은 `origin/main`의 `56db574`에 있으나 현재 `feature/sy`에는
> 병합하지 않았으므로, 이 작업만으로 Gazebo 실제 연동 완료로 보지 않는다.
>
> 구현된 범위는 다음과 같다.
>
> - B4 `battery_required=false`는 기존 위험 판단을 그대로 유지한다.
> - `true`일 때 timestamp, stale, `present`, percentage `[0, 1]`, 충전 상태
>   (CHARGING/DISCHARGING/NOT_CHARGING/FULL)를 검증한다.
> - LOW는 22% 이하로 분류하고 상태 변화 시 경고 로그를 남긴다. LOW를
>   감속·정지로 바꾸는 action level은 기본 비활성이다.
> - 누락·stale·NaN·범위 초과·`present=false`·UNKNOWN 충전 상태는 기존
>   `SafetyState`의 UNKNOWN 안전 차단 경로로 보낸다.
> - robot_1과 robot_2는 각 namespace의 상대 토픽과 각 B4 인스턴스의
>   독립 캐시로 분리한다. A4는 기존 SafetyState 게이트를 사용하므로
>   공통 메시지 구조를 변경하지 않았다.
>
> 아직 구현하지 않은 범위는 CRITICAL 10% 기준, LOW의 감속·정지 정책,
> 충전 중 주행·팔 동작 정책, 공식 배터리 reason code, D2의 신규 작업 배정
> 제한, A2의 producer 커밋을 현재 브랜치에 반영하는 작업이다. 이 항목들은
> 팀 합의 또는 담당자 연동 후 별도 변경으로 처리한다.

이 문서는 기존 `StateMeta`, `SafetyState`, `RobotSummary`, `FleetSnapshot`을 재사용해 배터리 기능을 연결하기 위한 구현 계약 초안이다. 배터리 측정의 출처와 안전 판정을 분리한다. A2가 측정값을 만들고, B4가 위험도를 판정하며, A4가 최종 구동·팔 차단을 수행하고, D2가 작업 배정 가능성을 계산한다.

## 1. 계약의 경계

| 구분 | 책임 | 계약상 의미 |
|---|---|---|
| A2 | 배터리 상태 생산·Gazebo 연동 | 측정값, 측정 시각, 충전 상태, 원시 유효성만 생산. LOW/CRITICAL을 결정하지 않음 |
| B4 | 위험도 판정 | 최신 `BatteryState`를 검증하고 LOW/CRITICAL/UNKNOWN을 판정. `SafetyState`에 안전 결과를 반영 |
| A4 | 최종 안전 게이트 | `SafetyState`와 배터리 입력이 유효·최신인지 확인하고 주행·팔 명령을 차단. 정지 확인을 `ActuationState`로 보고 |
| D2 | 배정 제한 | `FleetSnapshot`의 로봇별 배터리 요약을 사용해 새 작업·운송·복구 작업의 배정 가능 여부를 결정 |

`BatteryState`는 안전 허가 메시지가 아니다. 배터리 상태가 정상이어도 `SafetyState`, `ActuationState`, lease, 통로 등의 기존 조건을 우회할 수 없다.

## 2. 배터리 상태 메시지 제안

제안 파일: `src/ax_interfaces/msg/BatteryState.msg`

```text
StateMeta meta                  # 배터리 측정 출처·측정 시각·revision·유효성
string robot_id                 # robot_1 또는 robot_2

# 아래 enum 숫자는 추천안이며 확정 전에는 공개 API로 사용하지 않는다.
uint8 CHARGE_UNKNOWN=0
uint8 DISCHARGING=1
uint8 CHARGING=2
uint8 FULL=3
uint8 charge_state

float32 soc_percent             # 0.0..100.0, 배터리 잔량 추정치
float32 voltage_v               # V, 알 수 없으면 NaN이 아니라 0.0 + validity_reason
float32 current_a               # A, 방전은 음수/충전은 양수 권장. 방향은 결정 필요
float32 temperature_c           # °C, 미지원 시 0.0 + validity_reason
bool charging_cable_connected   # 충전기 연결 감지. 시뮬레이터에서도 명시적으로 설정
uint32 sample_sequence          # A2 생산자 내부 표본 순서
```

### 2.1 필드 규칙

- `meta`는 기존 모든 상태 메시지와 같은 `StateMeta`를 첫 필드로 둔다. `meta.header.stamp`는 측정 시각이며 ROS sim time이다. 수신 시각을 측정 시각으로 덮어쓰지 않는다.
- `robot_id`는 토픽 namespace의 로봇과 반드시 같아야 한다. `robot_1`과 `robot_2`를 한 메시지에 합치지 않는다.
- 잔량의 운영 기준값은 `soc_percent` 하나로 통일한다. 전압을 잔량으로 환산하는 규칙은 A2 내부에 숨기고, B4·D2가 전압을 임의로 잔량으로 재계산하지 않는다.
- 수치 필드는 정상 상태에서 유한한 값이어야 한다. `soc_percent`는 `[0, 100]`, 전압은 `> 0`, 온도는 장치 범위 안이어야 한다. 미지원 값은 NaN/무한대 대신 `meta.valid=false`와 구체적 `validity_reason`으로 표시한다.
- `sample_sequence`는 `StateMeta.revision`과 별개다. `revision`은 계약상 상태 변화 버전이고, `sample_sequence`는 생산자 표본 순서를 추적하는 진단용 값이다.
- `meta.valid=false`, `meta.revision=0`, 유효성 검사 실패, 시계 무효, 또는 수신 측 freshness 만료이면 배터리 상태는 `UNKNOWN`으로 취급한다. 마지막 정상 값을 계속 사용해 허가를 유지하지 않는다.

### 2.2 토픽 및 QoS 제안

| 항목 | 제안 |
|---|---|
| 로봇별 상태 | `/ax/robot_<n>/battery/state` · `BatteryState` |
| 발행자/소비자 | A2 → B4, A4, D2/D4 |
| QoS | 기존 `STATE`: reliable, transient_local, keep_last 1 |
| 발행 주기 | 10 Hz 권장 |
| freshness 초안 | 0.5 s 권장. 최종값은 Gazebo 센서 주기·제어 주기 시험으로 확정 |
| 로봇 범위 | `robot_1`, `robot_2` 각각 독립 topic·독립 revision·독립 latch |

배터리 상태가 오래되면 해당 로봇만 `UNKNOWN` 및 차단 대상으로 삼는다. `robot_1`의 stale/invalid가 `robot_2`의 배터리 상태나 허가를 덮어쓰지 않는다. 단, 두 로봇이 하나의 공동 운반 작업 중이면 D2가 작업 전체를 보류할 수 있다.

## 3. 기존 메시지 재사용과 확장

### 3.1 `SafetyState`

기존 `SafetyState`의 다음 필드를 그대로 사용한다.

- `level`: `CLEAR`, `SLOWDOWN_REQUIRED`, `STOP_REQUIRED`, `UNKNOWN`
- `reason_code`: 판정 근거
- `robot_id`, `meta`, `allow_forward`, `allow_backward`, `allow_rotate`, `max_linear_speed`, `max_angular_speed`
- `sensor_valid` 등 기존 안전 입력 유효성 필드

B4는 배터리 상태를 별도 입력으로 받아 다음 결과를 기존 안전 결과에 합성한다.

| 배터리 판정 | `SafetyState.level` 추천 | A4 허가 추천 |
|---|---:|---|
| 정상·최신 | 기존 환경 안전 판정 유지 | 기존 조건을 계속 적용 |
| LOW | 기존 정책에 따라 CLEAR 또는 SLOWDOWN_REQUIRED | 새 작업은 금지하고, 진행 중 동작은 결정된 감속/안전 지점 정책 적용 |
| CRITICAL | `STOP_REQUIRED` | 주행·팔 모두 false, 정지 확인까지 유지 |
| UNKNOWN(누락·stale·invalid) | `UNKNOWN` 또는 `STOP_REQUIRED` | 안전 보수 정책상 주행·팔 false |

LOW가 곧바로 `STOP_REQUIRED`인지, UNKNOWN을 `STOP_REQUIRED`로 승격할지는 시스템 위험 분석으로 결정해야 한다. A4는 어떤 경우에도 최신 유효 배터리 근거 없이 허가를 유지하지 않는다.

### 3.2 `RobotSummary`와 `FleetSnapshot`

상세 상태는 `BatteryState` 토픽이 authoritative source이고, 중앙 배정용 요약만 기존 `RobotSummary`에 추가한다.

```text
StateMeta battery_meta        # 배터리 요약의 출처·시각·revision·유효성
float32 battery_soc_percent   # BatteryState.soc_percent 복사값
uint8 battery_charge_state    # BatteryState.charge_state 복사값
uint8 battery_risk_level      # 제안: UNKNOWN/NORMAL/LOW/CRITICAL
bool battery_task_allowed     # D2가 새 작업에 사용할 수 있는지
```

위 필드는 제안이며 기존 `RobotSummary`의 영역별 meta 규칙을 따른다. `FleetSnapshot.meta`가 유효해도 `battery_meta`가 stale/invalid이면 배터리 영역만 UNKNOWN이다. D2는 누락을 정상·가용으로 채우지 않고 `missing_sources`에 `battery:<robot_id>`를 기록하는 방식을 권장한다.

`RobotSummary`에 새 필드를 추가하면 `ax_interfaces`의 메시지 계약 버전과 관련 snapshot schema를 함께 올려야 한다. 필드가 확정되기 전에는 임시로 D2가 `BatteryState`를 직접 구독할 수 있으나, 중앙 snapshot의 배정 근거에는 최종적으로 요약을 포함해야 한다.

## 4. LOW/CRITICAL 및 충전 정책: 추천안과 결정 항목

### 4.1 추천 초기값

| 항목 | 추천안 | 결정 필요 |
|---|---|---|
| LOW 진입 | `soc_percent <= 20%`를 3회 연속 관측 | 실제 배터리 모델·작업 시간·안전 여유 검증 |
| CRITICAL 진입 | `soc_percent <= 10%` 또는 A2가 저전압 보호 신호를 보고 | 전압 기준, 시뮬레이터 모델, 보호 신호 정의 |
| 히스테리시스 | LOW 해제 `>= 25%`, CRITICAL 해제 `>= 20%`를 3회 연속 관측 | 충전기 연결 여부와 함께 쓸지 결정 |
| LOW 동작 | 신규 장거리·고부하·팔 작업 배정 금지, 현재 원자적 동작 완료 후 안전 지점 대기/충전 이동만 허용 | “현재 작업 완료”의 경계와 충전 지점 경로 |
| CRITICAL 동작 | A4가 주행·팔을 즉시 차단하고 실제 정지 확인. 팔은 안전 자세로의 제한된 retract만 별도 허용 가능 | retract를 정지 전에 허용할지, 안전 자세 정의 |
| UNKNOWN 동작 | 해당 로봇 주행·팔 차단, D2 신규 배정 금지 | 통신 일시 장애에 대한 별도 복구 시간 허용 여부 |
| 충전 중 | 기본 추천은 주행·팔 금지, 충전 도킹/해제는 명시적 상태 전이로 처리 | Gazebo에서 케이블 연결 중 이동을 허용할지, 도킹 동작 예외 |
| 자동 복구 | 자동으로 `CLEAR`로 복귀하지 않음. 최신 상태, 충전 상태, 정지 확인, 원인 해소, D4 재배정/재개 조건을 모두 확인 후 해제 | 사람 승인 필요 여부와 D4/A4 해제 주체 |

임계값은 전압을 단순 병행 OR 조건으로 넣기 전에 실제 배터리 모델에서 `soc_percent`와 일관되는지 검증한다. `LOW`·`CRITICAL`의 숫자와 enum 값은 이 문서만으로 확정하지 않는다.

### 4.2 배터리 reason code 후보

기존 D-19 안전 사유 목록에 아래 후보를 추가하는 방안을 추천하지만, 이름과 우선순위는 결정이 필요하다.

`BATTERY_LOW`, `BATTERY_CRITICAL`, `BATTERY_STATE_STALE`, `BATTERY_STATE_INVALID`, `BATTERY_CHARGING_LOCK`, `BATTERY_RECOVERY_PENDING`

동시에 여러 사유가 있으면 가장 보수적인 level을 선택하고, 기존 `SafetyState.reason_code`가 단일 문자열인 한 대표 사유만 넣는다. 상세 원인은 별도 진단 로그/실패 사건에 남긴다. 새로운 enum을 `SafetyState.level`에 추가하지 않고 기존 4개 level을 유지하는 것을 추천한다.

## 5. 정지와 복구 상태 흐름

```text
NORMAL/LOW
  -> CRITICAL 또는 UNKNOWN
  -> B4 SafetyState(STOP_REQUIRED/UNKNOWN)
  -> A4 drive_allowed=false, arm_allowed=false
  -> ActuationState STOP_REQUESTED -> STOPPING -> STOPPED
  -> D2 해당 작업 보류·새 배정 차단
  -> 충전/상태 복구 + 최신성 확인 + 원인 해소
  -> A4 ClearFault/재개 조건 확인 + D2 재배정 세대 갱신
  -> 정상 작업 재개
```

- `STOP_REQUIRED` 발행만으로 실제 정지가 완료됐다고 간주하지 않는다. A4는 기존 `ActuationState`의 `STOPPED`와 정지 조건을 확인한다.
- 정지 중에는 Nav2 목표 취소, 속도 0 출력, 보호 팔 명령 차단을 기존 A4 경로로 처리한다. 배터리 기능이 제어기 입력을 직접 우회하지 않는다.
- 복구 시 이전 task/lease를 무조건 재개하지 않는다. D2/D4가 assignment generation, 작업 보류, 재개 전 조건을 다시 확인한다. 공동 운반이면 두 로봇의 배터리 상태를 모두 확인한다.
- 배터리 위험으로 인한 정지는 환경 장애물 정지와 같은 기존 안전 정지 우선순위를 따른다. 배터리 복구가 다른 안전 잠금을 해제하지 않는다.

## 6. 담당자별 작업 계획

### A2 현승님 — 배터리 상태 생산 및 Gazebo 연동

1. `BatteryState.msg` 확정안에 맞춰 Gazebo 배터리 플러그인/모델 입력을 정의한다.
2. `robot_1`, `robot_2` 각각 `/ax/robot_<n>/battery/state`를 발행하고 `robot_id`·namespace 일치 검사를 넣는다.
3. sim time, `StateMeta` session/revision, 표본 sequence, 충전 연결 상태를 구현한다.
4. 정상·충전·저잔량·저전압·센서 누락·시계 무효·생산자 재시작 fixture를 만든다.
5. 수치 범위/유한성 검증과 producer unit test, 두 로봇 독립성 ROS test를 추가한다.

완료 기준: 각 로봇이 10 Hz 권장 주기로 최신 상태를 발행하고, stale/invalid를 정상 잔량으로 위장하지 않으며, Gazebo에서 충전 상태 전이가 재현된다.

### B4 승연 — 배터리 위험도 판단 및 `SafetyState` 반영

1. `BatteryState` 입력 검증기와 freshness 판정기를 기존 B4 관측 검증 구조에 추가한다.
2. LOW/CRITICAL/UNKNOWN 규칙, 히스테리시스, 연속 샘플 조건을 설정 파일로 분리한다. 숫자는 결정 전 parameter로 둔다.
3. 기존 환경·위치 안전 판정과 배터리 결과를 합성해 `SafetyState`를 발행한다.
4. reason code 후보를 단일 대표 사유로 선택하는 우선순위와 상세 진단 기록을 구현한다.
5. 두 로봇 입력을 섞지 않는 테스트와 stale/invalid/복구/동시 장애 테스트를 추가한다.

완료 기준: 배터리 위험 시 기존 `SafetyState` 주기·QoS·`StateMeta` 규칙을 지키고, 오래된 값으로 CLEAR를 발행하지 않는다.

### A4 승연 — 위험 시 주행·팔 동작 차단

1. A4 safety gate가 `SafetyState`의 배터리 reason/level과 freshness를 기존 차단 조건으로 처리하도록 연결한다.
2. 위험 시 주행 속도 0, 방향 허가 false, `arm_allowed=false`를 보장한다.
3. `STOP_REQUESTED → STOPPING → STOPPED` 피드백과 실제 정지 확인을 배터리 시나리오로 검증한다.
4. 충전 도킹/안전 retract 예외가 승인되기 전까지는 일반 주행·팔 명령과 분리해 잠근다.
5. 복구 시 자동 CLEAR를 허용하지 않고 기존 `ClearFault`·재개 조건·다른 차단 사유 유지 규칙을 적용한다.

완료 기준: robot_1 위험이 robot_2 A4 게이트에 영향을 주지 않고, 위험 로봇은 stale/invalid 상태에서도 허가를 내지 않으며, 정지 확인 전 재개되지 않는다.

### D2 현승님 — 배터리 부족 시 작업 배정 제한

1. `RobotSummary`에 배터리 요약 필드를 추가할지 결정 후 `FleetSnapshot` 생성·검증을 반영한다.
2. LOW 로봇의 신규 장거리/고부하/팔 작업 제한, CRITICAL/UNKNOWN 로봇의 신규 작업 금지를 배정 규칙에 추가한다.
3. 공동 운반은 양쪽 로봇의 배터리 조건을 모두 만족해야 하며, 한쪽 위험이면 그룹 작업을 보류한다.
4. 배터리 stale/invalid를 `missing_sources`와 배정 거절 사유로 남긴다.
5. 복구 후 새 assignment generation, 작업 재개 조건, 예비 로봇 재배정을 기존 D4 원장과 연결한다.

완료 기준: D2가 배터리 상태를 정상으로 추측해 배정하지 않고, LOW/CRITICAL/UNKNOWN/복구 각 상태에서 배정 결과와 근거를 재현할 수 있다.

## 7. 구현 순서와 통합 시험

1. 계약 결정: 임계값·히스테리시스·충전 중 동작·reason code·`RobotSummary` 확장 여부를 확정한다.
2. A2가 메시지와 Gazebo producer를 먼저 구현하고, 고정 fixture를 제공한다.
3. B4가 순수 위험 규칙과 `SafetyState` 통합을 구현한다.
4. A4가 위험 입력에 대한 최종 차단·정지 확인·복구 잠금을 구현한다.
5. D2가 snapshot·배정 제한을 연결한다.
6. 두 로봇 동시 시험과 기존 안전 회귀 시험을 수행한다.

필수 시나리오:

- robot_1만 LOW/CRITICAL, robot_2는 정상
- 두 로봇의 배터리 상태를 서로 다른 속도로 발행
- 배터리 Topic 중단, `meta.valid=false`, revision 역행, sim clock 무효
- LOW 경계 진입·회복 히스테리시스
- CRITICAL 중 주행 중 정지와 팔 작업 중 정지
- 충전 연결·충전 중 이동 명령·충전 완료
- 정지 확인 전 재개 요청, 다른 안전 사유가 남은 상태의 복구 요청
- 공동 운반 중 한 로봇만 위험, 예비 로봇 재배정 및 assignment generation 갱신

## 8. 확정이 필요한 결정 목록

| ID | 결정 |
|---|---|
| BAT-DEC-01 | `soc_percent` 기준 LOW/CRITICAL 수치와 진입·해제 연속 샘플 수 |
| BAT-DEC-02 | 전압·온도·저전압 보호 신호를 위험 판정에 포함할지 |
| BAT-DEC-03 | LOW에서 현재 작업을 어느 안전 경계까지 허용할지 |
| BAT-DEC-04 | CRITICAL 시 안전 자세 retract를 허용할지, 즉시 팔 정지할지 |
| BAT-DEC-05 | 충전 케이블 연결 중 주행·팔 동작을 허용할지 |
| BAT-DEC-06 | 배터리 stale/invalid의 안전 level을 `UNKNOWN` 또는 `STOP_REQUIRED` 중 어느 쪽으로 강제할지 |
| BAT-DEC-07 | 새 배터리 reason code의 공식 명칭·우선순위 |
| BAT-DEC-08 | `RobotSummary`에 배터리 요약 필드를 추가할지와 snapshot schema version |
| BAT-DEC-09 | 복구를 완전 자동으로 할지, 운영자 승인 또는 D4 승인 단계를 둘지 |

이 결정이 끝나기 전에는 enum 숫자나 임계값을 코드에 하드코딩하지 않고, 구현 중에는 명시적 parameter와 `TBD` 테스트 fixture로 유지한다.
