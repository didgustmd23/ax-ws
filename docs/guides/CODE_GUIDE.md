# 팀원용 코드 읽는 순서

## 실행 경로

1. `remote_controller/install.cmd` → `install-shortcuts.ps1`: PC별 경로 탐색과 바로가기 생성.
2. `remote_controller/ax-launch.ps1`: Windows 창을 열고 Ubuntu 세션 관리자를 호출.
3. `tools/desktop_session.py`: 소유 프로세스 확인, 중복 실행 잠금, 시작·종료·준비 확인.
4. `src/ax_control/launch/simulation.launch.py`: A2 제어 진입점. 아래 launch를 포함하고 로봇별 제어기 spawner·흡착 공구 노드를 띄운다.
   제어기 설정은 `src/ax_control/config`(`common_controllers.yaml` → `ax1_controllers.yaml`·`ax2_controllers.yaml`)가 단일 기준이다.
5. `src/ax_description/launch/robot_test.launch.py`: Xacro 전개, 로봇 생성, 센서 브리지 연결. 단독 실행 시 제어기 없이 모델·센서만 띄운다.
6. `tools/remote_control.html` → `remote_control.py` → `keyboard_teleop.py`: 브라우저 입력 → HTTP 큐 → ROS 제어.

Start는 Ubuntu/Gazebo, Remote는 리모컨, Stop은 해당 AX 세션과 전용 창만 다룬다.
전체 WSL 재시작은 확인창이 있는 `remote_controller/recover.ps1`에만 있다.
일반 PowerShell·브라우저·다른 workspace의 프로세스를 일괄 종료하지 않는다.

## A3 모의 실행기 코드·실행 경로 (김성민 W0)

- `src/ax_task_executor/setup.py`: 두 노드의 `console_scripts` 등록.
- `src/ax_task_executor/ax_task_executor/mock_executor.py`: `CONFIRM_LOAD` 요청 검사 → 진행 피드백 → 시험 결과 반환 → `/a3_mock/result_display` 발행.
- `src/ax_task_executor/ax_task_executor/gazebo_mock_display.py`: 결과 Topic 구독 → Gazebo 서비스 호출 → 정적 표시등 생성.

Ubuntu에서 `ax_task_executor`까지 빌드하고 설치 환경을 source한 뒤, 기존 시뮬레이션과 별도로 각 터미널에서 실행한다.

```bash
ros2 run ax_task_executor gazebo_mock_display
ros2 run ax_task_executor mock_executor --ros-args -p mock_load:=EMPTY
```

요청 경로는 계약 경로 `/ax/robot_2/executor/execute_local_task`다(상태는 `MOCK_ONLY`). Gazebo와 표시 노드는 같은 `GZ_PARTITION`을 사용해야 한다.
환경 설정·빌드·Goal 전송은 [A3 패키지 안내](../../src/ax_task_executor/README.md),
계약 검토 범위와 미구현 항목은 [A3 K0 검토 기록](A3_K0_REVIEW.md)을 따른다.

## 모델 수정 경로

| 수정 대상 | 먼저 읽을 파일 | 생성/소비 경로 |
|---|---|---|
| AX_1 링크·관절·바퀴 | `urdf/ax.urdf.xacro` | `ax_1.urdf`는 호환용 생성물 |
| AX_2 링크·적재함 | `urdf/ax_2.urdf.xacro` | 생성된 치수·물리 Xacro를 포함 |
| V2 원점·가상 질량 | `config/ax1_asset_v2.yaml` | `tools/generate_asset_v2.py` → V2 메시·manifest |
| 충돌·질량·관성 | `tools/generate_physical_properties.py` | 물리 Xacro·JSON 보고서 |
| 관절 제한 | `config/arm_joint_limits.yaml` | URDF와 시험·키보드 도구에서 공통 사용 |
| 센서 장착 | `config/sensor_mounts.yaml` | `urdf/sensors.xacro`에서 사용 |
| 다중 흡착판 · 12컵·TCP | `config/epick.yaml`, `urdf/epick_adapter.xacro` | 사진 기반 자체 형상. epick은 호환 이름 |
| 통합 시험 배치 | `worlds/ax_test.world` | 로봇 생성은 월드가 아니라 launch가 담당 |

표의 `urdf/`, `config/`, `worlds/`는 `src/ax_description/` 기준이다.
생성 파일을 직접 수정하면 다음 생성 때 사라진다. 원본을 수정하고 생성기와 검사를 함께 실행한다.
설정 파일 주석이 바뀌어도 파일 바이트 해시는 달라지므로 manifest 갱신이 필요할 수 있다.

## 좌표·시간·명령의 의미

- `origin xyz/rpy`는 부모 링크 또는 해당 요소가 속한 링크 기준이다. 바닥 기준 좌표로 해석하지 않는다.
- 길이 m, 질량 kg, 관절각 rad, 선속도 m/s, 각속도 rad/s, 가속도 rad/s², 회전 effort N·m.
- 원본/V2 OBJ 정점은 mm다. URDF의 `scale=0.001`로 m로 변환한다.
- base_link 방향은 +X 전방, +Y 왼쪽, +Z 위쪽이다.
- `/clock`은 시뮬레이션 진행/메시지 stamp에, `time.monotonic()`은 실제 대기 제한·연결 만료에 사용한다.
- HTTP 202, Action 수락, 실제 관절 이동 성공은 각각 다른 상태다.
- 키보드/리모컨의 팔 각도 설정은 1회 이동량이다. 실시간 충돌 회피·손가락 개폐는 구현돼 있지 않다.
- 충돌 선별·하중 치구·TCP 시험은 지정된 모델 조건의 검사다. 실물 정격이나 흡착 성공을 뜻하지 않는다.

## 주석을 직접 넣지 않는 파일

- JSON에는 주석 문법이 없으므로 [설정 설명](../../src/ax_description/config/README.md)을 따른다.
- `physical_properties.xacro`, `ax_2_geometry.xacro`, `ax_2_physical_properties.xacro`, `ax_1.urdf`는 생성물이다.
- 외부 `epick_description`/`robotiq_description`의 원본·활성 매크로는 해시/라이선스 추적을 위해 유지한다.
- 기존 `docs/evidence/`는 과거 실행 기록이다. 주석 보강만으로 과거 실험을 다시 수행했다고 기록하지 않는다.
- 공유 설계도 HTML·메시·라이선스·공유 인터페이스의 기존 상세 설명은 해당 원본과 계약을 따른다.

## 확인

ROS 환경을 불러온 workspace 루트에서:

```bash
python3 tools/generate_asset_v2.py --check
python3 tools/check_portability.py
```

프로세스 종료 시험은 자신이 만든 시험 프로세스만 정리하도록 구성돼 있다.
실제 Gazebo 전체 회귀 도구는 별도의 시험 세션을 실행하므로 단순 문서 확인과 구분한다.

A2 제어 시험 도구는 `src/ax_control/scripts/check_*.py`이며 실행 중인 격리 시뮬레이션에 붙어 동작한다.
사용법·판정 범위는 [ax_control 안내](../../src/ax_control/README.md#개발-시험)를 따른다.
Gazebo 독립 관측(링크 자세 → 관절각·TCP·물체 자세)은 `gazebo_observer.py` 하나에 두고, 순수 계산은 `tools/tests/test_gazebo_observer.py`가 Gazebo 없이 검사한다.
결과 파일 쓰기(임시 파일 교체 저장)는 `result_record.py`에 두고, 도구별 결과 기록은 `tools/tests/test_stop_feedback.py`·`test_tool_attach.py`가 검사한다.
