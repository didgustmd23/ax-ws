# 실행·검증 도구 (`tools/sim`)

변경 유형별 검증과 격리된 headless Gazebo 실행을 위한 팀 공유 스크립트다. bash·python3·ROS 2 Jazzy만 사용하며 특정 편집기나 AI 도구에 의존하지 않는다. 모든 결과는 실행 폴더 `${AX_RUN_ROOT:-/tmp/ax_runs}/<시각>_<난수>_<종류>/`에 남고 저장소·`docs/evidence/`에는 쓰지 않는다.

| 스크립트 | 용도 |
|---|---|
| `ax_verify.py` | 변경 유형별 검증, `summary.tsv` 생성 |
| `ax_sim.py` | 격리 Gazebo `start`·`status`·`probe`·`stop`·`smoke` |
| `ax_regen_physics.py` | 물리 특성 생성 파일 `check`(쓰기 없음)·`regen`(명시적 교체) |
| `comm_probe.py` | 메시지 수신 확인(`ax_sim.py`가 호출) |
| `owned_procs.sh`, `own_procs.py` | 소유 프로세스 추적·종료 도우미(기존 도우미를 복사해 동작은 그대로 두고 주석·서식만 정리) |
| `test_owned_procs.sh`, `test_proc.py`, `owned_procs_v1_ppid.sh`, `tests/` | 회귀 시험. v1은 비교 시험용 이전 추적기다 |

## 검증

```bash
python3 tools/sim/ax_verify.py                 # auto: 현재 변경으로 종류 선택
python3 tools/sim/ax_verify.py model unit pkg=ax_control
python3 tools/sim/ax_verify.py auto comm-smoke --isolated-build --sim-model both
```

| 종류 | 내용 | 빌드 필요 |
|---|---|---|
| `auto` | staged·unstaged·untracked 변경을 수집하고(이름 변경은 두 경로, 삭제는 옛 경로) `--base REF`면 커밋 차이도 포함해 아래 종류를 선택한다. 분류하지 못한 경로는 INFO로 보고한다. `comm-smoke`는 자동 선택하지 않는다 | |
| `docs` | Markdown 상대 링크·이미지 경로 존재. 앵커는 검사하지 않는다 | |
| `model` | `ax.urdf.xacro` 집게 4종과 `ax_2.urdf.xacro`의 xacro·`check_urdf` | ○ |
| `unit` | `python3 -m unittest discover -s tools/tests` | ○ |
| `physics` | 생성기를 사본 트리에서 실행해 생성 파일 5개와 바이트 비교. 작업 트리에 쓰지 않는다 | |
| `simtools` | 이 폴더의 회귀 시험 | |
| `pkg=NAME,…` | `colcon test`. 결과는 이번 실행의 `--test-result-base`로 한정하며 결과 파일이 0개면 NOT_RUN | ○ |
| `comm-smoke` | 기본 통신 smoke(아래) | ○ |

결과 상태는 PASS·FAIL·BLOCKED(같은 실행의 선행 단계 실패)·NOT_RUN(실행 불가)·INFO(판정 근거 아님)이다. 종료 코드는 0 모든 필수 단계 PASS, 1 FAIL 있음, 2 FAIL은 없으나 BLOCKED·NOT_RUN 있음(미완료), 3 사용법 오류다.

빌드가 필요한 단계는 같은 실행의 `colcon build`와 패키지 경로 확인(`build:env`)이 통과해야 실행한다. 실패하면 BLOCKED이며 이전 `install/`로 대신 실행하지 않는다. 이미 source한 작업 공간 경로는 환경에서 제거한다. `--isolated-build`는 build·install을 실행 폴더 안에 만든다.

`summary.tsv`는 단계별 상태·종료 코드·시간·경고 줄 수·전체 로그 경로를 담는다. 로그는 잘라내지 않으므로 요약을 확인한 뒤 필요한 로그를 직접 열어 검토한다.

## 기본 통신 smoke와 격리 Gazebo

```bash
python3 tools/sim/ax_sim.py smoke --model both
RUN=$(python3 tools/sim/ax_sim.py start --model ax1)   # 실행 폴더 경로 출력
python3 tools/sim/ax_sim.py probe "$RUN"
python3 tools/sim/ax_sim.py stop "$RUN"
```

- 기본 통신 smoke는 메시지 수신만 확인한다. `/clock` 2회 수신·시간 증가, 로봇별 `joint_states`(이름 목록 비어 있지 않음)·`odom` 1회 수신이다. `ax1` 단독은 기존 계약대로 namespace 없음, `ax2`·`both`는 `/ax/robot_N`이다(`simulation.launch.py`의 namespace 규칙과 함께 갱신). 토픽·노드 수는 참고 정보다. **제어기 활성·명령 추종·정지 검증이 아니다.**
- domain은 기본 60~99에서 `$AX_RUN_ROOT/alloc`의 잠금 아래 할당하고, partition은 실행마다 새로 만든다. 102 같은 값도 유효한 ID지만 Linux 기본 설정의 권장 구간(0~101) 밖이다. 중복은 **같은 할당 디렉터리를 공유하는 실행끼리만** 막는다. 다른 `AX_RUN_ROOT`나 이 도구 밖에서 띄운 ROS·Gazebo와의 격리는 보장하지 않는다. 시작 전 같은 domain의 다른 노드를 한 번 찾아 기록하지만 참고용이다.
- 할당 기록은 시작 중(`STARTING`)에 시작 담당 프로세스의 PID·시작 시각을 담으며, 그 프로세스가 살아 있는 동안 회수하지 않는다. 그 밖에는 실행 중인 소유 프로세스가 없을 때만 회수한다. 할당·회수·해제는 같은 잠금 아래에서 처리한다.
- `stop`은 저장된 소유 기록·토큰을 복원해 SIGINT → SIGTERM → SIGKILL 순으로 그 실행의 프로세스에만 신호를 보내고, `own_init`을 다시 호출하지 않는다. 종료 후 남은 좀비(종료됐지만 부모가 회수하지 않은 프로세스)는 실행 중으로 세지 않고 따로 기록한다. 실행 폴더의 로그·환경·`stop.json`은 남긴다. 반복 `stop`은 안전하며 이전 결과를 다시 확인한다. 기존 도우미의 `own_cleanup`은 확인 전에 기록을 지우므로 이 경로에서는 쓰지 않는다.

## 물리 특성 생성 파일

```bash
python3 tools/sim/ax_regen_physics.py check   # 쓰기 없음
python3 tools/sim/ax_regen_physics.py regen   # 작업 트리 교체
```

`tools/generate_physical_properties.py`는 출력 경로 인자가 없고 스크립트 위치 기준 경로에 쓴다. 그래서 생성기 사본을 사본 트리에서 실행한다. 사본 트리의 `meshes`는 생성기가 읽기만 하는 입력 링크이며, 링크가 원본 쓰기를 막아 주지는 않는다. 그래서 메시와 작업 트리 출력이 실행 전후로 바뀌지 않았는지 확인한다. 줄바꿈(CRLF/LF)만 다른 파일은 따로 표시한다.

`regen`은 5개 출력이 모두 생성된 경우에만 교체한다. 기존 파일을 실행 폴더에 백업하고, 같은 폴더에 임시 파일로 준비한 뒤 교체한다. 도중에 실패하면 이미 교체한 파일을 백업으로 되돌리고 임시 파일을 지운다. 결과는 `regen.json`에 남는다.

## 모델 인계 검증과의 차이

`verify_handoff.py`는 제어기·인터페이스·구독자 준비 확인 뒤 AX_1의 팔·손가락 동작과 오차를 검사한다. 기본 통신 smoke의 상위 호환이나 안전 인증 시험이 아니다.

```bash
# 작업공간 setup을 적용한 사용자 터미널에서 실행한다.
python3 tools/verify_handoff.py --dependencies-only
python3 tools/verify_handoff.py --headless --domain-id auto --output-dir /tmp/ax_handoff_review
```

실행마다 새로운 출력 폴더를 지정한다. 이 도구는 Gazebo와 동작 명령을 실행한다. 종료 코드는 0 통과, 1 준비·조회·정리 실패 또는 미완료, 2 동작 후 목표 오차 초과다. `ax_verify.py`의 종료 코드 2(미완료)와 의미가 다르다. 통과해도 G1·하중·흡착 성공을 뜻하지 않는다.

## 회귀 시험

```bash
python3 -m unittest discover -s tools/sim/tests -v
bash tools/sim/test_owned_procs.sh
```
