# W1 A4·B4 통합 검증 기록

## 범위

- 대상 브랜치: `feature/sy`
- 기준 커밋: `06f6308` (`origin/main` fast-forward 병합 후)
- 기존 기록: `W1_A4_B4_VERIFICATION_2026-11-13.md`는 변경하지 않음
- 기존 W1 설치 안내, README, 테스트 결과와 로그는 보존함

## Git 통합 결과

- 작업 시작 시 `feature/sy`의 미커밋 변경은 없었음
- `git fetch --prune origin`으로 원격 브랜치 갱신
- 최신 `origin/main`을 `feature/sy`에 fast-forward 병합
- 충돌 없음
- `origin/feature/sy`의 W1 A4·B4 커밋은 이미 `origin/main`에 포함되어 있었음

## 팀원 변경 확인

- `origin/main`에서 반영된 변경: 월드 선택·지도 검증(B2/B3), 로봇 테스트 런치와 관련 문서·테스트
- `origin/GK`에만 존재하는 A3 후보: `c126807 feat(a3): validate runtime assignment goal and lease authority`
- `c126807`은 `origin/main`에 포함되지 않았으며, 브랜치가 명확하므로 이번 통합에는 포함하지 않음
- `origin/feature/robot-model-control`의 최신 커밋은 `origin/main` 병합 이력에 포함됨

## 새로 실행한 검증

환경: Windows 11 계열 작업 폴더, Python 3.14 venv. `pytest`, `numpy`, `PyYAML`, `tzdata`, `scipy`를 venv에 설치함. ROS 2/ Gazebo 도구는 이 환경에 없음.

### 통과

```text
src/ax_safety/tools/tests/test_b4_risk_rules.py: 11 passed
```

### 부분 실행 결과

```text
tools/tests/test_map_accuracy_world.py
tools/tests/test_map_adopt_gate.py
src/ax_safety/tools/tests/test_b4_risk_rules.py
21 passed, 12 failed, 3 skipped, 13 subtests passed
```

남은 실패는 다음 환경 제약으로 재현됨.

- `cv2` 미설치로 지도 격자 비교 일부가 import 실패
- 테스트가 POSIX 경로(`/w/ws`, `/w`)를 전제로 하므로 Windows `Path` 비교에서 portable path 검증 실패
- `test_w1_models.py`와 월드 런치 검증은 ROS Python 패키지 `xacro`, `rclpy`가 없어 수집·실행 불가
- `desktop_session.py` 및 셸 기반 테스트는 Windows에 `fcntl`, `bash`가 없어 실행 불가
- `colcon`과 `cmake`도 현재 환경에 없어 ROS 빌드 미실행

기존 W1 실행 결과와 Ubuntu 24.04/ROS 2 Jazzy/Gazebo Harmonic 빌드·런치 결과는 기존 검증 기록에 남아 있으며, 이번 보고서는 그 파일을 덮어쓰지 않음.
