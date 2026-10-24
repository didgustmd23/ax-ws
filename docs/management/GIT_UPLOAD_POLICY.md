# AX 프로젝트 Git 업로드 파일 분류

작성 기준: 2026-10-21, `/home/yhs/ax_ws`의 현재 파일 구성.

Git에는 로봇을 다시 빌드·실행하는 데 필요한 소스와 모델, 팀 공유 문서를 보관한다. 빌드 결과·개인 환경·인증정보·임시 파일은 제외한다. 이 기준을 반영한 [루트 .gitignore](../../.gitignore)를 추가했다. 이번 작업에서는 파일 삭제·Git 초기화·커밋·원격 업로드를 수행하지 않았다.

## 1. Git에 올리지 않는 파일

### 1.1 빌드 결과와 실행 산출물

| 현재 경로 | 제외 이유 | 필요한 경우의 보관 방법 |
|---|---|---|
| `build/` | colcon/CMake 빌드 중간 파일, 캐시, 로컬 절대 경로 포함 | 소스에서 재빌드 |
| `install/` | 설치 산출물·로컬 경로·심볼릭 링크 포함 | 대상 PC에서 colcon 빌드 |
| `log/` | 실행·빌드마다 생성되는 로그 | 필요한 결과만 검토 후 `docs/evidence/`에 선별 보관 |
| `legacy_before_rename/` | 이름 변경 전 build/install/log 보관본, 이전 경로·링크 포함 | 필요하면 별도 백업으로 관리 |

현재 관측 용량은 각각 약 1.4 MiB, 728 KiB, 1.7 MiB, 4.2 MiB다. 용량보다 재현성과 로컬 경로 의존성 때문에 제외한다.

`install/`을 공유하는 대신 다음처럼 소스에서 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ax_description ax_control
source install/setup.bash
```

### 1.2 Python 캐시·편집기 임시 파일

| 파일 종류 | 현재 확인한 예 | 처리 |
|---|---|---|
| `__pycache__/`, `*.pyc`, `*.pyo` | `src/ax_control/launch/__pycache__/` 등 Python 캐시 6개 | 제외 |
| `*.bak` | `src/ax_description/urdf/ax.urdf.xacro.bak` | 제외, 변경 이력은 Git으로 관리 |
| `*:Zone.Identifier` | `src/ax_description/urdf/structure_demo.urdf:Zone.Identifier` | Windows 다운로드 부가 정보이므로 제외 |
| `*.swp`, `*.swo`, `*~` | 이후 편집 과정에서 생성될 수 있음 | 제외 |
| `.DS_Store`, `Thumbs.db` | 이후 OS 사용 중 생성될 수 있음 | 제외 |
| `.pytest_cache/`, `.mypy_cache/`, `.ruff_cache/`, `.coverage*`, `htmlcov/` | 이후 검사 실행 시 생성될 수 있음 | 제외 |
| `.venv/`, `venv/`, `*.egg-info/` | 로컬 Python 환경·생성 메타데이터 | 제외, 의존성 정의는 별도 관리 |

위 예시는 조사 당시 상태다. 파일명 목록에 없는 항목도 같은 성격이면 동일 기준을 적용한다.

### 1.3 인증정보와 개인 환경

| 경로·종류 | 기준 |
|---|---|
| `.aws/` | 인증정보·개인 클라우드 설정을 업로드하지 않음 |
| 현재 환경의 `.codex/`, `.agents/` | 개인 도구·실행 환경용 디렉터리로 기본 제외 |
| `CLAUDE.md`, `.claude/` | Claude 하네스(작업 지침·스킬·에이전트·개인 설정)는 로컬 전용으로 커밋하지 않음 |
| `.env`, `.env.*` | 토큰·비밀번호·개인 접속 정보가 들어가는 실제 환경 파일 제외 |
| `*.pem`, `*.key` | 키 파일 기본 제외. 공유가 필요한 공개 인증서는 별도 경로·정책으로 구분 |
| GitHub 토큰, API 키, SSH 개인키, 클라우드 키 | 파일 위치·확장자와 무관하게 업로드 금지 |

현재 `.aws/`, `.codex/`, `.agents/`는 관리 환경의 경로로 보이며 내부 인증정보는 열람하지 않았다. 현재 공유 가능한 소스·문서 영역에 대해 제한적인 비밀정보 패턴 검사를 수행했으며 일치 파일은 없었다. 이 검사가 모든 형태의 비밀정보 부재를 보장하지는 않는다.

`.env.example`, `.env.sample`, `.env.template`은 가짜 값·빈 값만 담은 템플릿일 때 포함할 수 있도록 예외를 두었다. 같은 이름이라도 실제 키가 있으면 포함하면 안 된다.

Claude 하네스 파일은 팀 공용 지침으로도 Git에 올리지 않는다. 공유가 필요한 프로젝트 규칙은 README·설계·인터페이스 문서의 관련 절에 반영한다.

## 2. 조건을 확인한 뒤 올리는 파일

| 경로·종류 | 현재 상태 | 업로드 기준 |
|---|---|---|
| `docs/evidence/` | 검증 JSON, 실행 로그, URDF 스냅샷, pose·토픽 기록 약 0.4 MiB | 문서의 검증 근거로 필요한 파일만 포함. 개인 경로·호스트명·IP·키 등 점검 |
| `docs/design/plan.pdf` | 설계 원문 약 178 KiB | 팀 공유 권한이 있는 문서라면 포함 |
| `meshes/AX_1.obj`, `AX_1.mtl` | 모델 원본, OBJ 약 7.5 MiB | 원본 출처·공유 권한을 확인하고 모델 이력 관리에 필요하면 포함 |
| 영상·스크린샷·대형 실험 데이터 | 현재 별도 대형 영상 없음 | 필요한 대표 결과만 선별. 큰 원시 데이터는 별도 저장소 고려 |
| rosbag 원시 기록 | 현재 기본 경로에 없음 | 향후 `bags/`, `recordings/`는 기본 제외. 필요한 작은 샘플은 위치·공개 범위를 명시 |
| IDE 설정 | 현재 별도 설정 없음 | 팀 공용 tasks·포맷 설정은 포함 가능, 개인 경로·접속 설정은 제외 |

`docs/evidence/`를 통째로 제외하면 [물리 특성 보완 결과](../validation/PHYSICAL_PROPERTIES_REVIEW.md), [집게 비교](../guides/GRIPPER_COMPARISON.md), [업로드 전 점검](../validation/PREUPLOAD_REVIEW.md)의 증거 링크가 깨진다. `.gitignore`에는 `*.log`나 `docs/evidence/` 전체 제외 규칙을 넣지 않았다. 검증 기록을 줄일 때는 연결된 문서도 함께 갱신한다.

원시 실행 로그를 매번 자동 커밋하는 방식도 피한다. 재현 명령·설정·판정 결과를 중심으로 보관하고 원본 로그는 필요한 검증 사례에 한해 선택한다.

## 3. Git에 포함해야 하는 파일

| 경로·종류 | 포함 이유 |
|---|---|
| `src/ax_description/`의 Xacro·URDF·launch·설정 | 로봇 모델·좌표·실행 구성 |
| `src/ax_control/`의 Python·launch·패키지 정의 | 제어·브리지 구현과 실행 의존성 |
| `src/robotiq_description/` | 현재 Robotiq 미리보기의 로컬 의존 패키지 |
| `.obj`, `.mtl`, `.dae`, `.stl` | 로봇 시각·충돌 모델의 실제 입력 자산 |
| `meshes/parts/`의 OBJ·MTL | 현재 URDF가 직접 참조하는 분리 모델 |
| `manifest.json`, `column_split_manifest.json` | 모델 분리 결과와 출처·대응 관계 |
| `src/robotiq_description/LICENSE` | 외부 모델의 BSD-3-Clause 라이선스 |
| `src/robotiq_description/UPSTREAM_FILES.json` | 외부 원본 commit·파일 SHA 기록 |
| `src/robotiq_description/upstream/*.original` | 가져온 외부 패키지의 원본 메타데이터·설치 규칙 보존 |
| `package.xml`, `CMakeLists.txt`, `setup.py`, `setup.cfg`, `resource/` | ROS 패키지 빌드·설치·검색에 필요 |
| `maps/warehouse_map.yaml`, `maps/warehouse_map.pgm` | 지도 메타데이터와 실제 지도 이미지. 한 쌍으로 관리 |
| `tools/verify_handoff.py` | 검증 재현 도구 |
| `README.md`, `docs/**/*.md`, 검토된 증거 | 팀 공유·설계·실행·검증 기록 |
| `.gitignore` | 생성 파일·개인 환경 제외 규칙 공유 |

### 이 프로젝트에서 특히 주의할 확장자

- **`*.obj`를 전역 제외하지 않는다.** 일반 C/C++ 프로젝트의 오브젝트 파일 제외 예시를 복사하면 AX의 3D 모델까지 누락된다.
- **`*.mtl`을 제외하지 않는다.** OBJ가 참조하는 재질이다. 예: `parts/ax_column_colors.mtl`.
- **`*.dae`, `*.stl`을 제외하지 않는다.** Robotiq 미리보기의 시각·충돌 메시다.
- **`*.original`을 일괄 제외하지 않는다.** 현재 Robotiq 패키지의 원본 보존 파일에 사용한다.
- **`*.json`, `*.yaml`, `*.pgm`을 일괄 제외하지 않는다.** 메타데이터·설정·지도까지 누락된다.
- **`*.log`를 일괄 제외하지 않는다.** 일반 로그는 `/log/`로 제외하고 검증 근거 로그는 선별 보관한다.

현재 가장 큰 단일 파일은 `AX_1.obj` 약 7.5 MiB다. 파일이 크다는 이유만으로 현재 URDF의 필수 자산을 제외하지 않는다. 원본 모델과 파생 모델의 관계는 manifest·README로 설명한다.

## 4. .gitignore 적용 범위

추가한 루트 규칙은 다음 성격으로 구성했다.

별도의 임시 Git 저장소에서 제외 대상 16개·유지 대상 17개 경로를 `git check-ignore`로 확인했다. 현재 프로젝트의 Git 메타데이터나 추적 상태를 변경한 검증은 아니다.

```text
제외: /build/, /install/, /log/, /legacy_before_rename/
제외: Python 캐시, 백업·편집기 임시 파일, Zone.Identifier
제외: 개인 도구 상태, 실제 .env, 인증 키
제외: 향후 /bags/, /recordings/ 원시 기록
유지: 로봇 모델·재질·지도·라이선스·출처·선별 검증 기록
```

루트의 `.git/`는 Git 자체의 관리 데이터로 일반 프로젝트 파일처럼 커밋하지 않는다. `.gitignore`로 소스 파일과 같이 관리할 대상이 아니다.

이 작업 환경에서는 `git rev-parse --is-inside-work-tree`가 저장소를 인식하지 못했다. 따라서 이미 추적 중인 파일·staging 상태·원격 저장소의 공개 범위는 확인하지 못했다. `.gitignore`가 추가되었다고 기존 Git 이력이나 이미 추적 중인 파일이 정리된 것은 아니다.

## 5. 실제 커밋 전 확인 방법

아래 명령은 유효한 Git 저장소가 있는 사용자 터미널에서 실행한다.

```bash
# 변경·추가 파일 확인
git status --short

# 어떤 규칙으로 제외되는지 확인
git check-ignore -v build/ install/ log/ legacy_before_rename/

# 이미 Git이 추적하지만 현재 ignore 규칙에 해당하는 파일 확인
git ls-files -ci --exclude-standard

# 커밋 후보의 파일 목록과 변경량 확인
git diff --cached --name-status
git diff --cached --stat

# 모델 파일이 실수로 제외되지 않았는지 확인
git check-ignore -v src/ax_description/meshes/parts/AX_1/ax_body.obj
git check-ignore -v src/robotiq_description/meshes/visual/2f_85/robotiq_base.dae
```

마지막 두 명령은 해당 모델이 제외되지 않으면 출력이 없고 종료 코드가 1인 것이 정상이다.

이미 추적 중인 제외 대상은 `.gitignore`만으로 추적이 해제되지 않는다. 확인한 경로에 한해 `git rm --cached`를 사용하여 추적만 해제하고 실제 파일은 남길 수 있다. 이 문서 작성 과정에서는 실행하지 않았다. 인증정보가 과거 커밋에 있었다면 ignore 추가만으로 해결되지 않으므로 해당 인증정보를 교체하고 이력을 별도로 처리한다.

`git add .` 전에 후보 목록을 확인한다. 소스·문서 영역에도 임시 파일이나 개인정보가 섞일 수 있으므로 디렉터리 이름만으로 안전하다고 판단하지 않는다.
