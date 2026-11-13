# 저장소 파일·경로·오류 점검 — 2026-11-13

기준 main `48a0ea6` 및 기존 W2 미커밋 작업을 보존한 상태에서 점검했다.
Git 추적 파일과 ignore되지 않은 새 파일을 검사했다. build/install/log/가상환경은 소스 중복으로 취급하지 않았다.
패키지·시험·안내 파일 17개를 이동하고 참조 경로를 수정했다. 커밋 전에는 Git 상태에 삭제/새 파일로 표시된다.
`moves.json`에 이전·새 경로와 이동 전후 정규화 SHA-256을 기록했다.

## 이동한 파일

| 이전 위치 | 현재 위치 | 이유 |
|---|---|---|
| `src/ax_safety/src/ax_safety/{CMakeLists.txt,package.xml,include,launch,config,ax_safety,src/a4}` | `src/ax_safety/` 바로 아래 같은 구성 | ZIP 도입 당시 중첩된 ROS 패키지 루트 정리. 12개 패키지 파일 |
| `src/ax_safety/tools/tests/`의 테스트·모의 서버 3개 | `tools/tests/` | 공용 검사 위치로 통일하고 기본 시험 수집에 포함 |
| `src/ax_safety/README_W1_A4_B4_폴더구분_적용가이드.md` | `src/ax_safety/README.md` | 현재 패키지 구조·실행 안내로 정리 |
| 루트 `W1_A4_B4_SETUP.txt` | `docs/guides/W1_A4_B4_SETUP.txt` | 설치 안내를 문서 폴더로 이동 |

현재 ROS package.xml은 13개이며 모두 `src/<package_name>/package.xml` 위치다. 같은 패키지 이름의 중복은 없다.
Python 모듈 `src/ax_safety/ax_safety/`와 C++ 헤더 namespace `include/ax_safety/`는 정상 구조이므로 유지한다.
기존 경로의 Python 캐시 3개는 `log/repo_layout_audit_2026-11-13/old_safety_cache/`에 옮겼고 빈 래퍼 폴더를 제거했다.
과거 검증 문서의 실행 경로는 당시 기록이므로 현재 경로처럼 덮어쓰지 않았다.

## 수정한 오류

- root README·W1 설치 안내·B3/B4 인계 테스트·C++ 시험 명령을 새 경로에 맞췄다.
- 설치 문서의 이전 `su` 브랜치를 `main`으로, 삭제된 `ax_test.world` 안내를 현재 `worlds.yaml` 기준으로 정정했다.
- B4가 빈 CorridorObservationList를 CLEAR·방향 허용으로 처리하는 오류를 수정했다.
  빈 목록은 이제 UNKNOWN·메타 무효·속도 0·전 방향 비허용이다. 수정 전 회귀 검사 1개 실패를 보존했다.
- B4 main의 Ctrl+C/외부 종료 예외와 중복 shutdown 처리를 보완했다. 3개 안전 노드 기동 후 정상 종료를 확인했다.
- A4 명령 확인의 deprecated SharedFuture 암시적 변환을 `.future.share()`로 바꿔 재빌드했다.
  주행/팔 범위·취소·중복 실행 정책을 새로 구현한 것은 아니다.

## 중복·과거 기록

동일 바이트의 비어 있지 않은 파일은 10그룹이었다. 불필요한 동일 소스 코드 그룹은 없었다.

- 실험 이력 4그룹: 다른 회차의 동일 오류 시험 결과, 최종 채택본과 해당 원본 회차의 accuracy/drive/session.
- 라이선스 1그룹: 각 ROS 패키지가 보관하는 같은 라이선스 본문.
- 모델/원본 이력 5그룹: 원본·분리본·보존 버전의 MTL과 모델 manifest.

이 자료는 버전·출처·회차 재현성을 위해 유지했다. 파일명이 같다는 이유로 namespace 파일이나 라이선스를 삭제하지 않았다.

추가로 아래 과거 자료의 형식·링크 제한을 확인했다. 실행 소스 오류와 구분하며 원본 해시는 유지한다.

| 과거 자료 | 확인한 문제·처리 |
|---|---|
| `wheel_slip/attachments/drive_candidate/plan.yaml` | 자유 형식 계획 텍스트여서 YAML 파싱 실패. 당시 manifest와 요약 스크립트가 해시를 참조하므로 원본 보존 |
| `wheel_slip/attachments/robot1/plan.yaml` | 설명 문장 안 콜론 등으로 YAML 파싱 실패. 실행 설정으로 사용하지 않고 원본 계획 기록 보존 |
| `wheel_slip/attachments/candidate_tests/run_conditions.sh` | `<tmp>`, `<workspace>`로 비식별화된 명령 기록이라 bash 문법 검사 실패. 그대로 실행하는 스크립트가 아님 |
| AWS upstream README 이미지 링크 9개 | 출처 보관 subset에 해당 이미지가 없음. pinned 원본을 수정하지 않고 상위 README에 제한을 명시 |

위 wheel_slip 경로는 `docs/evidence/` 기준이다. 자세한 오류 위치는 `repository_scan.json`, shell 결과는 `bash_syntax.json`에 있다.
감사 도구의 `passed`는 현재 실행 소스·패키지 배치 기준이며 과거 자료가 모두 파싱된다는 뜻은 아니다.

## 검증

| 검사 | 결과 |
|---|---|
| 소스·구성 파싱·현재 문서 링크 | Python AST, strict JSON, XML/Xacro/URDF/SDF/world, YAML. 현재 실행 영역 오류·현재 문서 누락 링크 0 |
| ROS 패키지 | 13개, 중복 없음, 모두 표준 루트. 새 WSL 사본에서 rosdep 충족·전체 빌드 성공 |
| 전체 pytest 1차 | 689개 수집: 686 통과, 2 기존 제외, 1 실패 |
| 1차 실패 원인·재검사 | Git 메타데이터 없이 복사한 검증 폴더에서 커밋 ID가 None. 원본 Git 작업공간에서 동일 검사 재실행 통과 |
| 최종 B4 수정 후 | 빈/정상/UNKNOWN/위치 무효 관측, B4 규칙, B3/B4 인계, W3 계약: 24개 통과 |
| C++ A4 단위 | 이동한 헤더·소스 경로로 컴파일·실행 성공 |
| 안전 launch | 기본 잠금 상태의 3개 노드 시작·정상 종료 확인, traceback 없음 |
| Shell·PowerShell | bash 8개 중 현재 실행 스크립트 포함 7개 통과, 과거 placeholder 기록 1개 제외. PowerShell 3개 파싱 오류 0 |

전체 pytest 원본 실패와 원본 작업공간의 성공 재검사를 모두 보존했다. 합쳐 한 번에 전부 통과한 로그로 재구성하지 않았다.
최초 전체 빌드에 있던 A4 deprecated 경고는 해당 패키지 수정 후 재빌드에서 사라졌다.
원본 빌드·pytest·기동/종료·문법 검사 로그는 `log/repo_layout_audit_2026-11-13/`(Git 제외)에 보존했다.

재실행:

```bash
python3 tools/check_repository_layout.py --output log/repository_audit.json
colcon build
source install/setup.bash
python3 -m pytest tools/tests -q
```

커밋 ID를 검사하는 시험은 실제 Git 작업공간에서 실행한다. ROS 시험은 공용 domain 예약 도우미를 사용한다.
이번 전체 빌드는 공백 없는 WSL 내부 소스 사본에서 수행했다.

## 남아 있는 동작 검토 항목

이번 파일 정리·추가 수정이 이전에 확인한 모든 안전 동작 결함을 해결한 것은 아니다.
다음은 이전 재현/코드 검토 항목이며 A4/B4 담당 엄승연과 별도 수정·회귀 확인이 필요하다.

- A4 가속도 제한이 낮아진 안전 속도 상한보다 큰 최종 명령을 만드는 경우.
- 팔 제어기 수락 응답 대기 중 취소하면 늦은 수락을 취소하지 못하는 경합.
- 동일 팔 request_id의 완료 후 재요청을 다시 실행하는 중복 방어 누락.
- 현재 관절 상태부터 첫 궤적 점까지의 속도·가속도 검증 누락.
- A4/B4의 만료 시계가 계약 CLOCK_BOOTTIME과 다른 부분. 실제 절전/복귀 시험은 미수행.

반면 빈 통로 목록 오류는 이번 회차에서 수정했다. 현재 빌드·기존 회귀 통과만으로 위 후속 항목을 완료 처리하지 않는다.
