# ax-ws GitHub 업로드 전 점검

점검일: 2026-10-22. 프로젝트 표시명·저장소 이름: **`ax-ws`**. 로컬 작업공간은 `~/ax_ws`, ROS 모델 이름은 `ax`이며 패키지 이름은 기존 값을 유지한다. GitHub 저장소 생성·Git 초기화·커밋·업로드는 수행하지 않았다.

## 점검 결론

기존 build/install을 사용하지 않는 별도 `/tmp` 경로의 새 빌드에서 3개 패키지가 통과했다. 집게 4종의 URDF·SDF 변환과 메시 참조 확인도 통과했다. 자동 검사에서 발견한 스타일 오류와 잘못된 문서 링크는 수정했다.

**공개 전 자체 코드의 라이선스 결정이 남아 있다.** XML 스키마 검사는 환경의 외부 파일 접근 제한으로 완료하지 못했다. 전체 자동 테스트 통과 또는 Gazebo 구동·파지·하중 안정성 검증 완료로 보고하지 않는다.

## 확인 결과

| 항목 | 결과 | 범위 |
|---|---|---|
| 별도 경로 새 빌드 | 통과 | `robotiq_description`, `ax_description`, `ax_control` |
| Python 구문 | 통과 | 업로드 후보의 Python 파일 |
| 코드 스타일 | 수정 후 통과 | ax_control의 flake8·pep257, ax_description의 flake8·pep257·CMake 검사 |
| 패키지 XML 스키마 | 환경 제한으로 실패 | 외부 `http://download.ros.org/schema/package_format3.xsd` 로드 불가. 검사 무시 설정은 추가하지 않음 |
| 저작권 헤더 테스트 | 기존 skip 유지 | ax_control에 기본 생성된 skip 표시가 있음. 통과로 집계하지 않음 |
| 집게별 모델 | 4종 통과 | Xacro·check_urdf·URDF→SDF, 설치된 메시 파일 존재 |
| ROS launch 인자 | 확인 | display의 집게 선택 인자 4종, 기본 robotiq |
| SDF 경고 | 기존 센서 확장 경고 | `gz_frame_id` 확장 경고가 있으나 변환 성공 |
| 호스트 고정 경로 | 실행 소스에서 발견 없음 | 패키지 공유 경로 사용. 과거 기록의 `/home/yhs`는 실행 의존성이 아님 |
| `/tmp/ax.urdf` 공유 | 기존 제한 확인 | 동시 Gazebo 실행 시 충돌할 수 있음. 단일 실행 전제 유지 |
| 제외 파일 | 확인 | build/install/log, 개인 설정·인증정보 경로가 업로드 후보에서 제외됨 |
| 파일 크기 | 100 MiB 초과 없음 | 최대 파일은 원본 AX_1.obj 약 7.49 MiB |
| 비밀정보 패턴 | 탐지 없음 | 제외 규칙 적용 후 UTF-8 텍스트 후보의 개인 키·주요 토큰 패턴 검사. 모든 비밀정보 부재를 보증하는 검사는 아님 |
| 외부 파일 출처 | 23개 해시 일치 | Robotiq 고정 commit의 Git blob SHA와 비교, LICENSE·출처 파일 유지 |
| Markdown 링크 | 수정 후 확인 | 로컬 링크 대상 존재 검사. 외부 웹 링크와 제목 앵커의 유효성은 이번 범위 밖 |

`colcon test-result`는 XML 검사 실패를 CTest 요약과 하위 결과에 중복 표기하여 실패 2개로 집계했다. 실제 실패 원인은 같은 스키마 로드 문제 1건이다. 네트워크가 가능한 개발 환경에서 기본 `colcon test`를 다시 실행해야 한다.

## 이번에 수정한 항목

- 프로젝트 소개를 `ax-ws`로 정리하고 [문서 안내](../README.md)를 추가했다. 파일 이동·삭제 없이 현재 문서와 과거 기록의 용도를 표시했다.
- 상세설계의 원문 PDF 링크를 실제 위치 `docs/design/plan.pdf`로 수정했다.
- launch·제어 Python의 import 순서, 줄 길이, 들여쓰기와 불필요한 공백을 수정했다. 명령 토픽과 제어 동작은 유지했다.
- `/cmd_vel` 브리지가 사용하는 `geometry_msgs` 실행 의존성을 ax_control에 추가했다.
- 패키지 설명의 오래된 primitive 표기와 setup.py의 설명 TODO를 수정했다.
- 계산 도구에 필요한 numpy·scipy·matplotlib 설치 항목을 프로젝트 README에 추가했다.

## 공개 전에 결정할 항목

| 항목 | 현재 상태 | 필요한 결정 |
|---|---|---|
| 자체 코드 라이선스 | ax_control의 package.xml·setup.py에 `TODO: License declaration`, 루트 공통 LICENSE 없음 | 권리자가 적용할 라이선스를 결정한 뒤 관련 메타데이터·LICENSE 정리. 이번 점검에서 임의 지정하지 않음 |
| 기존 라이선스 일관성 | ax_description은 Apache-2.0 선언과 LICENSE, 외부 Robotiq은 BSD-3-Clause | 자체 코드의 범위를 정리하고 외부 BSD 고지 유지 |
| 유지관리자 연락처 | package.xml과 setup.py에 `yhs@todo.todo` 예시 주소 | 공개할 연락처로 교체할지 결정 |

## 현재 기능의 실제 한계

- 로봇팔의 가속도·TCP 속도는 계획값이며 현재 Float64 브리지가 강제하지 않는다.
- 실제 관절 상태 피드백, 동적 하중·토크·PID·차체 전도 검증은 후속 작업이다.
- RViz 집게 4종 비교와 Gazebo에서의 실제 파지 기능은 별개다. 현재 Gazebo 실행은 primitive 구성이다.
- 정적 COM 계산에서는 일부 미필터링 자세가 지지 영역 밖으로 나간다. 현재 전 가동 범위에서 2 kg 운반을 보장하지 않는다.
- GUI 표시·노드 간 통신·Gazebo 물리 실행은 이번 업로드 점검에서 새로 검증하지 않았다.

이 항목은 소스 공유 자체의 실패가 아니라 개발 진행 상태다. README와 인계 시 완료 기능으로 표현하지 않는다.

## 업로드 파일과 재검사

보관·제외 기준은 [GIT_UPLOAD_POLICY](../management/GIT_UPLOAD_POLICY.md)를 따른다. 빌드 산출물은 넣지 않고 소스·메시·재질·지도·문서·선별한 검증 자료·외부 라이선스를 포함한다.

이번 점검에서는 실제 프로젝트 Git 인덱스를 만들지 않고 `/tmp`의 임시 Git 메타데이터로 `.gitignore` 적용 후 후보를 확인했다. 파일별 점검 결과는 [파일 검사 기록](../evidence/preupload/file_checks.json), 모델별 결과는 [모델 검사 기록](../evidence/preupload/model_checks.json)에 보관한다. 문서나 파일을 추가하면 후보 수와 크기는 달라진다.

네트워크와 ROS 환경을 갖춘 PC에서 다음 검사를 실행한다.

```bash
cd ~/ax_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ax_description ax_control
source install/setup.bash
colcon test --packages-select ax_description ax_control
colcon test-result --verbose
```

프로젝트 상태 요약과 문서 선택은 [문서 안내](../README.md), 실제 물리 근사와 검사 한계는 [물리 특성 보완 결과](PHYSICAL_PROPERTIES_REVIEW.md)를 참고한다.
