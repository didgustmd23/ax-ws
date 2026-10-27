# ax-ws 문서 안내

현행 문서 정리 기준: 2026-10-27(작업트리). 구현·시험 기록은 각 문서에 명시된 날짜와 범위를 따른다. 프로젝트 표시명과 GitHub 저장소 이름은 **`ax-ws`**다. ROS 패키지는 `ax_description`, `ax_control`, `robotiq_description`을 유지하고 로봇 모델 이름은 `ax`를 사용한다. 로컬 작업공간 경로 `~/ax_ws`는 유지한다.

## 현재 상태와 문서 선택

- 실행·공개 명령은 [프로젝트 README](../README.md)와 [현재 인터페이스](guides/INTERFACES.md)를 먼저 읽는다.
- ros2_control·개발용 팔 보호는 구현됐지만 **G1은 미충족**이다. 현재 잔여 결함은 [인터페이스의 미해결 항목](guides/INTERFACES.md#알려진-제한과-미해결-항목)에 모은다.
- [통합 상세설계](design/DETAILED_DESIGN.md)는 목표 계약, [팀 공통 일정](management/TEAM_SCHEDULE.md)은 A1~E4 산출물·의존성·단계·검증·현재 기반을 관리한다. 번호별 담당은 확정했고 기간은 미정이다.
- 물리·집게·업로드의 과거 기록은 당시 모델과 조건에 한정된다. 과거 수치나 문서 수정 자체를 현재 시험 통과로 해석하지 않는다.

- 설계는 본설계 하나, 전체 일정·업무 분담은 공통 일정 하나로 관리한다. 채택된 임시 초안은 흡수 후 제거한다.

## 전체 시스템 구성도

[![AX 전체 시스템 — 자동 입고·중앙 계획·D 실행·예비 로봇·통합 관제](assets/diagrams/ax-system.webp)](https://didgustmd23.github.io/ax-ws/overview.html)

[인터랙티브 전체 구성도](https://didgustmd23.github.io/ax-ws/overview.html) · [HTML 내려받기](assets/diagrams/ax-system.html) · [이미지 크게 보기 · WebP](assets/diagrams/ax-system.webp)

## 구성도 모음

이미지를 누르면 GitHub Pages의 인터랙티브 구성도가 열린다. 배포 전에는 HTML을 내려받아 브라우저로 연다. WebP는 투명 배경이며, 아래 파일만 Git·Pages 공유 대상으로 사용한다. `.archify/`의 JSON·작업 원본·글꼴·검증 자료는 제외한다.

| 구성도 | 범위 | HTML | 투명 이미지 | Pages |
|---|---|---|---|---|
| 전체 시스템 | 자동 입고·중앙 계획·D 실행·역할별 예비·통합 관제 | [열기](assets/diagrams/ax-system.html) | [WebP](assets/diagrams/ax-system.webp) | [overview](https://didgustmd23.github.io/ax-ws/overview.html) |
| 현재 로봇 제어 | 팔 명령 보호를 포함한 작업트리 개발 구성 | [열기](assets/diagrams/ax-control.html) | [WebP](assets/diagrams/ax-control.webp) | [internal](https://didgustmd23.github.io/ax-ws/internal.html) |
| 공동 운반 | 입고·적재·배치·재고 갱신·실패 복구·예비 배정 | [열기](assets/diagrams/ax-transfer.html) | [WebP](assets/diagrams/ax-transfer.webp) | [transfer](https://didgustmd23.github.io/ax-ws/transfer.html) |
| 최종 차단 권고 | 중앙 계획→D 검증→A 최종 차단·주행/조작 경로 | [열기](assets/diagrams/ax-safety.html) | [WebP](assets/diagrams/ax-safety.webp) | [safety](https://didgustmd23.github.io/ax-ws/safety.html) |
| 패키지 구성 | A~E·공통 목표 패키지와 현재 구현 구분 | [열기](assets/diagrams/ax-packages.html) | [WebP](assets/diagrams/ax-packages.webp) | [packages](https://didgustmd23.github.io/ax-ws/packages.html) |
| 문서 폴더 | 본설계·팀 공통 일정·실행 계약·공유 자산 | [열기](assets/diagrams/ax-docs.html) | [WebP](assets/diagrams/ax-docs.webp) | [documents](https://didgustmd23.github.io/ax-ws/documents.html) |

구성도는 개인 배정 없이 A~E 기능과 호출·관측 경계를 표시한다.

Pages 링크는 main 반영 및 배포 후 갱신된다. 현재 제어 구조는 미커밋 팔 보호 코드를 포함한 작업트리 설명이며, 고정 커밋의 동작 검증 결과를 뜻하지 않는다.

## 처음 읽는 순서

1. [프로젝트 README](../README.md): 설치·빌드·실행.
2. [업로드 전 점검](validation/PREUPLOAD_REVIEW.md): 점검 결과, 수정 항목, 공개 전 남은 결정.
3. [현재 인터페이스](guides/INTERFACES.md): 링크·관절·토픽·단위·제공 범위.
4. [AX 5축 사양](design/AX_5DOF_ARM_SPEC.md): 적용값과 설계 목표.
5. [충돌·질량 특성 보완 결과](validation/PHYSICAL_PROPERTIES_REVIEW.md): 실제 반영한 근사와 정적 검사 한계.
6. [통합 상세설계](design/DETAILED_DESIGN.md), [팀 공통 일정](management/TEAM_SCHEDULE.md): 전체 목표와 역할별 후속 작업.

## 현재 구현과 사용 방법

| 문서 | 내용 | 읽을 때 구분할 점 |
|---|---|---|
| [README](../README.md) | 작업공간 설치·실행·공통 모델 인계 | 실행 입구 |
| [tools/sim](../tools/sim/README.md) | 변경 유형별 검증, 격리 Gazebo 실행·기본 통신 smoke, 물리 생성 파일 비교 | smoke는 메시지 수신만 확인. 제어기 활성·추종·정지 검증이 아님 |
| [INTERFACES](guides/INTERFACES.md) | 프레임·관절·제어 토픽·피드백 제공 상태 | 현재 코드 인터페이스 |
| [AX_5DOF_ARM_SPEC](design/AX_5DOF_ARM_SPEC.md) | 5축 구조·질량·속도·하중 목표·검증 순서 | 질량·속도는 반영, 2 kg 운반은 미검증 목표 |
| [PHYSICAL_PROPERTIES_REVIEW](validation/PHYSICAL_PROPERTIES_REVIEW.md) | 부품별 충돌 상자·가상 질량 분포·관성·정적 지지 여유 | 동적 전도·충돌 없는 작업 범위의 검증과 다름 |
| [GRIPPER_COMPARISON](guides/GRIPPER_COMPARISON.md) | 집게 4종 선택(RViz 기본 가상 흡착 헤드)과 RViz 개폐 조작, 흡착 구현 제약, EPick 도입 계획 | RViz 모델 비교. Gazebo 흡착·파지 구현 완료가 아님 |
| [GIT_UPLOAD_POLICY](management/GIT_UPLOAD_POLICY.md) | 포함·제외 파일과 확인 방법 | build/install/log 제외, 모델 자산·외부 LICENSE 포함 |
| [PREUPLOAD_REVIEW](validation/PREUPLOAD_REVIEW.md) | 업로드 전 소스·빌드·보안 패턴·문서·라이선스 점검 | 점검 범위와 미확정 사항 포함 |

## 설계·일정·도입 검토

| 문서 | 내용 | 상태 |
|---|---|---|
| [DETAILED_DESIGN](design/DETAILED_DESIGN.md) | 흡착 팔 로봇·적재함 로봇의 협업 그룹, A~E 기능, 상태 머신·통신·검증 목표 | 운영 무인·자동 입고·중앙 전체 계획·D 실행·역할별 예비·인증/관제 목표. 구현 완료 아님 |
| [TEAM_SCHEDULE](management/TEAM_SCHEDULE.md) | 팀 공통 일정, A1~E4·공통 계약·통합/장시간 시험·완료 관문 | 1 이종수·2 양현승·3 김성민·4 엄승연. 기간 미정, 기존 구현 근거와 전체 산정 분리 |
| [EXTERNAL_REPOSITORY_REVIEW](design/EXTERNAL_REPOSITORY_REVIEW.md) | Robotiq·UR5e·ROSbot의 적용 검토 | 도입 당시 비교 자료. Robotiq 모델 일부는 이후 도입됨 |
| [plan.pdf · Git 이력 원문](https://github.com/didgustmd23/ax-ws/blob/560e57e38afae17e1e29b2387b63f60b2ddee219/docs/design/plan.pdf) | 최초 통합 상세설계 원문 | 현재 작업트리에서 삭제 상태, 저장된 커밋 참조 |

## 모델 출처

| 문서 | 용도 |
|---|---|
| [분리 메시 README](../src/ax_description/meshes/parts/README.md) | AX_1·AX_2·상자별 OBJ 분리, 재생성·좌표·원통 분리 규칙 |
| [Robotiq README](../src/robotiq_description/README.md) | 도입 commit·모델 전용 범위·원본 파일 해시·라이선스 |

## 검증 자료를 읽는 방법

- [교체 전 모델의 물리 특성 검사 자료](evidence/physical_properties/): 정적 COM 분포, 수치 JSON, SDF 변환 결과. [분포 그림](evidence/physical_properties/com_projection.png)의 점은 집게 도달 위치가 아니라 전체 무게중심 투영점이다.
- [초기 팔 사양 적용 검사](evidence/model/arm_spec_v01/): 무게중심 보완 전 질량·속도 반영 기록.
- [집게 비교 검사](evidence/grippers/gripper_comparison/), [Robotiq 최초 검사](evidence/grippers/robotiq_preview/): 당시 모델·mimic 검사 범위.
- [업로드 전 점검 자료](evidence/preupload/): 이번 파일 분류·모델 검사 결과.

문서의 설계 목표, 코드 반영, 정적 계산 통과, Gazebo 실행 통과는 서로 다른 상태다. 현재 팔은 전 가동 범위의 하중 운반이나 실제 흡착·파지를 검증한 상태가 아니다. 현재 설정 확인은 소스와 인터페이스·물리 보완 문서를 우선한다.
