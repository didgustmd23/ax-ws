# ax-ws 문서 안내

정리 기준: 2026-10-22. 프로젝트 표시명과 GitHub 저장소 이름은 **`ax-ws`**다. ROS 패키지는 `ax_description`, `ax_control`, `robotiq_description`을 유지하고 로봇 모델 이름은 `ax`를 사용한다. 로컬 작업공간 경로 `~/ax_ws`는 유지한다.

## 폴더 분류

```text
docs/
├── README.md                 문서 안내
├── design/                   통합 설계·팔 사양·외부 저장소 검토·원문 PDF
├── guides/                   인터페이스·집게 사용법
├── management/               A 담당 일정·Git 업로드 정책
├── validation/               물리 특성·업로드 전 점검 보고서
└── evidence/                 검증 원본 자료
    ├── README.md             검증 자료 안내
    ├── physical_properties/  무게중심 그림·정적 계산·SDF 검사
    ├── preupload/            업로드 전 파일·모델 검사
    ├── model/                팔·기둥·초기 사양 검사
    ├── grippers/             집게 비교·Robotiq 검사
    └── archive/              이전 모델의 날짜별 실행 기록
```

## 처음 읽는 순서

1. [프로젝트 README](../README.md): 설치·빌드·실행.
2. [업로드 전 점검](validation/PREUPLOAD_REVIEW.md): 점검 결과, 수정 항목, 공개 전 남은 결정.
3. [현재 인터페이스](guides/INTERFACES.md): 링크·관절·토픽·단위·제공 범위.
4. [AX 5축 사양](design/AX_5DOF_ARM_SPEC.md): 적용값과 설계 목표.
5. [충돌·질량 특성 보완 결과](validation/PHYSICAL_PROPERTIES_REVIEW.md): 실제 반영한 근사와 정적 검사 한계.
6. [통합 상세설계](design/DETAILED_DESIGN.md), [A 일정](management/A_DETAILED_SCHEDULE.md): 전체 목표와 역할별 후속 작업.

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
| [DETAILED_DESIGN](design/DETAILED_DESIGN.md) | 흡착 팔 로봇·적재함 로봇의 협업 그룹, A/B/C/D 역할, 상태 머신·통신·검증 목표 | 원문과 2026-10-22 결정 기반 목표 설계. 현재 구현 전체를 뜻하지 않음 |
| [A_DETAILED_SCHEDULE](management/A_DETAILED_SCHEDULE.md) | A 담당 16주 상대 일정, 협업·완료 관문 | 계획 문서. 시작 당시 상태와 현재 진행 상태를 구분 |
| [EXTERNAL_REPOSITORY_REVIEW](design/EXTERNAL_REPOSITORY_REVIEW.md) | Robotiq·UR5e·ROSbot의 적용 검토 | 도입 당시 비교 자료. Robotiq 모델 일부는 이후 도입됨 |
| [plan.pdf](design/plan.pdf) | 최초 통합 상세설계 원문 | 원문 보관 |

## 모델 출처

| 문서 | 용도 |
|---|---|
| [분리 메시 README](../src/ax_description/meshes/parts/README.md) | AX_1·AX_2·상자별 OBJ 분리, 재생성·좌표·원통 분리 규칙 |
| [Robotiq README](../src/robotiq_description/README.md) | 도입 commit·모델 전용 범위·원본 파일 해시·라이선스 |

## 검증 자료를 읽는 방법

- [최신 물리 특성 검사 자료](evidence/physical_properties/): 정적 COM 분포, 수치 JSON, SDF 변환 결과. [분포 그림](evidence/physical_properties/com_projection.png)의 점은 집게 도달 위치가 아니라 전체 무게중심 투영점이다.
- [초기 팔 사양 적용 검사](evidence/model/arm_spec_v01/): 무게중심 보완 전 질량·속도 반영 기록.
- [집게 비교 검사](evidence/grippers/gripper_comparison/), [Robotiq 최초 검사](evidence/grippers/robotiq_preview/): 당시 모델·mimic 검사 범위.
- [업로드 전 점검 자료](evidence/preupload/): 이번 파일 분류·모델 검사 결과.

문서의 설계 목표, 코드 반영, 정적 계산 통과, Gazebo 실행 통과는 서로 다른 상태다. 현재 팔은 전 가동 범위의 하중 운반이나 실제 흡착·파지를 검증한 상태가 아니다. 현재 설정 확인은 소스와 인터페이스·물리 보완 문서를 우선한다.
