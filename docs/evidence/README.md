# 검증 자료 분류

보고서는 [validation](../validation/), 설계 목표는 [design](../design/), 전체 문서 안내는 [docs README](../README.md)를 참고한다.

| 폴더 | 자료 | 해석 범위 |
|---|---|---|
| [physical_properties](physical_properties/) | `com_projection.png`, `static_stability.json`, `sdf_checks.json` | 가상 질량 분포의 정적 계산과 모델 변환. 실제 파지·동적 전도 검증이 아님 |
| [preupload](preupload/) | `file_checks.json`, `model_checks.json` | 업로드 전 검사 시점의 파일·모델 상태. 최신 파일 목록을 자동 추적하는 자료가 아님 |
| [model/arm_spec_v01](model/arm_spec_v01/) | 초기 질량·속도 적용 검사 | 무게중심 보완 전 기록 |
| [model/arm_mesh_mount](model/arm_mesh_mount/) | 팔 메시 장착 검사 | 해당 장착 단계의 형상·변환 검사 |
| [model/column_yaw](model/column_yaw/) | 원통 회전 최초 분리 검사 | 상단 회전을 별도로 두었던 과거 구성 |
| [model/single_column_yaw](model/single_column_yaw/) | 원통 단일 회전 검사 | 중복 상단 yaw 제거 당시 검사 |
| [grippers/gripper_comparison](grippers/gripper_comparison/) | 집게 4종 비교 검사 | URDF 구조·개구 폭·mimic 출력 |
| [grippers/robotiq_preview](grippers/robotiq_preview/) | Robotiq 장착·메시·mimic 검사 | 최초 모델 통합 시점의 검사 |
| [archive](archive/) | `20261015_*`별 URDF·로그·pose·토픽·결과 JSON | 이전 ax1/ax 모델 실행 기록. 현재 형상·질량·하중 성능의 증거로 사용하지 않음 |

원본 PDF와 검증 자료의 내용은 폴더 정리 과정에서 변경하지 않았다. 과거 로그·JSON 안의 절대 경로나 출력 경로는 기록 당시 값이며, 현재 실행 경로로 해석하지 않는다.

무게중심 계산 코드의 출력 경로는 기존 `docs/evidence/physical_properties/`를 유지한다. 새 실행 로그는 작업 목적에 맞는 하위 폴더에 저장하고 보고서에 실행 조건과 검증 범위를 함께 적는다.
