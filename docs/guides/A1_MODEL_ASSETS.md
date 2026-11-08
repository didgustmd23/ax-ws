# A1 모델 자산·EPick·관절 검증

이 문서는 README가 연결한 공통 일정의 A1 산출물 중 메시 버전, EPick description 선별 도입,
모델·관절 검증의 구현 범위를 기록한다. A2의 실제 흡착 부착/해제나 A4 안전 게이트 구현 완료를 뜻하지 않는다.

## 메시 자산 버전

`meshes/parts/AX_1/` 원본 CAD는 보존한다. `tools/generate_asset_v2.py`가
`config/ax1_asset_v2.yaml`의 보정된 링크 원점·방향을 적용해
`meshes/versions/AX_1_v2/`에 11개 OBJ를 생성한다.

기구 치수를 임의로 바꾸지 않고, 수정된 관절축·받침 배치를 링크 로컬 좌표에 반영한 버전이다.
입출력 단위는 mm이고 URDF scale은 0.001이다. 정점·법선을 변환하고 면 연결·재질은 보존한다.
manifest에 원본/출력 SHA-256, 면·정점 수, 왕복 오차, 관절 프레임과 제한 파일 해시를 남긴다.
URDF visual 원점은 0으로 통일하고 collision·관성 생성기도 같은 로컬 메시를 읽는다.
V2 차체 collision은 60개다. 이전 58개 버전과 개수 자체를 완료 기준으로 삼지 않고
메시 전체 정점 포함과 양의 관성·좌표 일치를 검사한다.

`config/arm_joint_limits.yaml`이 위치·속도·가속도·effort의 개발용 공통 기준이다.
URDF의 팔 limit은 이 파일을 읽으며 자산 manifest도 해당 해시를 참조한다.

```bash
python3 tools/generate_asset_v2.py
python3 tools/generate_physical_properties.py
python3 tools/generate_asset_v2.py --check
python3 tools/generate_physical_properties.py --check
```

관절 원점·제한·메시를 바꾸면 위 생성과 검증을 함께 실행한다.

## EPick 선별 도입

원본: [PickNikRobotics/ros2_epick_gripper](https://github.com/PickNikRobotics/ros2_epick_gripper/tree/d07391307d2d16cacf88daa168ce02839589397e).
고정 commit은 `d07391307d2d16cacf88daa168ce02839589397e`, 라이선스는 BSD-3-Clause다.
`src/epick_description/`에 본체 STL, description helper, 원본 사본·라이선스·파일 해시를 보존한다.
원본에는 제품 관성이 없어 본체와 컵에 명시적인 가상 질량·관성을 추가했다.

기본 `tool:=epick`은 사용자 참고 사진을 바탕으로 만든 하우징·주름관·12컵 헤드를 생성한다. 전체 색상은 단일 짙은 회색이다. EPick 제품 CAD를 그대로 재현한 모델은 아니다. 기존 `tool:=suction` 4컵 헤드는 비교용이다.
하드웨어 드라이버, fake 압력/물체 감지, MoveIt용 `gripper` 진공 revolute 관절은 활성 모델에 넣지 않는다.
흡착 성공 신호를 생성하지 않으며 진공 부착/해제 동작을 구현한 것으로 보고하지 않는다.

`config/epick.yaml`은 개발용 컵 크기·가상 질량·ToF 위치·TCP 보정을 관리한다.
현재 임시 헤드 폭 170mm·깊이 130mm, 컵 4×3개, 컵별 지름 28mm다. 헤드 자체 장착점(epick_base_link)에서 접촉면까지 117.3mm,
공구 연결부(gripper_base_link)에서 TCP까지는 150.34361mm다. 집게의 형상·장착 위치를 유지하고 흡착판 장착 오프셋을 맞춘 값이다.
`config/epick.yaml`의 `head_visual`과 컵 치수를 수정하고 `python3 tools/generate_parcel_suction_head.py`, `python3 tools/export_ax1_urdf.py`를 실행하면 외형·가상 관성을 다시 생성한다. 이후 필요한 패키지를 빌드한다.
외형은 `meshes/tools/parcel_suction_head.stl`이며 색상은 `epick_adapter.xacro`에서 지정한다. OBJ/MTL 재질 때문에 흰색으로 표시되는 문제를 피하도록 재질 없는 STL을 사용한다. 본체는 보수적인 상자, 컵은 12개 원통으로 충돌을 구성한다. ToF는 넓어진 헤드 바깥의 105mm 위치로 옮겨 자기 차단을 피한다.
교체 비교를 위해 다중 흡착판 · 12컵의 가상 질량·무게중심·관성을 손가락 집게 · Robotiq 2F-85의 기본 손가락 자세에 맞췄다.
흡착판 본체 0.87086427304kg + 컵 0.04kg + ToF 0.01kg = 0.92086427304kg으로 집게 말단 총질량과 같다.
장착점 `gripper_base_link` 기준 공구 무게중심은 약 (0.0674194, -0.00004294, 0.00000939)m다.
외형·충돌·TCP는 각 공구의 기하를 유지한다. 손가락 자세가 바뀌거나 물체를 다른 위치에 잡으면 질량 분포도 달라진다.
실물 부품 동등성이나 전체 동적 안정성 보장이 아닌 무부하 시뮬레이션 비교 설정이다. 집게·컵·ToF 구성을 바꾸면 다시 계산해야 한다.
`epick_tcp`는 변형되지 않은 컵 끝면, `tool_tof_link`는 실제 ToF 측정 원점이다.
ToF는 공구 축에서 0.105 m 떨어진 위치에서 안쪽으로 20도 기울인 광선이다.
TCP 정렬·평행한 0.2m 상자를 컵 면 간격 0·0.05·0.15·0.30m에서 측정했고 최대 법선 거리 오차는 약 0.9mm였다.
접촉면에서 광선이 상자 가장자리 안쪽 약 3.1mm를 지나므로 위치 오차가 크면 놓칠 수 있다.
컵 전체 접촉이나 12컵 개별 흡착 성공을 뜻하지 않는다.

TCP/ToF 보정 시험은 컵 끝면에서 0.10·0.05·0 m 떨어진 기준 평면을 Gazebo에 배치한다.
기대 광선 거리는 `0.0237311643262128 + gap / cos(20도)` m다.
컵 면 법선 간격은 `(raw - 0.0237311643262128) * cos(20도)`이며 평행한 표면에만 적용한다.
`tcp_stroke_compensation_m=0`은 이 기하 모델에서 검증한다. 실물 컵 변형·진공 압력 보정은 아니다.
최종 제품 질량 특성과 컵 구성은 별도 미확정 사항으로 유지한다.

## 관절·자기 충돌·하중·정지 검증

팔 5축의 명목 위치 범위를 각 41점, 총 205점으로 검사한다.
각 축을 독립적으로 양쪽 한계의 0.01 rad 안쪽까지 실제 구동하고 중립으로 복귀한다.
중립뿐 아니라 시드 고정 결합 자세 128개도 검사한다.

`tools/arm_geometry.py`는 URDF collision을 OBB로 보수 근사하고 고정 링크 그룹과 인접 관절을 구분한다.
자기 충돌·지면 침범·관절 한계 위반 경로는 시험 목표로 보내지 않는다.
공간 전체의 무충돌을 보장하는 것이 아니며, 한 축씩 안전해도 여러 축을 조합하면 충돌할 수 있다.
검사기는 모델 시험용이고 일반 외부 명령을 차단하는 A4 런타임 안전 게이트는 아니다.

하중은 0·0.5·1·2 kg을 TCP에 고정한 **관성 시험 치구**다. 실제 흡착으로 들었다는 의미가 아니다.
각 하중에서 지정 자세 3개를 3회 반복하고 1초 유지해 오차를 기록한다(총 36개).
각 하중에서 진행 중 Action 취소 후 실제 관절 속도와 2초간 위치 드리프트도 확인한다.
허용 오차 0.03 rad, 정지 속도 0.01 rad/s이며 움직임 시간은 속도·가속도 제한으로 계산한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ax_description ax_interfaces ax_control
source install/setup.bash
python3 tools/export_ax1_urdf.py
python3 tools/audit_arm_range.py
python3 tools/verify_a1_envelope.py
python3 tools/verify_epick_tcp.py
```

기본 실행은 `ros2 launch ax_control simulation.launch.py`다. 제어기(바퀴·팔)와 흡착 공구 경로는 A2 `ax_control`이 띄운다. `ros2 launch ax_description robot_test.launch.py`는 제어기 없이 모델·센서만 띄우는 시험용이다.
센서 렌더링을 생략하는 `sensors:=false`는 관절 시험 성능을 위한 별도 조건이며
일반 센서 구성이 검증됐다는 근거로 쓰지 않는다.

최종 결과는 [A1 검증 자료](../evidence/a1/)에 보관한다.
이 결과는 지정한 가상 질량·마찰·컵·자세의 개발 검증이며 실물 정격·연속 작업·G1 안전 관문을 인증하지 않는다.

## W0 이종수 담당 기술 검토 — 2026-11-08

W0의 A1 모델 재구성 시작 목표는 달성했다. 아래는 코드·검사 기준의 담당 검토이며,
본인/공동 승인이나 K0 전체 확정을 대신하지 않는다. 공통 승인 상태는 INTERFACE_CONTRACT 7.3절을 따른다.

| 확인 사항 | 현재 기준 |
|---|---|
| 역할·분리 | robot_1/3 팔 로봇, robot_2/4 운송 로봇. `/ax/robot_<n>/` 토픽, `robot_<n>/` TF 접두사 |
| 단위·축 | m·kg·rad·s. 차체 +X 전방·+Y 왼쪽·+Z 위, 카메라 광학 +Z 전방·+X 오른쪽·+Y 아래 |
| 공구 기준 | `epick_tcp`와 별칭 `tool_tcp` 일치. 기본 집게 패드 중앙과 흡착판 컵 면 중앙이 장착부 기준 0.15034361m |
| ToF 계약 | `/sensors/tool_tof/raw` → B1 → `/sensors/tool_tof`, Range·30Hz·0.02~0.40m, `tool_tof_link` 기준 광선 거리 |
| 무효값 | NaN=판별 불가, +inf=범위 안 반사 없음, -inf=최소거리 미만. 차체 반사 필터는 raw를 보존하며 processed만 NaN |
| 시각·진단 | 원본 stamp/frame 유지, 측정 시각 TF를 최대 0.15s 대기. 동기화와 유효 깊이 진단은 분리 |
| 적재 기준 | `cargo_link`는 접촉판 윗면. 접촉 신호는 A3의 적재/LOST 판정이나 C4 보유 성공이 아님 |
| C1 후속 계약 | 현재 `perception/detections`는 픽셀 Detection2DArray다. `/ax/fleet/receipt/observations`의 ReceiptObservation(입고 구역·식별·map pose) 구현으로 취급하지 않는다. C2/B2 없이 3D pose나 물품 ID를 만들어 채우지 않는다 |
| D1 후속 계약 | Nav2 NavigateToPose를 사용하고 속도는 `/nav/cmd_vel` → A4 → 제어기로 연결해야 한다. 현재 리모컨의 직접 제어는 개발 시험 전용이다 |
| E1 후속 계약 | 중앙 계획은 AgentPlan Topic으로 E3 어댑터를 거쳐 D4에 제출한다. 발신 ID·세션·세대는 LLM이 만들지 않고, 계획으로 좌표·속도를 직접 구동하지 않는다. 아직 구현 완료 아님 |

ToF 기울기·보정식과 아래 주행/적재 외곽은 소비자에게 전달할 변경이다.
공유 메시지 타입·실행 권한·위험 판정은 이번 A1 검토에서 변경하지 않는다.

## 주행·적재 모델 기준

수치는 `config/a1_validation.yaml`의 `mobility`에 둔다. S005는 옆방향 `slip1=0.05 m/s/N`의
개발용 물리 가정으로 양쪽 모델에 반영한다. 마찰계수·질량·COM·제어기 설정을 조작해 회전 중심을 맞추지 않는다.
질량 배분은 가상 예산을 메시 근사 체적에 나눈 결과이지 COM을 바퀴 중앙에 맞춘 설계가 아니다.
무부하 전체 COM(base_link 기준)은 robot_1 neutral `(0.095637, 0.054021, 0.542698)m`,
주행 자세 `(0.101589, -0.045190, 0.542698)m`, robot_2 `(0.025680, -0.000224, 0.250728)m`다.

- robot_1: 기본 흡착판·무부하, 관절 순서 기둥/어깨/팔꿈치/손목/공구에 `[-1,0,0,0,0]rad`.
  팔을 새로 접는 기구 변경이 아니라 기둥 방향만 바꾼 주행 자세다. 원점에서 전환 경로 35점 표본 충돌 검사와
  각 축 ±0.02rad의 243개 외곽 표본을 확인했다. 센서 포함 표본 AABB는 x `[-0.76,0.80]`, y `[-0.393881,0.388]m`.
  여유 포함 외곽은 x `[-0.78,0.82]`, y `[-0.42,0.42]m`다. 자세 전환은 정지 상태에서 하고,
  전환 중에는 neutral의 왼쪽 돌출 약 0.225m를 포함한 실제 팔 외곽이 필요하다.
- robot_2: 기존 0.72×0.72m·두께 2mm의 강체 접촉판을 수동 지지면으로 사용한다.
  `cargo_link` 기준 상자 외곽 영역은 x/y ±0.34m(가장자리 20mm 여유)로 둔다.
  고정 관절·스트랩·자동 잠금 장치를 숨겨 추가하지 않는다. 판의 실제 재료 강도·휘어짐은 재현하지 않는다.
  선반 포함 collision 외곽은 x `[-0.60,0.64]`, y `[-1.001028,0.995096]m`이며 차체 폭만 쓰면 안 된다.
  여유 포함 x `[-0.62,0.66]`, y `[-1.03,1.03]m`를 관련 담당자에게 제공한다. 폭 1.9m 통로는 이 외곽으로 통과 불가다.

모델 검사 재현 명령은 `python3 tools/verify_a1_envelope.py --mobility`다.
격리한 평면 월드에서 두 로봇의 회전·직진·후진·곡선과 20cm 상자의 0.5/1/2kg 지지를 검사한다.
출력은 기존 Git 제외 폴더 `log/a1/mobility_summary.json`이다. A2 설정을 변경하거나 A4 주행 허용을 자동 구현하지 않는다.
실제 주행 게이트·A3 적재 판정·B2 운영 지도 채택은 각 담당자의 연결·검증이 별도로 필요하다.

## A1 → B2 S005 설정 차이 — 2026-11-09

현재 개발 모델은 공통 `ax_wheel_contact` 매크로에서 S005를 적용하므로 두 로봇의 바퀴에 함께 반영된다.
`tools/tests/test_map_tools.py`의 `test_describe_model_matches_operating_xacro`는 S005 반영 전의
`slip1=None`을 고정 기대값으로 사용해 실패한다. 지도 생성기의 파싱 실패나 제어기 설정 충돌은 아니다.

| 항목 | 현재 A1 개발 모델 | B2 시험 기대값 |
|---|---|---|
| `mu1` | 0.1 | 0.1 |
| `mu2` | 1.0 | 1.0 |
| `fdir1` | `0 0 1` | `0 0 1` |
| `slip1` | 0.05 m/s/N | `None` (명시 설정 없음) |

`None`은 추출기가 설정을 찾지 못했다는 값이며 모든 물리 엔진에서 반드시 0과 같다는 뜻은 아니다.
`slip1`은 접촉 마찰 방향의 힘에 따른 미끄럼 순응 파라미터이며 타이어 미끄럼률 5%가 아니다.
현재 A1 회귀 검사는 두 모델의 변환 SDF에서 바퀴별 `slip1=0.05`를 확인한다.
기존 물리 시험 기록은 당시 입력을 보존하며 현재 모델의 통과 근거로 자동 승계하지 않는다.

B2 담당자는 개발 모델 기준을 함께 확인한 후 시험 기대값과 지도 생성 시 모델·마찰 해시를 맞춘다.
이종수의 이번 수정에서는 B2 시험·지도·위치 추정 코드나 A2 제어기 값을 변경하지 않는다.
현재 B2 전달 폴더에는 채택한 운영 지도가 없으므로 기존 운영 지도와 충돌했다고 단정하지 않는다.
