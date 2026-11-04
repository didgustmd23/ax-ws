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

기본 `tool:=epick`은 본체와 단일 컵을 생성한다. 기존 `tool:=suction` 4컵 헤드는 비교용이다.
하드웨어 드라이버, fake 압력/물체 감지, MoveIt용 `gripper` 진공 revolute 관절은 활성 모델에 넣지 않는다.
흡착 성공 신호를 생성하지 않으며 진공 부착/해제 동작을 구현한 것으로 보고하지 않는다.

`config/epick.yaml`은 개발용 컵 크기·가상 질량·ToF 위치·TCP 보정을 관리한다.
현재 단일 컵 반경 0.012 m, 높이 0.015 m, 본체 길이 0.1023 m다.
`epick_tcp`는 변형되지 않은 컵 끝면, `tool_tof_link`는 실제 ToF 측정 원점이다.
ToF는 공구 축에서 0.06 m 떨어진 위치의 평행 광선이며 컵 전체 접촉을 뜻하지 않는다.

TCP/ToF 보정 시험은 컵 끝면에서 0.10·0.05·0 m 떨어진 기준 평면을 Gazebo에 배치한다.
센서 원점~컵 끝면 오프셋 0.0223 m에 각 간격을 더한 기대 거리와 비교한다.
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
