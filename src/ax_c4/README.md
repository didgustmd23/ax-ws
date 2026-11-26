# C4 ManipulateObject Action 서버

## 역할

`ax_c4`는 기존 `ManipulateObject` Action을 받아 PICK/PLACE 실행 순서를
관리한다. C4는 팔 제어기를 직접 호출하지 않는다. 팔 궤적은 A4
`ProtectedArmTrajectory` Action으로, 흡착·해제는 A2 `ToolCommand` Service로
요청한다.

## 안전 경계

- A3 lease가 허용되지 않으면 물리 명령을 시작하지 않는다.
- 같은 `execution_id`의 활성 Goal은 중복 실행하지 않는다.
- 같은 실행 ID의 완료 결과는 재사용하고, 인자가 다르면 거절한다.
- 부착·해제 Service의 `accepted=true`는 완료가 아니다.
- `AttachState`가 최신·유효하고 대상 물체와 일치할 때만 PICK/PLACE 결과를 확정한다.
- 취소 시 새 하위 명령을 만들지 않으며 자동 RELEASE를 하지 않는다.
- 공통 계약에 없는 IK·충돌 회피·차체 이동은 구현하지 않는다.

## 개발용 모의 실행

실제 C2/C3 후보와 A3/A2/A4 서버가 아직 연결되지 않은 환경에서는 다음처럼
모의 응답을 사용할 수 있다.

```bash
ros2 run ax_c4 manipulate_object_server --ros-args \
  -p robot_id:=robot_1 -p mock_execution:=true
```

모의 실행은 테스트 목적이다. 실제 조작 성공이나 Gazebo 통합 완료를 의미하지
않는다. 운영 모드에서는 계약에 정의된 궤적 제공자가 연결되지 않으면
`TRAJECTORY_UNAVAILABLE` 또는 하위 노드 unavailable 오류로 종료한다.

## 구현 상태

- 구현: Goal 검증, PICK/PLACE 상태 흐름, 중복·lease·예산·취소 처리,
  A4 Action/A2 Service 연결 골격, AttachState 검증, ToolHoldState 발행
- 미연동: C2/C3 후보 검증·IK·충돌 회피, 실제 A3 lease 서버, 실제 A2/A4
  종단 실행, D2 작업 상태·D4 예산 원장
