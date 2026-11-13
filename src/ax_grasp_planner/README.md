# C3 집기 후보·기본 순위 — 김성민 W3

관측 표면에서 흡착 공구의 중심·주변 후보와 접근/이탈 pose를 계산하는 규칙 기반 라이브러리다.
실제 팔 실행·물체 검출·3D 자세 추정·학습 모델은 구현하지 않는다. 다른 담당자 소스와 공통 타입은 변경하지 않았다.

## 입력과 내부 API

`Surface`는 C2가 실제 관측에서 제공해야 한다: robot/object ID, scene_revision, stamp, frame,
표면 중심·바깥 법선·폭 방향 접선·폭/깊이, 신뢰도, 흡착 가능 표면 여부·평탄도·질량.
`Constraints`는 검증한 공구 접촉 폭/깊이·허용 질량·접근/이탈 거리·여유·유효기간·제약 버전이다.
공구 크기는 A1/A2와 합의한 값을 전달한다. 이 라이브러리에 실물 공구 성능을 임의로 넣지 않는다.

```python
plan = plan_candidates(surface, constraints, now_s,
                       reachability=actual_ik_check,
                       collision=actual_approach_target_retreat_check)
selected = select_candidate(plan, now_s,
                            scene_revision=current_scene_revision,
                            constraint_version=current_constraint_version,
                            reachability=actual_ik_check,
                            collision=actual_approach_target_retreat_check,
                            preference=preference_from_accepted_c4_goal)
```

- 도달 검증기는 5축 관절 한계·방향 제약을 포함한 실제 IK 판정을 수행해야 한다.
  접근·목표·이탈 pose 모두 True이어야 하며, 충돌 검증기는 그 사이 전체 경로를 검사해야 한다.
- 검증기 없음·예외·False·미확인은 유효 후보가 아니다. 거리만으로 reachable=true라고 하지 않는다.
- 흡착 헤드 전체 접촉 크기와 가장자리 여유가 표면에 들어가는 후보만 생성한다.
- 중심 우선의 규칙 점수로 정렬하고, XGBoost가 없으므로 success_probability=-1이다.
- 선호는 현재 집합/장면·등록 후보일 때만 우선 검토하며 현재 도달·충돌 검사를 다시 통과해야 한다.
  오래되거나 부적합한 선호는 기본 순위로 대체한다. 후보가 없으면 None/사유를 반환한다.
- 좌표 약속: 표면 normal은 바깥 방향, TCP +Z는 표면으로 접근하는 방향(-normal)이다.
  실제 `epick_tcp` 보정·접촉 위치 변환과 이 약속은 A1/A2/C4와 통합 전에 확인해야 한다.
  다른 축을 쓰는 공구를 이 후보로 바로 실행하면 안 된다.

## 후보 게시

`CandidatePublisher.publish_plan(plan)`은 실제 C2/C4 연결부가 호출하는 내부 API다.
출력은 기존 `/ax/<robot>/manipulation/candidates` CandidateSetList이다(STATE QoS, 1Hz).
각 후보의 tool_target·rule_score·success_probability·reachable/collision_free를 게시한다.
접근/이탈 pose는 내부 Candidate에 있으며 공통 메시지에 필드를 임의 추가하지 않았다.
동일 관측 재게시로 벽시계 기한을 늘리지 않고, 오래된 후보는 무효 상태로 게시한다.
같은 물체의 기존 후보보다 장면 버전·제약 버전·관측 시각 중 하나라도 이전이면
`publish_plan`은 `ValueError`로 거절하고 기존 후보를 유지한다. 비교와 저장은 잠금으로 묶어
동시에 도착한 결과도 같은 기준을 적용한다. 버전 이력은 노드 실행 중 물체별로 관리한다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash  # 작업공간에서 colcon build 후 실행
ros2 launch ax_grasp_planner candidate_publisher.launch.py robot_id:=robot_1 use_sim_time:=true
python3 tools/tests/test_c3_grasp_planner.py
python3 tools/tests/test_w3_contracts_ros.py
```

현재 C2 표면 snapshot의 공통 ROS 타입/Topic과 C4 내부 호출부가 아직 없으므로 새로운 입력 Topic을
발명하지 않았다. 노드를 단독 실행하면 NO_VALID_CANDIDATES이며 가짜 좌표를 생성하지 않는다.
시험의 도달·충돌 검증기는 모의 함수다. 실제 IK·MoveIt·Gazebo 집기 성공을 검증한 것이 아니다.
손가락 집게 전략, XGBoost 학습·확률 보정, 실제 장애물 지도 연계는 이번 규칙 기반 흡착 범위 밖이다.
Gazebo에서는 `/clock`과 `use_sim_time:=true`를 함께 사용한다. 실제 시계를 쓰려면 입력 노드와
함께 `use_sim_time:=false`로 설정한다. launch는 C2 입력이나 IK·충돌 검증기를 생성하지 않는다.
사유 코드 9종의 뜻과 D-13 추가 제안은 [계약 인계 문서](../../docs/guides/C3_D3_CONTRACT_HANDOFF.md)에 정리했다.
