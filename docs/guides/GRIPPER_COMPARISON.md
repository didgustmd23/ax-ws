# 집게 모델 교체와 비교

`display.launch.py`의 `gripper_model` 인자로 네 종류를 선택한다. 실행당 하나만 장착하며, 교체할 때는 Ctrl+C로 종료하고 다른 인자로 다시 실행한다. 실행 중 교체와 여러 로봇의 동시 비교는 구현하지 않았다.

설계상 robot_1의 **기준 공구는 Robotiq EPick**이며 최종 컵 구성은 미정이다. EPick 모델은 아직 도입하지 않았다. 아래 네 모델은 모두 외형·운동학 비교용이며, 가상 4컵 헤드 `suction`은 EPick 도입 전까지 RViz 기본값으로 둔다. 이 기본값은 흡착 부착·해제 구현 완료가 아니다. 공통 Xacro와 Gazebo simulation의 기본 공구는 계속 `primitive`이고, Gazebo 공구 선택과 제어 구성은 별도 전환 작업이다.

## 선택 가능한 모델

| 값 | 모델 | 개폐·크기 | 로봇 전체 링크 / 관절 |
|---|---|---|---|
| `robotiq` | 오픈소스 Robotiq 2F-85 | 최대 개구 85 mm, 연결기구 연동 | 24 / 23 |
| `primitive` | 기존 임시 평행 집게 | 양쪽을 같은 변위로 움직이면 내부 간격 15~75 mm | 17 / 16 |
| `parallel_wide` | AX 가상 대형 평행 집게 | 내부 간격 20~450 mm, 손가락 길이 220 mm | 19 / 18 |
| `suction` (RViz 기본값, 비교용) | AX 가상 4컵 흡착 헤드 | 판 180×140 mm, 컵 지름 50 mm × 4개 | 21 / 20 |

두 신규 모델은 `src/ax_description/urdf/test_grippers.xacro`에 정의한다. 가상 설계의 질량·힘 설정은 시험용 값이며 파지 성능 검증 결과가 아니다. 기존 차체·바퀴·기둥·팔과 손목 회전 관절을 공통으로 사용한다. 집게 길이가 달라지므로 끝단 좌표는 달라진다.

## 실행

처음 한 번 변경 내용을 빌드한다.

```bash
cd ~/ax_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ax_description ax_control
source install/setup.bash
```

인자를 생략하면 `suction`으로 실행된다. 다른 모델은 아래 명령 중 하나로 실행한다. 새 터미널에서도 ROS와 작업공간 setup을 적용한다.

```bash
# 상자 폭에 맞춰 벌어지는 대형 평행 집게
ros2 launch ax_description display.launch.py gripper_model:=parallel_wide

# 4컵 흡착 헤드
ros2 launch ax_description display.launch.py gripper_model:=suction

# Robotiq 2F-85
ros2 launch ax_description display.launch.py gripper_model:=robotiq

# 기존 임시 집게
ros2 launch ax_description display.launch.py gripper_model:=primitive
```

RViz에서 Fixed Frame을 `base_link`, RobotModel의 Description Topic을 `/robot_description`으로 설정한다. 비교 시에는 기존 display 실행과 Gazebo 실행을 종료하여 `/robot_description`, `/joint_states`가 충돌하지 않도록 한다.

## GUI로 움직이는 방법

| 모델 | 조작 관절 | 범위 / 동작 |
|---|---|---|
| `robotiq` | `robotiq_85_left_knuckle_joint` | 0~0.8 rad, 증가하면 닫힘. 나머지 5개 관절은 mimic 연동 |
| `primitive` | `left_finger_joint`, `right_finger_joint` | 각각 -0.01~0.02 m. 대칭 개폐하려면 두 값을 같게 설정 |
| `parallel_wide` | `wide_left_finger_joint` | 0~0.215 m, 증가하면 열림. 오른쪽은 mimic 연동 |
| `suction` | 개폐 관절 없음 | 헤드·컵은 고정 링크. 팔과 손목으로 자세를 바꿈 |

넓은 집게는 처음에 최대 개구로 표시한다. 내부 간격은 `20 + 2000 × q` mm다. 예를 들어 300 mm 간격은 `q=0.14 m`다. `wide_grasp_frame`은 두 손가락 사이 중심, `suction_grasp_frame`은 네 컵 접촉면의 중심에 둔다. 기본 자세에서 신규 헤드의 로컬 +Z는 기존 어댑터 +X, 즉 아래쪽을 향한다.

## Robotiq 장착과 출처

Robotiq 2F-85는 공통 어댑터 `gripper_base_link`에 `xyz="0.02 0 0"`, `rpy="0 π/2 0"`로 고정 연결했다. 로봇의 기본 조립 자세에서 Robotiq 로컬 +Z가 어댑터 +X, 즉 아래쪽을 향한다. 원본 모델을 AX 크기에 맞춰 확대하지 않았다.

Robotiq 자체는 본체와 좌우 연결기구를 포함한 링크 9개이며, 회전 관절 6개 중 독립 입력 1개와 mimic 5개로 개폐한다. AX 원본 OBJ의 손가락·중앙 패드·링크 쌍은 이 모델에 사용하지 않는다. `include_ros2_control="false"`로 호출하므로 실물 드라이버·serial·ros2_control 제어기는 활성화하지 않는다.

모델 전용 부분을 고정 commit에서 가져왔으며 원본·BSD-3-Clause 라이선스·23개 파일의 SHA 기록은 [Robotiq 패키지 안내](../../src/robotiq_description/README.md), [LICENSE](../../src/robotiq_description/LICENSE), [UPSTREAM_FILES.json](../../src/robotiq_description/UPSTREAM_FILES.json)에 보관한다. 같은 작업공간에 동명 `robotiq_description` 패키지를 중복 추가하지 않는다.

최초 장착·메시 검사 기록은 [asset_and_mount_checks.json](../evidence/grippers/robotiq_preview/asset_and_mount_checks.json), 최초 mimic 출력 검사는 [joint_state_publisher_checks.json](../evidence/grippers/robotiq_preview/joint_state_publisher_checks.json)에 남아 있다. 당시 정적·프로세스 내부 검사 결과이며 GUI·DDS 통신·Gazebo 파지 성공 기록이 아니다.

## 현재 시험 범위

RViz에서 외형·개구 폭·관절 연동·팔 자세를 비교할 수 있다. RViz는 상자를 물리적으로 붙잡는 시뮬레이터가 아니다. 넓은 집게의 마찰 파지와 흡착 헤드의 진공 힘·접촉 판정·부착/해제는 아직 구현하지 않았다.

흡착은 접근 조건 확인 후 Gazebo에서 물체와 연결하고 명시적 해제 요청으로 연결을 제거하는 방식으로 구현하기로 결정했다. 후보인 Gazebo `DetachableJoint`(작업 환경 gz-sim 8.15에서 `attach_topic`·`detach_topic`·`output_topic`·`child_model`·`child_link` 옵션 확인)에는 다음 제약이 있어 도입 전에 시험한다.

- 연결 대상은 플러그인 설정의 `child_model`·`child_link`로 지정한다. 상자가 여럿이거나 `object_id`가 바뀌는 시나리오의 대상 지정 방식을 먼저 정한다.
- 플러그인은 거리·표면 방향·접촉을 검사하지 않는다. 접근 조건 검사는 C의 요청과 A의 실행 허용 검사 쪽에 별도로 구현한다.
- 시작 시 연결 상태와 접촉 중 재연결 동작을 시험으로 확인한다.
- 고정 연결 단계는 흡착력·전단력·박리 모멘트 검증이 아니다. 흡착 ON 요청, 실제 부착 완료, 유지, 실제 해제 완료를 구분하고, 안전 정지 시 자동 해제하지 않는다.

`simulation.launch.py`와 `/gripper/command`는 기존 `primitive` 전용이며, 이번 선택 인자는 RViz 실행에 적용한다.

## Robotiq EPick 도입 계획

결정 사항이며 아직 도입하지 않았다. 상세 계약은 [통합 상세설계](../design/DETAILED_DESIGN.md) 7.7절을 따른다.

| 항목 | 방향 |
|---|---|
| 도입 범위 | [ros2_epick_gripper](https://github.com/PickNikRobotics/ros2_epick_gripper)의 `epick_description` 선별 도입. 고정 commit·BSD-3-Clause LICENSE·파일 해시 보존. 기존 Robotiq 2F-85 subset과 같은 방식 |
| 제외 | `epick_controllers`·`epick_driver` 직접 사용, MoveIt 패키지. 압력 모델 없이 생성한 fake 압력·감지값을 실제 집기 성공으로 사용하지 않음 |
| 흡착 제어 | A의 허용 경로를 거치는 별도 노드 + Gazebo 플러그인 |
| 보유 판정 | 지정 물체의 실제 부착 상태. 압력 기반 상태와 구분하며, 검증된 압력 모델·실물 피드백은 향후 보조 근거로 추가 가능 |
| TCP | C의 공구 목표 기준은 `epick_tcp`(로컬 +Z가 접근 방향). `tcp_stroke_compensation`은 접촉 모델 검증 후 확정 |
| 미정 | 최종 컵 구성, 제품 질량 특성(원본 description에 `<inertial>` 없음), 가상 진공 관절 `gripper`의 제외·이름 변경, Jazzy 빌드 결과 |

원본 `main`(`d073913`, 2026-10-22 확인)은 단일 컵 매크로를 제공하며 Jazzy 수정은 미병합 브랜치에만 있었다. `epick_driver`가 요구하는 `serial` 패키지는 작업 PC와 Jazzy apt에서 찾지 못했다. 드라이버는 초기 범위에서 제외하므로 영향이 없지만, 고정 commit 빌드로 Jazzy 호환성을 확인한 뒤 확정한다.

Gazebo에서 실제 들어 올리는 성능을 비교하려면 모델별 제어기, 접촉·마찰 또는 흡착 구현, 상자 질량·크기·배치와 성공 판정을 추가해야 한다. 이전에 확인한 팔의 바닥 도달 범위도 모델별 끝단 기준으로 다시 검토해야 한다. 집게 교체 자체로 바닥 도달 문제가 해결된 것은 아니다.

## 확인 결과

- 3개 패키지 빌드와 네 모델의 Xacro 변환·`check_urdf` 통과.
- 공통 차체·팔 링크/관절 동일, 선택하지 않은 임시 손가락과 손가락 제어기 중복 없음.
- 모든 참조 메시 파일 존재, 잘못된 모델 이름 거부 확인.
- 설치된 `joint_state_publisher`를 프로세스 내부에서 실행하여 넓은 집게 초기값과 세 개구 위치의 mimic 출력 확인: 20 / 235 / 450 mm.
- 흡착 컵 4개의 고정 연결과 같은 접촉면 높이 확인.

검증 환경의 DDS 소켓 권한 제한으로 GUI 표시·노드 간 통신·Gazebo 물리 파지는 검증하지 않았다. 수치 기록: [model_checks.json](../evidence/grippers/gripper_comparison/model_checks.json).
