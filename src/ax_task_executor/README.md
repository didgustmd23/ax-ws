# A3 첫 과제: 모의 Action 결과를 Gazebo에서 보기

이 예제는 실제 적재 확인이 아니다. 모의 서버가 반환한 결과를 충돌 없는 정적
표시등으로 보여준다. EMPTY=빨강, STOWED=초록, UNKNOWN=노랑이다.
표시등 위치는 월드 좌표 (0, -3, 2) m이며 Entity Tree 이름은
`a3_mock_indicator_EMPTY`처럼 표시된다. 기존 로봇·월드·제어 코드는 변경하지 않는다.
시험 표시등은 Gazebo 종료 시 사라진다. 노드만 종료하면 마지막 표시등은 남는다.

## 준비

프로젝트 루트에서 기존 설명·제어 패키지를 Ubuntu에 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /home/freem/ax_main_logs build \
  --base-paths src --build-base /home/freem/ax_main_build \
  --install-base /home/freem/ax_main_install \
  --packages-up-to ax_description ax_interfaces ax_control ax_task_executor
```

## 실행 (Ubuntu 터미널 3개)

기존 A3 모의 서버와 Gazebo를 먼저 종료한다. 각 터미널에서 프로젝트 루트로
이동하고 아래 환경을 설정한다. Gazebo와 표시 노드의 GZ_PARTITION은 같아야 한다.
위 빌드에는 A3 패키지도 포함된다. 빌드한 설치 환경을 source하면 아래 `ros2 run` 명령을 사용할 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/freem/ax_main_install/setup.bash
export GZ_PARTITION=a3_first_demo
```

터미널 1: 기존 창고와 과제 로봇 실행.

```bash
ros2 launch ax_control simulation.launch.py \
  world:=aws_warehouse.world robots:=2 sensors:=false
```

터미널 2: 모의 결과 표시 노드 실행.

```bash
ros2 run ax_task_executor gazebo_mock_display
```

터미널 3: A3 모의 서버 실행.

```bash
ros2 run ax_task_executor mock_executor \
  --ros-args -p mock_load:=EMPTY
```

터미널 2와 3은 준비 후 계속 실행해 둔다.

## 요청 및 확인 (추가 Ubuntu 터미널)

같은 ROS 환경을 source한 후 요청한다.

```bash
ros2 action send_goal /a3_mock/robot_2/execute_local_task \
  ax_interfaces/action/ExecuteLocalTask \
  "{task_kind: 5, object_id: 'test_box', execution: {execution_id: 'visual-empty-001', operation: 'CONFIRM_LOAD'}}" \
  --feedback
```

응답은 `outcome: 2`, `load_state: 2`, `MOCK_EMPTY`, `ABORTED`가 정상이다.
표시 노드 로그에 `MOCK result displayed: EMPTY`가 나오면 Gazebo Entity Tree에서
`a3_mock_indicator_EMPTY`를 찾아 더블클릭해 해당 위치로 시야를 이동한다.
Gazebo 버전에 따라 우클릭 메뉴의 Move To 기능을 사용할 수 있다.
빨간 표시등은 빈 적재함이라는 **시험 응답**을 뜻한다. 실제 적재함이 비었음을
관측했다는 의미가 아니다. Gazebo에서 상자를 옮겨도 이 모의 결과는 바뀌지 않는다.

다른 결과는 서버의 `mock_load`를 `STOWED` 또는 `UNKNOWN`으로 변경해 재시작한 뒤
새 execution_id로 요청한다. 실제 관측·취소·운영 lease 검증은 이 예제 범위 밖이다.

## 중복 요청과 재시작

수락 전에 SQLite 원장에 로봇 ID·실행 ID·요청 내용·최초 `mock_load`를 저장한다.
같은 로봇의 같은 실행 ID와 같은 요청은 거절하지 않고 기존 실행 기록을 사용한다.
완료 결과가 있으면 그대로 반환하며, 수락 후 결과 저장 전에 재시작한 경우 최초 시험 입력으로
모의 결과를 완성한다. 이 복구 방식은 물리 명령이 없는 모의 서버에만 적용한다.
동일 ID로 물체·배정·작업 인자 등을 바꾸면 거절한다. 계약상 재전송 시 달라질 수 있는
`execution.task_revision`은 비교에서 제외한다. 다른 결과를 시험하려면 새 실행 ID를 사용한다.

기본 원장은 `${ROS_HOME:-~/.ros}/a3_mock/executions.sqlite3`이며 로봇 ID별로 기록이 분리된다.
`--ros-args -p execution_db:=/절대/경로/executions.sqlite3`로 위치를 지정할 수 있다.
재시작할 때 같은 경로를 유지해야 하며, 원장을 삭제하거나 다른 경로를 쓰면 이전 기록을
복구할 수 없다. 원장을 쓰지 못하면 요청을 거절하고, 결과 저장 실패 시 성공을 반환하지 않는다.
운영용 배정 검증·lease·물리 작업 복구는 별도 구현 대상이다.

표시가 없으면 Gazebo가 재생 중인지, world 이름(기본 ax_test)과 GZ_PARTITION이 같은지
확인한다. 서비스 연결 실패 시 표시 노드는 경고를 출력하고 재시도한다.

## 로봇 1에서도 실행하기

기본 `robot_id`는 `robot_2`이므로 기존 로봇 2 명령은 그대로 사용할 수 있다.
로봇 1은 위와 같은 빌드·환경 설정 후 별도 터미널 두 개에서 실행한다.
두 로봇을 함께 시험할 때도 Gazebo는 한 번만 실행한다.

```bash
# 로봇 1 표시 노드 (별도 터미널)
ros2 run ax_task_executor gazebo_mock_display --ros-args \
  -r __node:=a3_gazebo_mock_display_robot_1 -p robot_id:=robot_1
```

```bash
# 로봇 1 모의 서버 (별도 터미널)
ros2 run ax_task_executor mock_executor --ros-args \
  -r __node:=a3_mock_executor_robot_1 -p robot_id:=robot_1 -p mock_load:=EMPTY
```

```bash
# 요청 (추가 터미널)
ros2 action send_goal /a3_mock/robot_1/execute_local_task \
  ax_interfaces/action/ExecuteLocalTask \
  "{task_kind: 5, object_id: 'test_box', execution: {execution_id: 'robot1-empty-001', operation: 'CONFIRM_LOAD'}}" \
  --feedback
```

로봇 1 결과는 `/a3_mock/robot_1/result_display`로 전달하며,
표시등은 `a3_mock_indicator_robot_1_EMPTY`처럼 생성된다. 위치는 (-1, -3, 2) m다.
로봇 2의 기존 결과 Topic·표시등 이름·위치는 유지된다. 각 표시 노드는 자기 로봇의
표시등만 제거하므로 두 결과가 서로 덮어쓰이지 않는다.
로봇 1도 STOWED·EMPTY·UNKNOWN 시험 입력만 사용하며 실제 로봇 구동·적재 관측은 하지 않는다.

## 확인한 범위

분리된 headless Gazebo 시험 월드에서 EMPTY Action 응답과
scene/info의 `a3_mock_indicator_EMPTY` 생성 확인. 실제 GUI의 색상·시야는 수동 확인 대상이다.

로봇 1 추가 후 ROS 환경에서 `python3 tools/tests/test_a3_mock_robots.py`로 두 로봇의
Action 경로·결과 Topic과 STOWED·EMPTY·UNKNOWN 결과를 확인했다. Gazebo 서비스는
모의 호출로 대체하여 표시등 이름·위치·삭제 범위의 분리를 검사했다.
두 로봇의 Gazebo GUI 동시 표시는 별도 수동 확인 대상이다.
