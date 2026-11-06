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
  --packages-up-to ax_description ax_interfaces ax_control
```

## 실행 (Ubuntu 터미널 3개)

기존 A3 모의 서버와 Gazebo를 먼저 종료한다. 각 터미널에서 프로젝트 루트로
이동하고 아래 환경을 설정한다. Gazebo와 표시 노드의 GZ_PARTITION은 같아야 한다.
Python 파일을 직접 실행하므로 A3 자체는 별도 빌드가 필요 없다.

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
python3 src/ax_task_executor/ax_task_executor/gazebo_mock_display.py
```

터미널 3: A3 모의 서버 실행.

```bash
python3 src/ax_task_executor/ax_task_executor/mock_executor.py \
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
새 execution_id로 요청한다. 관측·취소·영속 원장·운영 lease 검증은 이 예제 범위 밖이다.

표시가 없으면 Gazebo가 재생 중인지, world 이름(기본 ax_test)과 GZ_PARTITION이 같은지
확인한다. 서비스 연결 실패 시 표시 노드는 경고를 출력하고 재시도한다.

## 확인한 범위

분리된 headless Gazebo 시험 월드에서 EMPTY Action 응답과
scene/info의 `a3_mock_indicator_EMPTY` 생성 확인. 실제 GUI의 색상·시야는 수동 확인 대상이다.
