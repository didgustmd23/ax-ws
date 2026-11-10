# D1 Nav2 기본 구성 — W2

2026-11-09: 두 로봇의 Nav2 설정·기동·모의 경로 계산을 확인했다.
실제 지도·B2 위치 추정·A4 게이트를 연결한 주행/정차 시험은 수행하지 않았다.

## 소유권과 입력

- D1은 controller_server·planner_server·bt_navigator만 실행한다. 지도 서버와 AMCL을 중복 실행하지 않는다.
- B2 입력: 공통 `/map`, `map → robot_<n>/odom` TF.
- A2 입력: `/ax/robot_<n>/diff_drive_controller/odom`, `robot_<n>/odom → base_link` TF.
- B1 입력: 로봇별 `sensors/front_lidar/scan_filtered`, `sensors/rear_lidar/scan_filtered`.
- TF는 공통 `/tf`·`/tf_static`, 로봇의 base/odom/센서 프레임은 접두사로 구분한다.
- 출력: `/ax/robot_<n>/nav/cmd_vel`, `geometry_msgs/TwistStamped`만 발행한다.
  제어기 입력에 직접 연결하지 않는다. A4 최종 게이트는 별도 담당 구현이다.
- NavigateToPose: `/ax/robot_<n>/navigate_to_pose`. D2 호출·실행 ID/UUID 영속화·취소·실패 처리는 후속 D1 작업이다.

## 설정

`launch/navigation.launch.py`가 A1 `config/a1_validation.yaml`의 footprint를 읽는다.
robot_1은 **epick·무부하·기록된 주행 자세** 조건의 외곽이다. 현재 팔 자세를 강제하거나 검사하지 않는다.
robot_2는 선반 포함 폭 2.06 m의 여유 포함 외곽이다. 1.9 m 통로에 맞도록 줄이지 않는다.
robot_3/4는 동일 역할의 개발 모델 외곽을 재사용하며, 이번 실행 검증은 robot_1/2만 했다.

- local costmap: 로봇별 odom 기준 6×6 m rolling window.
- global costmap: map 기준, B2 공통 지도 + 로봇별 전후 LiDAR.
- 두 costmap 모두 polygon footprint·장애물·inflation layer를 사용한다.
- 각 LiDAR 관측의 갱신 기대 시간은 0.5 s다. 미갱신 시 costmap을 최신으로 간주하지 않는다.
- 개발값: 선속도 0.15 m/s, 각속도 0.2 rad/s, 가속도 0.15 m/s²·0.2 rad/s²,
  목표 허용 오차 0.1 m·0.1 rad. 실제 정차 허용값이나 성능 검증값이 아니다.
- A* Navfn + DWB footprint collision critic. 센서 높이 밖의 장애물·팔 자세·적재 안정성·통로 허가를 판정하지 않는다.
- W2 BT는 경로 계산·추종만 연결한다. Spin/BackUp/자율 복구/강제 costmap 지우기는 추가하지 않았다.
- 기본 `autostart=false`: 실제 입력과 A4 연결을 확인하기 전 주행 기능을 활성화하지 않는다.
  lifecycle manager의 자동 프로세스 재시작은 구성하지 않았다.

## 빌드와 실행

Ubuntu 24.04 / ROS Jazzy / Nav2 1.3.13에서 확인했다.

```bash
sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ax_navigation ax_perception ax_control ax_localization
source install/setup.bash
ros2 launch ax_navigation navigation.launch.py robot_ids:=robot_1,robot_2
```

실제 운용 전에 B2 지도·위치 신뢰성·TF, A2 odom, B1 scan, A4 구동 경로를 공동 확인해야 한다.
모의 검사는 다음과 같이 별도 domain에서 실행한다.

```bash
# 두 터미널 모두 같은 값. 실제 시뮬레이션/운용이 없는 domain을 사용한다.
export ROS_DOMAIN_ID=180 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch ax_navigation navigation.launch.py
# 다른 터미널
python3 tools/check_w2_navigation.py
```

검사기가 **모의** `/clock`·지도·TF·odom·scan을 발행하고 lifecycle을 활성화한다.
두 로봇에 ComputePathToPose만 요청하며 NavigateToPose·FollowPath·바퀴 명령은 보내지 않는다.
확인 뒤 두 프로세스를 종료한다. 모의 입력을 실제 운용 domain에 섞지 않는다.

결과: [W2 증거](../../docs/evidence/w2/). 실제 주행·정차 W3, 복구 W4와 G1은 미완료다.
