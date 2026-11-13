# B1·D1 W2 인계 자료 — 2026-11-13

생산 담당 이종수. 기준 main `48a0ea6` 및 이 회차의 검사 도구 수정본.
아래는 전달할 인터페이스·검증 범위이며 공동 시험이나 수신 확인 완료 기록이 아니다.

## 공통 입력·시각

- 센서 루트 `/ax/robot_<n>/sensors/`, 센서 frame `robot_<n>/...`.
- `header.stamp`는 관측 ROS sim time을 유지한다. B1 무수신·ToF 대기는 CLOCK_BOOTTIME으로 잰다.
- 센서 QoS는 SENSOR_DATA(best effort, volatile). 큰 영상·점군 발행자에 Fast DDS 32 MB SHM 프로필 사용.
- B1 개발 기본값: 표본 나이 0.5 s, 미래 허용 0.05 s, RGB-D 동기화 0.03 s,
  무수신 STALE 2.0 s, ToF TF 대기 0.15 s. 진단 `/sensors/health`는 DiagnosticArray, 0.5 s마다 발행한다.
- 소비자의 만료 조건은 별도로 적용한다. 예를 들어 B4 센서 만료 0.25 s를 B1의 2.0 s 진단 주기로 대체하지 않는다.
- 두 로봇의 15 sim s 관측에서 RGB-D 최근접 시각 차 0, 관측 입력/출력 stamp 불일치 0.
  정격률·장시간·4대·실물 성능의 승인 근거는 아니다.

## 양현승 — B2·A2·C2

- 전후 LiDAR: `front_lidar/scan_filtered`, `rear_lidar/scan_filtered`, LaserScan·m, 10 Hz(sim).
- C2용 깊이·점군: `front_rgbd/processed/depth_image`, `front_rgbd/processed/points`,
  CameraInfo는 `front_rgbd/processed/camera_info`. 광학 +X 오른쪽, +Y 아래, +Z 전방.
- 원시 Harmonic 점군 대신 B1이 재생성한 processed 점군을 사용한다. RGB-D 15 Hz(sim).
- D1은 B2의 공통 `/map`, `map → robot_<n>/odom`과 A2 odom·`odom → base_link`를 소비한다.
  D1에서 지도 서버나 AMCL을 중복 실행하지 않는다.
- AX_2 footprint는 x −0.63~0.67, y −0.53~0.53 m, 적재물은 외곽 내부 여부 별도 확인.
- 함께 확인할 것: 현재 월드·지도 revision·센서 장착/프레임·TF 시각 대응,
  B2 신뢰성 저하와 재위치 뒤 D1 허용 조건. 실제 연결은 W3 시험에서 확인한다.

## 김성민 — B3·A3

- B3 관측에 사용할 로봇 frame·map revision은 B2와 대조한다. 센서 거리 검사 통과는 영역 승인이 아니다.
- cargo_contact는 원시 접촉 관측이며 적재 STOWED/EMPTY/LOST 판정 자체가 아니다.
- 작업대·통로 경계·접근 영역과 최신 AX_2 외곽의 일치를 확인한다.
- 함께 확인할 것: 미승인 영역·무효 위치·관측 만료 시 UNKNOWN 전달, D3 허가·이탈과의 연결.

## 엄승연 — B4·A4·C4

- D1 출력 `/ax/robot_<n>/nav/cmd_vel`, geometry_msgs/TwistStamped.
  D1은 `/diff_drive_controller/cmd_vel`에 직접 발행하지 않는다.
- 전후 scan_filtered는 10 Hz(sim). B4는 각 메시지의 시각과 자기 무수신 기한을 직접 검사한다.
- A4에 필요한 주행 자세·실제 관절 상태·permit은 D1 설정만으로 확인되지 않는다.
  robot_1 footprint는 epick·무부하·기록된 주행 자세 조건이다.
- 팔 RGB-D: 대표 자세 3개에서 좌/중앙/우 10·15 cm 표적 관측, 25 cm는 가림.
  이 결과는 고정 기하 치구의 TF/렌더링 일관성이며 실제 운동·독립 외부 보정이 아니다.
- 팔 RGB 참고 영상에는 로봇 전경 형상이 크게 포함된다. RGB 전체 영상이 물체 검출에 적합하다고
  판정한 것이 아니며, 작업 자세·관심 픽셀을 선택한 뒤 별도 검증해야 한다.
- 공구 ToF는 센서 원점에서 광선상 거리다. 컵 간격·흡착 성공·전체 공구 충돌 검사로 해석하지 않는다.
- 함께 확인할 것: 앞서 발견한 A4 결함의 수정·회귀 상태, 실제 B4→A4→A2 차단·정지,
  C2/C4 대표 작업 자세에서 표면이 보이는 픽셀·거리. W3 실제 주행 시험과 구분해 기록한다.

## 공동 확인 시 남길 것

참여자·소스/모델/지도 revision·시나리오·허용값·실제 결과·원본 로그 경로를 기록한다.
합의나 실제 시험이 없는 항목을 완료 처리하지 않는다. 안전·최신성 기준을 완화해 연결하지 않는다.
