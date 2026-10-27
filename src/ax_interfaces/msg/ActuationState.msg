# A4 → B4·D·상대 로봇 A4. 실제 구동·정지 상태(설계 8절 안전 실행). 정지 요청과 실제 정지를 구분한다.
uint8 UNKNOWN=0                 # 구동 상태 불명. 주행·팔 허용으로 해석하지 않는다
uint8 RUNNING=1                 # 구동 중(정지 요청 없음)
uint8 STOP_REQUESTED=2          # 정지 요청을 받았으나 아직 감속 전
uint8 STOPPING=3                # 감속·정지 진행 중
uint8 STOPPED=4                 # 실제 정지 확인(D-06 임계값 유지)
uint8 FAULT=5                   # 구동 고장. 해제 전 구동 금지
StateMeta meta                  # 상태 출처·시각·revision·유효성
string robot_id                 # 대상 로봇 ID(예: robot_1)
uint8 state                     # 위 상수 중 현재 구동 상태
bool drive_allowed              # A 최종 게이트의 현재 주행 허용
bool arm_allowed                # A 최종 게이트의 현재 팔 동작 허용
string[] block_reasons          # 예: HANDOFF_LOCK, SAFETY_STOP, PEER_STATE_STALE
float32 linear_speed            # 실측 선속도 크기, m/s
float32 angular_speed           # 실측 각속도 크기, rad/s
float32 max_joint_speed         # 팔 관절 실측 속도 크기의 최댓값, rad/s
