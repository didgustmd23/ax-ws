# A → B·D. 실제 구동·정지 상태(설계 8절 안전 실행). 정지 요청과 실제 정지를 구분한다.
uint8 UNKNOWN=0
uint8 RUNNING=1
uint8 STOP_REQUESTED=2
uint8 STOPPING=3
uint8 STOPPED=4
uint8 FAULT=5
StateMeta meta
string robot_id
uint8 state
bool drive_allowed              # A 최종 게이트의 현재 주행 허용
bool arm_allowed
string[] block_reasons          # 예: HANDOFF_LOCK, SAFETY_STOP, PEER_STATE_STALE
float32 linear_speed            # 실측, m/s
float32 angular_speed           # rad/s
float32 max_joint_speed         # 팔 관절 최대 실측 속도, rad/s
