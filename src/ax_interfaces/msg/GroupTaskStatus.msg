# D → 관제·Agent·A. 그룹 작업 상태(설계 8절).
uint8 UNSPECIFIED=0
uint8 QUEUED=1
uint8 ASSIGNED=2
uint8 RUNNING=3
uint8 COMPLETED=4
uint8 FAILED=5
uint8 CANCELED=6
uint8 RETRY_PENDING=7
string team_id
string task_id
uint64 task_revision
uint8 state
string object_id
string source_location_id
string destination_location_id
# 인계 상태(설계 9.3절). A4 최종 게이트의 입력이며 0(UNKNOWN)·만료면 두 로봇 모두 주행 금지
uint8 TRANSFER_UNKNOWN=0
uint8 TRANSFER_NONE=1           # 인계 없음 또는 시작 전
uint8 TRANSFER_LOCKED=2         # 두 로봇 실제 정지·잠금 확인. 두 로봇 주행 금지
uint8 TRANSFER_IN_PROGRESS=3    # 적재·하역 중. 두 로봇 주행 금지
uint8 TRANSFER_UNLOCKED=4       # 적재 확인·흡착 해제·팔 후퇴 확인 후 잠금 해제
string manipulator_robot_id     # 이 작업의 조작 역할 로봇
string transporter_robot_id     # 이 작업의 운송 역할 로봇
uint64 assignment_generation
string transfer_id
uint8 transfer_state
string[] hold_ids
string failure_event_id
