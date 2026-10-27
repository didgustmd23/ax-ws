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
string transfer_id
string transfer_state           # 예: LOCKED, LOADING, UNLOCKED
string[] hold_ids
string failure_event_id
