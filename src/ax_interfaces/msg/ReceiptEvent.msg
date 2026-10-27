# D 내부·관제. 반복 관측과 신규 입고를 구분한 입고 사건.
uint8 PENDING_IDENTIFICATION=0
uint8 TASK_CREATED=1
uint8 DUPLICATE=2
uint8 REJECTED=3
string receipt_event_id
string object_id
string zone_id
string[] observation_ids
uint8 state
string task_id
