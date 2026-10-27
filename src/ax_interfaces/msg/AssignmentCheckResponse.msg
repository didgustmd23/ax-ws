# D → A. 배정 유효성 응답. 권한 기한은 A의 요청 시각 tA + validity_s이며 수신 시각으로 늘리지 않는다.
string request_id
string a_session_id             # 요청의 A 세션을 그대로 돌려준다. A는 다른 세션 응답을 버린다
string d_session_id
uint64 assignment_generation
uint64 sequence
bool valid
float32 validity_s              # H
string task_state
string reason_code
