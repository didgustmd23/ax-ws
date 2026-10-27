# D → A. 배정 유효성 응답. 권한 기한은 A의 요청 시각 tA + validity_s이며 수신 시각으로 늘리지 않는다.
string request_id               # 대응하는 요청 ID
string a_session_id             # 요청의 A 세션을 그대로 돌려준다. A는 다른 세션 응답을 버린다
string d_session_id             # D 프로세스 세션 ID. 바뀌면 A는 이전 권한을 버린다
uint64 assignment_generation    # D 원장의 현재 배정 세대
uint64 sequence                 # D가 붙이는 응답 순번. 늦게 온 이전 응답을 가려낸다
bool valid                      # 배정이 현재도 유효한지
float32 validity_s              # 확인 유효 시간 H, s. A는 요청 시각 tA(CLOCK_BOOTTIME)+H를 기한으로 쓴다. valid=false면 권한 없음
string task_state               # D 원장의 작업 상태 이름(예: RUNNING)
string reason_code              # 무효 사유 코드(예: ASSIGNMENT_STALE)
