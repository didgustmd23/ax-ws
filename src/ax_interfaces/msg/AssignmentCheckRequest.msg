# A → D. 배정 유효성 확인 요청(설계 7.11절). 요청 시각 tA는 A가 로컬 단조 시계로 기록한다.
string request_id
string a_session_id             # A 프로세스 세션. D는 재시작 전 세션의 요청을 거절한다
string robot_id
string assignment_id
uint64 assignment_generation
