# A → D. 배정 유효성 확인 요청(설계 7.11절). 요청 시각 tA는 A가 로컬 단조 시계로 기록한다.
string request_id               # 요청 ID. 확인마다 새로 발급(요청 시각 tA와 짝)
string a_session_id             # A 프로세스 세션. D는 재시작 전 세션의 요청을 거절한다
string robot_id                 # 요청한 로봇 ID
string assignment_id            # 확인할 배정 ID
uint64 assignment_generation    # A가 알고 있는 배정 세대
