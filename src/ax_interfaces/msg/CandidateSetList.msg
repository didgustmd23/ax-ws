# C → D. 로봇 1대가 게시 중인 모든 물체의 조작 후보 목록. 목록 전체가 최신 상태다.
StateMeta meta                  # 목록 전체의 출처·시각·revision·유효성
string robot_id                 # 후보를 게시한 로봇 ID
CandidateSet[] candidate_sets   # 물체별 후보 묶음 목록
