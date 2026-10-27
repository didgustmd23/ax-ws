# C → D. 조작 후보 snapshot(설계 7.13절). 읽기 전용 게시이며 집기 실행이 아니다.
StateMeta meta                  # 후보 묶음의 출처·시각·revision·유효성
string robot_id                 # 후보를 계산한 로봇 ID
string object_id                # 대상 물체 ID
string candidate_set_id         # 후보 묶음 ID
uint64 scene_revision           # 후보 계산에 쓴 장면 revision
uint32 constraint_version       # 후보 제약 조건 버전
string[] default_ranking        # candidate_id 순서
GraspCandidate[] candidates     # 후보 목록
