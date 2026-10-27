# C → D. 조작 후보 snapshot(설계 7.13절). 읽기 전용 게시이며 집기 실행이 아니다.
StateMeta meta
string robot_id
string object_id
string candidate_set_id
uint64 scene_revision
uint32 constraint_version
string[] default_ranking        # candidate_id 순서
GraspCandidate[] candidates
