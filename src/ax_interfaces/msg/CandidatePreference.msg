# D → A → C. 로봇 Agent의 후보 선호 전달. C가 재검증하고 부적합하면 무시한다.
string proposal_id              # 선호를 낸 Agent 제안 ID
string candidate_set_id         # 대상 후보 묶음 ID
string candidate_id             # 선호하는 후보 ID
uint64 scene_revision           # 선호 근거 장면 revision. 다르면 C가 무시
