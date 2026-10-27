# D → Agent·로그. 제안·계획 검증 결과.
uint8 UNSPECIFIED=0
uint8 ACCEPTED=1
uint8 REJECTED=2
uint8 RECORDED_ONLY=3           # ADVISORY·RECOVERY_SHADOW에서 기록만 함
string sender_agent_id
string proposal_id
string plan_id
uint64 snapshot_id
uint8 decision
string reason_code
