# D → 관제·Agent. Agent별 수신 상태(설계 5.3절).
uint8 UNKNOWN=0                 # ACTIVE로 간주하지 않는다
uint8 ACTIVE=1
uint8 QUARANTINED=2
uint8 RECOVERY_SHADOW=3
string sender_agent_id
uint64 agent_receive_generation
uint8 state
string reason_code
