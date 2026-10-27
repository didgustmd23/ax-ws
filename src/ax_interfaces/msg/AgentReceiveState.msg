# D → 관제·Agent. Agent별 수신 상태(설계 5.3절).
uint8 UNKNOWN=0                 # ACTIVE로 간주하지 않는다
uint8 ACTIVE=1                  # 제안 수신·검증 대상
uint8 QUARANTINED=2             # 격리됨. 제안을 받지 않음
uint8 RECOVERY_SHADOW=3         # 복귀 시험 중. 제안은 기록·검증만 함
string sender_agent_id          # 대상 Agent ID
uint64 agent_receive_generation # D가 발급한 Agent 수신 세대. 격리 진입과 ACTIVE 복귀 때 증가하며 이전 세대 제안은 거절
uint8 state                     # 위 상수 중 현재 수신 상태
string reason_code              # 격리·복귀 사유 코드
