# E → D·관제. Agent 추론 처리 상태(설계 8절). ACCEPTED는 실행 성공이 아니다.
uint8 DISABLED=0
uint8 IDLE=1
uint8 REASONING=2
uint8 VALIDATING=3
uint8 ACCEPTED=4
uint8 REJECTED=5
uint8 FALLBACK=6
StateMeta meta
string sender_agent_id
uint8 state
string model_id
float32 last_latency_s          # 마지막 추론 1회 소요 시간(어댑터 단조 시계)
