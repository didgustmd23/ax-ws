# D가 위임한 재시도 예산(설계 7.10절). 받은 쪽은 목록 밖 ID를 만들어 실행하지 않는다.
string budget_grant_id
string recovery_context_id
string kind                     # GRASP_ATTEMPT, REOBSERVATION, RECOVERY_MOVE 등
uint32 granted_attempts         # 0이면 시도 불가
string[] allowed_execution_ids
float64 time_budget_s           # 감독용 단조 시계 기준. 0 이하는 무효 예산이며 시도하지 않는다
