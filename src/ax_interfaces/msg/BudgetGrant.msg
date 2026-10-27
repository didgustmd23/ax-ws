# D가 위임한 재시도 예산(설계 7.10절). 받은 쪽은 목록 밖 ID를 만들어 실행하지 않는다.
string budget_grant_id          # 예산 ID
string recovery_context_id      # 예산이 속한 복구 문맥 ID
string kind                     # 예산 종류: GRASP_ATTEMPT(집기 물리 시도), REOBSERVATION(재관측 주기), RECOVERY_MOVE(복구 주행), NAV_RETRY(일반 주행 재시도) 중 하나
uint32 granted_attempts         # 0이면 시도 불가
string[] allowed_execution_ids  # 이 예산으로 실행할 수 있는 execution_id 목록
float64 time_budget_s           # 시간 예산, s(CLOCK_BOOTTIME 기준). 0 이하는 무효 예산이며 시도하지 않는다
