# 물리 시도 1회의 사용 기록. 시작된 시도는 환불하지 않는다.
uint8 UNSPECIFIED=0             # 채우지 않은 기록. 시도 상태 불명으로 취급
uint8 STARTED=1                 # 시도 시작(예산 차감)
uint8 FINISHED=2                # 시도 종료(결과는 ResultStatus)
uint8 UNCERTAIN=3               # 결과 미확정. D는 실제 상태와 대조 전까지 예약을 유지한다
string budget_grant_id          # 사용한 예산 ID
string execution_id             # 이 시도의 실행 ID
uint32 attempt_index            # 시도 순번. 0부터 시작(D-03)
string kind                     # 예산 종류(BudgetGrant.kind와 같음)
uint8 state                     # 위 상수 중 시도 상태
builtin_interfaces/Time started_at # ROS sim time. 물리 성능 기록용
float64 elapsed_s               # 보고 측이 CLOCK_BOOTTIME으로 잰 경과 시간. 시간 예산 집계용
