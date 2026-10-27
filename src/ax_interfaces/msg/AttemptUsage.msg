# 물리 시도 1회의 사용 기록. 시작된 시도는 환불하지 않는다.
uint8 UNSPECIFIED=0
uint8 STARTED=1
uint8 FINISHED=2
uint8 UNCERTAIN=3               # 결과 미확정. D는 실제 상태와 대조 전까지 예약을 유지한다
string budget_grant_id
string execution_id
uint32 attempt_index
string kind
uint8 state
builtin_interfaces/Time started_at   # ROS sim time. 물리 성능 기록용
float64 elapsed_s               # 보고 측이 CLOCK_BOOTTIME으로 잰 경과 시간. 시간 예산 집계용
