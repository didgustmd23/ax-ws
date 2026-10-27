# A → C. lease 철회. ManipulateObject cancel과 함께 보내며 같은 ID에서 멱등 처리한다.
string goal_id                  # 철회 대상 Goal UUID. C는 execution_id·lease_generation·goal_id가 모두 현재 것과 같을 때만 처리
string execution_id             # 철회할 실행 ID
uint64 lease_generation         # 철회할 lease 세대
string reason_code              # 철회 사유 코드(D-12 구동 차단 사유 또는 D-13 오류 코드, 예: SAFETY_STOP)
