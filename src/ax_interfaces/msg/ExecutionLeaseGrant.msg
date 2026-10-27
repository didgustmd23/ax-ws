# A → C. lease 응답. 기한은 C의 요청 시각 tC + lease_s이며 0 < lease_s <= min(상한, tA + H - 현재).
string request_id               # 대응하는 lease 요청 ID. 재동기화 중인 C는 가장 최근 request_id의 grant만 쓴다
string goal_id                  # 요청의 goal_id를 그대로 돌려준다. C는 다른 Goal의 응답을 버린다
string execution_id             # lease 대상 실행 ID
uint64 lease_generation         # 부여한 lease 세대. FIRST면 A 원장의 마지막 세대+1이며 같은 execution_id의 이전 세대는 철회, RENEW면 같은 세대(기한만 새로 계산)
bool granted                    # lease 부여 여부
float32 lease_s                 # lease 허용 기간 L, s. C는 자기 요청 시각 tC(CLOCK_BOOTTIME)+L을 기한으로 쓴다. granted=false면 무시
string reason_code              # 거절 사유 코드(예: LEASE_RESYNC_UNCERTAIN이면 C는 Action을 UNCERTAIN으로 끝낸다)
