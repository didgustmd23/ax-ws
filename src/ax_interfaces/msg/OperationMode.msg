# D → 전체. 운영 모드와 시스템 운영 상태(설계 5.1·9.5절).
uint8 OFF=0
uint8 ADVISORY=1
uint8 ASSISTED=2
uint8 SYSTEM_UNKNOWN=0          # 정상으로 간주하지 않는다
uint8 SYSTEM_NORMAL=1
uint8 SYSTEM_SAFE_STOP=2
StateMeta meta
uint8 mode
uint8 system_state
string safe_stop_reason
bool restoring                  # 재시작 복원 중. 새 일반 작업·Agent 반영 금지
