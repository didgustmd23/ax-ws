# D → 전체. 운영 모드와 시스템 운영 상태(설계 5.1·9.5절).
uint8 OFF=0                     # Agent 미사용, 규칙 기반 운영
uint8 ADVISORY=1                # Agent 제안을 기록·검증만 함
uint8 ASSISTED=2                # 검증된 Agent 제안을 D가 반영
uint8 SYSTEM_UNKNOWN=0          # 정상으로 간주하지 않는다
uint8 SYSTEM_NORMAL=1           # 시스템 정상 운영
uint8 SYSTEM_SAFE_STOP=2        # 전체 안전 정지
StateMeta meta                  # 상태 출처·시각·revision·유효성
uint8 mode                      # 위 OFF·ADVISORY·ASSISTED 중 운영 모드
uint8 system_state              # 위 SYSTEM_ 상수 중 시스템 상태
string safe_stop_reason         # SAFE_STOP 사유 코드
bool restoring                  # 재시작 복원 중. 새 일반 작업·Agent 반영 금지
