# E → D·관제. Agent 추론 처리 상태(설계 8절). ACCEPTED는 실행 성공이 아니다.
uint8 DISABLED=0                # Agent 비활성(OFF 모드 또는 미기동)
uint8 IDLE=1                    # 대기 중. RECORDED_ONLY 결정을 받으면 이 상태로 돌아간다
uint8 REASONING=2               # 추론 중
uint8 VALIDATING=3              # 어댑터가 출력 형식·스키마를 검사하거나 D 결정을 기다리는 중
uint8 ACCEPTED=4                # D가 제안을 수락함
uint8 REJECTED=5                # D가 제안을 거절함
uint8 FALLBACK=6                # 형식 오류·지연으로 출력 폐기, 규칙 기반 진행
StateMeta meta                  # 상태 출처·시각·revision·유효성
string sender_agent_id          # 이 상태의 Agent ID
uint8 state                     # 위 상수 중 현재 처리 상태
string model_id                 # 사용 중인 추론 모델 식별자
float32 last_latency_s          # 마지막 추론 1회 소요 시간(어댑터 단조 시계)
