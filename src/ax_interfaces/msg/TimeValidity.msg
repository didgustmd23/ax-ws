# A 공용 감독기 → D·A 실행 계층. sim time 진행 상태(설계 7.6절).
uint8 UNKNOWN=0                 # 판정 불가. 시간 기반 판정 금지
uint8 RUNNING=1                 # sim time 정상 진행
uint8 PAUSED=2                  # sim time 정지
uint8 REVERSED=3                # sim time 역행(리셋 등)
uint8 RTF_OUT_OF_RANGE=4        # 실시간 배율이 허용 범위 밖
StateMeta meta                  # 상태 출처·시각·revision·유효성
uint8 state                     # 위 상수 중 시간 상태
float32 measured_rtf            # /clock 진행량 ÷ CLOCK_BOOTTIME 경과
float32 window_s                # 측정 창, CLOCK_BOOTTIME 초
