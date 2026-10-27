# A 공용 감독기 → D·A 실행 계층. sim time 진행 상태(설계 7.6절).
uint8 UNKNOWN=0
uint8 RUNNING=1
uint8 PAUSED=2
uint8 REVERSED=3
uint8 RTF_OUT_OF_RANGE=4
StateMeta meta
uint8 state
float32 measured_rtf            # /clock 진행량 ÷ CLOCK_BOOTTIME 경과
float32 window_s                # 측정 창, CLOCK_BOOTTIME 초
