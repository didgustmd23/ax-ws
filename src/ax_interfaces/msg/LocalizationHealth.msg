# B2 → B3·B4·D·인계 상대 로봇 C4. 위치 추정 신뢰성과 위치 이상 탐지 결과.
uint8 UNKNOWN=0                 # 판정 불가. OK로 보지 않는다
uint8 OK=1                      # 위치 추정 정상
uint8 DEGRADED=2                # 신뢰도 저하
uint8 LOST=3                    # 위치 상실
StateMeta meta                  # 상태 출처·시각·revision·유효성
string robot_id                 # 대상 로봇 ID
uint8 state                     # 위 상수 중 위치 추정 상태
float32 position_std_m          # 위치 표준편차 추정, m
float32 yaw_std_rad             # 방향 표준편차 추정, rad
float32 anomaly_score           # Isolation Forest 이상 점수, 0~1로 정규화하고 클수록 이상. 모델 불가면 0이고 anomaly_model_available=false. 진단 보조이며 안전 판단을 대체하지 않는다
bool anomaly_model_available    # 이상 탐지 모델 사용 가능 여부
