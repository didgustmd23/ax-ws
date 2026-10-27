# B → A·D. 위치 추정 신뢰성과 위치 이상 탐지 결과.
uint8 UNKNOWN=0
uint8 OK=1
uint8 DEGRADED=2
uint8 LOST=3
StateMeta meta
string robot_id
uint8 state
float32 position_std_m          # 위치 표준편차 추정, m
float32 yaw_std_rad
float32 anomaly_score           # Isolation Forest 점수. 진단 보조이며 안전 판단을 대체하지 않는다
bool anomaly_model_available
