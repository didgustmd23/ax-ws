# B → A·D. 로봇별 위험 판단(설계 7.2절). 전체 정지와 상태 만료는 방향별 허가보다 우선한다.
uint8 CLEAR=0                   # 위험 없음(설계 7.2절 숫자). meta 무효면 UNKNOWN 취급
uint8 SLOWDOWN_REQUIRED=1       # 감속 필요
uint8 STOP_REQUIRED=2           # 정지 필요
uint8 UNKNOWN=3                 # 판정 불가. 정지로 취급
StateMeta meta                  # 상태 출처·시각·revision·유효성
string robot_id                 # 대상 로봇 ID
uint8 level                     # 위 상수 중 위험 수준
string reason_code              # 판단 사유 코드(계약 D-19 안전 사유 목록)
bool sensor_valid               # 안전 센서 전체 유효 여부
bool front_sensor_valid         # 전방 센서 유효 여부
bool rear_sensor_valid          # false면 해당 로봇 후진 금지
bool localization_valid         # 위치 추정 유효 여부
bool allow_forward              # 전진 허용
bool allow_backward             # 후진 허용
bool allow_rotate               # 제자리 회전 허용
float32 max_linear_speed        # B가 권고하는 선속도 상한, m/s. 0이면 정지. level·만료가 우선
float32 max_angular_speed       # B가 권고하는 각속도 상한, rad/s. 0이면 회전 금지. level·만료가 우선
