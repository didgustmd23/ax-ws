# B → A·D. 로봇별 위험 판단(설계 7.2절). 전체 정지와 상태 만료는 방향별 허가보다 우선한다.
uint8 CLEAR=0
uint8 SLOWDOWN_REQUIRED=1
uint8 STOP_REQUIRED=2
uint8 UNKNOWN=3
StateMeta meta
string robot_id
uint8 level
string reason_code
bool sensor_valid
bool front_sensor_valid
bool rear_sensor_valid          # false면 해당 로봇 후진 금지
bool localization_valid
bool allow_forward
bool allow_backward
bool allow_rotate
float32 max_linear_speed        # m/s
float32 max_angular_speed       # rad/s
