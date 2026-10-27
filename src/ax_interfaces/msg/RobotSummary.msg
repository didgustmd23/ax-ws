# FleetSnapshot에 넣는 로봇 1대의 요약. *_valid가 false인 상태값은 UNKNOWN으로 취급한다.
string robot_id
string role                     # MANIPULATOR, TRANSPORTER
string[] capabilities
string availability             # AVAILABLE, BUSY, FAULT, RESERVE
string assignment_id
uint64 assignment_generation
string location_id
uint8 safety_level              # SafetyState 상수(0=CLEAR는 설계 7.2 값이므로 safety_valid로 보호)
bool safety_valid
uint8 actuation_state           # ActuationState 상수
bool actuation_valid
uint8 localization_state        # LocalizationHealth 상수
bool localization_valid
bool load_applicable            # 운송 역할만 true
uint8 load_state                # LoadState 상수
bool load_valid
