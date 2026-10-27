# FleetSnapshot에 넣는 로봇 1대의 요약. 영역별 meta가 그 상태값의 출처·시각·revision·유효성이다.
# 영역 meta가 무효이거나 revision 0이면 그 상태값은 UNKNOWN으로 취급한다(바깥 snapshot meta보다 우선).
string robot_id
string role                     # MANIPULATOR, TRANSPORTER
string[] capabilities
string availability             # AVAILABLE, BUSY, FAULT, RESERVE
string assignment_id
uint64 assignment_generation
string location_id
StateMeta safety_meta
uint8 safety_level              # SafetyState 상수(0=CLEAR는 설계 7.2 값이므로 safety_meta로 보호)
StateMeta actuation_meta
uint8 actuation_state           # ActuationState 상수
StateMeta localization_meta
uint8 localization_state        # LocalizationHealth 상수
bool load_applicable            # 운송 역할만 true
StateMeta load_meta
uint8 load_state                # LoadState 상수
