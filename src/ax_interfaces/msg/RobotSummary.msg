# FleetSnapshot에 넣는 로봇 1대의 요약. 영역별 meta가 그 상태값의 출처·시각·revision·유효성이다.
# 영역 meta가 무효·만료·revision 0이면 그 상태값은 UNKNOWN이다. 영역 meta가 바깥 meta보다 우선하되, 바깥 snapshot meta가 무효면 전체 무효다.
string robot_id                 # 로봇 ID
string role                     # 역할: MANIPULATOR(조작) 또는 TRANSPORTER(운송)
string[] capabilities           # 수행 가능 기능 목록(예: SUCTION_PICK, CARRY)
string availability             # 가용 상태: AVAILABLE(배정 가능), BUSY(작업 중), FAULT(고장), STANDBY_SPARE(예비). 이 4개만 쓰고 빈 문자열은 가용으로 보지 않는다
string assignment_id            # 현재 배정 ID. 없으면 빈 문자열
uint64 assignment_generation    # 현재 배정 세대
string location_id              # 가장 가까운 등록 지점 ID(정차 확인 아님). 모르면 빈 문자열
StateMeta safety_meta           # 안전 판정의 출처·시각·revision·유효성
uint8 safety_level              # SafetyState 상수(0=CLEAR는 설계 7.2 값이므로 safety_meta로 보호)
StateMeta actuation_meta        # 구동 상태의 출처·시각·revision·유효성
uint8 actuation_state           # ActuationState 상수
StateMeta localization_meta     # 위치 추정 상태의 출처·시각·revision·유효성
uint8 localization_state        # LocalizationHealth 상수
bool load_applicable            # 운송 역할만 true
StateMeta load_meta             # 적재 판정의 출처·시각·revision·유효성
uint8 load_state                # LoadState 상수
