# A(운송 로봇) → D. 적재 판정(설계 7.8절). 순간 접촉 해제만으로 LOST로 판정하지 않는다.
uint8 UNKNOWN=0
uint8 STOWED=1
uint8 EMPTY=2
uint8 LOST=3
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_SENSOR=1
uint8 SOURCE_GROUND_TRUTH=2     # 적재 영역 판정의 Gazebo 객체 위치 사용(제한적 예외)
uint8 SOURCE_COMBINED=3
StateMeta meta
string robot_id
string object_id
uint8 state
bool contact
bool in_zone
uint8 source
