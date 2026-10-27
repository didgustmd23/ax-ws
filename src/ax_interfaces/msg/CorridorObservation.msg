# B → D. 통로 진입·완전 이탈 관측(설계 7.3절). 외형에는 접힌 팔·적재물을 포함한다.
uint8 OUTSIDE=0
uint8 APPROACHING=1
uint8 INSIDE=2
uint8 UNKNOWN=3
StateMeta meta
string corridor_id
string robot_id
uint8 observation
bool position_valid
