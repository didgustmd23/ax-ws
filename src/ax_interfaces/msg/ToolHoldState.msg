# C → D. 공구 보유 판정. 보유는 지정 물체의 실제 부착 상태이며 fake 압력값을 근거로 쓰지 않는다.
uint8 UNKNOWN=0
uint8 HELD=1
uint8 RELEASED=2
StateMeta meta
string robot_id
string object_id
uint8 state
string evidence                 # 판정 근거 요약(예: ATTACH_SIGNAL+ARM_CAMERA)
