# C → D. 공구 보유 판정. 보유는 지정 물체의 실제 부착 상태이며 fake 압력값을 근거로 쓰지 않는다.
uint8 UNKNOWN=0                 # 보유 판정 불명. 미보유로 보지 않는다
uint8 HELD=1                    # 지정 물체 보유 확인
uint8 RELEASED=2                # 해제 확인
StateMeta meta                  # 상태 출처·시각·revision·유효성
string robot_id                 # 대상 로봇 ID
string object_id                # 판정 대상 물체 ID
uint8 state                     # 위 상수 중 보유 판정
string evidence                 # 판정 근거 요약(예: ATTACH_SIGNAL+ARM_CAMERA)
