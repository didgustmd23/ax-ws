# A3(운송 로봇) → D2·상대 로봇 A4. 적재 판정(설계 7.8절). 순간 접촉 해제만으로 LOST로 판정하지 않는다.
uint8 UNKNOWN=0                 # 적재 상태 불명. 비었다고 보지 않는다
uint8 STOWED=1                  # 적재함에 적재됨
uint8 EMPTY=2                   # 비어 있음 확인
uint8 LOST=3                    # 적재물 상실(D-08 조건)
uint8 SOURCE_UNKNOWN=0          # 판정 출처 불명
uint8 SOURCE_SENSOR=1           # 센서 기반 판정
uint8 SOURCE_GROUND_TRUTH=2     # 적재 영역 판정의 Gazebo 객체 위치 사용(제한적 예외)
uint8 SOURCE_COMBINED=3         # 센서와 정답 기반 판정 결합
StateMeta meta                  # 상태 출처·시각·revision·유효성
string robot_id                 # 운송 로봇 ID
string object_id                # 적재 대상 물체 ID
uint8 state                     # 위 적재 상수 중 판정
bool contact                    # 적재함 접촉 감지 여부
bool in_zone                    # 물체가 허용 적재 영역 안에 있는지
uint8 source                    # 위 SOURCE_ 상수 중 판정 출처
