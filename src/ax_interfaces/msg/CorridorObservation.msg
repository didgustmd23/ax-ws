# B3 → D3·A4(CorridorObservationList 항목). 통로 진입·완전 이탈 관측(설계 7.3절). 외형에는 접힌 팔·적재물을 포함한다.
uint8 OUTSIDE=0                 # 통로 밖(완전 이탈, 설계 7.3절 숫자). meta 유효·position_valid=true일 때만 해제 근거
uint8 APPROACHING=1             # 통로 진입선 앞 접근 구역 안에서 통로 쪽을 향함
uint8 INSIDE=2                  # 통로 안
uint8 UNKNOWN=3                 # 판정 불가. 점유로 취급하며 position_valid=false인 관측도 이 값으로 본다
StateMeta meta                  # 관측 출처·시각·revision·유효성
string corridor_id              # 대상 통로 ID
string robot_id                 # 관측 대상 로봇 ID
uint8 observation               # 위 상수 중 관측 결과
bool position_valid             # false면 OUTSIDE(0)여도 통로 해제 근거로 쓰지 않는다
