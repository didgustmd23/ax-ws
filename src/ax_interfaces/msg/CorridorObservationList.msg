# B3 → D3·A4. 로봇 1대의 통로별 관측 목록. 가까운 모든 통로를 한 메시지로 보낸다.
StateMeta meta                  # 목록 전체의 출처·시각·revision·유효성
string robot_id                 # 관측 대상 로봇 ID
CorridorObservation[] observations # 통로별 관측 목록
