# 모든 상태 메시지의 공통 머리. 상태의 출처·버전·유효성을 함께 보낸다.
std_msgs/Header header          # stamp: 관측·판정 시각(ROS sim time), frame_id: 기준 프레임(없으면 빈 문자열)
uint32 schema_version           # 메시지 계약 버전. 의미가 바뀌면 올린다
string source_id                # 발행 노드·모듈 식별자(예: b4_safety_monitor)
string source_session_id        # 생산자 프로세스 시작마다 새로 발급. 소비자는 (session, revision)으로 비교
uint64 revision                 # 같은 세션에서 상태 변경마다 1씩 증가. 0은 아직 발행 전(UNKNOWN)
bool valid                      # 생산자가 판단한 유효성. false면 소비자는 성공·정상으로 해석하지 않는다
string validity_reason          # valid=false 사유 코드(예: SENSOR_STALE)
