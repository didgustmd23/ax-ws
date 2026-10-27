# A → C. 원시 부착·해제 신호. 보유 판정은 C가 이 신호와 관측을 묶어 따로 발행한다.
StateMeta meta
string robot_id
string tool_id                  # 예: epick
string object_id                # 부착된 물체. 없으면 빈 문자열
bool attached
