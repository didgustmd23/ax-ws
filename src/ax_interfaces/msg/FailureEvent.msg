# 실패 사건(설계 9.1·9.5·9.6절). 사람 응답을 기다리지 않고 종료·격리·알림을 기록한다.
uint8 UNSPECIFIED=0
uint8 CLASSIFIED=1
uint8 UNCLASSIFIED=2
string failure_event_id
string owner_domain             # A, B, C, D, E
string code                     # 9.1절 오류 코드
uint8 classification
string task_id
string object_id
string robot_id
string recovery_context_id
bool recovery_exhausted
bool post_review_required       # POST_REVIEW_REQUIRED 알림 대상
builtin_interfaces/Time occurred_at  # ROS sim time
string detail
