# 집기 후보 1개. 좌표는 C가 실제 관측으로 계산하며 LLM이 만들지 않는다.
string candidate_id
geometry_msgs/PoseStamped tool_target   # epick_tcp 기준 공구 목표 pose
float32 rule_score
float32 success_probability     # XGBoost 추정. 모델 불가 시 -1
bool reachable
bool collision_free
