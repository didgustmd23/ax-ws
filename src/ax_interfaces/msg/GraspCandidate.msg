# 집기 후보 1개. 좌표는 C가 실제 관측으로 계산하며 LLM이 만들지 않는다.
string candidate_id             # 후보 ID
geometry_msgs/PoseStamped tool_target # robot_<n>/epick_tcp가 도달할 목표 pose. 기준 프레임은 header.frame_id(예: map)
float32 rule_score              # 규칙 기반 점수(클수록 우선)
float32 success_probability     # XGBoost 성공 확률 0~1. 모델을 쓸 수 없으면 -1이며 이때 rule_score로만 순위를 정한다
bool reachable                  # 후보 계산 시점 C의 도달 검사 결과. 실행 전 C가 재검증
bool collision_free             # 후보 계산 시점 C의 충돌 검사 결과. 실행 전 C가 재검증
