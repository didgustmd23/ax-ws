# 등록 지점에서 역할별 정차 자세.
string robot_role               # MANIPULATOR, TRANSPORTER
geometry_msgs/PoseStamped pose  # map 기준 정차 pose
float32 position_tolerance_m    # 정차 위치 허용 오차, m
float32 yaw_tolerance_rad       # 정차 방향 허용 오차, rad
