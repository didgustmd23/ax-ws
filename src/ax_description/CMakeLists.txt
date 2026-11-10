# 설치 경로를 기준으로 launch/config/메시/스크립트를 배포한다. build/install은 사용자 PC에서 다시 만든다.
# 이 패키지를 구성할 수 있는 최소 CMake 버전.
cmake_minimum_required(VERSION 3.8)
# package.xml의 name과 동일한 ROS 패키지 이름.
project(ax_description)

# ROS 2의 CMake 기반 패키지 등록·설치 기능.
find_package(ament_cmake REQUIRED)

# OBJ/MTL 메시와 URDF/Xacro를 패키지 공유 경로에 설치한다.
# package://ax_description/... 참조는 이 설치 결과에서 파일을 찾는다.
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# 실행 구성과 Gazebo 월드를 설치한다. Python 캐시 폴더는 배포하지 않는다.
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# 센서 장착, 물리 특성, 적재 높이 제한 등의 설정 파일을 배포한다.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ROS 실행 파일로 등록한다: ToF 거리 변환. 적재 높이 감시는 현재 설치·실행하지 않는다.
# ros2 run ax_description <파일명> 및 launch의 Node에서 사용한다.
install(PROGRAMS scripts/tof_range.py DESTINATION lib/${PROJECT_NAME})

# ament 인덱스에 등록하여 ros2와 다른 패키지가 설치 결과를 찾도록 한다.
ament_package()
