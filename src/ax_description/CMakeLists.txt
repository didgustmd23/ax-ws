# 설치 경로를 기준으로 launch/config/메시/스크립트를 배포한다. build/install은 사용자 PC에서 다시 만든다.
cmake_minimum_required(VERSION 3.8)
project(ax_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/tof_range.py DESTINATION lib/${PROJECT_NAME})

ament_package()
