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
