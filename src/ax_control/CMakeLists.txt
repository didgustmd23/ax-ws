cmake_minimum_required(VERSION 3.8)
project(ax_control)

find_package(ament_cmake REQUIRED)

# 코드를 빌드하지 않는 설정 패키지다. launch와 config만 설치한다.
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()