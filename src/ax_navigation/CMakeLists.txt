cmake_minimum_required(VERSION 3.8)
project(ax_navigation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES scripts/navigation_execution.py scripts/navigation_client.py scripts/navigation_watchdog.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
