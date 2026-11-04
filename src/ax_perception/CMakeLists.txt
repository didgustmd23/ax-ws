# B1 Python 노드와 launch만 설치한다. 모델·제어기는 해당 패키지가 소유한다.
cmake_minimum_required(VERSION 3.8)
project(ax_perception)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/sensor_processing.py scripts/object_detection.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
