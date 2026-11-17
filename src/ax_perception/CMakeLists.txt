# B1 Python 노드와 launch만 설치한다. 모델·제어기는 해당 패키지가 소유한다.
cmake_minimum_required(VERSION 3.8)
project(ax_perception)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/sensor_processing.py scripts/object_detection.py DESTINATION lib/${PROJECT_NAME})
# C2 입력 노드와 그 노드가 import하는 C2 라이브러리. 라이브러리는 실행 스크립트와 같은 폴더에 두어
# ros2 run 시 설치본 모듈을 쓴다(소스 경로 sys.path 불필요). C3 어댑터(c2_surface_adapter.py)는 노드가 쓰지 않아 설치하지 않는다.
install(PROGRAMS scripts/c2_surface_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/surface_estimation.py scripts/c2_detection_link.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
