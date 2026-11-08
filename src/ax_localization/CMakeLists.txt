# B2 위치 추정·지도 작성 Python 도구와 설정·launch·지도만 설치한다. 모델·제어기·센서 처리는 해당 패키지가 소유한다.
cmake_minimum_required(VERSION 3.8)
project(ax_localization)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/mapping_drive.py scripts/check_map_accuracy.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/map_tools.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})
ament_package()
