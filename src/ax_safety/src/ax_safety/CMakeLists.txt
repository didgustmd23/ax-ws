# ax_safety는 A4 주행/팔 최종 차단 노드와 B4 위험 판정 Python 노드를
# 함께 설치하는 ROS 2 패키지이다. A2 컨트롤러, A3 lease 서버,
# B1~B3 상태 메시지와 연결되는 실행 파일·설정·헤더를 여기서 묶는다.
cmake_minimum_required(VERSION 3.8)
project(ax_safety)


# A4 구현은 C++17 문법과 표준 라이브러리를 사용한다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 안전 코드의 암시적 변환·미사용 변수·비표준 문법을 빌드 때 확인한다.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ax_interfaces REQUIRED)
# 보호 팔 서버가 command digest를 계산할 때 사용하는 OpenSSL SHA-256을 찾는다.
find_package(OpenSSL REQUIRED)


# A4 주행 게이트: Nav2의 명령을 검사해 A2 diff_drive_controller로 전달한다.
add_executable(a4_safety_gate src/a4/safety_gate.cpp)
target_include_directories(a4_safety_gate PRIVATE include)
ament_target_dependencies(a4_safety_gate rclcpp geometry_msgs nav_msgs sensor_msgs ax_interfaces)
# A4 보호 팔 서버: A3 lease와 SafetyState/ActuationState를 확인한 뒤
# A2 FollowJointTrajectory action으로만 팔 명령을 전달한다.
add_executable(a4_protected_arm_server src/a4/protected_arm_server.cpp)
target_include_directories(a4_protected_arm_server PRIVATE include)
ament_target_dependencies(a4_protected_arm_server rclcpp rclcpp_action control_msgs trajectory_msgs ax_interfaces)
target_link_libraries(a4_protected_arm_server OpenSSL::Crypto)


# 실행 파일은 ROS 2가 찾는 패키지별 lib 디렉터리에 설치한다.
install(TARGETS a4_safety_gate a4_protected_arm_server DESTINATION lib/${PROJECT_NAME})
# 외부 패키지가 include/ax_safety/... 경로로 A4 헤더를 사용할 수 있게 한다.
install(DIRECTORY include/ DESTINATION include)
# ax_safety/b4 아래 Python 모듈을 ROS Python 패키지로 설치한다.
ament_python_install_package(${PROJECT_NAME})
# launch/config는 실행 시 패키지 share 디렉터리에서 읽고, B4 노드는 실행 이름을 고정한다.
install(PROGRAMS ax_safety/b4/b4_risk_node.py DESTINATION lib/${PROJECT_NAME} RENAME b4_risk_node)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


# package.xml의 build_type과 이 CMake 구성을 ament 패키지로 마무리한다.
ament_package()
