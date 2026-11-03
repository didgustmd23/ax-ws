cmake_minimum_required(VERSION 3.8)
project(ax_control)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

# 흡착 부착·해제 Gazebo 플러그인. 기본 DetachableJoint로 임의 물체를 붙일 수 없어 직접 작성했다.
add_library(ax_suction_attach_system SHARED src/suction_attach_system.cpp)
target_compile_features(ax_suction_attach_system PRIVATE cxx_std_17)
target_link_libraries(ax_suction_attach_system
  gz-sim::gz-sim
  gz-plugin::register
  gz-transport::gz-transport
  gz-msgs::gz-msgs
)

install(
  TARGETS ax_suction_attach_system
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 공구 노드만 설치한다. check_*.py는 개발 시험 도구라 소스에서 직접 실행한다.
install(
  PROGRAMS scripts/suction_tool_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()