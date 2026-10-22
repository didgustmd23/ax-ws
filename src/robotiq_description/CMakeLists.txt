cmake_minimum_required(VERSION 3.10)
project(robotiq_description)

# AX model-only subset of the upstream description package. No hardware driver.
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE README.md UPSTREAM_FILES.json DESTINATION share/${PROJECT_NAME})
ament_package()
