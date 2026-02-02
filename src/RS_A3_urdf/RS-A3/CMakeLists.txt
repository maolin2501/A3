cmake_minimum_required(VERSION 3.8)
project(rs_a3_urdf_legacy)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES RS-A3.urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
