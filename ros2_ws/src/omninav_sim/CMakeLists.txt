cmake_minimum_required(VERSION 3.16)
project(omninav_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch worlds config meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
