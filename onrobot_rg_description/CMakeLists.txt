cmake_minimum_required(VERSION 3.16)
project(onrobot_rg_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()