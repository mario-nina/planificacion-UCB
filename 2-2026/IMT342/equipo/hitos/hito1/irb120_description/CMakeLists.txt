cmake_minimum_required(VERSION 3.8)
project(irb120_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

ament_package()
