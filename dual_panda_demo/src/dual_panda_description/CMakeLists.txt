cmake_minimum_required(VERSION 3.10.2)
project(dual_panda_description)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})
