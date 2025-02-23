cmake_minimum_required(VERSION 3.20)
project(dual_panda_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)


find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometric_shapes REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)


add_executable(my_move_group
    src/my_move_group.cpp)     
ament_target_dependencies(my_move_group
    rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
    moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3
    visualization_msgs moveit_visual_tools)  

add_executable(my_moveit_cpp
    src/my_moveit_cpp.cpp)     
ament_target_dependencies(my_moveit_cpp
    rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
    moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3
    visualization_msgs moveit_visual_tools)
 
# target_compile_definitions(MyPlan PRIVATE JZJ_ROBOT)

install(TARGETS
    my_move_group my_moveit_cpp
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)  

ament_package()
