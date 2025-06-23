cmake_minimum_required(VERSION 3.14)
project(medrct_ros2_app VERSION 0.0.1)

# cmake_policy(SET CMP0148 OLD)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(medrct_common REQUIRED)
find_package(medrct_env REQUIRED)
find_package(medrct_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ros2_teleop_node
  src/ros2_teleop_node.cc)

ament_target_dependencies(ros2_teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
)
target_link_libraries(ros2_teleop_node
  medrct_common::medrct_common
  medrct_env::medrct_env
  medrct_ros2::medrct_ros2)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ros2_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
