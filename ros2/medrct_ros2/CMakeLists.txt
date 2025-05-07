cmake_minimum_required(VERSION 3.5)
project(medrct_ros2 VERSION 0.0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(medrct_common REQUIRED)

# ament_package()

include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/ros2_singleton.cc
  src/ros2_stream_factory.cc)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(${PROJECT_NAME} medrct_common::medrct_common)

# Scripts
# add_executable(input_buffer_streamer scripts/input_buffer_streamer.cc)
# target_link_libraries(input_buffer_streamer PRIVATE
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   medrct_common::medrct_common
# )
# add_executable(input_callback_streamer scripts/input_callback_streamer.cc)
# target_link_libraries(input_callback_streamer PRIVATE
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   medrct_common::medrct_common
# )
# add_executable(output_streamer scripts/output_streamer.cc)
# target_link_libraries(output_streamer PRIVATE
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   medrct_common::medrct_common
# )

# Install
install(DIRECTORY include/
DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
)

# Export
install(
  EXPORT ${PROJECT_NAME}-targets
  FILE "${PROJECT_NAME}-targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# # Install CMake files:
configure_file(
  "cmake/${PROJECT_NAME}-config.cmake"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  @ONLY
)
install(
  FILES
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  DESTINATION
    "lib/cmake/${PROJECT_NAME}"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-version.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)

############
# Testing ##
############

# Add gtest based cpp test target and link libraries
option(BUILD_TESTING "Build testing" ON)
if(BUILD_TESTING)
  # include(FetchContent)
  # FetchContent_Declare(
  #   googletest
  #   URL https://github.com/google/googletest/archive/34ad51b3dc4f922d8ab622491dd44fc2c39afee9.zip
  # )
  # # For Windows: Prevent overriding the parent project's compiler/linker settings
  # set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  # FetchContent_MakeAvailable(googletest)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions
    tests/conversions_test.cc
    tests/main.cc)
  target_include_directories(test_conversions PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_conversions
    sensor_msgs geometry_msgs
  )
  target_link_libraries(test_conversions
    medrct_common::medrct_common
    Eigen3::Eigen
    Threads::Threads)
endif()

ament_package()
