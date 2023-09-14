cmake_minimum_required(VERSION 3.0.2)
project(medrct_ros VERSION 0.0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(yaml-cpp REQUIRED)
find_package(medrct_common REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# Library
add_library(${PROJECT_NAME}
  src/ros_singleton.cc
  src/ros_stream_factory.cc)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC medrct_common::medrct_common)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

# Scripts
add_executable(input_buffer_streamer scripts/input_buffer_streamer.cc)
target_link_libraries(input_buffer_streamer PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  medrct_common::medrct_common
)
add_executable(input_callback_streamer scripts/input_callback_streamer.cc)
target_link_libraries(input_callback_streamer PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  medrct_common::medrct_common
)
add_executable(output_streamer scripts/output_streamer.cc)
target_link_libraries(output_streamer PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  medrct_common::medrct_common
)

# Install
install(DIRECTORY include/
DESTINATION include)

install(
  TARGETS ${PROJECT_NAME} input_buffer_streamer input_callback_streamer output_streamer
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

# Install CMake files:
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
  enable_testing()
  include(GoogleTest)
  include(Dart)

  add_executable(test_conversions
  tests/conversions_test.cc
  tests/main.cc
  )

  target_compile_options(test_conversions
  PRIVATE
  -Wno-ignored-qualifiers -Wno-unused-variable -Wno-pointer-arith
  -Wno-unused-but-set-variable
  )
  target_link_libraries(test_conversions PRIVATE
  gtest ${PROJECT_NAME} medrct_common::medrct_common
  Eigen3::Eigen
  Threads::Threads
  )
  target_include_directories(test_conversions PRIVATE include)
  gtest_discover_tests(test_conversions DISCOVERY_TIMEOUT 60)
endif()
