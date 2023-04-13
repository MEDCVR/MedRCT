cmake_minimum_required(VERSION 3.0.2)
project(medrct_ros_app VERSION 0.0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(medrct_common REQUIRED)
find_package(medrct_controller REQUIRED)
find_package(medrct_env REQUIRED)
find_package(medrct_ros REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(ros_teleop_node src/ros_teleop_node.cc)
target_link_libraries(ros_teleop_node PRIVATE
  ${catkin_LIBRARIES}
  medrct_common::medrct_common
  medrct_controller::medrct_controller
  medrct_env::medrct_env
  medrct_ros::medrct_ros)

install(
  TARGETS ros_teleop_node
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
)

# Export
# install(
#   EXPORT ${PROJECT_NAME}-targets
#   FILE "${PROJECT_NAME}-targets.cmake"
#   NAMESPACE ${PROJECT_NAME}::
#   DESTINATION lib/cmake/${PROJECT_NAME}
# )

install(DIRECTORY config
  DESTINATION .)

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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# option(BUILD_TESTING "Build testing" ON)
# if(BUILD_TESTING)
#   enable_testing()
#   include(GoogleTest)
#   include(Dart)

#   add_executable(test_conversions
#   tests/conversions_test.cc
#   tests/main.cc
#   )

#   target_compile_options(test_conversions
#   PRIVATE
#   -Wno-ignored-qualifiers -Wno-unused-variable -Wno-pointer-arith
#   -Wno-unused-but-set-variable
#   )
#   target_link_libraries(test_conversions PRIVATE
#   gtest ${PROJECT_NAME} medrct_common::medrct_common
#   Eigen3::Eigen
#   Threads::Threads
#   )
#   target_include_directories(test_conversions PRIVATE include)
#   gtest_discover_tests(test_conversions DISCOVERY_TIMEOUT 60)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
