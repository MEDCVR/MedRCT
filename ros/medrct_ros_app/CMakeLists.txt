cmake_minimum_required(VERSION 3.14)
project(medrct_ros_app VERSION 0.0.1)

cmake_policy(SET CMP0148 OLD)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(medrct_common REQUIRED)
find_package(medrct_env REQUIRED)
find_package(medrct_ros REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(ros_teleop_node
  src/ros_teleop_node.cc)

target_link_libraries(ros_teleop_node PRIVATE
  ${catkin_LIBRARIES}
  medrct_common::medrct_common
  medrct_env::medrct_env
  medrct_ros::medrct_ros)

install(
  TARGETS ros_teleop_node
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Export
# install(
#   EXPORT ${PROJECT_NAME}-targets
#   FILE "${PROJECT_NAME}-targets.cmake"
#   NAMESPACE ${PROJECT_NAME}::
#   DESTINATION lib/cmake/${PROJECT_NAME}
# )

# install(DIRECTORY config
#   DESTINATION .)

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

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_package()

