cmake_minimum_required(VERSION 3.0.2)
project(medrct_franka_env VERSION 0.0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(${PROJECT_NAME} SHARED
  src/franka_kinematics_factory.cc
  src/franka_kinematics_solver.cc
  src/franka_kinematics.cpp
)

find_package(yaml-cpp REQUIRED)
find_package(medrct_common REQUIRED)
find_package(medrct_env REQUIRED)

target_include_directories(${PROJECT_NAME}
PUBLIC
$<INSTALL_INTERFACE:include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
)

# include_directories(
# include

# )

target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  medrct_common::medrct_common
  medrct_env::medrct_env
)

#############
## Testing ##
#############

option(BUILD_TESTING "Build testing" ON)
if(BUILD_TESTING)
  enable_testing()
  include(GoogleTest)

  add_executable(franka_tests # name tests cannot be used with catkin dependency
    tests/main.cc
    tests/franka_kinematics_test.cc
    tests/franka_loader_kinematics_test.cc
  )

  target_compile_options(franka_tests
    PRIVATE
    -Wno-ignored-qualifiers -Wno-unused-variable -Wno-pointer-arith
    -Wno-unused-but-set-variable
  )
  target_link_libraries(franka_tests
    PRIVATE gtest ${PROJECT_NAME}
    Threads::Threads)
  target_include_directories(franka_tests PRIVATE include)

  gtest_discover_tests(franka_tests DISCOVERY_TIMEOUT 60)
endif()

# Install
install(DIRECTORY include/
DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
)

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
