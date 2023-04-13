cmake_minimum_required(VERSION 3.0.2)
project(medrct_env VERSION 0.0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Library
add_library(${PROJECT_NAME} SHARED
  src/description/chain.cc
  src/description/joint.cc
  src/description/kinematics_tree.cc
  src/description/object.cc
  src/description/world.cc
  src/kinematics/simple_forward_kinematics.cc
)

target_include_directories(${PROJECT_NAME}
PUBLIC
$<INSTALL_INTERFACE:include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
)

# target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME})
find_package(yaml-cpp REQUIRED)

find_package(medrct_common REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC medrct_common::medrct_common)

find_package(urdfdom REQUIRED COMPONENTS urdfdom_model)
target_link_libraries(${PROJECT_NAME} PUBLIC urdfdom_model)

# Scripts
# add_executable(kdl_psm_test benchmark/kdl_psm_test.cc)

# find_package(orocos_kdl REQUIRED)
# target_link_libraries(kdl_psm_test orocos-kdl)

#############
## Testing ##
#############

option(BUILD_TESTING "Build testing" ON)
if(BUILD_TESTING)
  enable_testing()
  include(GoogleTest)
  include(Dart)

  add_executable(tests
    tests/main.cc
    tests/simple_fk_test.cc
  )
  target_compile_options(tests
    PRIVATE
    -Wno-ignored-qualifiers -Wno-unused-variable -Wno-pointer-arith
    -Wno-unused-but-set-variable
  )
  target_link_libraries(tests
    PRIVATE gtest ${PROJECT_NAME}
    Threads::Threads)
  target_include_directories(tests PRIVATE include)
  gtest_discover_tests(tests DISCOVERY_TIMEOUT 60)

  add_executable(world_test tests/world_test.cc)
  target_compile_options(world_test
    PRIVATE
    -Wno-ignored-qualifiers -Wno-unused-variable -Wno-pointer-arith
    -Wno-unused-but-set-variable
  )
  target_link_libraries(world_test
    PRIVATE gtest ${PROJECT_NAME}
    Threads::Threads)
  target_compile_definitions(world_test PRIVATE TEST_WORLD_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/world_test.urdf")
  target_compile_definitions(world_test PRIVATE CMAKE_CURRENT_LIST_DIR="${CMAKE_CURRENT_LIST_DIR}")

  target_include_directories(world_test PRIVATE include)
  gtest_discover_tests(world_test DISCOVERY_TIMEOUT 60)
endif()

install(DIRECTORY include/
DESTINATION include)

install(
  TARGETS ${PROJECT_NAME} #kdl_psm_test
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
