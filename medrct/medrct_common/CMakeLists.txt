cmake_minimum_required(VERSION 3.0.2)
project(medrct_common VERSION 0.0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build Library
add_library(${PROJECT_NAME} SHARED
  src/controller/controller_factory.cc
  src/controller/controller_manager_config.cc
  src/controller/controller_manager.cc
  src/controller/controller.cc
  src/controller/utils.cc
  src/intra_stream/intra_common.cc
  src/intra_stream/intra_executor.cc
  src/intra_stream/intra_stream_factory.cc
  src/stream/condition_wait.cc
  src/stream/stream.cc
  src/file_paths.cc
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
find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
target_link_libraries(${PROJECT_NAME} PUBLIC Boost::boost
    ${CMAKE_DL_LIBS}
    Boost::system
    Boost::filesystem
    Boost::serialization
    yaml-cpp)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
set(CMAKE_PKG_REQ_PUB "${CMAKE_PKG_REQ_PUB} Threads")

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

find_package(spdlog REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC spdlog)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  PACKAGE_BUILD_DIR="${CMAKE_CURRENT_BINARY_DIR}"
  PACKAGE_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}")

# Build Scripts
add_executable(run_intra_stream scripts/run_intra_stream.cc)
target_link_libraries(run_intra_stream ${PROJECT_NAME})

add_executable(test_eigen scripts/test_eigen.cc)
target_link_libraries(test_eigen ${PROJECT_NAME})

add_executable(test_yaml scripts/test_yaml.cc)
target_link_libraries(test_yaml ${PROJECT_NAME} yaml-cpp)

#############
## Testing ##
#############

option(BUILD_TESTING "Build testing" ON)
if(BUILD_TESTING)
  enable_testing()
  include(GoogleTest)
  include(Dart)

  add_executable(tests
  tests/test_main.cc
  tests/intra_stream_test.cc
  tests/joint_state_test.cc
  tests/logger_test.cc
  tests/quaternion_test.cc
  tests/transform_test.cc
  tests/twist_test.cc
  tests/vector3_test.cc
  )

  target_compile_options(tests
  PRIVATE
  -Wno-ignored-qualifiers -Wno-unused-variable -Wno-pointer-arith
  -Wno-unused-but-set-variable
  )
  target_link_libraries(tests PRIVATE
  gtest ${PROJECT_NAME}
  Eigen3::Eigen
  Threads::Threads
  spdlog
  )
  target_include_directories(tests PRIVATE include)
  gtest_discover_tests(tests DISCOVERY_TIMEOUT 60)

  ## Plugin Library
  add_library(${PROJECT_NAME}_test_plugin_multiply SHARED tests/dummy_plugins/test_plugin_multiply.cpp)
  target_link_libraries(${PROJECT_NAME}_test_plugin_multiply PUBLIC ${PROJECT_NAME} ${Boost_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_test_plugin_multiply
  PRIVATE
  -Wno-ignored-qualifiers -Wno-unused-variable -Wno-pointer-arith
  -Wno-unused-but-set-variable
  )

  ## Plugin loader unit test
  add_executable(plugin_loader_unit_test
  tests/plugin_loader_unit.cc
  )
  target_link_libraries(plugin_loader_unit_test PRIVATE
  gtest ${PROJECT_NAME}
  Threads::Threads
  spdlog
  yaml-cpp
  )
  target_compile_definitions(plugin_loader_unit_test PRIVATE TEST_PLUGIN_DIR="${CMAKE_CURRENT_BINARY_DIR}")
  target_compile_options(plugin_loader_unit_test
  PRIVATE
  -Wno-ignored-qualifiers -Wno-unused-variable -Wno-pointer-arith
  -Wno-unused-but-set-variable
  )
  gtest_discover_tests(plugin_loader_unit_test DISCOVERY_TIMEOUT 60)

endif()

# Install
install(DIRECTORY include/
DESTINATION include)

install(
  TARGETS ${PROJECT_NAME} run_intra_stream test_eigen test_yaml
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
