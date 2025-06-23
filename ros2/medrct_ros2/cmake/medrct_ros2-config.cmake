# Report to users when this package has been found:
if(NOT @PROJECT_NAME@_FIND_QUIETLY)
  message(STATUS "Found @PROJECT_NAME@: @PROJECT_VERSION@ (${@PROJECT_NAME@_DIR})")
endif()

if(NOT TARGET @PROJECT_NAME@::@PROJECT_NAME@)

  include(CMakeFindDependencyMacro)
  foreach(public_link_library @CMAKE_PKG_REQ_PUB@)
    find_dependency(${public_link_library} REQUIRED)
  endforeach()

  find_package(PkgConfig REQUIRED)

  foreach(public_link_library @PKG_CONFIG_REQ_PUB@)
    pkg_check_modules(pkg REQUIRED ${public_link_library})
  endforeach()

  include(${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@-targets.cmake)

  if (NOT TARGET @PROJECT_NAME@)
    # Add support for `target_link_libraries(my_project @PROJECT_NAME@)`:
    add_library(@PROJECT_NAME@ INTERFACE IMPORTED)
    target_link_libraries(@PROJECT_NAME@ INTERFACE @PROJECT_NAME@::@PROJECT_NAME@)

    # Add support for `ament_target_dependencies(my_project @PROJECT_NAME@)`:
    get_property(definitions TARGET @PROJECT_NAME@::@PROJECT_NAME@ PROPERTY INTERFACE_COMPILE_DEFINITIONS)
    get_property(libraries TARGET @PROJECT_NAME@::@PROJECT_NAME@ PROPERTY INTERFACE_LINK_LIBRARIES)
    get_property(include_dirs TARGET @PROJECT_NAME@::@PROJECT_NAME@ PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    set(@PROJECT_NAME@_DEFINITIONS ${definitions})
    set(@PROJECT_NAME@_LIBRARIES ${libraries})
    set(@PROJECT_NAME@_INCLUDE_DIRS ${include_dirs})
  endif()
endif()
