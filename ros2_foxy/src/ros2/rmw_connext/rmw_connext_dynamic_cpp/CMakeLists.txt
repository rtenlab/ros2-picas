cmake_minimum_required(VERSION 3.5)

project(rmw_connext_dynamic_cpp)

set(CONNEXT_DYNAMIC_DISABLE $ENV{CONNEXT_DYNAMIC_DISABLE}
  CACHE BOOL "If Connext Dynamic should be disabled.")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(connext_cmake_module REQUIRED)
find_package(Connext QUIET MODULE)
if(Connext_FOUND)
  find_package(Connext MODULE)
endif()
if(NOT Connext_FOUND OR CONNEXT_DYNAMIC_DISABLE)
  ament_package()
  if(NOT Connext_FOUND)
    message(WARNING "Could not find RTI Connext - skipping '${PROJECT_NAME}'")
  else()
    message(STATUS "Connext dynamic rmw implementation explicitly disabled - skipping '${PROJECT_NAME}'")
  endif()
  return()
endif()

find_package(rcutils REQUIRED)

find_package(rosidl_typesupport_introspection_c)
find_package(rosidl_typesupport_introspection_cpp)
if(NOT rosidl_typesupport_introspection_c_FOUND AND NOT rosidl_typesupport_introspection_cpp_FOUND)
  ament_package()
  message(WARNING "Could not find ROSIDL TypeSupport for Connext (rosidl_typesupport_introspection_c|cpp) - skipping '${PROJECT_NAME}'")
  return()
endif()

find_package(rmw REQUIRED)
find_package(rmw_connext_shared_cpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

ament_export_dependencies(
  rcutils
  rmw rmw_connext_shared_cpp
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp
)

register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_introspection_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_introspection_cpp"
)

add_library(rmw_connext_dynamic_cpp SHARED src/functions.cpp src/publish_take.cpp src/rmw_node_info_and_types.cpp)
ament_target_dependencies(rmw_connext_dynamic_cpp
  "rcutils"
  "rosidl_typesupport_introspection_c"
  "rosidl_typesupport_introspection_cpp"
  "rmw"
  "rmw_connext_shared_cpp"
  "rosidl_runtime_c"
  "rosidl_runtime_cpp"
  "Connext")
if(Connext_GLIBCXX_USE_CXX11_ABI_ZERO)
  target_compile_definitions(rmw_connext_dynamic_cpp
    PRIVATE Connext_GLIBCXX_USE_CXX11_ABI_ZERO)
endif()
ament_export_libraries(rmw_connext_dynamic_cpp)

# On Windows this adds the RMW_BUILDING_DLL definition.
# On Unix (GCC or Clang) it hides the symbols by default with -fvisibility=hidden.
configure_rmw_library(rmw_connext_dynamic_cpp)
# Additionally, on Windows, add the ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_BUILDING_DLL definition.

if(WIN32)
  target_compile_definitions(rmw_connext_dynamic_cpp
    PRIVATE "_CRT_NONSTDC_NO_DEPRECATE")
  target_compile_definitions(rmw_connext_dynamic_cpp
    PRIVATE "ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_BUILDING_DLL")
endif()

if(NOT WIN32)
  set(rmw_connext_dynamic_cpp_ADDITIONAL_LIBRARIES dl pthread)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  TARGETS rmw_connext_dynamic_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

get_rmw_connext_output_filter("rmw_connext_output_filter")
ament_index_register_resource("rmw_output_prefixes" CONTENT "${rmw_connext_output_filter}")
