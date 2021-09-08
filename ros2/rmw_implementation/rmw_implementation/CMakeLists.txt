cmake_minimum_required(VERSION 3.5)

project(rmw_implementation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rcutils REQUIRED)

find_package(rmw_implementation_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

get_default_rmw_implementation(RMW_IMPLEMENTATION)

message(STATUS "")
message(STATUS "Default rmw implementation: '${RMW_IMPLEMENTATION}'")
message(STATUS "")

get_available_rmw_implementations(available_rmw_implementations)
list(LENGTH available_rmw_implementations count_available_rmw_implementations)
if(count_available_rmw_implementations EQUAL 1)
  set(rmw_implementation_disable_runtime_selection_default ON)
else()
  set(rmw_implementation_disable_runtime_selection_default OFF)
endif()

option(RMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION "\
  Use only the RMW implementation specified at build time via the \
  'RMW_IMPLEMENTATION' environment variable and the 'RMW_IMPLEMENTATION' \
  CMake option, ignoring the runtime value of the 'RMW_IMPLEMENTATION' \
  environment variable"
  ${rmw_implementation_disable_runtime_selection_default})

if(RMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION)
  message(STATUS "Runtime selection of RMW disabled; Only using "
    "'${RMW_IMPLEMENTATION}'")
else()
  message(STATUS "Runtime selection of RMW enabled")

  # provides FindPoco.cmake and Poco on platforms without it
  find_package(poco_vendor REQUIRED)
  find_package(Poco REQUIRED COMPONENTS Foundation)
  find_package(rmw REQUIRED)

  add_library(${PROJECT_NAME} SHARED
    src/functions.cpp)
  ament_target_dependencies(${PROJECT_NAME}
    "Poco"
    "rmw")
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC "DEFAULT_RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}")
  configure_rmw_library(${PROJECT_NAME})

  ament_export_libraries(${PROJECT_NAME})

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

register_rmw_implementation(
  "c:rosidl_typesupport_c"
  "cpp:rosidl_typesupport_cpp"
)

ament_package(
  CONFIG_EXTRAS "rmw_implementation-extras.cmake.in"
)
