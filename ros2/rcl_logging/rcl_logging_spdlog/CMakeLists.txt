cmake_minimum_required(VERSION 3.5)

project(rcl_logging_spdlog)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(spdlog_vendor REQUIRED) # Provides spdlog 1.3.1 on platforms without it.
find_package(spdlog REQUIRED)

include_directories(include)

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(${PROJECT_NAME} src/rcl_logging_spdlog.cpp)
target_link_libraries(${PROJECT_NAME} spdlog::spdlog)

ament_target_dependencies(${PROJECT_NAME}
  rcutils
  spdlog
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RCL_LOGGING_BUILDING_DLL")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake rcutils)
ament_export_libraries(${PROJECT_NAME})
ament_package()
