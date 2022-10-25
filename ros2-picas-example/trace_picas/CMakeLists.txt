cmake_minimum_required(VERSION 3.5)
project(trace_picas)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(include)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# In this package, node is not needed.
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(SOURCES
  src/trace_function.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(trace_picas ${rclcpp_LIBRARIES})

ament_target_dependencies(trace_picas rclcpp)
ament_export_dependencies(rclcpp)
ament_export_targets(export_trace HAS_LIBRARY_TARGET)


#ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_trace
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
