cmake_minimum_required(VERSION 3.5)
project(picas_example_mt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-reorder)
endif()

if(${PICAS})
  add_compile_options(-DPICAS)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(test_msgs REQUIRED)
find_package(trace_picas REQUIRED)

add_executable(example_mt src/example_mt.cpp)
ament_target_dependencies(example_mt rclcpp test_msgs trace_picas)

install(TARGETS
  example_mt
  DESTINATION lib/${PROJECT_NAME})    

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

