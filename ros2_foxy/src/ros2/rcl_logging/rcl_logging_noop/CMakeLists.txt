cmake_minimum_required(VERSION 3.5)

project(rcl_logging_noop)

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

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(${PROJECT_NAME}_sources
  src/rcl_logging_noop/rcl_logging_noop.cpp
)

if(NOT WIN32)
  add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources})
else()
  # TODO(nburek): There is an issue with build on Windows where it is looking for a static lib
  # and can't find it when trying to call find_package() for this package. This explicitly creates
  # it.
  add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})
endif()

ament_target_dependencies(${PROJECT_NAME}
  rcutils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_export_dependencies(ament_cmake)
ament_export_libraries(${PROJECT_NAME})
ament_package()
