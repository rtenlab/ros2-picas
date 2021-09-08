cmake_minimum_required(VERSION 3.5)

project(rcpputils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

ament_export_include_directories(include)

add_library(${PROJECT_NAME}
  src/find_library.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
)
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "RCPPUTILS_BUILDING_LIBRARY")
endif()
ament_target_dependencies(${PROJECT_NAME} rcutils)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wthread-safety -Werror)
  endif()

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_thread_safety_annotations test/test_thread_safety_annotations.cpp)

  ament_add_gtest(test_join test/test_join.cpp)

  ament_add_gtest(test_split test/test_split.cpp)

  ament_add_gtest(test_filesystem_helper test/test_filesystem_helper.cpp)

  ament_add_gtest(test_find_and_replace test/test_find_and_replace.cpp)

  ament_add_gtest(test_pointer_traits test/test_pointer_traits.cpp)

  ament_add_gtest(test_endian test/test_endian.cpp)

  add_library(test_library SHARED test/test_library.cpp)
  ament_add_gtest(test_find_library test/test_find_library.cpp)
  target_link_libraries(test_find_library ${PROJECT_NAME} test_library)
  set_tests_properties(test_find_library PROPERTIES
    ENVIRONMENT
      "_TEST_LIBRARY_DIR=$<TARGET_FILE_DIR:test_library>;_TEST_LIBRARY=$<TARGET_FILE:test_library>")
endif()

ament_package()

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
