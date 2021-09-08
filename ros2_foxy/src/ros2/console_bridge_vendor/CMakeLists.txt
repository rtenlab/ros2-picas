cmake_minimum_required(VERSION 3.5)

project(console_bridge_vendor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

option(FORCE_BUILD_VENDOR_PKG
  "Build console_bridge from source, even if system-installed package is available"
  OFF)

if(NOT FORCE_BUILD_VENDOR_PKG)
  find_package(console_bridge QUIET)
endif()

macro(build_console_bridge)
  include(GNUInstallDirs)

  set(extra_cmake_args
    "-DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}"
    "-DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_INCLUDEDIR}"
  )
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
    if(ANDROID)
      if(DEFINED ANDROID_ABI)
        list(APPEND extra_cmake_args "-DANDROID_ABI=${ANDROID_ABI}")
      endif()
      if(DEFINED ANDROID_CPP_FEATURES)
        list(APPEND extra_cmake_args "-DANDROID_CPP_FEATURES=${ANDROID_CPP_FEATURES}")
      endif()
      if(DEFINED ANDROID_FUNCTION_LEVEL_LINKING)
        list(APPEND extra_cmake_args "-DANDROID_FUNCTION_LEVEL_LINKING=${ANDROID_FUNCTION_LEVEL_LINKING}")
      endif()
      if(DEFINED ANDROID_NATIVE_API_LEVEL)
        list(APPEND extra_cmake_args "-DANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL}")
      endif()
      if(DEFINED ANDROID_NDK)
        list(APPEND extra_cmake_args "-DANDROID_NDK=${ANDROID_NDK}")
      endif()
      if(DEFINED ANDROID_STL)
        list(APPEND extra_cmake_args "-DANDROID_STL=${ANDROID_STL}")
      endif()
      if(DEFINED ANDROID_TOOLCHAIN_NAME)
        list(APPEND extra_cmake_args "-DANDROID_TOOLCHAIN_NAME=${ANDROID_TOOLCHAIN_NAME}")
      endif()
    endif()
  else()
    list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
    list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  endif()
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

  include(ExternalProject)

  # Get console_bridge 0.5.1
  externalproject_add(console_bridge-1.0.1
    URL https://github.com/ros/console_bridge/archive/1.0.1.tar.gz
    URL_MD5 9f5351994c7690b37f88e9f70e478061
    TIMEOUT 600
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
      ${extra_cmake_args}
      -Wno-dev
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
  )

  set(console_bridge_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/${CMAKE_INSTALL_INCLUDEDIR})
  set(console_bridge_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/${CMAKE_INSTALL_LIBDIR})
  set(console_bridge_LIBRARIES console_bridge)
endmacro()

if(NOT console_bridge_FOUND OR "${console_bridge_VERSION}" VERSION_LESS 1.0.1)
  build_console_bridge()
  set(extra_test_dependencies console_bridge-1.0.1)
endif()

# TODO(blast545): This section and package.xml should be updated once copyright auto is solved.
# See: https://github.com/ament/ament_lint/issues/237
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(performance_test_fixture REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  get_target_property(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS
    performance_test_fixture::performance_test_fixture INTERFACE_INCLUDE_DIRECTORIES)

  add_performance_test(benchmark_log test/benchmark/benchmark_log.cpp)
  if(TARGET benchmark_log)
    target_include_directories(benchmark_log PRIVATE ${console_bridge_INCLUDE_DIRS})
    target_link_libraries(benchmark_log ${console_bridge_LIBRARIES})
    if(${CMAKE_VERSION} VERSION_LESS "3.13")
      link_directories(${console_bridge_LIBRARY_DIRS})
    else()
      target_link_directories(benchmark_log PRIVATE ${console_bridge_LIBRARY_DIRS})
    endif()
    if(extra_test_dependencies)
      add_dependencies(benchmark_log ${extra_test_dependencies})
    endif()
  endif()
endif()

# this ensures that the package has an environment hook setting the PATH
ament_package()
