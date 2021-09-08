cmake_minimum_required(VERSION 3.5)

project(spdlog_vendor)

find_package(ament_cmake REQUIRED)

option(FORCE_BUILD_VENDOR_PKG
  "Build spdlog from source, even if system-installed package is available"
  OFF)

if(NOT FORCE_BUILD_VENDOR_PKG)
  find_package(spdlog QUIET)
endif()

macro(build_spdlog)
  set(extra_cmake_args)
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
    list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  endif()
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
  list(APPEND extra_cmake_args "-DSPDLOG_BUILD_BENCH=OFF")
  list(APPEND extra_cmake_args "-DSPDLOG_BUILD_TESTS=OFF")
  list(APPEND extra_cmake_args "-DSPDLOG_BUILD_EXAMPLES=OFF")

  include(ExternalProject)
  # Get spdlog 1.3.1
  ExternalProject_Add(spdlog-1.3.1
    URL https://github.com/gabime/spdlog/archive/v1.3.1.tar.gz
    URL_MD5 3c17dd6983de2a66eca8b5a0b213d29f
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
endmacro()

if(NOT spdlog_FOUND OR "${spdlog_VERSION}" VERSION_LESS 1.3.1)
  build_spdlog()
endif()

# this ensures that the package has an environment hook setting the PATH
ament_package()
