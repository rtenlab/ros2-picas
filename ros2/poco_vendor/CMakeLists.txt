cmake_minimum_required(VERSION 3.5)
project(poco_vendor VERSION "1.1.1")

option(FORCE_BUILD_VENDOR_PKG
  "Build Poco from source, even if system-installed package is available"
  OFF)

# Can work with poco 1.4.1p1 (earliest to use recursive mutexes on Linux)
# 1.6.1 is the first version to ship with PocoConfigVersion.cmake
if(NOT FORCE_BUILD_VENDOR_PKG)
  find_package(Poco "1.6.1" COMPONENTS Foundation QUIET)
endif()


if(NOT Poco_FOUND)
  set(POCO_CXX_FLAGS ${CMAKE_CXX_FLAGS})

  # If Poco was not found, download and build from source
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(WIN32)
    list(APPEND POCO_CXX_FLAGS "/wd4244 /wd4530 /wd4577")
  else()
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=-Wno-shift-negative-value")
    list(APPEND extra_cmake_args "-DCMAKE_CXX_STANDARD=14")
    if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND NOT ANDROID)
      list(APPEND extra_cmake_args "-DPOCO_UNBUNDLED:BOOL=ON")
    endif()
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
        list(APPEND extra_cmake_args
          "-DANDROID_FUNCTION_LEVEL_LINKING=${ANDROID_FUNCTION_LEVEL_LINKING}")
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
  endif()
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${POCO_CXX_FLAGS}")
  list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  include(ExternalProject)

  ExternalProject_Add(poco-1.8.0.1-release
    URL https://github.com/pocoproject/poco/archive/poco-1.8.0.1-release.tar.gz
    URL_MD5 07aa03d7976d0dbc141d95821c104c10
    TIMEOUT 600
    CMAKE_ARGS
      -DENABLE_CRYPTO:BOOL=OFF
      -DENABLE_DATA:BOOL=OFF
      -DENABLE_JSON:BOOL=OFF
      -DENABLE_MONGODB:BOOL=OFF
      -DENABLE_NET:BOOL=OFF
      -DENABLE_NETSSL:BOOL=OFF
      -DENABLE_PAGECOMPILER_FILE2PAGE:BOOL=OFF
      -DENABLE_PAGECOMPILER:BOOL=OFF
      -DENABLE_REDIS:BOOL=OFF
      -DENABLE_UTIL:BOOL=OFF
      -DENABLE_XML:BOOL=OFF
      -DENABLE_ZIP:BOOL=OFF
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/poco_external_project_install
      ${extra_cmake_args}
      -Wno-dev
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/poco_external_project_install/
    DESTINATION ${CMAKE_INSTALL_PREFIX})
else()
  message(STATUS "Found Poco ${Poco_VERSION}")
endif()

configure_file(poco_vendorConfig.cmake.in
  "${PROJECT_BINARY_DIR}/poco_vendorConfig.cmake" @ONLY)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/poco_vendorConfig-version.cmake"
  COMPATIBILITY AnyNewerVersion)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME})

install(FILES
  "${PROJECT_BINARY_DIR}/poco_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/poco_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
