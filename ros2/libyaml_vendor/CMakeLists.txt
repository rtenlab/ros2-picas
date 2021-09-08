cmake_minimum_required(VERSION 3.5)

project(libyaml_vendor)

find_package(ament_cmake REQUIRED)

macro(build_libyaml)
  set(extra_cmake_args)
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  list(APPEND extra_cmake_args -DBUILD_SHARED_LIBS=ON)
  list(APPEND extra_cmake_args -DBUILD_TESTING=OFF)
  # Disable MSVC warnings
  if(CMAKE_C_COMPILER_ID MATCHES "MSVC")
    set(win_c_flags "${CMAKE_C_FLAGS} /wd4244 /wd4267 /wd4996")
    list(APPEND extra_cmake_args -DCMAKE_C_FLAGS=${win_c_flags})
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
  endif()
  include(ExternalProject)
  ExternalProject_Add(libyaml-10c9078
    URL https://github.com/yaml/libyaml/archive/10c907871f1ccd779c7fccf6b81a62762b5c4e7b.zip
    URL_MD5 b595f0ab0735c04c7f6ed0a798a3eff7
    TIMEOUT 60
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/libyaml_install
      ${extra_cmake_args}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/libyaml_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    PATTERN config.h EXCLUDE
  )
endmacro()

build_libyaml()

ament_export_libraries(yaml)

ament_package()
