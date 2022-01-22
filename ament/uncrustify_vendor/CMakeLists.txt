cmake_minimum_required(VERSION 3.10)

project(uncrustify_vendor)

option(FORCE_BUILD_VENDOR_PKG
  "Build uncrustify from source, even if system-installed package is available"
  OFF)

find_package(ament_cmake REQUIRED)

macro(build_uncrustify)
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
  endif()

  include(ExternalProject)

  # Pinning to tip of master at the time of a desired bugfix
  set(uncrustify_version 45b836cff040594994d364ad6fd3f04adc890a26)
  # Get uncrustify 0.68.1
  ExternalProject_Add(uncrustify-${uncrustify_version}
    GIT_REPOSITORY https://github.com/uncrustify/uncrustify.git
    GIT_TAG ${uncrustify_version}
    GIT_CONFIG advice.detachedHead=false
    # Suppress git update due to https://gitlab.kitware.com/cmake/cmake/-/issues/16419
    # See https://github.com/ament/uncrustify_vendor/pull/22 for details
    UPDATE_COMMAND ""
    TIMEOUT 600
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
      ${extra_cmake_args}
      -Wno-dev
    PATCH_COMMAND
      ${CMAKE_COMMAND} -E chdir <SOURCE_DIR> git apply -p1 --ignore-space-change --whitespace=nowarn ${CMAKE_CURRENT_SOURCE_DIR}/install-patch.diff
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
  )
endmacro()

execute_process(
  COMMAND uncrustify --version
  RESULT_VARIABLE res
  OUTPUT_VARIABLE out
  ERROR_VARIABLE err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
set(need_local_build FALSE)
if(NOT res EQUAL 0)
  set(need_local_build TRUE)
else()
  # version of uncrustify before 0.65 user a different versionning scheme so the regex wont be matched
  string(REGEX REPLACE "^Uncrustify-(.*)_f$" "\\1" version_prefix_match "${out}")
  if(version_prefix_match STREQUAL "" OR version_prefix_match VERSION_LESS 0.68)
    set(need_local_build TRUE)
  endif()
endif()
if(FORCE_BUILD_VENDOR_PKG OR need_local_build)
  build_uncrustify()
endif()

# this ensures that the package has an environment hook setting the PATH
ament_package()
