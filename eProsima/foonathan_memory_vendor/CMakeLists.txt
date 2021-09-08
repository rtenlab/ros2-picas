cmake_minimum_required(VERSION 3.5)
project(foonathan_memory_vendor VERSION "1.0.0")

find_package(foonathan_memory QUIET)

if(NOT foonathan_memory_FOUND)
  ###############################################################################
  # Default shared libraries
  ###############################################################################
  # Global flag to cause add_library() to create shared libraries if on.
  # If set to true, this will cause all libraries to be built shared
  # unless the library was explicitly added as a static library.
  option(BUILD_SHARED_LIBS "Create shared libraries by default" ON)

  if(BUILD_SHARED_LIBS)
    # Library will be statically created with PIC code
    list(APPEND extra_cmake_args -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
  endif()

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
  include(ExternalProject)

  if(INSTALLER_PLATFORM)
    set(PATCH_COMMAND_STR PATCH_COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt.patch && sed -i -e "s/INSTALLER_PLATFORM/${INSTALLER_PLATFORM}/g" CMakeLists.txt)
  endif()

  externalproject_add(foo_mem-ext
  GIT_REPOSITORY https://github.com/foonathan/memory.git
  GIT_TAG c619113
  TIMEOUT 600
  # Avoid the update (git pull) and so the recompilation of foonathan_memory library each time.
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DFOONATHAN_MEMORY_BUILD_EXAMPLES=OFF
    -DFOONATHAN_MEMORY_BUILD_TESTS=OFF
    -DFOONATHAN_MEMORY_BUILD_TOOLS=ON
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/foo_mem_ext_prj_install
    ${extra_cmake_args}
    -Wno-dev
    ${PATCH_COMMAND_STR}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/foo_mem_ext_prj_install/
  DESTINATION ${CMAKE_INSTALL_PREFIX})
else()
  message(STATUS "Found foonathan_memory ${foonathan_memory_VERSION}")
endif()

configure_file(foonathan_memory_vendorConfig.cmake.in
  "${PROJECT_BINARY_DIR}/foonathan_memory_vendorConfig.cmake" @ONLY)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/foonathan_memory_vendorConfig-version.cmake"
  COMPATIBILITY AnyNewerVersion)

find_package(ament_cmake_test QUIET)
if(BUILD_TESTING)
  find_package(ament_cmake_copyright QUIET)
  find_package(ament_cmake_lint_cmake QUIET)
  find_package(ament_cmake_xmllint QUIET)
  if(ament_cmake_copyright_FOUND)
    ament_copyright()
  endif()
  if(ament_cmake_lint_cmake_FOUND)
    ament_lint_cmake()
  endif()
  if(ament_cmake_xmllint_FOUND)
    ament_xmllint()
  endif()
endif()

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME})

install(FILES
  "${PROJECT_BINARY_DIR}/foonathan_memory_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/foonathan_memory_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
