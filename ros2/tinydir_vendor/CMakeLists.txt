cmake_minimum_required(VERSION 3.5)
project(tinydir_vendor)

list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")

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
externalproject_add(tinydir-1.2.4
  PREFIX tinydir-1.2.4
  URL https://github.com/cxong/tinydir/archive/1.2.4.tar.gz
  URL_MD5 15978c5f498e2ff3d26a9975c734cda0
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
    ${extra_cmake_args}
  BUILD_IN_SOURCE 1
  PATCH_COMMAND
    ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/tinydir_cmakelists.txt" <SOURCE_DIR>/CMakeLists.txt
)

# The external project installs into our build directory.
# When the vendor packge is installs, then we install the external project as well.
install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}
  USE_SOURCE_PERMISSIONS
)

configure_file(tinydir_vendorConfig.cmake.in "${PROJECT_BINARY_DIR}/tinydir_vendorConfig.cmake" @ONLY)
configure_file(tinydir_vendorConfig-version.cmake.in "${PROJECT_BINARY_DIR}/tinydir_vendorConfig-version.cmake" @ONLY)

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
  "${PROJECT_BINARY_DIR}/tinydir_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/tinydir_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
