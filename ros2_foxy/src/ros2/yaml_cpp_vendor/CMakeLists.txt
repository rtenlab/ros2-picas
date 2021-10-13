cmake_minimum_required(VERSION 3.5)

project(yaml_cpp_vendor)

find_package(ament_cmake REQUIRED)

set(PACKAGE_VERSION "1.0.0")

macro(build_yaml_cpp)
  set(extra_cmake_args)
  set(YAML_C_FLAGS ${CMAKE_C_FLAGS})
  set(YAML_CXX_FLAGS ${CMAKE_CXX_FLAGS})

  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(NOT WIN32)
    list(APPEND YAML_C_FLAGS "-w")
    list(APPEND YAML_CXX_FLAGS "-std=c++14 -w")
  endif()

  list(APPEND extra_cmake_args "-DYAML_CPP_BUILD_TESTS=OFF")
  list(APPEND extra_cmake_args "-DYAML_CPP_BUILD_TOOLS=OFF")
  list(APPEND extra_cmake_args "-DYAML_CPP_BUILD_CONTRIB=OFF")
  list(APPEND extra_cmake_args "-DBUILD_SHARED_LIBS=ON")
  list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=${YAML_C_FLAGS}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${YAML_CXX_FLAGS}")

  if(WIN32 AND NOT ${CMAKE_VERBOSE_MAKEFILE})
    set(should_log ON)  # prevent warnings in Windows CI
  else()
    set(should_log OFF)
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  endif()

  include(ExternalProject)
  # This specific version (past the current latest release of 0.5.3) is required to make
  # yaml-cpp relocatable, hopefully it is released again soon.
  # See: https://github.com/jbeder/yaml-cpp/pull/538
  # Latest release fails to compile on recent visual studio (VS2017 v15.8.1)
  # See: https://github.com/jbeder/yaml-cpp/pull/597
  ExternalProject_Add(yaml_cpp-0f9a586
    URL https://github.com/jbeder/yaml-cpp/archive/0f9a586ca1dc29c2ecb8dd715a315b93e3f40f79.zip
    URL_MD5 ec76c27ebd07d5178cbe85b773df8e62
    TIMEOUT 600
    LOG_CONFIGURE ${should_log}
    LOG_BUILD ${should_log}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/yaml_cpp_install
      ${extra_cmake_args}
      -Wno-dev
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/yaml_cpp_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/yaml_cpp_vendor
  )
endmacro()

build_yaml_cpp()

if(WIN32)
  ament_environment_hooks(env_hook/yaml_cpp_vendor_library_path.bat)
  set(ENV_VAR_NAME "PATH")
  set(ENV_VAR_VALUE "opt\\yaml_cpp_vendor\\bin")
else()
  ament_environment_hooks(env_hook/yaml_cpp_vendor_library_path.sh)
  if(APPLE)
    set(ENV_VAR_NAME "DYLD_LIBRARY_PATH")
  else()
    set(ENV_VAR_NAME "LD_LIBRARY_PATH")
  endif()
  set(ENV_VAR_VALUE "opt/yaml_cpp_vendor/lib")
endif()
ament_environment_hooks(env_hook/yaml_cpp_vendor_library_path.dsv.in)

ament_package(
  CONFIG_EXTRAS "yaml_cpp_vendor-extras.cmake.in"
)
