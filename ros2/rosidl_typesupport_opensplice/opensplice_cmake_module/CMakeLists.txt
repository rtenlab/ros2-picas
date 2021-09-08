cmake_minimum_required(VERSION 3.5)

project(opensplice_cmake_module NONE)

find_package(ament_cmake REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(OpenSplice MODULE)
if(OpenSplice_HOME)
  normalize_path(OpenSplice_HOME "${OpenSplice_HOME}")
  # if the home is in the install space make the environment hook relocatable
  if(OpenSplice_HOME STREQUAL CMAKE_INSTALL_PREFIX)
    set(OpenSplice_HOME "$AMENT_CURRENT_PREFIX")
  endif()
  file(TO_NATIVE_PATH "${OpenSplice_HOME}" OpenSplice_HOME)
  if(WIN32)
    ament_environment_hooks(env_hook/opensplice.bat.in)
  else()
    ament_environment_hooks(env_hook/opensplice.sh.in)
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "opensplice_cmake_module-extras.cmake"
)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
