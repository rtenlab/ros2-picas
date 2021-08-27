cmake_minimum_required(VERSION 3.5)

project(connext_cmake_module)

find_package(ament_cmake REQUIRED)

set(${PROJECT_NAME}_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(Connext QUIET MODULE)
if(Connext_FOUND)
  find_package(Connext MODULE)
endif()
if(Connext_HOME)
  file(TO_NATIVE_PATH "${Connext_HOME}" Connext_HOME)
  if(WIN32)
    ament_environment_hooks(env_hook/connext.bat.in)
  else()
    ament_environment_hooks(env_hook/connext.sh.in)
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "connext_cmake_module-extras.cmake"
)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})
