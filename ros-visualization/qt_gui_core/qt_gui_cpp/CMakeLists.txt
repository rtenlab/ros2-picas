cmake_minimum_required(VERSION 3.5)
project(qt_gui_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

if(WIN32)
  message(STATUS "${PROJECT_NAME} is not yet supported on Windows. Package will not be built.")
  ament_package()
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/plugin.xml"
    "<library path=\"src\"/>")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/plugin.xml"
    DESTINATION share/${PROJECT_NAME})
  return()
endif()

find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

add_subdirectory(src)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(pluginlib rcpputils)
ament_export_include_directories(include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# TODO(brawner) add ament_lint_auto tests here. Much of the c++ code needs to
# be updated for ROS2 style

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(qt_gui_cpp test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 90)
endif()

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)
