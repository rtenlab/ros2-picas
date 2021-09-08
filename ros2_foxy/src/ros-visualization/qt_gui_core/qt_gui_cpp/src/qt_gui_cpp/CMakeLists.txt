set(Python_ADDITIONAL_VERSIONS "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

set(qt_gui_cpp_SRCS
  composite_plugin_provider.cpp
  generic_proxy.cpp
  plugin_bridge.cpp
  plugin_context.cpp
  plugin_descriptor.cpp
  plugin_provider.cpp
  recursive_plugin_provider.cpp
  settings.cpp
)

set(qt_gui_cpp_HDRS
  ../../include/qt_gui_cpp/plugin.h
  ../../include/qt_gui_cpp/plugin_bridge.h
  ../../include/qt_gui_cpp/plugin_context.h
)

set(qt_gui_cpp_INCLUDE_DIRECTORIES
  ${PYTHON_INCLUDE_DIR}
  ${PROJECT_NAME}/include/qt_gui_cpp
)

set(qt_gui_cpp_LINK_LIBRARIES
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

qt5_wrap_cpp(qt_gui_cpp_MOCS ${qt_gui_cpp_HDRS})

ament_export_dependencies(pluginlib TinyXML2)
add_library(${PROJECT_NAME} ${qt_gui_cpp_SRCS} ${qt_gui_cpp_MOCS})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PRIVATE ${qt_gui_cpp_INCLUDE_DIRECTORIES})
ament_target_dependencies(${PROJECT_NAME} pluginlib TinyXML2)

if (WIN32)
  # On Windows systems, Visual Studio does not currently set __cplusplus correctly unless this
  # compile option is added. __cplusplus is used by a header in pluginlib/class_loader.hpp
  message(STATUS "On Windows, setting /Zc:__cplusplus")
  target_compile_options(${PROJECT_NAME} PUBLIC "/Zc:__cplusplus")
endif()

target_link_libraries(${PROJECT_NAME} ${qt_gui_cpp_LINK_LIBRARIES} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
