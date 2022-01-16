cmake_minimum_required(VERSION 3.5)
project(urdf_parser_plugin)

find_package(ament_cmake_ros REQUIRED)
find_package(urdfdom_headers REQUIRED)

# Create and export a header-only target
add_library(urdf_parser_plugin INTERFACE)
target_include_directories(urdf_parser_plugin INTERFACE
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(urdf_parser_plugin INTERFACE
  "urdfdom_headers"
)
install(TARGETS urdf_parser_plugin EXPORT urdf_parser_plugin-export)
ament_export_targets(urdf_parser_plugin-export)
ament_export_dependencies(urdfdom_headers)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
