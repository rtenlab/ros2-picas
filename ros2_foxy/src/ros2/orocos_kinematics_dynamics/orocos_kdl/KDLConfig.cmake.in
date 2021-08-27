# - Config file for the orocos-kdl package
# It defines the following variables
#  orocos_kdl_INCLUDE_DIRS - include directories for Orocos KDL
#  orocos_kdl_LIBRARIES    - libraries to link against for Orocos KDL
#  orocos_kdl_PKGCONFIG_DIR - directory containing the .pc pkgconfig files

# Compute paths

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
get_filename_component(_orocos_kdl_include_dir "${CMAKE_CURRENT_LIST_DIR}/../../../include" ABSOLUTE)
set(orocos_kdl_INCLUDE_DIRS "${_orocos_kdl_include_dir}" "${Eigen3_INCLUDE_DIRS}")

if(NOT TARGET orocos-kdl)
  include("${CMAKE_CURRENT_LIST_DIR}/OrocosKDLTargets.cmake")
endif()

set(orocos_kdl_LIBRARIES orocos-kdl)
set(orocos_kdl_TARGETS orocos-kdl)

# where the .pc pkgconfig files are installed
get_filename_component(orocos_kdl_PKGCONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/pkgconfig" ABSOLUTE)
