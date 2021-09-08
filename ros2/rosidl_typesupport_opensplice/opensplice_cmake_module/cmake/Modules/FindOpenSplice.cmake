# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###############################################################################
#
# CMake module for finding PrismTech OpenSplice.
#
# Input variables:
#
# - OSPL_HOME (optional): When specified, header files and libraries
#   will be searched for in `${OSPL_HOME}/include` and
#   `${OSPL_HOME}/lib` respectively.
#
# Output variables:
#
# - OpenSplice_FOUND: flag indicating if the package was found
# - OpenSplice_INCLUDE_DIRS: Paths to the header files
# - OpenSplice_LIBRARIES: Name to the C++ libraries including the path
# - OpenSplice_LIBRARY_DIRS: Paths to the libraries
# - OpenSplice_DEFINITIONS: Definitions to be passed on
# - OpenSplice_IDLPP: Path to the idl2code generator
#
# Example usage:
#
#   find_package(opensplice_cmake_module REQUIRED)
#   find_package(OpenSplice MODULE)
#   # use OpenSplice_* variables
#
###############################################################################

# lint_cmake: -convention/filename, -package/stdargs

if(DEFINED OpenSplice_FOUND)
  return()
endif()

set(OpenSplice_FOUND FALSE)

# check if provided OSPL_HOME is from an "official" binary package
set(_ospl_home "")
if(DEFINED ENV{OSPL_HOME})
  file(TO_CMAKE_PATH "$ENV{OSPL_HOME}" _ospl_home)
endif()
if(WIN32)
  set(_ospl_release_file release.bat)
else()
  set(_ospl_release_file release.com)
endif()
if(NOT _ospl_home STREQUAL "" AND NOT EXISTS "${_ospl_home}/${_ospl_release_file}")
  set(_ospl_home "")
endif()

if(NOT _ospl_home STREQUAL "")
  # look inside of OSPL_HOME if defined
  message(STATUS "Found PrismTech OpenSplice: ${_ospl_home}")
  set(OpenSplice_HOME "${_ospl_home}")
  set(OpenSplice_INCLUDE_DIRS
    "${_ospl_home}/include"
    "${_ospl_home}/include/sys"
    "${_ospl_home}/include/dcps/C++/SACPP")
  set(OpenSplice_LIBRARIES
    "cmxml"
    "dcpsgapi"
    "dcpssac"
    "ddsconfparser"
    "ddsconf"
    "ddsdatabase"
    "ddsi2"
    "ddskernel"
    "ddsosnet"
    "ddsos"
    "ddsserialization"
    "ddsuser"
    "ddsutil"
    "spliced"
    # we can't link against both sacpp and isocpp at the same time
    #"dcpsisocpp"
    "dcpssacpp")
  set(OpenSplice_LIBRARY_DIRS "${_ospl_home}/lib")
  set(OpenSplice_DEFINITIONS "")
  set(OpenSplice_IDLPP "${_ospl_home}/bin/idlpp")
  set(OpenSplice_FOUND TRUE)
else()
  # try to find_package() it
  set(quiet "")
  if(OpenSplice_FIND_QUIETLY)
    set(quiet "QUIET")
  endif()
  find_package(opensplice ${quiet} NO_MODULE COMPONENTS CXX
    PATHS $ENV{AMENT_PREFIX_PATH} /usr /usr/local)
  if(OPENSPLICE_FOUND)
    message(STATUS "Found PrismTech OpenSplice: ${opensplice_DIR}")
    set(OpenSplice_HOME "${OPENSPLICE_PREFIX}")
    set(OpenSplice_INCLUDE_DIRS ${OPENSPLICE_INCLUDE_DIRS})
    set(OpenSplice_LIBRARIES "")
    foreach(_lib ${OPENSPLICE_LIBRARIES})
      get_filename_component(_name "${_lib}" NAME_WE)
      # we can't link against both sacpp and isocpp at the same time
      if(NOT _name STREQUAL "libdcpsisocpp" AND NOT _name STREQUAL "dcpsisocpp")
        list(APPEND OpenSplice_LIBRARIES "${_lib}")
      endif()
    endforeach()
    set(OpenSplice_LIBRARY_DIRS "")
    set(OpenSplice_DEFINITIONS ${OPENSPLICE_DEFINITIONS})
    set(OpenSplice_IDLPP "${OPENSPLICE_IDLPP}")
    set(OpenSplice_FOUND TRUE)
  endif()
endif()

# convert libraries into absolute paths
set(_libs ${OpenSplice_LIBRARIES})
set(OpenSplice_LIBRARIES)
foreach(_library ${_libs})
  if(NOT IS_ABSOLUTE "${_library}")
    set(_lib "NOTFOUND")
    find_library(
      _lib "${_library}"
      HINTS ${OpenSplice_LIBRARY_DIRS})
    if(NOT _lib)
      message(FATAL_ERROR "OpenSplice exports the library '${_library}' which couldn't be found")
    elseif(NOT IS_ABSOLUTE "${_lib}")
      message(FATAL_ERROR "The OpenSplice library '${_library}' was found at '${_lib}' which is not an absolute path")
    elseif(NOT EXISTS "${_lib}")
      message(FATAL_ERROR "The OpenSplice library '${_library}' was found at '${_lib}' which doesn't exist")
    endif()
    set(_library "${_lib}")
  endif()
  list(APPEND OpenSplice_LIBRARIES "${_library}")
endforeach()

if(NOT WIN32)
  list(APPEND OpenSplice_LIBRARIES "pthread" "dl")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenSplice
  FOUND_VAR OpenSplice_FOUND
  REQUIRED_VARS
    OpenSplice_INCLUDE_DIRS
    OpenSplice_LIBRARIES
    OpenSplice_IDLPP
)
