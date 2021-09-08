cmake_minimum_required(VERSION 3.5)

project(tinyxml2_vendor)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(TinyXML2 REQUIRED)

configure_file(tinyxml2_vendorConfig.cmake.in
"${PROJECT_BINARY_DIR}/tinyxml2_vendorConfig.cmake" @ONLY)
configure_file(tinyxml2_vendorConfig-version.cmake.in
"${PROJECT_BINARY_DIR}/tinyxml2_vendorConfig-version.cmake" @ONLY)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME})

install(FILES
  "${PROJECT_BINARY_DIR}/tinyxml2_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/tinyxml2_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
