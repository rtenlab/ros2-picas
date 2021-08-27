project(lgsvl_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(MSG_FILES
  "BoundingBox2D.msg"
  "BoundingBox3D.msg"
  "CanBusData.msg"
  "DetectedRadarObjectArray.msg"
  "DetectedRadarObject.msg"
  "Detection2D.msg"
  "Detection2DArray.msg"
  "Detection3D.msg"
  "Detection3DArray.msg"
  "Signal.msg"
  "SignalArray.msg"
  "Ultrasonic.msg"
  "VehicleControlData.msg"
  "VehicleStateData.msg"
  "VehicleOdometry.msg"
)

if(${ROS_VERSION} EQUAL 1)
  cmake_minimum_required(VERSION 2.8.3)

  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    sensor_msgs
    std_msgs
  )

  # Default to C++11
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 11)
  endif()

  add_message_files(
    DIRECTORY msg
    FILES ${MSG_FILES}
  )

  generate_messages(
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    sensor_msgs
    std_msgs
  )
elseif(${ROS_VERSION} EQUAL 2)
  cmake_minimum_required(VERSION 3.5)

  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # Apend "msg/" to each file name
  set(TEMP_LIST "")
  foreach(MSG_FILE ${MSG_FILES})
    list(APPEND TEMP_LIST "msg/${MSG_FILE}")
  endforeach()
  set(MSG_FILES ${TEMP_LIST})

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
    DEPENDENCIES
      builtin_interfaces
      geometry_msgs
      sensor_msgs
      std_msgs
    ADD_LINTER_TESTS
  )

  ament_export_dependencies(rosidl_default_runtime)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()
