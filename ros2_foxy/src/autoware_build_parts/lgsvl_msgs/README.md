# ROS Package lgsvl_msgs for LG SVL Automotive Simulator

This repository contains ROS message definitions for lgsvl_msgs to subscribe ROS messages being published by LG SVL Automotive Simulator via rosbridge.

```text
  - Detection3DArray.msg    # A list of 3D detections
  - Detection3D.msg         # 3D detection including id, label, score, and 3D bounding box
  - BoundingBox3D.msg       # A 3D bounding box definition
  - Detection2DArray.msg    # A list of 2D detections
  - Detection2D.msg         # 2D detection including id, label, score, and 2D bounding box
  - BoundingBox2D.msg       # A 2D bounding box definition
  - SignalArray.msg         # A list of traffic light detections
  - Signal.msg              # 3D detection of a traffic light including id, label, score, and 3D bounding box
  - CanBusData.msg          # Can bus data for an ego vehicle published by the simulator
  - VehicleControlData.msg  # Vehicle control commands that the simulator subscribes to
  - VehicleStateData.msg    # Description of the full state of an ego vehicle
  - Ultrasonic.msg          # Minimum detected distance by an ultrasonic sensor
  - VehicleOdometry.msg     # Odometry for an ego vehicle
```

## Copyright and License

Copyright (c) 2018-2020 LG Electronics, Inc.

This software contains code licensed as described in LICENSE.
