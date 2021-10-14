^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package can_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-11-05)
------------------
* port can_msgs to ROS2
* Contributors: Joshua Whitley, Mathias Lüdtke

0.8.2 (2019-11-04)
------------------

0.8.1 (2019-07-14)
------------------
* Set C++ standard to c++14
* Contributors: Harsh Deshpande

0.8.0 (2018-07-11)
------------------

0.7.8 (2018-05-04)
------------------

0.7.7 (2018-05-04)
------------------

0.7.6 (2017-08-30)
------------------

0.7.5 (2017-05-29)
------------------

0.7.4 (2017-04-25)
------------------

0.7.3 (2017-04-25)
------------------

0.7.2 (2017-03-28)
------------------

0.7.1 (2017-03-20)
------------------

0.7.0 (2016-12-13)
------------------

0.6.5 (2016-12-10)
------------------
* hamonized versions
* styled and sorted CMakeLists.txt
  * removed boilerplate comments
  * indention
  * reviewed exported dependencies
* styled and sorted package.xml
* Adds message_runtime to can_msgs dependencies.
  Added the missing dependency, also changes message_generation to a build_depend.
* Finalizes work on the socketcan_bridge and can_msgs.
  Readies the packages socketcan_bridge and can_msgs for the merge with ros_canopen.
  Bumps the version for both packages to 0.1.0. Final cleanup in CMakeLists, added
  comments to the shell script and launchfile used for testing.
* Introduces the can_msgs package for message types.
  Package to hold CAN message types, the Frame message type can contain the data
  as returned by SocketCAN.
* Contributors: Ivor Wanders, Mathias Lüdtke
