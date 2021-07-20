Change history
==============

2.1.1 (2020-04-25)
------------------
* Enable linter tests on unique_identifier_msgs (`#5 <https://github.com/ros2/unique_identifier_msgs/issues/5>`_)
* Contributors: Jorge Perez

2.1.0 (2019-04-14)
------------------
* Added mapping rule for ros1_bridge (`#3 <https://github.com/ros2/unique_identifier_msgs/issues/3>`_)
* Contributors: Paul Bovbel

2.0.0 (2018-11-21)
------------------

* Migrating from its old (language mixed) repository

  Old Repo: https://github.com/ros-geographic-info/unique_identifier

  Name Change: uuid_msgs -> unique_identifier_msgs

  The name change is to line it up with the names of the langauge specific wrappers that will exist around this message artifact.
  As this exactly describes the semantic meaning of the message (a UUID is defined as exactly a 128 bit unique identifier).
* Dropped std_msgs dependency
* Update maintainer

1.1.0 (2018-11-01)
------------------
* Renamed (match c++/python package names): uuid_msgs -> unique_identifier_msgs
* Renamed (exact semantic): UniqueID.msg -> UUID.msg

1.0.4 (2014-04-30)
------------------

 * add architecture independent tag

1.0.1 (2013-03-25)
-------------------

 * Hydro release update.

1.0.0 (2013-03-18)
-------------------

 * Hydro release.
 * Convert to catkin (`#1`_).

0.9.0 (2013-01-03)
------------------

 * Initial release to Groovy.

0.8.0 (2012-07-19)
------------------

 * Initial release to Fuerte.
 * Provides uuid_msgs/UniqueID message.

.. _`#1`: https://github.com/ros-geographic-info/unique_identifier/issues/1
