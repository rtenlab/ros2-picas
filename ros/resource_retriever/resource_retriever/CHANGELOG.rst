^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package resource_retriever
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.2 (2020-04-09)
------------------

2.2.1 (2019-12-05)
------------------
* Catch ament_index_cpp::PackageNotFoundError (`#33 <https://github.com/ros/resource_retriever/issues/33>`_)
* Contributors: Shane Loretz

2.2.0 (2019-09-26)
------------------

2.1.1 (2019-05-08)
------------------
* Changed to export resource retriever and to enable hooks. (`#26 <https://github.com/ros/resource_retriever/issues/26>`_)
* Contributors: Steven! Ragnarök

2.1.0 (2018-06-21)
------------------
* Make sure to export the include directory for resource_retriever. (`#22 <https://github.com/ros/resource_retriever/issues/22>`_)
* Contributors: Chris Lalancette

1.12.3 (2017-03-27)
-------------------
* Fix C++11 to use set_directory_properties
* Make Shane and Chris the maintainers.
* Python3 compatibility (`#10 <https://github.com/ros/resource_retriever/issues/10>`_)
  * Replace urlgrabber with urllib[2]
  As urlgrabber is not supported for Python 3 replace it with either the built-in urllib (Python 2) or urllib2 (Python 3)
  * Use rospkg instead of system call for rospack
  * Add test for python functionality
  * Fix rospkg dependency definition
* Update URL in http test to something which exists (`#8 <https://github.com/ros/resource_retriever/issues/8>`_)
  * Update URL in http test to something which exists
* Contributors: Chris Lalancette, Mike Purvis, Ruben Smits

1.12.2 (2016-06-10)
-------------------
* fix failing build due to cmake error (`#6 <https://github.com/ros/resource_retriever/issues/6>`_)
* Contributors: Jackie Kay

1.12.1 (2016-06-10)
-------------------
* Fix warnings in test (`#5 <https://github.com/ros/resource_retriever/issues/5>`_)
  add spaces around ROS_PACKAGE_NAME
* Merge pull request `#4 <https://github.com/ros/resource_retriever/issues/4>`_ from DLu/kinetic-devel
  Add c++11 flag
* Contributors: David V. Lu!!, Jackie Kay, Steven Peters

1.12.0 (2016-03-23)
-------------------
* resource_retriever: adding missing dep
  Using the python resource_retriever requires the `python-urlgrabber` system dependency: http://rosindex.github.io/d/python-urlgrabber/
* Contributors: Jonathan Bohren

1.11.6 (2014-11-30)
-------------------

1.11.5 (2014-07-24)
-------------------

1.11.4 (2014-07-07)
-------------------

1.11.3 (2014-06-24)
-------------------

1.11.2 (2014-03-22)
-------------------

1.11.1 (2014-03-20)
-------------------

1.11.0 (2014-02-21)
-------------------

1.10.18 (2013-12-04)
--------------------
* add DEPENDS for kdl_parser
* Contributors: Ioan Sucan

1.10.16 (2013-11-18)
--------------------
* check for CATKIN_ENABLE_TESTING

1.10.15 (2013-08-17)
--------------------

* resource_retriever: install python package using setup.py
