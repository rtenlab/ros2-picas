^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libstatistics_collector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2020-05-27)
------------------
* Added quality declaration (`#21 <https://github.com/ros-tooling/libstatistics_collector/issues/21>`_)
  * Added quality declaration
  * Added feedback
  * Fixed rep link
  * Fixed QD
  * added feedback
  * Added feedback
* Added Doxyfile (`#23 <https://github.com/ros-tooling/libstatistics_collector/issues/23>`_)
* Run CI on Focal (`#20 <https://github.com/ros-tooling/libstatistics_collector/issues/20>`_)
* Run lint worflow on Docker (`#19 <https://github.com/ros-tooling/libstatistics_collector/issues/19>`_)
* Fix annotation syntax for thread safety attributes (`#18 <https://github.com/ros-tooling/libstatistics_collector/issues/18>`_)
* Remove unused strategy matrix for ASAN CI job (`#17 <https://github.com/ros-tooling/libstatistics_collector/issues/17>`_)
* Refactor workflow to extract CW reporting (`#15 <https://github.com/ros-tooling/libstatistics_collector/issues/15>`_)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Thomas Moulard

1.0.0 (2020-04-29)
------------------
* Bump actions versions (`#14 <https://github.com/ros-tooling/libstatistics_collector/issues/14>`_)
  * Bump actions versions
  * Use upload-artifact v1
* Bump setup-ros to 0.0.20 (`#10 <https://github.com/ros-tooling/libstatistics_collector/issues/10>`_)
  0.0.20 is also installing numpy which currently
  prevents the CI runs from succeeding.
* Log workflow results to CloudWatch (`#11 <https://github.com/ros-tooling/libstatistics_collector/issues/11>`_)
* Add repo activity workflow (`#12 <https://github.com/ros-tooling/libstatistics_collector/issues/12>`_)
* export targets in a addition to include directories / libraries (`#8 <https://github.com/ros-tooling/libstatistics_collector/issues/8>`_)
* Fix windows warning (`#6 <https://github.com/ros-tooling/libstatistics_collector/issues/6>`_)
* Apply windows vibility fix changes (`#5 <https://github.com/ros-tooling/libstatistics_collector/issues/5>`_)
  * Apply windows vibility fix changes
  * Add test fixes
  * Add ament_cmake_ros dependency
  * Alphasort CmakeLists.txt
  * Alphasort package.xml
  * Alphasort collector.hpp
  * Add quotes in cmake
* Fix README test badge (`#4 <https://github.com/ros-tooling/libstatistics_collector/issues/4>`_)
  * Fix README test badge
  * Fix bracket
* Fix license format and add linter actions (`#3 <https://github.com/ros-tooling/libstatistics_collector/issues/3>`_)
* Update GH Actions badge
* Move libstatistics_collector folder from system_metrics_collector (`#2 <https://github.com/ros-tooling/libstatistics_collector/issues/2>`_)
  * Moved libstatistics_collector folder from system_metrics_collector
  * Address review comments
  * Add actions, reflect changes in statistics_msgs
  * Add CONTRIBUTING.md
  Co-authored-by: Prajakta Gokhale <prajaktg@amazon.com>
* Initial commit
* Contributors: Devin Bonnie, Dirk Thomas, Emerson Knapp, Prajakta Gokhale, Thomas Moulard
