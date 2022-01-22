^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libstatistics_collector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2021-08-05)
------------------
* Remove cloudwatch reporting (`#110 <https://github.com/ros-tooling/libstatistics_collector/issues/110>`_)
* Replace index.ros.org links -> docs.ros.org. (`#94 <https://github.com/ros-tooling/libstatistics_collector/issues/94>`_)
* Use latest versions of CI actions (`#92 <https://github.com/ros-tooling/libstatistics_collector/issues/92>`_)
* Contributors: Chris Lalancette, Emerson Knapp

1.1.0 (2021-03-19)
------------------
* fix: measured values after the decimal point are truncated `#79 <https://github.com/ros-tooling/libstatistics_collector/issues/79>`_ (`#80 <https://github.com/ros-tooling/libstatistics_collector/issues/80>`_)
* Update linter to run on rolling+focal (`#81 <https://github.com/ros-tooling/libstatistics_collector/issues/81>`_)
* Add automerge.yml config file (`#70 <https://github.com/ros-tooling/libstatistics_collector/issues/70>`_)
* Update QD to QL 1 (`#68 <https://github.com/ros-tooling/libstatistics_collector/issues/68>`_)
* Updated QD (`#64 <https://github.com/ros-tooling/libstatistics_collector/issues/64>`_)
* Updated QD Performance tests (`#58 <https://github.com/ros-tooling/libstatistics_collector/issues/58>`_)
* Added benchmark test to libstatistics_collector (`#57 <https://github.com/ros-tooling/libstatistics_collector/issues/57>`_)
  * Added benchmark test to libstatistics_collector
  * cppcheck supressed unknown macro warning - macos
  * Reset heap counters
  * Added feedback
  * Remove unknownMacro suppression from CMakeLists.txt
  * Added feedback
  * moved benchmark test to test/benchmark
  * Added feedback
  Co-authored-by: Devin Bonnie <47613035+dabonnie@users.noreply.github.com>
* Report failed workflows (`#56 <https://github.com/ros-tooling/libstatistics_collector/issues/56>`_)
  Allow codecov failures to be silent
* Add default CODEOWNERS file (`#55 <https://github.com/ros-tooling/libstatistics_collector/issues/55>`_)
* Remove repo activity from individual repositories in favor of centralized reporting (`#52 <https://github.com/ros-tooling/libstatistics_collector/issues/52>`_)
* Don't attempt to report if originating from a fork (`#43 <https://github.com/ros-tooling/libstatistics_collector/issues/43>`_)
* Removed doxygen warnings (`#41 <https://github.com/ros-tooling/libstatistics_collector/issues/41>`_)
  Co-authored-by: Anas Abou Allaban <allabana@amazon.com>
* Add autoapprove action for dependabot (`#40 <https://github.com/ros-tooling/libstatistics_collector/issues/40>`_)
* Create Dependabot config file (`#31 <https://github.com/ros-tooling/libstatistics_collector/issues/31>`_)
  * Create Dependabot config file
  * Randomize time of run
  Co-authored-by: dependabot-preview[bot] <27856297+dependabot-preview[bot]@users.noreply.github.com>
  Co-authored-by: Prajakta Gokhale <prajaktg@amazon.com>
* Updated QD to 3 (`#30 <https://github.com/ros-tooling/libstatistics_collector/issues/30>`_)
* Add Security Vulnerability Policy pointing to REP-2006. (`#24 <https://github.com/ros-tooling/libstatistics_collector/issues/24>`_)
  Co-authored-by: Emerson Knapp <537409+emersonknapp@users.noreply.github.com>
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Devin Bonnie, Emerson Knapp, Lucas Han, Prajakta Gokhale, Stephen Brawner, hsgwa

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
