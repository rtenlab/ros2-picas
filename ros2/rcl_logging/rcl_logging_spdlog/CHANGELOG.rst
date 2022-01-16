^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcl_logging_spdlog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.2 (2021-04-06)
------------------
* updating quality declaration links (re: `ros2/docs.ros2.org#52 <https://github.com/ros2/docs.ros2.org/issues/52>`_) (`#73 <https://github.com/ros2/rcl_logging/issues/73>`_)
* Contributors: shonigmann

2.1.1 (2021-01-25)
------------------
* Include what you use (`#71 <https://github.com/ros2/rcl_logging/issues/71>`_)
* Contributors: Ivan Santiago Paunovic

2.1.0 (2020-12-08)
------------------
* Update QD to QL 1 (`#66 <https://github.com/ros2/rcl_logging/issues/66>`_)
* Make sure to check return value from external_initialize. (`#65 <https://github.com/ros2/rcl_logging/issues/65>`_)
* updated QD section 3.i and 3ii and spelling error (`#63 <https://github.com/ros2/rcl_logging/issues/63>`_)
* rcl_logging_spdlog: Increased QL to 2 in QD
* Updated spdlog QL in QD
* Make internal dependencies private (`#60 <https://github.com/ros2/rcl_logging/issues/60>`_)
* [rcl_logging_spdlog] Add warnings (`#54 <https://github.com/ros2/rcl_logging/issues/54>`_)
* Allow configuring logging directory through environment variables (`#53 <https://github.com/ros2/rcl_logging/issues/53>`_)
* Update the maintainers. (`#55 <https://github.com/ros2/rcl_logging/issues/55>`_)
* Added benchmark test to rcl_logging_spdlog (`#52 <https://github.com/ros2/rcl_logging/issues/52>`_)
* Used current_path() function from rcpputils (`#51 <https://github.com/ros2/rcl_logging/issues/51>`_)
* Add fault injection unittest to increase coverage (`#49 <https://github.com/ros2/rcl_logging/issues/49>`_)
* Contributors: Alejandro Hernández Cordero, Audrow Nash, Chris Lalancette, Christophe Bedard, Shane Loretz, Stephen Brawner, ahcorde, brawner

2.0.1 (2020-07-21)
------------------
* Bump QD to level 3 and updated QD (`#44 <https://github.com/ros2/rcl_logging/issues/44>`_)
* Added Doxyfile and fixed related warnings (`#42 <https://github.com/ros2/rcl_logging/issues/42>`_)
* Contributors: Alejandro Hernández Cordero

2.0.0 (2020-06-18)
------------------
* Use new package with rcl logging interface (`#41 <https://github.com/ros2/rcl_logging/issues/41>`_)
* Increased test coverage (`#40 <https://github.com/ros2/rcl_logging/issues/40>`_)
* Add Security Vulnerability Policy pointing to REP-2006.
* Rename Quality_Declaration.md -> QUALITY_DECLARATION.md
* Contributors: Chris Lalancette, Scott K Logan

1.0.0 (2020-05-26)
------------------
* Add some preliminary functional tests (`#36 <https://github.com/ros2/rcl_logging/issues/36>`_)
* warn about unused return value for set_logger_level (`#38 <https://github.com/ros2/rcl_logging/issues/38>`_)
* Added features to rcl_logging_spdlog (`#35 <https://github.com/ros2/rcl_logging/issues/35>`_)
* Added public API documentation for log4cxx and spdlog (`#32 <https://github.com/ros2/rcl_logging/issues/32>`_)
* Current state Quality Declaration, Contributing and Readme files (`#29 <https://github.com/ros2/rcl_logging/issues/29>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Jorge Perez, Scott K Logan

0.4.0 (2020-04-24)
------------------
* Export targets in addition to include directories / libraries (`#31 <https://github.com/ros2/rcl_logging/issues/31>`_)
* Make spdlog an exec_depend (`#27 <https://github.com/ros2/rcl_logging/issues/27>`_)
* Code style only: wrap after open parenthesis if not in one line (`#24 <https://github.com/ros2/rcl_logging/issues/24>`_)
* Bypass spdlog singleton registry (`#23 <https://github.com/ros2/rcl_logging/issues/23>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Ivan Santiago Paunovic

0.3.3 (2019-10-23)
------------------
* Fix Clang warning about possible uninitialized variable (`#21 <https://github.com/ros2/rcl_logging/issues/21>`_)
* Contributors: Jacob Perron

0.3.2 (2019-10-07)
------------------
* spdlog is a header-only library, so the exported dep isn't needed. (`#19 <https://github.com/ros2/rcl_logging/issues/19>`_)
* Contributors: Chris Lalancette

0.3.1 (2019-10-03)
------------------
* Implement a backend logger using spdlog. (`#17 <https://github.com/ros2/rcl_logging/issues/17>`_)
* Contributors: Chris Lalancette

0.3.0 (2019-09-26)
------------------

0.2.1 (2019-05-08)
------------------

0.2.0 (2019-03-09)
------------------

0.1.0 (2018-12-07)
------------------
