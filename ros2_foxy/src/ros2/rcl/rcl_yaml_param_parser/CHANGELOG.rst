^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcl_yaml_param_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2021-04-14)
-------------------
* Update quality declaration links (re: `ros2/docs.ros2.org#52 <https://github.com/ros2/docs.ros2.org/issues/52>`_) (`#910 <https://github.com/ros2/rcl/issues/910>`_)
* Contributors: Simon Honigmann

1.1.10 (2020-12-09)
-------------------
* Update build.ros2.org links (`#868 <https://github.com/ros2/rcl/issues/868>`_)
* Update QD to QL 1 (`#867 <https://github.com/ros2/rcl/issues/867>`_)
* Update QD (`#843 <https://github.com/ros2/rcl/issues/843>`_)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard, Jorge Perez, Stephen Brawner

1.1.9 (2020-11-03)
------------------
* Add mocking unit tests for rcl_yaml_param_parser (coverage part 3/3) (`#772 <https://github.com/ros2/rcl/issues/772>`_)
* Add fault-injection unit tests (coverage part 2/3) (`#766 <https://github.com/ros2/rcl/issues/766>`_)
* Add basic unit tests for refactored functions in rcl_yaml_param_parser (coverage part 1/3) (`#771 <https://github.com/ros2/rcl/issues/771>`_)
* Fix mem leaks in unit test from 776 (`#779 <https://github.com/ros2/rcl/issues/779>`_)
* remove debugging statements. (`#755 <https://github.com/ros2/rcl/issues/755>`_)
* Removed variant benchmark
* Several memory-related fixes for rcl_variant_t benchmarks (`#813 <https://github.com/ros2/rcl/issues/813>`_)
* Improved rcl_yaml_param_parser benchmark test (`#810 <https://github.com/ros2/rcl/issues/810>`_)
* Added benchmark test to rcl_yaml_param_parser (`#803 <https://github.com/ros2/rcl/issues/803>`_)
* Contributors: Alejandro Hernández Cordero, Scott K Logan, ahcorde, brawner, tomoya

1.1.8 (2020-10-07)
------------------
* Fix yaml parser error when meets .nan (refactor on `#754 <https://github.com/ros2/rcl/issues/754>`_) (`#781 <https://github.com/ros2/rcl/issues/781>`_) (`#785 <https://github.com/ros2/rcl/issues/785>`_)
* Refactor parser.c for better testability (`#754 <https://github.com/ros2/rcl/issues/754>`_) (`#784 <https://github.com/ros2/rcl/issues/784>`_)
* Don't overwrite cur_ns pointer if reallocation fails (`#780 <https://github.com/ros2/rcl/issues/780>`_) (`#783 <https://github.com/ros2/rcl/issues/783>`_)
* Set yaml_variant values to NULL on finalization (`#765 <https://github.com/ros2/rcl/issues/765>`_) (`#782 <https://github.com/ros2/rcl/issues/782>`_)
* Fix rcl_parse_yaml_file() error handling (`#776 <https://github.com/ros2/rcl/issues/776>`_) (`#786 <https://github.com/ros2/rcl/issues/786>`_)
* Contributors: Michel Hidalgo, Stephen Brawner

1.1.7 (2020-08-03)
------------------
* Removed doxygen warnings (`#712 <https://github.com/ros2/rcl/issues/712>`_) (`#724 <https://github.com/ros2/rcl/issues/724>`_)
* Contributors: Alejandro Hernández Cordero

1.1.6 (2020-07-07)
------------------

1.1.5 (2020-06-03)
------------------

1.1.4 (2020-06-02)
------------------

1.1.3 (2020-06-01)
------------------
* Add Security Vulnerability Policy pointing to REP-2006 (`#661 <https://github.com/ros2/rcl/issues/661>`_)
* Contributors: Chris Lalancette

1.1.2 (2020-05-28)
------------------

1.1.1 (2020-05-26)
------------------
* Increase rcl_yaml_param_parser test coverage (`#656 <https://github.com/ros2/rcl/issues/656>`_)
* Contributors: Stephen Brawner

1.1.0 (2020-05-22)
------------------
* Update Quality Declaration for 1.0 (`#647 <https://github.com/ros2/rcl/issues/647>`_)
* Contributors: brawner

1.0.0 (2020-05-12)
------------------

0.9.1 (2020-05-08)
------------------
* Included features (`#644 <https://github.com/ros2/rcl/issues/644>`_)
* Quality Declarations for rcl_action, rcl_lifecycle, yaml_parser (`#641 <https://github.com/ros2/rcl/issues/641>`_)
* Contributors: Alejandro Hernández Cordero, brawner

0.9.0 (2020-04-29)
------------------
* Added rcl yaml param parser doxyfile (`#634 <https://github.com/ros2/rcl/issues/634>`_)
* Fixed rcl_yaml_param_parser package description (`#637 <https://github.com/ros2/rcl/issues/637>`_)
* Fix usage to not expose underlying yaml (`#630 <https://github.com/ros2/rcl/issues/630>`_)
* Export targets in a addition to include directories / libraries (`#621 <https://github.com/ros2/rcl/issues/621>`_)
* Remove usage of undefined CMake variable (`#620 <https://github.com/ros2/rcl/issues/620>`_)
* Fix memory leaks (`#564 <https://github.com/ros2/rcl/issues/564>`_)
* Code style only: wrap after open parenthesis if not in one line (`#565 <https://github.com/ros2/rcl/issues/565>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, y-okumura-isp

0.8.3 (2019-11-08)
------------------

0.8.2 (2019-10-23)
------------------
* Specify test working directory (`#529 <https://github.com/ros2/rcl/issues/529>`_)
* Remove the maximum string size. (`#524 <https://github.com/ros2/rcl/issues/524>`_)
* Contributors: Chris Lalancette, Dan Rose

0.8.1 (2019-10-08)
------------------

0.8.0 (2019-09-26)
------------------
* Enable incremental parameter yaml file parsing. (`#507 <https://github.com/ros2/rcl/issues/507>`_)
* Support parameter overrides and remap rules flags on command line (`#483 <https://github.com/ros2/rcl/issues/483>`_)
* Increase MAX_STRING_SIZE (`#487 <https://github.com/ros2/rcl/issues/487>`_)
* include actual size in error message (`#490 <https://github.com/ros2/rcl/issues/490>`_)
* Avoid C4703 error on UWP (`#282 <https://github.com/ros2/rcl/issues/282>`_)
* [YAML Parser] Support parameter value parsing (`#471 <https://github.com/ros2/rcl/issues/471>`_)
* [YAML Parser] Depend on rcutils only (`#470 <https://github.com/ros2/rcl/issues/470>`_)
* Accept quoted int or float values as strings (`#464 <https://github.com/ros2/rcl/issues/464>`_)
* Fix memory corruption when maximum number of parameters is exceeded (`#456 <https://github.com/ros2/rcl/issues/456>`_)
* Contributors: Dirk Thomas, Esteve Fernandez, Jacob Perron, Michel Hidalgo, hyunseok-yang, ivanpauno

0.7.4 (2019-05-29)
------------------
* Allow empty strings if they are quoted. (`#450 <https://github.com/ros2/rcl/issues/450>`_)
* Contributors: Ralf Anton Beier

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------

0.7.1 (2019-04-29)
------------------

0.7.0 (2019-04-14)
------------------
* Corrected bool reading from yaml files. (`#415 <https://github.com/ros2/rcl/issues/415>`_)
* Added launch along with launch_testing as test dependencies. (`#393 <https://github.com/ros2/rcl/issues/393>`_)
* Set symbol visibility to hidden for rcl. (`#391 <https://github.com/ros2/rcl/issues/391>`_)
* Contributors: Michel Hidalgo, Sachin Suresh Bhat, ivanpauno

0.6.4 (2019-01-11)
------------------

0.6.3 (2018-12-13)
------------------

0.6.2 (2018-12-13)
------------------

0.6.1 (2018-12-07)
------------------
* No changes.

0.6.0 (2018-11-16)
------------------
* Updated to use new error handling API from rcutils (`#314 <https://github.com/ros2/rcl/issues/314>`_)
* Fixed FQN=//node_name when ns is / (`#299 <https://github.com/ros2/rcl/issues/299>`_)
* Fixed documentation issues (`#288 <https://github.com/ros2/rcl/issues/288>`_)
* Fixed to deallocate ret_val to avoid memory leak (`#278 <https://github.com/ros2/rcl/issues/278>`_)
* Contributors: Chris Ye, William Woodall, dhood

0.5.0 (2018-06-25)
------------------
* Added functions to parse YAML parameter files. (`#235 <https://github.com/ros2/rcl/issues/235>`_)
* Contributors: Shane Loretz, William Woodall, anup-pem, dhood
