^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_typesupport_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2021-04-14)
------------------
* Update quality declaration links (re: `ros2/docs.ros2.org#52 <https://github.com/ros2/docs.ros2.org/issues/52>`_) (`#109 <https://github.com/ros2/rosidl_typesupport/issues/109>`_)
* Contributors: Simon Honigmann

1.0.1 (2020-12-08)
------------------
* Update Quality Declaration to QL 1 (`#97 <https://github.com/ros2/rosidl_typesupport/issues/97>`_)
* Add mock for rcutils_get_symbol failure (`#93 <https://github.com/ros2/rosidl_typesupport/issues/93>`_) (`#94 <https://github.com/ros2/rosidl_typesupport/issues/94>`_)
* Added benchmark test to rosidl_typesupport_c/cpp (`#84 <https://github.com/ros2/rosidl_typesupport/issues/84>`_)
* Catch exception from has_symbol (`#86 <https://github.com/ros2/rosidl_typesupport/issues/86>`_)
* Handle rcpputils::find_library_path() failure (`#85 <https://github.com/ros2/rosidl_typesupport/issues/85>`_)
* Add fault injection macros and unit tests (`#80 <https://github.com/ros2/rosidl_typesupport/issues/80>`_)
* Remove rethrow in extern c code (`#82 <https://github.com/ros2/rosidl_typesupport/issues/82>`_)
* Add Security Vulnerability Policy pointing to REP-2006. (`#76 <https://github.com/ros2/rosidl_typesupport/issues/76>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Jose Luis Rivero, Jose Tomas Lorente, Louise Poubel, Michel Hidalgo, Stephen Brawner

1.0.0 (2020-05-26)
------------------
* Addresses test failures in release mode (`#75 <https://github.com/ros2/rosidl_typesupport/issues/75>`_)
* Add tests for type_support functions (`#63 <https://github.com/ros2/rosidl_typesupport/issues/63>`_)
* Contributors: Stephen Brawner

0.9.2 (2020-05-19)
------------------

0.9.1 (2020-05-19)
------------------
* Force extension points to be registered in order (`#73 <https://github.com/ros2/rosidl_typesupport/issues/73>`_)
* Update API documentation and quality declarations (`#74 <https://github.com/ros2/rosidl_typesupport/issues/74>`_)
* Complete feature documentation (`#72 <https://github.com/ros2/rosidl_typesupport/issues/72>`_)
* Add API documentation for public functions (`#64 <https://github.com/ros2/rosidl_typesupport/issues/64>`_)
* Add current quality level declarations (`#67 <https://github.com/ros2/rosidl_typesupport/issues/67>`_)
* Contributors: Ivan Santiago Paunovic, brawner

0.9.0 (2020-04-24)
------------------
* Fix single typesupport build exposing build directory in include dirs (`#71 <https://github.com/ros2/rosidl_typesupport/issues/71>`_)
* Export targets in addition to include directories / libraries (`#69 <https://github.com/ros2/rosidl_typesupport/issues/69>`_ `#70 <https://github.com/ros2/rosidl_typesupport/issues/70>`_)
* Fix build with single introspection typesupport (`#68 <https://github.com/ros2/rosidl_typesupport/issues/68>`_)
* Update includes to use non-entry point headers from detail subdirectory (`#66 <https://github.com/ros2/rosidl_typesupport/issues/66>`_)
* Rename rosidl_generator_c namespace to rosidl_runtime_c (`#65 <https://github.com/ros2/rosidl_typesupport/issues/65>`_)
* Remove dependency on rmw_implementation (`#62 <https://github.com/ros2/rosidl_typesupport/issues/62>`_)
* Added rosidl_runtime c depencency (`#58 <https://github.com/ros2/rosidl_typesupport/issues/58>`_)
* Removed poco dependency (`#59 <https://github.com/ros2/rosidl_typesupport/issues/59>`_)
* Remove OpenSplice dependencies (`#56 <https://github.com/ros2/rosidl_typesupport/issues/56>`_)
* Depend on rcpputils` for find_library (`#47 <https://github.com/ros2/rosidl_typesupport/issues/47>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Eric Cousineau, Jacob Perron, Sean Kelly

0.8.0 (2019-09-26)
------------------

0.7.1 (2019-05-08)
------------------
* update code to match refactoring of rosidl definitions (`#49 <https://github.com/ros2/rosidl_typesupport/issues/49>`_)
* remove usage of UnknownMessageType (`#48 <https://github.com/ros2/rosidl_typesupport/issues/48>`_)
* Contributors: Dirk Thomas

0.7.0 (2019-04-14)
------------------
* Using ament_target_dependencies where possible (`#46 <https://github.com/ros2/rosidl_typesupport/issues/46>`_)
* change generators to IDL-based pipeline (`#39 <https://github.com/ros2/rosidl_typesupport/issues/39>`_)
* Contributors: Dirk Thomas, ivanpauno

0.6.2 (2019-01-11)
------------------
* include available typesuppports in error message (`#43 <https://github.com/ros2/rosidl_typesupport/issues/43>`_)
* Change uncrustify max line length to 0 (`#42 <https://github.com/ros2/rosidl_typesupport/issues/42>`_)
* Contributors: Dirk Thomas, Jacob Perron

0.6.1 (2018-12-07)
------------------
* Merge pull request `#41 <https://github.com/ros2/rosidl_typesupport/issues/41>`_ from ros2/hidmic/trim-action-targets-names
* Contributors: Michel Hidalgo

0.6.0 (2018-11-16)
------------------
* Add typesupport for actions in c and c++ (`#36 <https://github.com/ros2/rosidl_typesupport/issues/36>`_)
* Allow generated IDL files (`#35 <https://github.com/ros2/rosidl_typesupport/issues/35>`_)
* Merge pull request `#33 <https://github.com/ros2/rosidl_typesupport/issues/33>`_ from ros2/hidmic/prepare_for_action_generation
* Removes remaininig srv folder assumptions.
* Makes rosidl interfaces generation action folder aware.
* update manifest to adhere to tag order in schema (`#30 <https://github.com/ros2/rosidl_typesupport/issues/30>`_)
* Contributors: Alexis Pojomovsky, Dirk Thomas, Michel Hidalgo, Shane Loretz

0.5.0 (2018-06-24)
------------------
* Prepare dependencies for bouncy release. (`#27 <https://github.com/ros2/rosidl_typesupport/issues/27>`_)
* add and use groups for generator and runtime packages (`#25 <https://github.com/ros2/rosidl_typesupport/issues/25>`_)
* Merge pull request `#23 <https://github.com/ros2/rosidl_typesupport/issues/23>`_ from ros2/misra_fixup
* Merge pull request `#22 <https://github.com/ros2/rosidl_typesupport/issues/22>`_ from ros2/use_typesupport_group
* use CMAKE_CURRENT_BINARY_DIR for arguments json (`#21 <https://github.com/ros2/rosidl_typesupport/issues/21>`_)
* use ament_cmake_ros (`#19 <https://github.com/ros2/rosidl_typesupport/issues/19>`_)
* Contributors: Dirk Thomas, Michael Carroll, Steven! Ragnarök

0.4.0 (2017-12-08)
------------------
* update service type support header name (`#15 <https://github.com/ros2/rosidl_typesupport/issues/15>`_)
* Contributors: Dirk Thomas, Mikael Arguedas
