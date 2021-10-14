^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package connext_cmake_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2021-04-14)
------------------
* Update maintainers (`#64 <https://github.com/ros2/rosidl_typesupport_connext/issues/64>`_)
* Contributors: Jacob Perron

1.0.2 (2020-09-29)
------------------

1.0.1 (2020-08-12)
------------------
* remove pthread link flags for Android NDK cross-compiling (`#56 <https://github.com/ros2/rosidl_typesupport_connext/issues/56>`_)
* Contributors: jayhou

1.0.0 (2020-05-19)
------------------
* Fix failure when find-packaging Connext Module and c/cxx languages are not enabled (`#55 <https://github.com/ros2/rosidl_typesupport_connext/issues/55>`_)
* Contributors: Ivan Santiago Paunovic

0.9.0 (2020-04-24)
------------------
* Fix CMake warning about using uninitialized variables (`#52 <https://github.com/ros2/rosidl_typesupport_connext/issues/52>`_)
* Add missing project() command to check_abi CMakeLists.txt (`#48 <https://github.com/ros2/rosidl_typesupport_connext/issues/48>`_)
* Set ddsgen and ddsgen_server paths to absolute paths for Windows (`#46 <https://github.com/ros2/rosidl_typesupport_connext/issues/46>`_)
* Ignore -Wclass-memaccess build warnings coming from Connext 5.3.1 (`#45 <https://github.com/ros2/rosidl_typesupport_connext/issues/45>`_)
* Contributors: Dirk Thomas, Jacob Perron, Stephen Brawner

0.8.4 (2019-12-04)
------------------

0.8.3 (2019-12-04)
------------------

0.8.2 (2019-10-23)
------------------

0.8.1 (2019-10-04)
------------------
* Inline ament_zsh_to_array logic (`#36 <https://github.com/ros2/rosidl_typesupport_connext/issues/36>`_)
* Contributors: Dirk Thomas

0.8.0 (2019-09-25)
------------------
* Use local function to prepend unique value (`#35 <https://github.com/ros2/rosidl_typesupport_connext/issues/35>`_)
* Contributors: Dirk Thomas

0.7.2 (2019-05-29)
------------------
* Generalize string matching for OSX Connext library in cmake module (`#28 <https://github.com/ros2/rosidl_typesupport_connext/issues/28>`_)
* Contributors: Jacob Perron

0.7.1 (2019-05-08)
------------------

0.7.0 (2019-04-13)
------------------

0.6.4 (2019-01-11)
------------------

0.6.3 (2018-12-13)
------------------
* Add case for 'x64Darwin17clang9.0' in Connext library path in find module (`#21 <https://github.com/ros2/rosidl_typesupport_connext/issues/21>`_)
* Contributors: Jacob Perron

0.6.2 (2018-12-07)
------------------

0.6.1 (2018-12-06)
------------------
* Reduce verbosity when Connext is not available (`#17 <https://github.com/ros2/rosidl_typesupport_connext/issues/17>`_)
* Contributors: Dirk Thomas

0.6.0 (2018-11-16)
------------------

0.5.3 (2018-06-28)
------------------

0.5.2 (2018-06-25)
------------------
* Re-export NDDSHOME variable in environment hook. (`#5 <https://github.com/ros2/rosidl_typesupport_connext/issues/5>`_)
  The `find_package(Connext)` function provided by connext_cmake_module
  relies on the NDDSHOME variable being set.
  In order for the connext_cmake_module package to be the only one which
  needs to source the RTI environment script on the buildfarm the
  environment hook must re-export it.
* Contributors: Steven! Ragnarök

0.5.1 (2018-06-24)
------------------
* Add build dependency on rti-connext-dds-5.3.1 (`#3 <https://github.com/ros2/rosidl_typesupport_connext/issues/3>`_)
* Contributors: Steven! Ragnarök

0.5.0 (2018-06-23)
------------------
* Extra fix for paths with spaces in them
* Only set NDDSHOME if not previously set
* Add support for Visual Studio 2017
* 0.4.0
* 0.0.3
* 0.0.2
