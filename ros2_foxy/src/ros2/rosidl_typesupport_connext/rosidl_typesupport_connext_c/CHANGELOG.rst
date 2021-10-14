^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_typesupport_connext_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2021-04-14)
------------------
* Update maintainers (`#64 <https://github.com/ros2/rosidl_typesupport_connext/issues/64>`_)
* Contributors: Jacob Perron

1.0.2 (2020-09-29)
------------------

1.0.1 (2020-08-12)
------------------
* fix capacity of serialized messages (`#58 <https://github.com/ros2/rosidl_typesupport_connext/issues/58>`_)
* Contributors: Dirk Thomas

1.0.0 (2020-05-19)
------------------

0.9.0 (2020-04-24)
------------------
* Switch take_response and take_request to rmw_service_info_t (`#53 <https://github.com/ros2/rosidl_typesupport_connext/issues/53>`_)
* Update includes to use non-entry point headers from detail subdirectory (`#51 <https://github.com/ros2/rosidl_typesupport_connext/issues/51>`_)
* Rename rosidl_generator_c namespace to rosidl_runtime_c (`#50 <https://github.com/ros2/rosidl_typesupport_connext/issues/50>`_)
* Replaced rosidl_generator_x for rosidl_runtime_x (`#49 <https://github.com/ros2/rosidl_typesupport_connext/issues/49>`_)
* Ignore -Wclass-memaccess build warnings coming from Connext 5.3.1 (`#45 <https://github.com/ros2/rosidl_typesupport_connext/issues/45>`_)
* Style update to match uncrustify with explicit language (`#44 <https://github.com/ros2/rosidl_typesupport_connext/issues/44>`_)
* Code style only: wrap after open parenthesis if not in one line (`#43 <https://github.com/ros2/rosidl_typesupport_connext/issues/43>`_)
* Fix different signedness compiler warnings (`#42 <https://github.com/ros2/rosidl_typesupport_connext/issues/42>`_)
* Remove redundant logic (`#41 <https://github.com/ros2/rosidl_typesupport_connext/issues/41>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Ingo Lütkebohle, Jacob Perron

0.8.4 (2019-12-04)
------------------

0.8.3 (2019-12-04)
------------------

0.8.2 (2019-10-23)
------------------
* Connext 6 compatibility (`#37 <https://github.com/ros2/rosidl_typesupport_connext/issues/37>`_)
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------

0.8.0 (2019-09-25)
------------------
* Remove non-package from ament_target_dependencies() (`#34 <https://github.com/ros2/rosidl_typesupport_connext/issues/34>`_)
* Contributors: Shane Loretz

0.7.2 (2019-05-29)
------------------

0.7.1 (2019-05-08)
------------------
* Combine package name with namespace in type support struct (`#32 <https://github.com/ros2/rosidl_typesupport_connext/issues/32>`_)
* Add WString support (`#29 <https://github.com/ros2/rosidl_typesupport_connext/issues/29>`_)
* Add cpplint flag to ignore long functions in generated code
* Simplify code using updated definition API (`#27 <https://github.com/ros2/rosidl_typesupport_connext/issues/27>`_)
* Update code to match refactoring of rosidl definitions (`#26 <https://github.com/ros2/rosidl_typesupport_connext/issues/26>`_)
* Remove usage of UnknownMessageType (`#25 <https://github.com/ros2/rosidl_typesupport_connext/issues/25>`_)
* Contributors: Dirk Thomas, Jacob Perron

0.7.0 (2019-04-13)
------------------
* Change generators to IDL-based pipeline (`#15 <https://github.com/ros2/rosidl_typesupport_connext/issues/15>`_)
* Removed Connext as target dependency, this library doesn't need to link with the RTI provided libraries (`#22 <https://github.com/ros2/rosidl_typesupport_connext/issues/22>`_)
* Contributors: Dirk Thomas, Johnny Willemsen

0.6.4 (2019-01-11)
------------------
* Change uncrustify max line length to 0 (`#23 <https://github.com/ros2/rosidl_typesupport_connext/issues/23>`_)
  This is for compatibility with uncrustify v0.68.
* Contributors: Jacob Perron

0.6.3 (2018-12-13)
------------------

0.6.2 (2018-12-07)
------------------
* Export dependency on rcutils instead of requiring it (`#19 <https://github.com/ros2/rosidl_typesupport_connext/issues/19>`_)
* Contributors: Steven! Ragnarök

0.6.1 (2018-12-06)
------------------
* Reduce verbosity when Connext is not available (`#17 <https://github.com/ros2/rosidl_typesupport_connext/issues/17>`_)
* Use uint8 (`#16 <https://github.com/ros2/rosidl_typesupport_connext/issues/16>`_)
* Contributors: Dirk Thomas, Karsten Knese

0.6.0 (2018-11-16)
------------------
* Allow generated IDL files (`#13 <https://github.com/ros2/rosidl_typesupport_connext/issues/13>`_)
* Rename dynamic array to sequence (`#14 <https://github.com/ros2/rosidl_typesupport_connext/issues/14>`_)
* Makes services' typesupport generation subfolder-aware (`#12 <https://github.com/ros2/rosidl_typesupport_connext/issues/12>`_)
  Necessary for actions typesupport generation.
* Avoid using undefined variable (`#10 <https://github.com/ros2/rosidl_typesupport_connext/issues/10>`_)
* Fix spacing between dereference and variable name (`#8 <https://github.com/ros2/rosidl_typesupport_connext/issues/8>`_)
  * To comply with uncrustify 0.67
* Contributors: Alexis Pojomovsky, Dirk Thomas, Michel Hidalgo, Mikael Arguedas, Shane Loretz

0.5.3 (2018-06-28)
------------------
* Update maintainer

0.5.2 (2018-06-25)
------------------

0.5.1 (2018-06-24)
------------------

0.5.0 (2018-06-23)
------------------
* Export / use typesupport libraries separately
* Expose raw CDR stream for publish and subscribe
* Remove topic partitions
* Add group_depends for typesupport
* Use CMAKE_CURRENT_BINARY_DIR for arguments json
* 0.4.0
* 0.0.3
* 0.0.2
