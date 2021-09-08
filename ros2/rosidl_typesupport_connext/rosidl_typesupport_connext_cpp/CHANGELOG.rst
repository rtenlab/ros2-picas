^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_typesupport_connext_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.4 (2019-12-04)
------------------
* try fallback to non-server mode if server mode failed (`#39 <https://github.com/ros2/rosidl_typesupport_connext/issues/39>`_)
* Revert "retry rtiddsgen(_server) invocation if it fails with a return code (`#38 <https://github.com/ros2/rosidl_typesupport_connext/issues/38>`_)"
* Contributors: Dirk Thomas

0.8.3 (2019-12-04)
------------------
* retry rtiddsgen(_server) invocation if it fails with a return code (`#38 <https://github.com/ros2/rosidl_typesupport_connext/issues/38>`_)
* Contributors: Dirk Thomas

0.8.2 (2019-10-23)
------------------
* Connext 6 compatibility (`#37 <https://github.com/ros2/rosidl_typesupport_connext/issues/37>`_)
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------

0.8.0 (2019-09-25)
------------------

0.7.2 (2019-05-29)
------------------

0.7.1 (2019-05-08)
------------------
* Combine package name with namespace in type support struct (`#32 <https://github.com/ros2/rosidl_typesupport_connext/issues/32>`_)
* Fix clang ignores (`#31 <https://github.com/ros2/rosidl_typesupport_connext/issues/31>`_)
* Add WString support (`#29 <https://github.com/ros2/rosidl_typesupport_connext/issues/29>`_)
* Add cpplint flag to ignore long functions in generated code
* Update code to match refactoring of rosidl definitions (`#26 <https://github.com/ros2/rosidl_typesupport_connext/issues/26>`_)
* Remove usage of UnknownMessageType (`#25 <https://github.com/ros2/rosidl_typesupport_connext/issues/25>`_)
* Contributors: Dirk Thomas, Jacob Perron

0.7.0 (2019-04-13)
------------------
* Change generators to IDL-based pipeline (`#15 <https://github.com/ros2/rosidl_typesupport_connext/issues/15>`_)
* Contributors: Dirk Thomas

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
* Makes services' typesupport generation subfolder-aware (`#12 <https://github.com/ros2/rosidl_typesupport_connext/issues/12>`_)
  Necessary for actions typesupport generation.
* Avoid using undefined variable (`#10 <https://github.com/ros2/rosidl_typesupport_connext/issues/10>`_)
* Fix spacing between dereference and variable name (`#8 <https://github.com/ros2/rosidl_typesupport_connext/issues/8>`_)
  * To comply with uncrustify 0.67
* Contributors: Alexis Pojomovsky, Dirk Thomas, Michel Hidalgo, Mikael Arguedas, Shane Loretz

0.5.3 (2018-06-28)
------------------

0.5.2 (2018-06-25)
------------------

0.5.1 (2018-06-24)
------------------

0.5.0 (2018-06-23)
------------------
* Use key rti-connext-dds-5.3.1 as package key name for connext
* Export / use typesupport libraries separately
* Expose raw CDR stream for publish and subscribe
* Remove topic partitions
* Add group_depends for typesupport
* Use CMAKE_CURRENT_BINARY_DIR for arguments json
* 0.4.0
* 0.0.3
* 0.0.2
