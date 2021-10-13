^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcl_logging_noop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2021-04-14)
------------------

1.0.1 (2020-07-21)
------------------

1.0.0 (2020-05-26)
------------------

0.4.0 (2020-04-24)
------------------
* Fix CMake warnings about using uninitialized variables (`#30 <https://github.com/ros2/rcl_logging/issues/30>`_)
* Contributors: Dirk Thomas

0.3.3 (2019-10-23)
------------------

0.3.2 (2019-10-07)
------------------
* Enable linters for noop and log4cxx. (`#12 <https://github.com/ros2/rcl_logging/issues/12>`_)
* Contributors: Steven! Ragnarök

0.3.1 (2019-10-03)
------------------

0.3.0 (2019-09-26)
------------------
* remove unused 'dependencies' CMake variable (`#16 <https://github.com/ros2/rcl_logging/issues/16>`_)
* fix package.xml schema violations (`#15 <https://github.com/ros2/rcl_logging/issues/15>`_)
* Contributors: Mikael Arguedas

0.2.1 (2019-05-08)
------------------
* Changing the default location for log files to be a local directory instead of /var/log/ros on linux due to permission issues. (`#9 <https://github.com/ros2/rcl_logging/issues/9>`_)
* Prototype to put things in ~/.ros/log
* Change the API to add an allocator to logging initialize. (`#10 <https://github.com/ros2/rcl_logging/issues/10>`_)
* Contributors: Chris Lalancette, Steven! Ragnarök

0.2.0 (2019-03-09)
------------------

0.1.0 (2018-12-07)
------------------
* First release.
* Contributors: Nick Burek, William Woodall
