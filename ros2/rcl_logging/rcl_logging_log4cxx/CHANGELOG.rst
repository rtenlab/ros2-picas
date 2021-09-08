^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcl_logging_log4cxx
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* fix package.xml schema violations (`#15 <https://github.com/ros2/rcl_logging/issues/15>`_)
* Contributors: Mikael Arguedas

0.2.1 (2019-05-08)
------------------
* Changing the default location for log files to be a local directory instead of /var/log/ros on linux due to permission issues. (`#9 <https://github.com/ros2/rcl_logging/issues/9>`_)
* Removes debugging fprintf
* Review fixes.
* Move basename down to rcutils layer.
* Prototype to put things in ~/.ros/log
* Changing the default location for log files to be a local directory instead of /var/log/ros on linux due to permission issues.
* Change the API to add an allocator to logging initialize. (`#10 <https://github.com/ros2/rcl_logging/issues/10>`_)
* Added include dir to installation of rcl_logging_log4cxx. (`#7 <https://github.com/ros2/rcl_logging/issues/7>`_)
* Contributors: Chris Lalancette, Nick Burek, Rasmus Skovgaard Andersen, Steven! Ragnarök, burekn

0.2.0 (2019-03-09)
------------------
* First release of rcl_logging_log4cxx (`#3 <https://github.com/ros2/rcl_logging/issues/3>`_)
  * Fixes for building and running on MAC with log4cxx from brew.
  * Fixing the case for finding log4cxx library.
  * Fixing the log4cxx string macro on Windows and the cmake find for windows as well.
  * Fixing the lib generation on windows.
  * Cleanup of formatting.
  * Use static library for windows instead of debug mode
  * Fix "unreferenced local variable" warning
  * Move C incompatible functions out of extern C block
  * Disable ddl-interface warnings
* Contributors: Nick Burek

0.1.0 (2018-12-07)
------------------
* preparing for first release
* Renaming everything with an 'rcl\_' prefix instead of an 'rc\_' prefix to match the change in 'rcl' (`#2 <https://github.com/ros2/rcl_logging/issues/2>`_)
* Contributors: Nick Burek, William Woodall
