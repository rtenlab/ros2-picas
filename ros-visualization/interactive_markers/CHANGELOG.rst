^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package interactive_markers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2019-10-23)
------------------
* Fix test flakes by waiting for pub/sub discovery (`#55 <https://github.com/ros-visualization/interactive_markers/issues/55>`_)
* Add parameters for QoS of update and feedback topics (`#54 <https://github.com/ros-visualization/interactive_markers/issues/54>`_)
* Port Python implementation to ROS 2 (`#53 <https://github.com/ros-visualization/interactive_markers/issues/53>`_)
  * Move Python files to their own directory
  * Install Python via ament_cmake_python
  * Make Python implementation ROS 2 compatible
  * Use docstrings
  * Minor refactor of callback logic
  * Guard against None values and KeyError's
  * Change insert() signature to take keyword arguments
  * Rename variables for clarity (e.g. 'cb' -> 'callback')
  * Fix PEP 257 errors
  * Remove unused setup.py
  * Enable flake8 tests and fix errors
  * Improve performance
  * Clear pending updates after applying all of them
  * Don't rely on user to apply any pose updates
  * Expose QoSProfile a parameter
  * Add Python implementation dependencies to package.xml
* Contributors: Jacob Perron

2.0.0 (2019-09-26)
------------------
* Add missing visibility macros (`#51 <https://github.com/ros-visualization/interactive_markers/issues/51>`_)
* Less verbose logging (`#45 <https://github.com/ros-visualization/interactive_markers/issues/45>`_)
* Rename enums to avoid collisions with MSVC compiler defines (`#49 <https://github.com/ros-visualization/interactive_markers/issues/49>`_)
* Catch polymorphic exceptions by reference (`#48 <https://github.com/ros-visualization/interactive_markers/issues/48>`_)
* Port to ROS 2 (`#44 <https://github.com/ros-visualization/interactive_markers/issues/44>`_)
    * Style and other aesthetic changes
    * Use tf2::BufferCoreInterface
    * Replace 'init' topic with a ROS service
    * Merge SingleClient logic into InteractiveMarkerClient
    * Remove notion of server ID
    * Add feedback publisher to client
    * Default to C++14 and set stricter compiler flags
    * Fix Windows compiler warnings
    * Remove StateMachine class
    * Fix Clang warnings
* Contributors: David Gossow, Jacob Perron, Scott K Logan

1.11.4 (2018-04-16)
-------------------
* Fixed a crash when updates arrive, or are being processed, while shutdown is called (`#36 <https://github.com/ros-visualization/interactive_markers/issues/36>`_)
* Contributors: Simon Schmeisser

1.11.3 (2016-08-24)
-------------------
* The ``processFeedback`` function of the menu handler no longer catches the ``KeyErrors`` of the feedback_cb.
  See: `#29 <https://github.com/ros-visualization/interactive_markers/issues/29>`_
* Added the ``empty()`` and ``size()`` members to ``InteractiveMarkerServer`` interface.
  See: `#30 <https://github.com/ros-visualization/interactive_markers/issues/30>`_
* Contributors: Blake Anderson, Guglielmo Gemignani

1.11.2 (2016-08-24)
-------------------
* Fix build when disabling tests with ``-DCATKIN_ENABLE_TESTING=OFF``.
  See: `#26 <https://github.com/ros-visualization/interactive_markers/issues/26>`_
* Fix use of uninitialized variables.
  See: `#24 <https://github.com/ros-visualization/interactive_markers/issues/24>`_
* Fix potential segfault when shutting down.
  See: `#25 <https://github.com/ros-visualization/interactive_markers/issues/25>`_
* Contributors: Alexis Ballier, David Gossow, Max Schwarz

1.11.1 (2014-12-16)
-------------------
* Added explicit keyword argument queue_size for publisher in Python code and use the same default queue_size value as C++.
* Fixed a SEGFAULT in setPose reported in `#18 <https://github.com/ros-visualization/interactive_markers/issues/18>`_
  Previously, calling setPose() on an interactive marker causes a SEGFAULT
  if applyChanges() was not called on the server at least once since the
  marker was created. I traced the actual SEGFAULT to the doSetPose
  function. The value of header passed from setPose() is invalid because,
  in this case, marker_context_it = marker_contexts\_.end().
  I added a check for this case and, if there is no marker is present,
  instead use the header from the pending update.
* Contributors: David Gossow, Mike Koval, William Woodall, ipa-fxm

1.11.0 (2014-02-24)
-------------------
* Adding William Woodall as maintainer
* fix threading bugs
  Fix locking of data structures shared across threads.
* Contributors: Acorn Pooley, William Woodall, hersh

1.10.2 (2014-02-03)
-------------------
* fix regression in menu_handler.py
  fixes `#14 <https://github.com/ros-visualization/interactive_markers/issues/14>`_
* Contributors: William Woodall

1.10.1 (2014-01-27)
-------------------
* cleanup python code and package contents
* remove useless dependencies
* Contributors: Vincent Rabaud, William Woodall

1.10.0 (2014-01-23)
-------------------
* remove debug statement that could produce segfault; init_it->msg->markers may be empty
* Contributors: Filip Jares
