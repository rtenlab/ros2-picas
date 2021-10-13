^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcpputils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2020-10-09)
------------------
* Add scope_exit helper. (`#78 <https://github.com/ros2/rcpputils//issues/78>`_) (`#103 <https://github.com/ros2/rcpputils//issues/103>`_)
* Contributors: Alejandro Hernández Cordero, Michel Hidalgo

1.3.0 (2020-07-21)
------------------
* Removed doxygen warnings (`#86 <https://github.com/ros2/rcpputils/issues/86>`_) (`#90 <https://github.com/ros2/rcpputils/issues/90>`_)
* Add clamp header (`#85 <https://github.com/ros2/rcpputils/issues/85>`_) (`#88 <https://github.com/ros2/rcpputils/issues/88>`_)
* Add remove_all to remove non-empty directories… (`#80 <https://github.com/ros2/rcpputils/issues/80>`_)
* Contributors: Alejandro Hernández Cordero, Hunter L. Allen, Karsten Knese, Victor Lopez

1.1.0 (2020-06-22)
------------------
* Fix parent_path() for empty paths and paths of length one (`#73 <https://github.com/ros2/rcpputils/issues/73>`_)
* Add get_executable_name() function (`#70 <https://github.com/ros2/rcpputils/issues/70>`_)
* Address memory leak in remove pointer test (`#72 <https://github.com/ros2/rcpputils/issues/72>`_)
* Add current_path to filesystem_helpers (`#63 <https://github.com/ros2/rcpputils/issues/63>`_)
* Align path combine behavior with C++17 (`#68 <https://github.com/ros2/rcpputils/issues/68>`_)
* Update quality declaration to QL 2 (`#71 <https://github.com/ros2/rcpputils/issues/71>`_)
* Contributors: Jacob Perron, Scott K Logan, Stephen Brawner

1.0.1 (2020-06-03)
------------------
* Include stdexcept in get_env.hpp (`#69 <https://github.com/ros2/rcpputils/issues/69>`_)
* Update quality declaration for version stability (`#66 <https://github.com/ros2/rcpputils/issues/66>`_)
* Handle empty paths in is_absolute (`#67 <https://github.com/ros2/rcpputils/issues/67>`_)
* Add Security Vulnerability Policy pointing to REP-2006 (`#65 <https://github.com/ros2/rcpputils/issues/65>`_)
* Contributors: Chris Lalancette, Scott K Logan, Steven! Ragnarök

1.0.0 (2020-05-26)
------------------
* Remove mention of random file from temporary_directory_path doc (`#64 <https://github.com/ros2/rcpputils/issues/64>`_)
* Contributors: Scott K Logan

0.3.1 (2020-05-08)
------------------
* Fix Action CI by using released upload-artifact instead of master (`#61 <https://github.com/ros2/rcpputils/issues/61>`_)
* Quality declaration (`#47 <https://github.com/ros2/rcpputils/issues/47>`_)
* Contributors: Emerson Knapp, brawner

0.3.0 (2020-04-24)
------------------
* Added shared library to feature list (`#58 <https://github.com/ros2/rcpputils/issues/58>`_)
* export targets in a addition to include directories / libraries (`#57 <https://github.com/ros2/rcpputils/issues/57>`_)
* remove pointer for smart pointer (`#56 <https://github.com/ros2/rcpputils/issues/56>`_)
* Added shared library class description to readme (`#53 <https://github.com/ros2/rcpputils/issues/53>`_)
* Increased shared library tests (`#51 <https://github.com/ros2/rcpputils/issues/51>`_)
* Removed duplicated split function (`#54 <https://github.com/ros2/rcpputils/issues/54>`_)
* Exposed get_env_var (`#55 <https://github.com/ros2/rcpputils/issues/55>`_)
* Added debug version for library names (`#52 <https://github.com/ros2/rcpputils/issues/52>`_)
* Added unload_library method to shared_library (`#50 <https://github.com/ros2/rcpputils/issues/50>`_)
* Included abstraction for rcutils::shared_library (`#49 <https://github.com/ros2/rcpputils/issues/49>`_)
* Add more documentation and include doxyfile (`#46 <https://github.com/ros2/rcpputils/issues/46>`_)
* Update README.md with license and build badges. (`#45 <https://github.com/ros2/rcpputils/issues/45>`_)
* Update README to mention assertion helper functions (`#43 <https://github.com/ros2/rcpputils/issues/43>`_)
* Add rcpputils::fs::file_size and rcpputils::fs::is_directory (`#41 <https://github.com/ros2/rcpputils/issues/41>`_)
* Make assert functions accept an optional string. (`#42 <https://github.com/ros2/rcpputils/issues/42>`_)
* Add functions for C++ assertions (`#31 <https://github.com/ros2/rcpputils/issues/31>`_)
* remove reference for pointer traits (`#38 <https://github.com/ros2/rcpputils/issues/38>`_)
* code style only: wrap after open parenthesis if not in one line (`#36 <https://github.com/ros2/rcpputils/issues/36>`_)
* Bug fixes for rcpputils::fs API (`#35 <https://github.com/ros2/rcpputils/issues/35>`_)
  * Ensure rcpputils::fs::create_directories works with absolute paths.
  * Implement temp_directory_path() for testing purposes.
  * Fix rcpputils::fs::path::parent_path() method.
* Add build and test workflow (`#33 <https://github.com/ros2/rcpputils/issues/33>`_)
* Add linting workflow (`#32 <https://github.com/ros2/rcpputils/issues/32>`_)
* Fix filesystem helpers for directory manipulation. (`#30 <https://github.com/ros2/rcpputils/issues/30>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Emerson Knapp, Karsten Knese, Michel Hidalgo, Zachary Michaels

0.2.1 (2019-11-12)
------------------
* add new function to remove the extension of a file (`#27 <https://github.com/ros2/rcpputils/pull/27>`_)
* Contributors: Anas Abou Allaban

0.2.0 (2019-09-24)
------------------
* find_library: Centralize functionality here (`#25 <https://github.com/ros2/rcpputils/issues/25>`_)
* Implement join() (`#20 <https://github.com/ros2/rcpputils/issues/20>`_)
* Rename test (`#21 <https://github.com/ros2/rcpputils/issues/21>`_)
* use _WIN32 instead of WIN32 (`#24 <https://github.com/ros2/rcpputils/issues/24>`_)
* Update README.md and package.xml (`#22 <https://github.com/ros2/rcpputils/issues/22>`_)
* Fix typo (`#23 <https://github.com/ros2/rcpputils/issues/23>`_)
* type trait rcpputils::is_pointer<T>` (`#19 <https://github.com/ros2/rcpputils/issues/19>`_)
* File extension addition for camera calibration parser (`#18 <https://github.com/ros2/rcpputils/issues/18>`_)
* Add endian helper until C++20 (`#16 <https://github.com/ros2/rcpputils/issues/16>`_)
* use iterators for split (`#14 <https://github.com/ros2/rcpputils/issues/14>`_)
* Add function 'find_and_replace' (`#13 <https://github.com/ros2/rcpputils/issues/13>`_)
* Contributors: Andreas Klintberg, Dirk Thomas, Jacob Perron, Karsten Knese, Michael Carroll, Michel Hidalgo, Tully Foote

0.1.0 (2019-04-13)
------------------
* Fixed leak in test_basic.cpp. (`#9 <https://github.com/ros2/rcpputils/issues/9>`_)
* Added CODEOWNERS file. (`#10 <https://github.com/ros2/rcpputils/issues/10>`_)
* Added commonly-used filesystem helper to utils. (`#5 <https://github.com/ros2/rcpputils/issues/5>`_)
* Fixed thread_safety_annotation filename to .hpp. (`#6 <https://github.com/ros2/rcpputils/issues/6>`_)
* Added section about DCO to CONTRIBUTING.md.
* Added thread annotation macros. (`#2 <https://github.com/ros2/rcpputils/issues/2>`_)
* Contributors: Dirk Thomas, Emerson Knapp, Michael Carroll, Thomas Moulard
