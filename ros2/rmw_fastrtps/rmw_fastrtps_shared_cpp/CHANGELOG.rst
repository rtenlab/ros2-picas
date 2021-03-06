^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_fastrtps_shared_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.1 (2019-10-23)
------------------
* Restrict traffic to localhost only if env var is provided (`#331 <https://github.com/ros2/rmw_fastrtps/issues/331>`_)
* Added new functions which can be used to get rmw_qos_profile_t from WriterQos and ReaderQos (`#328 <https://github.com/ros2/rmw_fastrtps/issues/328>`_)
* Renamed dds_qos_to_rmw_qos to dds_attributes_to_rmw_qos (`#330 <https://github.com/ros2/rmw_fastrtps/issues/330>`_)
* Contributors: Brian Marchi, jaisontj

0.8.0 (2019-09-25)
------------------
* Correct error message (`#320 <https://github.com/ros2/rmw_fastrtps/issues/320>`_)
* Return specific error code when node is not found (`#311 <https://github.com/ros2/rmw_fastrtps/issues/311>`_)
* Correct linter failure (`#318 <https://github.com/ros2/rmw_fastrtps/issues/318>`_)
* Fix bug in graph API by node (`#316 <https://github.com/ros2/rmw_fastrtps/issues/316>`_)
* fix method name change from 1.8.1->1.9.0 (`#302 <https://github.com/ros2/rmw_fastrtps/issues/302>`_)
* Add missing lock guards for discovered_names and discovered_namespaces (`#301 <https://github.com/ros2/rmw_fastrtps/issues/301>`_)
* Add function for getting clients by node (`#293 <https://github.com/ros2/rmw_fastrtps/issues/293>`_)
* Enable manual_by_node and node liveliness assertion (`#298 <https://github.com/ros2/rmw_fastrtps/issues/298>`_)
* Enable assert liveliness on publisher. (`#296 <https://github.com/ros2/rmw_fastrtps/issues/296>`_)
* Use rcpputils::find_and_replace instead of std::regex_replace (`#291 <https://github.com/ros2/rmw_fastrtps/issues/291>`_)
* Fix a comparison with a sign mismatch (`#288 <https://github.com/ros2/rmw_fastrtps/issues/288>`_)
* Implement get_actual_qos() for subscriptions (`#287 <https://github.com/ros2/rmw_fastrtps/issues/287>`_)
* add missing qos setings in get_actual_qos() (`#284 <https://github.com/ros2/rmw_fastrtps/issues/284>`_)
* Fix ABBA deadlock.
* Contributors: Chris Lalancette, Emerson Knapp, Jacob Perron, M. M, Scott K Logan, William Woodall, ivanpauno

0.7.3 (2019-05-29)
------------------
* Protection of discovered_names and discovered_namespaces (`#283 <https://github.com/ros2/rmw_fastrtps/issues/283>`_)
* Disable all liveliness until it is actually supported (`#282 <https://github.com/ros2/rmw_fastrtps/issues/282>`_)
* Contributors: Emerson Knapp, MiguelCompany

0.7.2 (2019-05-20)
------------------
* fix log_debug typo in rmw_count (`#279 <https://github.com/ros2/rmw_fastrtps/issues/279>`_)
* Fastrtps18 event callbacks policies (`#275 <https://github.com/ros2/rmw_fastrtps/issues/275>`_)
* Centralize topic name creation logic and update to match FastRTPS 1.8 API (`#272 <https://github.com/ros2/rmw_fastrtps/issues/272>`_)
* Contributors: 1r0b1n0, Emerson Knapp, Nick Burek

0.7.1 (2019-05-08)
------------------
* Support arbitrary message namespaces  (`#266 <https://github.com/ros2/rmw_fastrtps/issues/266>`_)
* Set more correct return values for unimplemented features (`#276 <https://github.com/ros2/rmw_fastrtps/issues/276>`_)
* Add qos interfaces with no-op (`#271 <https://github.com/ros2/rmw_fastrtps/issues/271>`_)
* Updates for preallocation API. (`#274 <https://github.com/ros2/rmw_fastrtps/issues/274>`_)
* Fix logging in rmw_node_info_and_types.cpp (`#273 <https://github.com/ros2/rmw_fastrtps/issues/273>`_)
* Contributors: Emerson Knapp, Jacob Perron, Michael Carroll, Ross Desmond, Thomas Moulard

0.7.0 (2019-04-13)
------------------
* Thread safety annotation - minimally intrusive first pass (`#259 <https://github.com/ros2/rmw_fastrtps/issues/259>`_)
* Add function to get publisher actual qos settings (`#267 <https://github.com/ros2/rmw_fastrtps/issues/267>`_)
* Fixed race condition between taking sample and updating counter. (`#264 <https://github.com/ros2/rmw_fastrtps/issues/264>`_)
* Fix cpplint error
* change count type to size_t to avoid warning (`#262 <https://github.com/ros2/rmw_fastrtps/issues/262>`_)
* update listener logic for accurate counting (`#262 <https://github.com/ros2/rmw_fastrtps/issues/262>`_)
* Make sure to include the C++ headers used by these headers. (`#256 <https://github.com/ros2/rmw_fastrtps/issues/256>`_)
* pass context to wait set and fini context (`#252 <https://github.com/ros2/rmw_fastrtps/issues/252>`_)
* Improve service_is_available logic to protect that client is waiting forever (`#238 <https://github.com/ros2/rmw_fastrtps/issues/238>`_)
* Merge pull request `#250 <https://github.com/ros2/rmw_fastrtps/issues/250>`_ from ros2/support_static_lib
* use namespace_prefix from shared package
* make namespace_prefix header public
* Use empty() to check for an empty string (`#247 <https://github.com/ros2/rmw_fastrtps/issues/247>`_)
* We can compare a std::string with a const char* using operator==, simplifies the code (`#248 <https://github.com/ros2/rmw_fastrtps/issues/248>`_)
* Use empty() instead of size() to check if a vector/map contains elements and fixed some incorrect logging (`#245 <https://github.com/ros2/rmw_fastrtps/issues/245>`_)
* Fix guard condition trigger error (`#235 <https://github.com/ros2/rmw_fastrtps/issues/235>`_)
* Contributors: Chris Lalancette, Dirk Thomas, DongheeYe, Emerson Knapp, Jacob Perron, Johnny Willemsen, Ricardo Gonz??lez, William Woodall, ivanpauno

0.6.1 (2018-12-06)
------------------
* Add topic cache object for managing topic relations (`#236 <https://github.com/ros2/rmw_fastrtps/issues/236>`_)
* Fix lint: remove trailing whitespace (`#244 <https://github.com/ros2/rmw_fastrtps/issues/244>`_)
* Fastrtps 1.7.0 (`#233 <https://github.com/ros2/rmw_fastrtps/issues/233>`_)
* RMW_FastRTPS configuration from XML only (`#243 <https://github.com/ros2/rmw_fastrtps/issues/243>`_)
* Methods to retrieve matched counts on pub/sub (`#234 <https://github.com/ros2/rmw_fastrtps/issues/234>`_)
* use uint8_array (`#240 <https://github.com/ros2/rmw_fastrtps/issues/240>`_)
* Contributors: Jacob Perron, Juan Carlos, Karsten Knese, Michael Carroll, MiguelCompany, Ross Desmond

0.6.0 (2018-11-16)
------------------
* use new error handling API from rcutils (`#231 <https://github.com/ros2/rmw_fastrtps/issues/231>`_)
* Add semicolons to all RCLCPP and RCUTILS macros. (`#229 <https://github.com/ros2/rmw_fastrtps/issues/229>`_)
* separating identity and permission CAs (`#227 <https://github.com/ros2/rmw_fastrtps/issues/227>`_)
* Include node namespaces in get_node_names (`#224 <https://github.com/ros2/rmw_fastrtps/issues/224>`_)
* allow builtin reader/writer to reallocate memory if needed (`#221 <https://github.com/ros2/rmw_fastrtps/issues/221>`_)
* Improve runtime performance of `rmw_count_XXX` functions (`#216 <https://github.com/ros2/rmw_fastrtps/issues/216>`_) (`#217 <https://github.com/ros2/rmw_fastrtps/issues/217>`_)
* Merge pull request `#218 <https://github.com/ros2/rmw_fastrtps/issues/218>`_ from ros2/pr203
* Refs `#3061 <https://github.com/ros2/rmw_fastrtps/issues/3061>`_. Leaving common code only on rmw_fastrtps_shared_cpp.
* Refs `#3061 <https://github.com/ros2/rmw_fastrtps/issues/3061>`_. Package rmw_fastrtps_cpp copied to rmw_fastrtps_shared_cpp.
* Contributors: Chris Lalancette, Dirk Thomas, Guillaume Autran, Michael Carroll, Miguel Company, Mikael Arguedas, William Woodall

0.5.1 (2018-06-28)
------------------

0.5.0 (2018-06-23)
------------------

0.4.0 (2017-12-08)
------------------
