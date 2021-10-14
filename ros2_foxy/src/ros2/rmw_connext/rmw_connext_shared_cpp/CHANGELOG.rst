^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connext_shared_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2020-12-08)
------------------
* Update maintainers (`#468 <https://github.com/ros2/rmw_connext/issues/468>`_) (`#470 <https://github.com/ros2/rmw_connext/issues/470>`_)
* Contributors: Alejandro Hernández Cordero

1.0.2 (2020-10-15)
------------------
* Fixed Foxy
* Updated rmw\_* return codes (`#463 <https://github.com/ros2/rmw_connext/issues/463>`_)
* Update graph API return codes. (`#459 <https://github.com/ros2/rmw_connext/issues/459>`_)
* Ensure compliant publisher API (`#445 <https://github.com/ros2/rmw_connext/issues/445>`_)
* Ensure compliant node construction/destruction API. (`#439 <https://github.com/ros2/rmw_connext/issues/439>`_)
* Contributors: Alejandro Hernández Cordero, Michel Hidalgo

1.0.1 (2020-07-07)
------------------
* Handle RMW_DEFAULT_DOMAIN_ID (`#427 <https://github.com/ros2/rmw_connext/issues/427>`_) (`#430 <https://github.com/ros2/rmw_connext/issues/430>`_)
* Contributors: Michel Hidalgo

1.0.0 (2020-05-12)
------------------
* Remove MANUAL_BY_NODE liveliness API (`#423 <https://github.com/ros2/rmw_connext/issues/423>`_)
* Contributors: Ivan Santiago Paunovic

0.9.1 (2020-05-08)
------------------
* Initialize RMW qos profiles to rmw_qos_profile_unknown. (`#422 <https://github.com/ros2/rmw_connext/issues/422>`_)
* Contributors: Michel Hidalgo

0.9.0 (2020-04-25)
------------------
* Add basic support for security logging plugin (`#404 <https://github.com/ros2/rmw_connext/issues/404>`_)
* security-context -> enclave (`#407 <https://github.com/ros2/rmw_connext/issues/407>`_)
* Do not force 510 locators compatibility mode (`#406 <https://github.com/ros2/rmw_connext/issues/406>`_)
* API changes to sync with one Participant per Context change in rmw_fastrtps (`#392 <https://github.com/ros2/rmw_connext/issues/392>`_)
* Correct error message when using a non supported event (`#400 <https://github.com/ros2/rmw_connext/issues/400>`_)
* Support for ON_REQUESTED_INCOMPATIBLE_QOS and ON_OFFERED_INCOMPATIBLE_QOS events (`#398 <https://github.com/ros2/rmw_connext/issues/398>`_)
* Increase the max_objects_per_thread (`#394 <https://github.com/ros2/rmw_connext/issues/394>`_)
  Increased to 8192, which should allow something like 60 - 70 participants.
* Add rmw\_*_event_init() functions (`#397 <https://github.com/ros2/rmw_connext/issues/397>`_)
* Fix build warnings due to -Wsign-compare with GCC 9 (`#396 <https://github.com/ros2/rmw_connext/issues/396>`_)
* Don't use RTPS Participant name as node name (`#393 <https://github.com/ros2/rmw_connext/issues/393>`_)
* Implement the rmw_get_publishers/subscriptions_info_by_topic() methods (`#391 <https://github.com/ros2/rmw_connext/issues/391>`_)
* Finding rmw_connext_shared_cpp must succeed, only rmw_connext_cpp can signal not-found when Connext is not available (`#389 <https://github.com/ros2/rmw_connext/issues/389>`_)
* Code style only: wrap after open parenthesis if not in one line (`#387 <https://github.com/ros2/rmw_connext/issues/387>`_)
* Avoid using build time Connext library paths, determine them when downstream packages are built (`#385 <https://github.com/ros2/rmw_connext/issues/385>`_)
* Stubs for rmw_get_publishers_info_by_topic and rmw_get_subscriptions_info_by_topic  (`#377 <https://github.com/ros2/rmw_connext/issues/377>`_)
* Filtering empty node names and namespaces (`#362 <https://github.com/ros2/rmw_connext/issues/362>`_)
* Contributors: CaptainTrunky, Chris Lalancette, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Jaison Titus, Kyle Fazzari, Miaofei Mei, Mikael Arguedas

0.8.1 (2019-10-23)
------------------
* Support localhost only communication (`#373 <https://github.com/ros2/rmw_connext/issues/373>`_)
* update logic / patches to optionally support Connext 6 (`#374 <https://github.com/ros2/rmw_connext/issues/374>`_)
* Contributors: Brian Marchi, Dirk Thomas

0.8.0 (2019-09-25)
------------------
* Return specific error when not finding a node name (`#365 <https://github.com/ros2/rmw_connext/issues/365>`_)
* Add function for getting clients by node (`#361 <https://github.com/ros2/rmw_connext/issues/361>`_)
* Use rpputils::find_and_replace instead of std::regex_replace (`#359 <https://github.com/ros2/rmw_connext/issues/359>`_)
* Implement get_actual_qos() for subscriptions (`#358 <https://github.com/ros2/rmw_connext/issues/358>`_)
* Contributors: Jacob Perron, M. M, ivanpauno

0.7.2 (2019-05-20)
------------------
* Include 'srv' in service type namespace. (`#356 <https://github.com/ros2/rmw_connext/issues/356>`_)
* Contributors: Michael Carroll, Michel Hidalgo

0.7.1 (2019-05-08)
------------------
* Combine package name with type namespace in type support struct (`#354 <https://github.com/ros2/rmw_connext/issues/354>`_)
* Implement QoS: liveliness, deadline, lifespan (`#352 <https://github.com/ros2/rmw_connext/issues/352>`_)
* Contributors: Devin Bonnie, Jacob Perron, Michael Carroll

0.7.0 (2019-04-13)
------------------
* Add rmw includes to cppcheck (`#348 <https://github.com/ros2/rmw_connext/issues/348>`_)
* pass context to wait set and fini context (`#343 <https://github.com/ros2/rmw_connext/issues/343>`_)
* Use narrow to downcast to a type specific datareader (`#339 <https://github.com/ros2/rmw_connext/issues/339>`_)
* deduplicate code (`#312 <https://github.com/ros2/rmw_connext/issues/312>`_)
* Don't use DURATION_INFINITE, it is a RTI specific type (`#335 <https://github.com/ros2/rmw_connext/issues/335>`_)
* Make use of DDS namespace instead of DDS\_ prefixed versions in global namespace (`#328 <https://github.com/ros2/rmw_connext/issues/328>`_)
* Directly assing mw_ret_t on declaring it and removed some mw_ret_t intermediate values which are not required (`#321 <https://github.com/ros2/rmw_connext/issues/321>`_)
* No need to use strcmp, we can compare a std::string with operator== t… (`#320 <https://github.com/ros2/rmw_connext/issues/320>`_)
* No need for DDS_InstanceStateKind scoping (`#319 <https://github.com/ros2/rmw_connext/issues/319>`_)
* Use empty() to check for an empty string (`#317 <https://github.com/ros2/rmw_connext/issues/317>`_)
* Use empty() instead of size() to check whether we have elements in th… (`#316 <https://github.com/ros2/rmw_connext/issues/316>`_)
* Fixed typo in comment (`#315 <https://github.com/ros2/rmw_connext/issues/315>`_)
* Contributors: Johnny Willemsen, Shane Loretz, William Woodall

0.6.1 (2018-12-06)
------------------
* Adds missing RMW_CONNEXT_SHARED_CPP_PUBLIC (`#314 <https://github.com/ros2/rmw_connext/issues/314>`_)
* Node graph impl (`#313 <https://github.com/ros2/rmw_connext/issues/313>`_)
* reduce verbosity when Connext is not available (`#311 <https://github.com/ros2/rmw_connext/issues/311>`_)
* Contributors: Dirk Thomas, Ross Desmond, Thomas Moulard

0.6.0 (2018-11-16)
------------------
* use new error handling API from rcutils (`#306 <https://github.com/ros2/rmw_connext/issues/306>`_)
* Add semicolons to all RCLCPP and RCUTILS macros. (`#304 <https://github.com/ros2/rmw_connext/issues/304>`_)
* separating identity and permission CAs (`#301 <https://github.com/ros2/rmw_connext/issues/301>`_)
* Include node namespaces in get_node_names (`#299 <https://github.com/ros2/rmw_connext/issues/299>`_)
* Contributors: Chris Lalancette, Michael Carroll, Mikael Arguedas, William Woodall

0.5.1 (2018-06-28)
------------------
* update maintainer
* Contributors: Dirk Thomas

0.5.0 (2018-06-23)
------------------
* Use key rti-connext-dds-5.3.1 as package key name for connext. (`#294 <https://github.com/ros2/rmw_connext/issues/294>`_)
* update usage of rcutils_join_path() (`#290 <https://github.com/ros2/rmw_connext/issues/290>`_)
* Remove topic partitions (`#285 <https://github.com/ros2/rmw_connext/issues/285>`_)
* Disable non-standard TypeCode (`#288 <https://github.com/ros2/rmw_connext/issues/288>`_)
* Merge pull request `#287 <https://github.com/ros2/rmw_connext/issues/287>`_ from ros2/misra_fixup
* Merge pull request `#276 <https://github.com/ros2/rmw_connext/issues/276>`_ from ros2/node_name_in_user_data
* get participant name from user data first
* add node name to user data
* Contributors: Dirk Thomas, Michael Carroll, Rohit Salem, Shane Loretz, Steven! Ragnarök, William Woodall

0.4.0 (2017-12-08)
------------------
* Merge pull request `#273 <https://github.com/ros2/rmw_connext/issues/273>`_ from ros2/fix_demangle
* Wait set two words (`#271 <https://github.com/ros2/rmw_connext/issues/271>`_)
* Merge pull request `#264 <https://github.com/ros2/rmw_connext/issues/264>`_ from dejanpan/master
* Merge pull request `#261 <https://github.com/ros2/rmw_connext/issues/261>`_ from dejanpan/master
* Remove obsolete warning suppressions (`#257 <https://github.com/ros2/rmw_connext/issues/257>`_)
* Merge pull request `#251 <https://github.com/ros2/rmw_connext/issues/251>`_ from ros2/uncrustify_master
* Update Connext license filter resource name to specify it's a prefix (`#242 <https://github.com/ros2/rmw_connext/issues/242>`_)
* update style to match latest uncrustify
