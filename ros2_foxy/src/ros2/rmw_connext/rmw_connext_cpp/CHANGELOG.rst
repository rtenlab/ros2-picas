^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connext_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2020-12-08)
------------------
* Fix wrong error messages (`#458 <https://github.com/ros2/rmw_connext/issues/458>`_) (`#478 <https://github.com/ros2/rmw_connext/issues/478>`_)
* Update maintainers (`#468 <https://github.com/ros2/rmw_connext/issues/468>`_) (`#470 <https://github.com/ros2/rmw_connext/issues/470>`_)
* Contributors: Alejandro Hernández Cordero, Ivan Santiago Paunovic

1.0.2 (2020-10-15)
------------------
* Return RMW_RET_UNSUPPORTED in rmw_get_serialized_message_size (`#467 <https://github.com/ros2/rmw_connext/issues/467>`_)
* returned RMW_RET_INCORRECT_RMW_IMPLEMENTATION in rmw_service_server_is_available (`#466 <https://github.com/ros2/rmw_connext/issues/466>`_)
* Update service/client request/response API error returns (`#465 <https://github.com/ros2/rmw_connext/issues/465>`_)
* Updated rmw\_* return codes (`#463 <https://github.com/ros2/rmw_connext/issues/463>`_)
* Update service/client construction/destruction API return codes. (`#464 <https://github.com/ros2/rmw_connext/issues/464>`_)
* Updated returns on rmw_take_serialized_with_info (`#462 <https://github.com/ros2/rmw_connext/issues/462>`_)
* Update gid API return codes. (`#461 <https://github.com/ros2/rmw_connext/issues/461>`_)
* updated error returns (`#456 <https://github.com/ros2/rmw_connext/issues/456>`_)
* Updated error returns on rmw_take (`#454 <https://github.com/ros2/rmw_connext/issues/454>`_)
* Update rmw_publish() error returns (`#452 <https://github.com/ros2/rmw_connext/issues/452>`_)
* Update rmw_publish_serialized_message() error returns (`#453 <https://github.com/ros2/rmw_connext/issues/453>`_)
* Ensure compliant matched pub/sub count API. (`#451 <https://github.com/ros2/rmw_connext/issues/451>`_)
* Ensure compliant subscription API. (`#450 <https://github.com/ros2/rmw_connext/issues/450>`_)
* Decouple rmw_destroy_publisher() outcome from global RMW error state. (`#449 <https://github.com/ros2/rmw_connext/issues/449>`_)
* Ensure compliant publisher API (`#445 <https://github.com/ros2/rmw_connext/issues/445>`_)
* Ensure compliant node construction/destruction API. (`#439 <https://github.com/ros2/rmw_connext/issues/439>`_)
* Amend rmw_init() implementation: require enclave. (`#437 <https://github.com/ros2/rmw_connext/issues/437>`_)
* Ensure compliant init/shutdown API implementation. (`#432 <https://github.com/ros2/rmw_connext/issues/432>`_)
* Ensure compliant init options API implementations. (`#431 <https://github.com/ros2/rmw_connext/issues/431>`_)
* Finalize context if and only if it's shutdown (`#428 <https://github.com/ros2/rmw_connext/issues/428>`_)
* Contributors: Alejandro Hernández Cordero, Jose Tomas Lorente, Michel Hidalgo

1.0.1 (2020-07-07)
------------------

1.0.0 (2020-05-12)
------------------
* Remove MANUAL_BY_NODE liveliness API (`#423 <https://github.com/ros2/rmw_connext/issues/423>`_)
* Contributors: Ivan Santiago Paunovic

0.9.1 (2020-05-08)
------------------
* Correct rmw_take() return value. (`#419 <https://github.com/ros2/rmw_connext/issues/419>`_)
* Switch to always using rtiddsgen (not the server). (`#421 <https://github.com/ros2/rmw_connext/issues/421>`_)
* Cast to size_t before doing comparison (`#420 <https://github.com/ros2/rmw_connext/issues/420>`_)
* Cast size_t to DDS_Long explicitly. (`#416 <https://github.com/ros2/rmw_connext/issues/416>`_)
* set connext buffer capacity (`#417 <https://github.com/ros2/rmw_connext/issues/417>`_)
* Retry calling the idl_pp up to 10 times. (`#415 <https://github.com/ros2/rmw_connext/issues/415>`_)
* Contributors: Chris Lalancette, Karsten Knese, Michel Hidalgo, Shane Loretz

0.9.0 (2020-04-25)
------------------
* Rename rosidl_message_bounds_t (`#413 <https://github.com/ros2/rmw_connext/issues/413>`_)
* More verbose error output when apply_patch.py fails (`#414 <https://github.com/ros2/rmw_connext/issues/414>`_)
* Switch take_response and take_request to rmw_service_info_t (`#412 <https://github.com/ros2/rmw_connext/issues/412>`_)
* Add support for taking a sequence of messages (`#408 <https://github.com/ros2/rmw_connext/issues/408>`_)
* Fix CMake warning about using uninitialized variables (`#411 <https://github.com/ros2/rmw_connext/issues/411>`_)
* security-context -> enclave (`#407 <https://github.com/ros2/rmw_connext/issues/407>`_)
* Replace rosidl_generator_x for rosidl_runtime_x (`#399 <https://github.com/ros2/rmw_connext/issues/399>`_)
* API changes to sync with one Participant per Context change in rmw_fastrtps (`#392 <https://github.com/ros2/rmw_connext/issues/392>`_)
* Support for ON_REQUESTED_INCOMPATIBLE_QOS and ON_OFFERED_INCOMPATIBLE_QOS events (`#398 <https://github.com/ros2/rmw_connext/issues/398>`_)
* Add rmw\_*_event_init() functions (`#397 <https://github.com/ros2/rmw_connext/issues/397>`_)
* Fix build warnings due to -Wsign-compare with GCC 9 (`#396 <https://github.com/ros2/rmw_connext/issues/396>`_)
* Implement the rmw_get_publishers/subscriptions_info_by_topic() methods (`#391 <https://github.com/ros2/rmw_connext/issues/391>`_)
* Finding rmw_connext_shared_cpp must succeed, only rmw_connext_cpp can signal not-found when Connext is not available (`#389 <https://github.com/ros2/rmw_connext/issues/389>`_)
* Code style only: wrap after open parenthesis if not in one line (`#387 <https://github.com/ros2/rmw_connext/issues/387>`_)
* Avoid using build time Connext library paths, determine them when downstream packages are built (`#385 <https://github.com/ros2/rmw_connext/issues/385>`_)
* Stubs for rmw_get_publishers_info_by_topic and rmw_get_subscriptions_info_by_topic  (`#377 <https://github.com/ros2/rmw_connext/issues/377>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Ingo Lütkebohle, Ivan Santiago Paunovic, Jacob Perron, Jaison Titus, Miaofei Mei, Michael Carroll, Mikael Arguedas

0.8.1 (2019-10-23)
------------------
* CMAKE_SOURCE_DIR -> CMAKE_CURRENT_SOURCE_DIR (`#378 <https://github.com/ros2/rmw_connext/issues/378>`_)
* use return_loaned_message_from (`#376 <https://github.com/ros2/rmw_connext/issues/376>`_)
* Support localhost only communication (`#373 <https://github.com/ros2/rmw_connext/issues/373>`_)
* Zero copy api (`#367 <https://github.com/ros2/rmw_connext/issues/367>`_)
* update logic / patches to optionally support Connext 6 (`#374 <https://github.com/ros2/rmw_connext/issues/374>`_)
* assert that unmodified lines in the patch match the input (`#371 <https://github.com/ros2/rmw_connext/issues/371>`_)
* supress invalid syntax warning for macro call (`#370 <https://github.com/ros2/rmw_connext/issues/370>`_)
* Fix build error (`#369 <https://github.com/ros2/rmw_connext/issues/369>`_)
* update signature for added pub/sub options (`#368 <https://github.com/ros2/rmw_connext/issues/368>`_)
* Contributors: Brian Marchi, Dan Rose, Dirk Thomas, Karsten Knese, William Woodall, ivanpauno

0.8.0 (2019-09-25)
------------------
* speed up copy with memcpy (`#366 <https://github.com/ros2/rmw_connext/issues/366>`_)
* Implement get_actual_qos() for subscriptions (`#358 <https://github.com/ros2/rmw_connext/issues/358>`_)
* add missing qos setings in get_actual_qos() (`#357 <https://github.com/ros2/rmw_connext/issues/357>`_)
* Contributors: Jacob Perron, M. M, Neil Puthuff

0.7.2 (2019-05-20)
------------------
* Add missing get_serialized_message_size method. (`#355 <https://github.com/ros2/rmw_connext/issues/355>`_)
* Contributors: Michael Carroll

0.7.1 (2019-05-08)
------------------
* Combine package name with type namespace in type support struct (`#354 <https://github.com/ros2/rmw_connext/issues/354>`_)
* Implement QoS: liveliness, deadline, lifespan (`#352 <https://github.com/ros2/rmw_connext/issues/352>`_)
* Stub out new allocation APIs. (`#353 <https://github.com/ros2/rmw_connext/issues/353>`_)
* Contributors: Devin Bonnie, Jacob Perron, Michael Carroll

0.7.0 (2019-04-13)
------------------
* Add function to get publisher actual qos settings (`#350 <https://github.com/ros2/rmw_connext/issues/350>`_)
* pass context to wait set and fini context (`#343 <https://github.com/ros2/rmw_connext/issues/343>`_)
* deduplicate code (`#312 <https://github.com/ros2/rmw_connext/issues/312>`_)
* Delete datareader on the subscriber and datawriter on the publisher, issue `#330 <https://github.com/ros2/rmw_connext/issues/330>`_ (`#337 <https://github.com/ros2/rmw_connext/issues/337>`_)
* Remove unnecessary argument name, issue `#331 <https://github.com/ros2/rmw_connext/issues/331>`_ (`#336 <https://github.com/ros2/rmw_connext/issues/336>`_)
* Make use of DDS namespace instead of DDS\_ prefixed versions in global namespace (`#328 <https://github.com/ros2/rmw_connext/issues/328>`_)
* Fixed typo in comments and remove trailing spaces (`#332 <https://github.com/ros2/rmw_connext/issues/332>`_)
* remove debug print (`#322 <https://github.com/ros2/rmw_connext/issues/322>`_)
* Contributors: Johnny Willemsen, William Woodall, ivanpauno

0.6.1 (2018-12-06)
------------------
* Node graph impl (`#313 <https://github.com/ros2/rmw_connext/issues/313>`_)
* refactor to support init options and context (`#308 <https://github.com/ros2/rmw_connext/issues/308>`_)
* Add implementation of matching publisher/subscriber counts (`#310 <https://github.com/ros2/rmw_connext/issues/310>`_)
* reduce verbosity when Connext is not available (`#311 <https://github.com/ros2/rmw_connext/issues/311>`_)
* use uint8_t array (`#309 <https://github.com/ros2/rmw_connext/issues/309>`_)
* Contributors: Dirk Thomas, Karsten Knese, Michael Carroll, Ross Desmond, William Woodall

0.6.0 (2018-11-16)
------------------
* use new error handling API from rcutils (`#306 <https://github.com/ros2/rmw_connext/issues/306>`_)
* Fix lint warning from invalid escape sequences (`#305 <https://github.com/ros2/rmw_connext/issues/305>`_)
* Include node namespaces in get_node_names (`#299 <https://github.com/ros2/rmw_connext/issues/299>`_)
* add rmw_get_serialization_format (`#298 <https://github.com/ros2/rmw_connext/issues/298>`_)
* Contributors: Jacob Perron, Karsten Knese, Michael Carroll, William Woodall

0.5.1 (2018-06-28)
------------------
* only deserialize when taken is true (`#297 <https://github.com/ros2/rmw_connext/issues/297>`_)
* Contributors: Karsten Knese

0.5.0 (2018-06-23)
------------------
* Use key rti-connext-dds-5.3.1 as package key name for connext. (`#294 <https://github.com/ros2/rmw_connext/issues/294>`_)
* Expose raw CDR stream for publish and subscribe (`#259 <https://github.com/ros2/rmw_connext/issues/259>`_)
* Remove topic partitions (`#285 <https://github.com/ros2/rmw_connext/issues/285>`_)
* Merge pull request `#287 <https://github.com/ros2/rmw_connext/issues/287>`_ from ros2/misra_fixup
* Merge pull request `#277 <https://github.com/ros2/rmw_connext/issues/277>`_ from ros2/compile_all_cpp_files
* Logging manipulation API from rmw_connext (`#266 <https://github.com/ros2/rmw_connext/issues/266>`_)
* Contributors: Karsten Knese, Michael Carroll, Rohit Salem, Sriram Raghunathan, Steven! Ragnarök

0.4.0 (2017-12-08)
------------------
* Merge pull request `#272 <https://github.com/ros2/rmw_connext/issues/272>`_ from ros2/rename_group
* Wait set two words (`#271 <https://github.com/ros2/rmw_connext/issues/271>`_)
* Merge pull request `#267 <https://github.com/ros2/rmw_connext/issues/267>`_ from ros2/rep149
* Merge pull request `#268 <https://github.com/ros2/rmw_connext/issues/268>`_ from ros2/ignore_unavailable_rmw
* Merge pull request `#252 <https://github.com/ros2/rmw_connext/issues/252>`_ from ros2/remove_indent_off
* Merge pull request `#251 <https://github.com/ros2/rmw_connext/issues/251>`_ from ros2/uncrustify_master
* remove obsolete INDENT-OFF usage
* Update Connext license filter resource name to specify it's a prefix (`#242 <https://github.com/ros2/rmw_connext/issues/242>`_)
* update style to match latest uncrustify
* Merge pull request `#249 <https://github.com/ros2/rmw_connext/issues/249>`_ from ros2/remove_unnecessary_define
