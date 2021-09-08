^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_connext_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
