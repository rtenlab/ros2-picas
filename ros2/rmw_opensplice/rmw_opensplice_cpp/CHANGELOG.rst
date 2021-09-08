^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_opensplice_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.1 (2019-10-23)
------------------
* use return_loaned_message_from (`#290 <https://github.com/ros2/rmw_opensplice/issues/290>`_)
* Add localhost parameter to create node function (`#288 <https://github.com/ros2/rmw_opensplice/issues/288>`_)
* zero copy api for opensplice (`#285 <https://github.com/ros2/rmw_opensplice/issues/285>`_)
* Change PrismTech -> ADLINK (`#287 <https://github.com/ros2/rmw_opensplice/issues/287>`_)
* Fix build error (`#286 <https://github.com/ros2/rmw_opensplice/issues/286>`_)
* update signature for added pub/sub options (`#284 <https://github.com/ros2/rmw_opensplice/issues/284>`_)
* Contributors: Brian Marchi, Dan Rose, Karsten Knese, William Woodall, ivanpauno

0.8.0 (2019-09-26)
------------------
* Return specific error when not finding a node name (`#281 <https://github.com/ros2/rmw_opensplice/issues/281>`_)
* use a common datareader to fetch node info from participants.(`#242 <https://github.com/ros2/rmw_opensplice/issues/242>`_) (`#275 <https://github.com/ros2/rmw_opensplice/issues/275>`_)
* Don't export rcpputils dependency (`#277 <https://github.com/ros2/rmw_opensplice/issues/277>`_)
* Add function for getting clients by node (`#274 <https://github.com/ros2/rmw_opensplice/issues/274>`_)
* Use rcpputils::find_and_replace instead of std::regex_replace (`#272 <https://github.com/ros2/rmw_opensplice/issues/272>`_)
* Fix a comparison with a sign mismatch (`#276 <https://github.com/ros2/rmw_opensplice/issues/276>`_)
* Implement get_actual_qos() for subscriptions (`#271 <https://github.com/ros2/rmw_opensplice/issues/271>`_)
* add missing qos setings in get_actual_qos() (`#270 <https://github.com/ros2/rmw_opensplice/issues/270>`_)
* Contributors: Jacob Perron, M. M, Scott K Logan, YuSheng, ivanpauno

0.7.1 (2019-05-08)
------------------
* Combine package name with message namespace in type support struct (`#268 <https://github.com/ros2/rmw_opensplice/issues/268>`_)
* fix windows build warnings (`#269 <https://github.com/ros2/rmw_opensplice/issues/269>`_)
* Implement QoS: liveliness, deadline, lifespan (`#266 <https://github.com/ros2/rmw_opensplice/issues/266>`_)
* API updates for preallocation work. (`#267 <https://github.com/ros2/rmw_opensplice/issues/267>`_)
* Contributors: Jacob Perron, M. M, Michael Carroll, Ross Desmond

0.7.0 (2019-04-13)
------------------
* Add function to get publisher actual qos settings (`#265 <https://github.com/ros2/rmw_opensplice/issues/265>`_)
* Add rmw includes to cppcheck (`#261 <https://github.com/ros2/rmw_opensplice/issues/261>`_)
* GUID fix for node graph implementation (`#255 <https://github.com/ros2/rmw_opensplice/issues/255>`_)
* pass context to wait set and fini context (`#257 <https://github.com/ros2/rmw_opensplice/issues/257>`_)
* Contributors: M. M, Shane Loretz, William Woodall, ivanpauno

0.6.2 (2018-12-12)
------------------
* fix compiler warnings on Windows introduced in `#251 <https://github.com/ros2/rmw_opensplice/issues/251>`_ (`#252 <https://github.com/ros2/rmw_opensplice/issues/252>`_)
* Contributors: Dirk Thomas

0.6.1 (2018-12-06)
------------------
* Node graph impl (`#251 <https://github.com/ros2/rmw_opensplice/issues/251>`_)
* refactor to support init options and context (`#246 <https://github.com/ros2/rmw_opensplice/issues/246>`_)
* add opensplice support for cdr serialization (`#247 <https://github.com/ros2/rmw_opensplice/issues/247>`_)
* Methods to retrieve matched counts on pub/sub (`#248 <https://github.com/ros2/rmw_opensplice/issues/248>`_)
* suppress warning on Windows (`#250 <https://github.com/ros2/rmw_opensplice/issues/250>`_)
* update to OpenSplice 6.9 (`#249 <https://github.com/ros2/rmw_opensplice/issues/249>`_)
* Contributors: Dirk Thomas, MarcelJordense, Michael Carroll, Ross Desmond, William Woodall

0.6.0 (2018-11-16)
------------------
* use new error handling API from rcutils (`#245 <https://github.com/ros2/rmw_opensplice/issues/245>`_)
* Add semicolons to all RCLCPP and RCUTILS macros. (`#244 <https://github.com/ros2/rmw_opensplice/issues/244>`_)
* Include node namespaces in get_node_names (`#243 <https://github.com/ros2/rmw_opensplice/issues/243>`_)
* add rmw_get_serialization_format (`#241 <https://github.com/ros2/rmw_opensplice/issues/241>`_)
* Contributors: Chris Lalancette, Karsten Knese, Michael Carroll, William Woodall

0.5.2 (2018-06-29)
------------------
* Don't error if Sample\_ prefix not found in service type (`#240 <https://github.com/ros2/rmw_opensplice/issues/240>`_)
* Contributors: dhood

0.5.1 (2018-06-28)
------------------
* accept RETCODE_PRECONDITION_NOT_MET when detaching condition (`#235 <https://github.com/ros2/rmw_opensplice/issues/235>`_)
* Contributors: Dirk Thomas

0.5.0 (2018-06-23)
------------------
* strip 'Sample\_' from reported types (`#234 <https://github.com/ros2/rmw_opensplice/issues/234>`_)
* homogenize opensplice keys and use libopensplice67 everywhere (`#232 <https://github.com/ros2/rmw_opensplice/issues/232>`_)
* gracefully fail when using serialized functions (`#228 <https://github.com/ros2/rmw_opensplice/issues/228>`_)
* Remove topic partitions (`#225 <https://github.com/ros2/rmw_opensplice/issues/225>`_)
* move function to rmw for reuse (`#222 <https://github.com/ros2/rmw_opensplice/issues/222>`_)
* API to manipulate log setting macros (`#209 <https://github.com/ros2/rmw_opensplice/issues/209>`_)
* Contributors: Dirk Thomas, Karsten Knese, Mikael Arguedas, Rohit Salem, Sriram Raghunathan

0.4.0 (2017-12-08)
------------------
* Merge pull request `#215 <https://github.com/ros2/rmw_opensplice/issues/215>`_ from ros2/rename_group
* Wait set two words (`#214 <https://github.com/ros2/rmw_opensplice/issues/214>`_)
* Merge pull request `#210 <https://github.com/ros2/rmw_opensplice/issues/210>`_ from ros2/rep149
* Merge pull request `#211 <https://github.com/ros2/rmw_opensplice/issues/211>`_ from ros2/ignore_unavailable_rmw
* Merge pull request `#208 <https://github.com/ros2/rmw_opensplice/issues/208>`_ from dejanpan/master
* Merge pull request `#206 <https://github.com/ros2/rmw_opensplice/issues/206>`_ from ros2/uncrustify_master
* Merge pull request `#203 <https://github.com/ros2/rmw_opensplice/issues/203>`_ from ros2/listener_trigger
* remove unnecessary define (`#204 <https://github.com/ros2/rmw_opensplice/issues/204>`_)
