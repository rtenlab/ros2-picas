^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_implementation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.2 (2019-11-13)
------------------
* Add support for Cyclone DDS. (`#71 <https://github.com/ros2/rmw_implementation/issues/71>`_)
* Contributors: Ruffin

0.8.1 (2019-10-23)
------------------
* use return_loaned_message_from (`#76 <https://github.com/ros2/rmw_implementation/issues/76>`_)
* Add localhost boolean parameter to create node function (`#75 <https://github.com/ros2/rmw_implementation/issues/75>`_)
* Zero copy api (`#69 <https://github.com/ros2/rmw_implementation/issues/69>`_)
* Add Python API for RMW implementation lookups (`#73 <https://github.com/ros2/rmw_implementation/issues/73>`_)
* update signature for added pub/sub options (`#74 <https://github.com/ros2/rmw_implementation/issues/74>`_)
* remove unneeded line from CMakeLists (`#70 <https://github.com/ros2/rmw_implementation/issues/70>`_)
* Make middleware selection more independent of build-time package availability (`#67 <https://github.com/ros2/rmw_implementation/issues/67>`_)
* Contributors: Brian Marchi, Dan Rose, Karsten Knese, Michel Hidalgo, William Woodall

0.8.0 (2019-09-25)
------------------
* Add function for getting clients by node (`#62 <https://github.com/ros2/rmw_implementation/issues/62>`_)
* add get_actual_qos() feature to subscriptions (`#61 <https://github.com/ros2/rmw_implementation/issues/61>`_)
* Contributors: Jacob Perron, M. M

0.7.1 (2019-05-08)
------------------
* add interfaces for rmw_take_event and assert_liveliness (`#60 <https://github.com/ros2/rmw_implementation/issues/60>`_)
* Rmw preallocate (`#51 <https://github.com/ros2/rmw_implementation/issues/51>`_)
* Contributors: Michael Carroll, Nick Burek

0.7.0 (2019-04-13)
------------------
* Add function rmw_get_actual_qos (`#56 <https://github.com/ros2/rmw_implementation/issues/56>`_)
* cmake: Add `RMW_IMPLEMENTATION_FORCE_POCO` (`#59 <https://github.com/ros2/rmw_implementation/issues/59>`_)
* add missing preload of rmw_set_log_severity (`#55 <https://github.com/ros2/rmw_implementation/issues/55>`_)
* Export threading library via extras and not ament_export_libraries to avoid warnings when cross-compiling (`#53 <https://github.com/ros2/rmw_implementation/issues/53>`_)
* pass context to wait set and fini context (`#52 <https://github.com/ros2/rmw_implementation/issues/52>`_)
* Contributors: Dirk Thomas, Eric Cousineau, Esteve Fernandez, William Woodall, ivanpauno

0.6.1 (2018-12-06)
------------------
* Add node graph functions (`#49 <https://github.com/ros2/rmw_implementation/issues/49>`_)
* add new functions (`#50 <https://github.com/ros2/rmw_implementation/issues/50>`_)
* Methods to retrieve matched count on pub/sub. (`#48 <https://github.com/ros2/rmw_implementation/issues/48>`_)
* Contributors: Michael Carroll, Ross Desmond, William Woodall

0.6.0 (2018-11-16)
------------------
* use semicolons after macros (`#47 <https://github.com/ros2/rmw_implementation/issues/47>`_)
* Include node namespaces in get_node_names. (`#46 <https://github.com/ros2/rmw_implementation/issues/46>`_)
* add rmw_get_serialization_format (`#43 <https://github.com/ros2/rmw_implementation/issues/43>`_)
* Contributors: Karsten Knese, Michael Carroll, William Woodall

0.5.1 (2018-07-17)
------------------
* avoid recursive find (`#44 <https://github.com/ros2/rmw_implementation/issues/44>`_)
* Contributors: Dirk Thomas

0.5.0 (2018-06-23)
------------------
* Prepare dependencies for bouncy release. (`#41 <https://github.com/ros2/rmw_implementation/issues/41>`_)
* _raw function (`#31 <https://github.com/ros2/rmw_implementation/issues/31>`_)
* print missing symbol name (`#40 <https://github.com/ros2/rmw_implementation/issues/40>`_)
* Merge pull request `#39 <https://github.com/ros2/rmw_implementation/issues/39>`_ from ros2/misra_fixup
* Change #if to #ifdef
* improve error messages (`#37 <https://github.com/ros2/rmw_implementation/issues/37>`_)
* API to enable log severity setting.  (`#30 <https://github.com/ros2/rmw_implementation/issues/30>`_)
* Contributors: Dirk Thomas, Karsten Knese, Michael Carroll, Sriram Raghunathan, Steven! Ragnarök

0.4.0 (2017-12-08)
------------------
* Merge pull request `#36 <https://github.com/ros2/rmw_implementation/issues/36>`_ from ros2/rename_group
* waitset -> wait_set (`#34 <https://github.com/ros2/rmw_implementation/issues/34>`_)
* Merge pull request `#32 <https://github.com/ros2/rmw_implementation/issues/32>`_ from ros2/rep149
* use format 3
* simplify code relaying all symbols (`#29 <https://github.com/ros2/rmw_implementation/issues/29>`_)
* Merge pull request `#27 <https://github.com/ros2/rmw_implementation/issues/27>`_ from ros2/fix_deadlock
* prefetch all symbols in rmw_init to avoid later race
* make resolved symbol static to significantly reduce the chance of a deadlock
* Merge pull request `#26 <https://github.com/ros2/rmw_implementation/issues/26>`_ from ros2/uncrustify_master
* update style to match latest uncrustify
* Contributors: Dirk Thomas, Karsten Knese, Mikael Arguedas, Morgan Quigley, William Woodall, dhood
