^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rmw_implementation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2021-04-14)
------------------

1.0.1 (2020-10-21)
------------------
* Fixed rmw_create_node arguments
* Add fault injection tests to construction/destroy APIs.  (`#144 <https://github.com/ros2/rmw_implementation/issues/144>`_)
* Fixed function rmw_create_node in tests
* Add tests bad type_support implementation (`#152 <https://github.com/ros2/rmw_implementation/issues/152>`_)
* Add tests for localhost-only node creation (`#150 <https://github.com/ros2/rmw_implementation/issues/150>`_)
* Use 10x the intraprocess delay to wait for sent requests. (`#148 <https://github.com/ros2/rmw_implementation/issues/148>`_)
* Added rmw_wait, rmw_create_wait_set, and rmw_destroy_wait_set tests (`#139 <https://github.com/ros2/rmw_implementation/issues/139>`_)
* Added rmw_service_server_is_available tests (`#140 <https://github.com/ros2/rmw_implementation/issues/140>`_)
* Add tests service/client request/response with bad arguments (`#141 <https://github.com/ros2/rmw_implementation/issues/141>`_)
* Fixed function rmw_create_node in tests
* Added rmw_wait, rmw_create_wait_set, and rmw_destroy_wait_set tests (`#139 <https://github.com/ros2/rmw_implementation/issues/139>`_)
* Added test for rmw_get_serialized_message_size (`#142 <https://github.com/ros2/rmw_implementation/issues/142>`_)
* Add service/client construction/destruction API test coverage. (`#138 <https://github.com/ros2/rmw_implementation/issues/138>`_)
* Added rmw_publisher_allocation and rmw_subscription_allocation related tests (`#137 <https://github.com/ros2/rmw_implementation/issues/137>`_)
* Add tests take serialized with info bad arguments (`#130 <https://github.com/ros2/rmw_implementation/issues/130>`_)
* Add gid API test coverage. (`#134 <https://github.com/ros2/rmw_implementation/issues/134>`_)
* Add tests take bad arguments  (`#125 <https://github.com/ros2/rmw_implementation/issues/125>`_)
* Bump graph API test coverage. (`#132 <https://github.com/ros2/rmw_implementation/issues/132>`_)
* Add tests take sequence serialized with bad arguments (`#129 <https://github.com/ros2/rmw_implementation/issues/129>`_)
* Add tests take sequence + take sequence with bad arguments (`#128 <https://github.com/ros2/rmw_implementation/issues/128>`_)
* Add tests take with info bad arguments (`#126 <https://github.com/ros2/rmw_implementation/issues/126>`_)
* Add tests for non-implemented rmw_take\_* functions (`#131 <https://github.com/ros2/rmw_implementation/issues/131>`_)
* Add tests publish serialized bad arguments (`#124 <https://github.com/ros2/rmw_implementation/issues/124>`_)
* Add tests publish bad arguments (`#123 <https://github.com/ros2/rmw_implementation/issues/123>`_)
* Add tests non-implemented functions + loan bad arguments (`#122 <https://github.com/ros2/rmw_implementation/issues/122>`_)
* Add missing empty topic name tests. (`#136 <https://github.com/ros2/rmw_implementation/issues/136>`_)
* Add rmw_get_serialization_format() smoke test. (`#133 <https://github.com/ros2/rmw_implementation/issues/133>`_)
* Complete publisher/subscription QoS query API test coverage. (`#120 <https://github.com/ros2/rmw_implementation/issues/120>`_)
* Remove duplicate assertions (`#121 <https://github.com/ros2/rmw_implementation/issues/121>`_)
* Add publisher/subscription matched count API test coverage. (`#119 <https://github.com/ros2/rmw_implementation/issues/119>`_)
* Fixed rmw_create_node
* Add serialize/deserialize API test coverage. (`#118 <https://github.com/ros2/rmw_implementation/issues/118>`_)
* Add subscription API test coverage. (`#117 <https://github.com/ros2/rmw_implementation/issues/117>`_)
* Extend publisher API test coverage (`#115 <https://github.com/ros2/rmw_implementation/issues/115>`_)
* Add node construction/destruction API test coverage. (`#112 <https://github.com/ros2/rmw_implementation/issues/112>`_)
* Check that rmw_init() fails if no enclave is given. (`#113 <https://github.com/ros2/rmw_implementation/issues/113>`_)
* Add init options API test coverage. (`#108 <https://github.com/ros2/rmw_implementation/issues/108>`_)
* Complete init/shutdown API test coverage. (`#107 <https://github.com/ros2/rmw_implementation/issues/107>`_)
* Add dependency on ament_cmake_gtest (`#109 <https://github.com/ros2/rmw_implementation/issues/109>`_)
* Add test_rmw_implementation package. (`#106 <https://github.com/ros2/rmw_implementation/issues/106>`_)
* Contributors: Alejandro Hernández Cordero, Geoffrey Biggs, Jose Tomas Lorente, Michel Hidalgo, Shane Loretz
