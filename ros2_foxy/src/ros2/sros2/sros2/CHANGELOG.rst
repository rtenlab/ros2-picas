^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.4 (2020-11-17)
------------------
* parameter_events topic is now absolute (`#233 <https://github.com/ros2/sros2/issues/233>`_) (`#245 <https://github.com/ros2/sros2/issues/245>`_)
* Contributors: Mikael Arguedas

0.9.3 (2020-09-16)
------------------
* add cyclonedds to the list of rmw using graph info topics (#231) (#238)
  Signed-off-by: Mikael Arguedas <mikael.arguedas@gmail.com>
* Add scope parameter (backport to Foxy) (#232)
  Signed-off-by: Jose Luis Rivero <jrivero@osrfoundation.org>
* Contributors: Jose Luis Rivero, Mikael Arguedas

0.9.2 (2020-06-06)
------------------
* Fix list keys verb (`#219 <https://github.com/ros2/sros2/issues/219>`_)
* Contributors: Mikael Arguedas

0.9.1 (2020-05-11)
------------------
* start validity date a day before to account for timezone mismatch (`#209 <https://github.com/ros2/sros2/issues/209>`_)
* Contributors: Mikael Arguedas

0.9.0 (2020-05-06)
------------------
* Use matching validity dates for cert and permissions (`#205 <https://github.com/ros2/sros2/issues/205>`_)
* permission signing should use permissions_ca (`#204 <https://github.com/ros2/sros2/issues/204>`_)
* Rename keystore root env from ROS_SECURITY_ROOT_DIRECTORY to ROS_SECURITY_KEYSTORE (`#200 <https://github.com/ros2/sros2/issues/200>`_)
* API cleanup
  * remove function leftover from old generation strategy (`#207 <https://github.com/ros2/sros2/issues/207>`_)
  * remove distribute_key completely (`#197 <https://github.com/ros2/sros2/issues/197>`_)
  * api: reorganize policy generation API (`#196 <https://github.com/ros2/sros2/issues/196>`_)
  * api: reorganize artifact generation API (`#195 <https://github.com/ros2/sros2/issues/195>`_)
  * api: reorganize key API (`#192 <https://github.com/ros2/sros2/issues/192>`_)
  * api: reorganize permission API (`#191 <https://github.com/ros2/sros2/issues/191>`_)
  * api: reorganize policy API (`#190 <https://github.com/ros2/sros2/issues/190>`_)
  * api: reorganize keystore API (`#188 <https://github.com/ros2/sros2/issues/188>`_)
* Security enclaves:
  * Use security contexts (`#177 <https://github.com/ros2/sros2/issues/177>`_)
  * security-context -> enclave (`#198 <https://github.com/ros2/sros2/issues/198>`_)
  * Update generate_policy verb for enclaves (`#203 <https://github.com/ros2/sros2/issues/203>`_)
  * reenable test_generate_policy_no_policy_file (`#206 <https://github.com/ros2/sros2/issues/206>`_)
* [ci] Add GitHub actions for linting and source-build CI (`#178 <https://github.com/ros2/sros2/issues/178>`_)
* [test] use test_msgs instead of std message packages (`#181 <https://github.com/ros2/sros2/issues/181>`_)
* [test] more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* [bookkeeping] Update maintainer to point to ros-security mailing list + fix package.xml (`#179 <https://github.com/ros2/sros2/issues/179>`_)
* Symlink CA certs instead of copy (`#176 <https://github.com/ros2/sros2/issues/176>`_)
* update from ros2cli API:
  * pass argv in add_arguments to add_subparsers_on_demand (`#175 <https://github.com/ros2/sros2/issues/175>`_)
  * switch to not deprecated API (`#174 <https://github.com/ros2/sros2/issues/174>`_)
  * Use ros2cli.node.NodeStrategy consistently. (`#173 <https://github.com/ros2/sros2/issues/173>`_)
* Contributors: Dirk Thomas, Ivan Santiago Paunovic, Kyle Fazzari, Michel Hidalgo, Mikael Arguedas, Ruffin

0.8.1 (2019-11-13)
------------------
* add profile for lifecycle nodes (`#146 <https://github.com/ros2/sros2/issues/146>`_)
* Cleanup changelog
* Update version in setup.py
* Contributors: Jacob Perron, Mikael Arguedas

0.8.0 (2019-09-26)
------------------
* Install an XML catalog so we can look this schema up locally (`#158 <https://github.com/ros2/sros2/issues/158>`_)
  Fixes a failure in test_policy_to_permissions when there's no internet.
* Fix missing resources needed for ament (`#160 <https://github.com/ros2/sros2/issues/160>`_)
* Install package manifest (`#159 <https://github.com/ros2/sros2/issues/159>`_)
* Disable flaky test (`#155 <https://github.com/ros2/sros2/issues/155>`_)
* Add mypy tests to check static typing (`#154 <https://github.com/ros2/sros2/issues/154>`_)
* Topics starting with tilde need a slash right after (`#152 <https://github.com/ros2/sros2/issues/152>`_)
* Update message content to match create_key message
* Create key and cert only once in generate_artifacts
* Fix certificate start date to work regardless of the timezone (`#148 <https://github.com/ros2/sros2/issues/148>`_)
* Use older pytest compatible with Ubuntu Bionic (`#145 <https://github.com/ros2/sros2/issues/145>`_)
* Add request service permissions in generated policies  (`#141 <https://github.com/ros2/sros2/issues/141>`_)
* Replace openssl subprocess calls with Python cryptography library
    * Remove use of subprocess for creating ca key and cert (`#126 <https://github.com/ros2/sros2/issues/126>`_)
    * Obtain S/MIME signature using cryptography library (`#129 <https://github.com/ros2/sros2/issues/129>`_)
    * Migrate permissions S/MIME to cryptography library (`#136 <https://github.com/ros2/sros2/issues/136>`_)
    * Migrate create_key to cryptography library (`#138 <https://github.com/ros2/sros2/issues/138>`_)
    * Remove now obsolete openssl dependency (`#140 <https://github.com/ros2/sros2/issues/140>`_)
* Factor out the hardcoded name 'sros2testCA' into a constant DEFAULT_COMMON_NAME (`#134 <https://github.com/ros2/sros2/issues/134>`_)
* Improve create_key tests (`#132 <https://github.com/ros2/sros2/issues/132>`_)
* Add test for create_key verb (`#125 <https://github.com/ros2/sros2/issues/125>`_)
* Add basic create_keystore test. (`#124 <https://github.com/ros2/sros2/issues/124>`_)
* Add tests for list_keys verb (`#123 <https://github.com/ros2/sros2/issues/123>`_)
* Add tests for generate_policy verb (`#122 <https://github.com/ros2/sros2/issues/122>`_)
* Guard against empty ROS graph when generating policy (`#118 <https://github.com/ros2/sros2/issues/118>`_)
* Guard against invalid key names (`#117 <https://github.com/ros2/sros2/issues/117>`_)
  In particular, guard against keys that only consist of whitespace and '/' characters.
* Contributors: Emerson Knapp, Jacob Perron, Kyle Fazzari, Mikael Arguedas, Peter Baughman, Ruffin, Siddharth Kucheria

0.7.0 (2019-05-08)
------------------
* Add generate_artifacts verb (`#107 <https://github.com/ros2/sros2/issues/107>`_)
* complete xml and not yaml files for create_permission (`#104 <https://github.com/ros2/sros2/issues/104>`_)
* Fix bug preventing generate_policy verb from working with publishers and services
* Add missing attributes to test permissions XML file
* add reference to schema in generated permission files (`#84 <https://github.com/ros2/sros2/issues/84>`_)
* Correct sros2 cli test folder location (`#83 <https://github.com/ros2/sros2/issues/83>`_)
* Use XML and XSLT to perform permission transform (`#72 <https://github.com/ros2/sros2/issues/72>`_)
* Contributors: Jacob Perron, Michael Carroll, Mikael Arguedas, Ruffin

0.6.2 (2019-02-08)
------------------

0.6.1 (2019-01-15)
------------------
* Restructured sros2 to enable additional packages in this repository. (`#74 <https://github.com/ros2/sros2/issues/74>`_)
* Added generate_permissions verb + update policy definition to support services and actions (`#71 <https://github.com/ros2/sros2/issues/71>`_)
* Contributors: Jacob Perron, Ross Desmond

0.6.0 (2018-12-07)
------------------
* Update package maintainer for sros2 (`#70 <https://github.com/ros2/sros2/issues/70>`_)
* separating identity and permission CAs (`#67 <https://github.com/ros2/sros2/issues/67>`_)
* ignore __pycache__ in git
* raise FileNotFoundError if provided permission file doesn't exist (`#64 <https://github.com/ros2/sros2/issues/64>`_)
* refer to new xsd now that it's available (`#62 <https://github.com/ros2/sros2/issues/62>`_)
* fixup path of RANDFILE to match rest of tutorial (`#61 <https://github.com/ros2/sros2/issues/61>`_)
* Contributors: Michael Carroll, Mikael Arguedas, William Woodall

0.5.0 (2018-06-27)
------------------
* Update docs for bouncy leveraging remapping for demo (`#53 <https://github.com/ros2/sros2/issues/53>`_)
* Windows tutorial tweaks (`#58 <https://github.com/ros2/sros2/issues/58>`_)
* publish ans subscribe to all parameter service topics (`#52 <https://github.com/ros2/sros2/issues/52>`_)
* remove partitions (`#45 <https://github.com/ros2/sros2/issues/45>`_)
* as of Bouncy access control is available for both Fast-RTPS and Connext (`#50 <https://github.com/ros2/sros2/issues/50>`_)
* add pytest markers to linter tests
* Remove outdated docker resources now that SROS2 ships as part of the core (`#48 <https://github.com/ros2/sros2/issues/48>`_)
* add X509 extensionCA:false (`#47 <https://github.com/ros2/sros2/issues/47>`_)
* enable_liveliness_protection (`#44 <https://github.com/ros2/sros2/issues/44>`_)
* set zip_safe to avoid warning during installation (`#42 <https://github.com/ros2/sros2/issues/42>`_)
* Linter fixup
* add special service rule only if not wildcarding everything (`#40 <https://github.com/ros2/sros2/issues/40>`_)
* remove whant now appears to be obsolete DCPS whitelisting (`#34 <https://github.com/ros2/sros2/issues/34>`_)
* fix sample_policy download command (`#37 <https://github.com/ros2/sros2/issues/37>`_)
* Fix access control for ardent (`#33 <https://github.com/ros2/sros2/issues/33>`_)
* advise to ask questions on ROS answers
* print full help when no command is passed (`#35 <https://github.com/ros2/sros2/issues/35>`_)
* add return code to all verb apis (`#28 <https://github.com/ros2/sros2/issues/28>`_)
* Contributors: Dirk Thomas, Mikael Arguedas, Shane Loretz, dhood

0.4.0 (2017-12-08)
------------------
* update maintainer
* update instructions now that connext security is supported on all pla… (`#30 <https://github.com/ros2/sros2/issues/30>`_)
* explicitly call out setting the variables (`#29 <https://github.com/ros2/sros2/issues/29>`_)
* remove test_suite, add pytest as test_requires (`#27 <https://github.com/ros2/sros2/issues/27>`_)
* update xml to match spec + connext 53 (`#16 <https://github.com/ros2/sros2/issues/16>`_)
* 0.0.3
* install/setup.bat -> <path to ros2 install>/setup.bat (`#25 <https://github.com/ros2/sros2/issues/25>`_)
* Add internal topics (without partition) to default allow rule (`#24 <https://github.com/ros2/sros2/issues/24>`_)
  The topic wildcard + partition wildcard doesn't match
* Correct ordering of string formatting params (`#23 <https://github.com/ros2/sros2/issues/23>`_)
  Topic and partition were swapped
* make policy filenames match
* Update OpenSSL install instructions for Windows
* Fix connext for node with default partition (`#20 <https://github.com/ros2/sros2/issues/20>`_)
* update style to satisfy new flake8 plugins (`#19 <https://github.com/ros2/sros2/issues/19>`_)
* implicitly inherit from object (`#18 <https://github.com/ros2/sros2/issues/18>`_)
* remove flake8 dependency from Dockerfile (`#17 <https://github.com/ros2/sros2/issues/17>`_)
* add issue template
* update dockerfile to use beta2 binaries (`#14 <https://github.com/ros2/sros2/issues/14>`_)
* add libssl-dev as an exec dependency (`#13 <https://github.com/ros2/sros2/issues/13>`_)
* OS X: fix typo for env. variable (`#15 <https://github.com/ros2/sros2/issues/15>`_)
* update links to master
* Split docs and update content (`#12 <https://github.com/ros2/sros2/issues/12>`_)
* use ros2 run (`#11 <https://github.com/ros2/sros2/issues/11>`_)
* 0.0.2
* Find openssl executable on osx and enforce minimum required version for all platforms (`#10 <https://github.com/ros2/sros2/issues/10>`_)
* Updates to Windows running instructions (`#9 <https://github.com/ros2/sros2/issues/9>`_)
  - Fixed the OpenSSL install link
  - Added OpenSSL to path
  - Updated talker and listener calls
  - Deleted source section
* fix fallback api without argcomplete (`#8 <https://github.com/ros2/sros2/issues/8>`_)
* fix wrong imports (`#7 <https://github.com/ros2/sros2/issues/7>`_)
* Add tools for security files generation (`#3 <https://github.com/ros2/sros2/issues/3>`_)
* Initial commit
* Contributors: Adam Allevato, Dirk Thomas, Mikael Arguedas, Morgan Quigley, Shane Loretz, Tully Foote, Víctor Mayoral Vilches, dhood
