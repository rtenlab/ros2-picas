^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ament_package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Revert "Generate Setuptools Dict Helper Method (`#126 <https://github.com/ament/ament_package/issues/126>`_) (`#128 <https://github.com/ament/ament_package/issues/128>`_)" (`#132 <https://github.com/ament/ament_package/issues/132>`_)

0.9.4 (2021-03-12)
------------------
* Generate Setuptools Dict Helper Method (`#126 <https://github.com/ament/ament_package/issues/126>`_) (`#128 <https://github.com/ament/ament_package/issues/128>`_)

0.9.3 (2020-12-07)
------------------
* make AMENT_TRACE_SETUP_FILES output sourceable (`#120 <https://github.com/ament/ament_package/issues/120>`_) (`#121 <https://github.com/ament/ament_package/issues/121>`_)
* Contributors: Dirk Thomas

0.9.2 (2020-06-05)
------------------
* add configure-time flag to skip parent_prefix_path (`#115 <https://github.com/ament/ament_package/issues/115>`_)
* Contributors: Dirk Thomas

0.9.1 (2020-05-12)
------------------
* [Windows] Fix _ament_python_executable fallback code path. (`#113 <https://github.com/ament/ament_package/issues/113>`_)
* Contributors: Dirk Thomas

0.9.0 (2020-04-24)
------------------
* Convert format to f-string. (`#112 <https://github.com/ament/ament_package/issues/112>`_)
* Contributors: Dirk Thomas

0.8.8 (2019-12-10)
------------------
* fix removing trailing semicolon logic in bat (`#111 <https://github.com/ament/ament_package/issues/111>`_)
* Contributors: Dirk Thomas

0.8.7 (2019-12-05)
------------------
* fix handling of empty env var (`#110 <https://github.com/ament/ament_package/issues/110>`_)
* Contributors: Dirk Thomas

0.8.6 (2019-12-04)
------------------
* improve error message when split for dsv line raises (`#108 <https://github.com/ament/ament_package/issues/108>`_)
* skip empty lines in dsv files (`#107 <https://github.com/ament/ament_package/issues/107>`_)
* fix performance regression in environment setup (`#106 <https://github.com/ament/ament_package/issues/106>`_)
* Contributors: Dirk Thomas

0.8.5 (2019-11-08)
------------------
* Fix sh command to remove trailing separator (`#105 <https://github.com/ament/ament_package/issues/105>`_)
* Always prepend with a trailing separator (`#104 <https://github.com/ament/ament_package/issues/104>`_)
* Contributors: Jacob Perron

0.8.4 (2019-10-23)
------------------
* add dsv type set-if-unset (`#102 <https://github.com/ament/ament_package/issues/102>`_)
* Add support for prepending multiple values to env variables from .dsv files (`#101 <https://github.com/ament/ament_package/issues/101>`_)
* Contributors: Dirk Thomas, Jacob Perron, Shane Loretz

0.8.3 (2019-10-11)
------------------
* add type 'set' for dsv files (`#95 <https://github.com/ament/ament_package/issues/95>`_)
* keep using default ignore list (`#96 <https://github.com/ament/ament_package/issues/96>`_)
* Contributors: Dirk Thomas

0.8.2 (2019-10-04 15:45)
------------------------
* provide ament_prepend_unique_value function in prefix level sh script (`#94 <https://github.com/ament/ament_package/issues/94>`_)
* Contributors: Dirk Thomas

0.8.1 (2019-10-04 14:34)
------------------------
* fix reading the package run dependencies (`#93 <https://github.com/ament/ament_package/issues/93>`_)
* if no package.dsv is available fallback to a local_setup file (`#92 <https://github.com/ament/ament_package/issues/92>`_)
* Contributors: Dirk Thomas

0.8.0 (2019-10-04 10:32)
------------------------
* perform environment calculation in Python (`#89 <https://github.com/ament/ament_package/issues/89>`_)
* escape closing parenthesis in local_setup.bat file (`#91 <https://github.com/ament/ament_package/issues/91>`_)
* add quotes around ament_python_executable variable in local_setup.bat (`#90 <https://github.com/ament/ament_package/issues/90>`_)
* fix pytest warning about unknown markers (`#88 <https://github.com/ament/ament_package/issues/88>`_)
* install resource marker file for package (`#87 <https://github.com/ament/ament_package/issues/87>`_)
* Contributors: Dirk Thomas

0.7.2 (2019-09-18)
------------------
* install package manifest (`#86 <https://github.com/ament/ament_package/issues/86>`_)
* Contributors: Dirk Thomas

0.7.1 (2019-08-21)
------------------
* add warning/fallback for AMENT_CURRENT_PREFIX if relocated (`#85 <https://github.com/ament/ament_package/issues/85>`_)
* Contributors: Dirk Thomas

0.7.0 (2019-04-08)
------------------
* add section about DCO to CONTRIBUTING.md
* Contributors: Dirk Thomas

0.6.0 (2018-11-13)
------------------
* Fix lint warnings from invalid escape sequences (`#82 <https://github.com/ament/ament_package/issues/82>`_)
* Contributors: Jacob Perron

0.5.2 (2018-07-19)
------------------
* fix custom zsh logic for handling arrays (`#80 <https://github.com/ament/ament_package/issues/80>`_)
* Contributors: Dirk Thomas

0.5.1 (2018-06-14)
------------------
* Use flake8 directly (`#77 <https://github.com/ament/ament_package/issues/77>`_)
* Drop dependency on pyparsing. (`#78 <https://github.com/ament/ament_package/issues/78>`_)
* Contributors: Steven! Ragnarök

0.5.0 (2018-06-13)
------------------
* remove all Python modules by the templates (`#75 <https://github.com/ament/ament_package/issues/75>`_)
* add pytest markers to linter tests
* support file attribution of license tag (`#73 <https://github.com/ament/ament_package/issues/73>`_)
* set zip_safe to avoid warning during installation (`#72 <https://github.com/ament/ament_package/issues/72>`_)
* Revert "Revert "consider condition for group membership (`#69 <https://github.com/ament/ament_package/issues/69>`_)" (`#70 <https://github.com/ament/ament_package/issues/70>`_)" (`#71 <https://github.com/ament/ament_package/issues/71>`_)
* Revert "consider condition for group membership (`#69 <https://github.com/ament/ament_package/issues/69>`_)" (`#70 <https://github.com/ament/ament_package/issues/70>`_)
* consider condition for group membership (`#69 <https://github.com/ament/ament_package/issues/69>`_)
* fix copyright year
* Contributors: Dirk Thomas, Mikael Arguedas, Tamaki Nishino
