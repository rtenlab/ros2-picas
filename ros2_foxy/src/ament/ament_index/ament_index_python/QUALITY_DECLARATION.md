This document is a declaration of software quality for the `ament_index_python` package, based on the guidelines in [REP-2004](https://github.com/ros-infrastructure/rep/blob/rep-2004/rep-2004.rst).

# `ament_index_python` Quality Declaration

The package `ament_index_python` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-quality-categories)

## Version Policy [1]

### Version Scheme [1.i]

`ament_index_python` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning)

### Version Stability [1.ii]

`ament_index_python` is at a stable version, i.e. >= 1.0.0. The current version can be found in its [package.xml](./package.xml).

### Public API Declaration [1.iii]

The public API of `ament_index_python` is composed of the functions available in the module, exported in the [init file](./ament_index_python/__init__.py) and the [command line utility script](./ament_index_python/cli.py).

### API Stability Policy [1.iv]

`ament_index_python` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Policy [1.v]

`ament_index_python` is a Python package, so it does not need to handle ABI stability.

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

`ament_index_python` will not break API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

## Change Control Process [2]

`ament_index_python` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process).

### Change Requests [2.i]

All changes will occur through a pull request, check the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers)

Currently nightly results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)

###  Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`ament_index_python` does not have a documented feature list. Although it currently states part of its conceptual overview [here](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md).

### Public API Documentation [3.ii]

Most `ament_index_python` API functions are documented using docstrings. However, this is not hosted anywhere.

### License [3.iii]

The license for `ament_index_python` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](../LICENSE) file.

There is an automated test which runs a linter that ensures each file has a license statement. [Here](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/) can be found a list with the latest results of the various linters being run on the package.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `ament_index_python`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/test_copyright/) .

## Testing [4]

### Feature Testing [4.i]

Each feature in `ament_index_python` has corresponding tests which simulate typical usage, and they are located in the [`test`](./test) directory.
New features are required to have tests before being added.
Currently nightly test results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)

New features are required to have tests before being added.

### Public API Testing [4.ii]

All the functionality of the declared API in this package is covered in its unit tests. Currently it has a line coverage of [96%](https://ci.ros2.org/job/ci_linux_coverage/85/cobertura/ament_index_python/).

### Coverage [4.iii]

`ament_index_python` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#code-coverage), and opts to use line coverage instead of branch coverage.

This includes:

- tracking and reporting line coverage statistics
- achieving and maintaining a reasonable branch line coverage (90-100%)
- no lines are manually skipped in coverage calculations

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

Current coverage statistics can be viewed [here](https://ci.ros2.org/job/ci_linux_coverage/85/cobertura/ament_index_python/)

### Performance [4.iv]

An environment variable defines the prefix paths of such resource indices and the API has a time complexity of `O(n)` where `n` is the number of prefix paths.
The time complexity to query information is either scaling linearly with the number of resource types or with the number of resources per type (depending on which dimension is requested).
If the content of a specific resource is retrieved the time complexity is linear to the size of the content as is the memory usage in that case since the content is returned to the caller.
The runtime cost of the implementation is dominated by the runtime cost of the underlying filesystem API, and the implemented logic doesn't add any significant overhead.

From a usage point of view it is also expected that the resource index is commonly only queried during startup and not at runtime of a production system.
Therefore `ament_index_python` does not conduct explicit performance tests.

### Linters and Static Analysis [4.v]

`ament_index_python` uses and passes all the ROS2 standard linters and static analysis tools for a Python package as described in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

Currently nightly test results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/ament_index_python.src.ament.ament_index.ament_index_python.test/)

## Dependencies [5]

`ament_index_python` has no run-time or build-time dependencies that need to be considered for this declaration.

## Platform Support [6]

`ament_index_python` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

`ament_index_python` does not have a Vulnerability Disclosure Policy

# Current status Summary

The chart below compares the requirements in the REP-2004 with the current state of the `ament_index_python package`.
|Number|  Requirement| Current state |
|--|--|--|
|1| **Version policy** |---|
|1.i|Version Policy available | ✓ |
|1.ii|Stable version |☓|
|1.iii|Declared public API|✓|
|1.iv|API stability policy|✓|
|1.v|ABI stability policy|✓|
|1.vi_|API/ABI stable within ros distribution|✓|
|2| **Change control process** |---|
|2.i| All changes occur on change request | ✓|
|2.ii| Contributor origin (DCO, CLA, etc) | ✓|
|2.iii| Peer review policy | ✓ |
|2.iv| CI policy for change requests | ✓ |
|2.v| Documentation policy for change requests | ✓ |
|3| **Documentation** | --- |
|3.i| Per feature documentation | ☓ |
|3.ii| Per public API item documentation | ☓ |
|3.iii| Declared License(s) | ✓ |
|3.iv| Copyright in source files| ✓ |
|3.v.a| Quality declaration linked to README | ✓ |
|3.v.b| Centralized declaration available for peer review |✓|
|4| Testing | --- |
|4.i| Feature items tests | ✓ |
|4.ii| Public API tests | ✓ |
|4.iii.a| Using coverage |✓ |
|4.iii.a| Coverage policy | ✓ |
|4.iv.a| Performance tests (if applicable) | ☓ |
|4.iv.b| Performance tests policy| ✓ |
|4.v.a| Code style enforcement (linters)| ✓ |
|4.v.b| Use of static analysis tools | ✓ |
|5| Dependencies | --- |
|5.i| Must not have ROS lower level dependencies | ✓ |
|5.ii| Optional ROS lower level dependencies| ✓ |
|5.iii| Justifies quality use of non-ROS dependencies |✓|
|6| Platform support | --- |
|6.i| Support targets Tier1 ROS platforms| ✓ |
|7| Security | --- |
|7.i| Vulnerability Disclosure Policy | ☓ |

Comparing this table with the [Quality Level Comparison Chart of REP2004](https://github.com/ros-infrastructure/rep/blob/d1074e43f25f957d75f50dbfda94ab10d86bcbfd/rep-2004.rst#quality-level-comparison-chart) lead us to decide that this package qualifies to Quality Level 4.
