This document is a declaration of software quality for the `spdlog_vendor` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# spdlog_vendor Quality Declaration

The package `spdlog_vendor` claims to be in the **Quality Level 1** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://www.ros.org/reps/rep-2004.html) of the ROS2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]

`spdlog_vendor` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#versioning).

### Version Stability [1.ii]

`spdlog_vendor` is at or above a stable version, i.e. `>= 1.0.0`.

### Public API Declaration [1.iii]

This is a vendor package for `spdlog` and as such does not declare its own API.

### API Stability Policy [1.iv]/[1.vi]

There is no policy for API stability. This is not a problem because the `spdlog_vendor` package importing the `spdlog` dependency is using a fixed version, in this case, [1.6.1](https://github.com/gabime/spdlog/releases/tag/v1.6.1).

### ABI Stability Policy [1.v]/[1.vi]

There is no policy for ABI stability. This is not a problem because the `spdlog_vendor` package importing the `spdlog` dependency is using a fixed version, in this case, [1.6.1](https://github.com/gabime/spdlog/releases/tag/v1.6.1).

## Change Control Process [2]

`spdlog_vendor` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#change-control-process).

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](./CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed, check the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#change-control-process) for additional information.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers).

Currently nightly results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastSuccessfulBuild/testReport/spdlog_vendor/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastSuccessfulBuild/testReport/spdlog_vendor/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastSuccessfulBuild/testReport/spdlog_vendor/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastSuccessfulBuild/testReport/spdlog_vendor/)

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`spdlog_vendor` does not have features other than importing the external dependency `spdlog` and therefore does not require feature documentation.

### Public API Documentation [3.ii]

`spdlog_vendor` does not have an API and therefore does not require API documentation.

### License [3.iii]

The license for `spdlog_vendor` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [LICENSE](./LICENSE) file. The vendored library, `spdlog` license is MIT as stated in its [Quality declaration](./SPDLOG_QUALITY_DECLARATION.md) document (Section 5.iii).

There is an automated test which runs a linter that ensures each file has a license statement. [Here](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastSuccessfulBuild/testReport/spdlog_vendor/) can be found a list with the latest results of the various linters being run on the package.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `spdlog_vendor`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastSuccessfulBuild/testReport/spdlog_vendor/copyright/).

## Testing [4]

`spdlog_vendor` is a package providing solely CMake files and therefore does not require feature/API tests and has no coverage or performance requirements.

### Feature Testing [4.i]

`spdlog_vendor` doesn't require feature tests, they aren't needed since the vendor package only imports an external library.

### Public API Testing [4.ii]

`spdlog_vendor` doesn't require public API tests, they aren't needed since the vendor package only imports an external library.

### Coverage [4.iii]

`spdlog_vendor` doesn't require code coverage, they aren't needed since the vendor package only imports an external library.

### Performance [4.iv]

`spdlog_vendor` doesn't require performance tests, they aren't needed since the vendor package only imports an external library.

### Linters and Static Analysis [4.v]

`spdlog_vendor` uses and passes all the ROS2 standard linters and static analysis tools as described in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

Currently nightly test results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastSuccessfulBuild/testReport/spdlog_vendor/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastSuccessfulBuild/testReport/spdlog_vendor/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastSuccessfulBuild/testReport/spdlog_vendor/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastSuccessfulBuild/testReport/spdlog_vendor/)

## Dependencies [5]

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

`spdlog_vendor` does not have direct/optional runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`spdlog_vendor` depends directly on the external dependency `spdlog`, which is qualified as quality level 3 in its [Quality Declaration](./SPDLOG_QUALITY_DECLARATION.md).

## Platform Support [6]

`spdlog_vendor` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).

# Current status Summary

The chart below compares the requirements in the REP-2004 with the current state of the spdlog package.
|Number|  Requirement| Current state |
|--|--|--|
|1| **Version policy** |---|
|1.i|Version Policy available | ✓ |
|1.ii|Stable version |✓|
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
|3.i| Per feature documentation | ✓ |
|3.ii| Per public API item documentation | ✓ |
|3.iii| Declared License(s) | ✓ |
|3.iv| Copyright in source files| ✓ |
|3.v.a| Quality declaration linked to README | ✓ |
|3.v.b| Centralized declaration available for peer review |✓|
|4| Testing | --- |
|4.i| Feature items tests | ✓ |
|4.ii| Public API tests | ✓ |
|4.iii.a| Using coverage |✓ |
|4.iii.a| Coverage policy | ✓ |
|4.iv.a| Performance tests (if applicable) | ✓ |
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
|7.i| Vulnerability Disclosure Policy | ✓ |

Comparing this table with the [Quality Level Comparison Chart of REP2004](https://github.com/ros-infrastructure/rep/blob/master/rep-2004.rst#quality-level-comparison-chart) lead us to decide that this package qualifies as Quality Level 1.
