This document is a declaration of software quality for the `foonathan_memory` package imported by `foonathan_memory_vendor`, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `foonathan_memory` Quality Declaration

This quality declaration claims that the third party package `foonathan_memory` is in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`foonathan_memory` does not have a declared versioning scheme.

### Version Stability [1.ii]

`foonathan_memory` currently has a version less than 1.0 but is in maintenance mode.
Large API changes may occur during minor version updates, and smaller API additions may occur during patch version updates.
A summary of its changes are contained in its [CHANGELOG](https://github.com/foonathan/memory/blob/master/CHANGELOG.MD).

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API with the exception of the `detail` directory and corresponding namespace.

All installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`foonathan_memory` is a third party package and does not follow the ROS distribution timeline.
It may introduce breaking API changes within a ROS distribution.
To avoid breaking API within a ROS distribution, its vendor package `foonathan_memory_vendor` shall import a specific version.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`foonathan_memory` is a third party package and does not follow the ROS distribution timeline.
It may introduce breaking ABI changes within a ROS distribution.
To avoid breaking API within a ROS distribution, its vendor package `foonathan_memory_vendor` shall import a specific version.

## Change Control Process [2]

`foonathan_memory` does not have a stated change control process.

### Change Requests [2.i]

This package does not require that changes occur through a pull request.
Many changes are committed directly by its maintainer.

### Contributor Origin [2.ii]

This package does not have a confirmation of contributor origin policy.

### Peer Review Policy [2.iii]

There is no stated peer review policy, but each pull request is at least reviewed by the core maintainer.

### Continuous Integration [2.iv]

Changes must pass CI on AMD 64 for:
- Ubuntu 16.04 for gcc 4.9 and 5.5
- MacOS with XCode 9.4 with Apple LLVM version 9.1.0
- Windows with Microsoft Visual Studio 2015, MSVC 18.0

ARM 64 support is not directly evaluated by this package's CI process.

Regular CI results can be seen here:
* [Ubuntu and OS X](https://travis-ci.org/github/foonathan/memory).
* [Windows](https://ci.appveyor.com/project/foonathan/memory/branch/master).

### Documentation Policy [2.v]

This package does not have a stated documentation policy.

## Documentation [3]

### Feature Documentation [3.i]

`foonathan_memory` has a [feature list](https://foonathan.net/memory/index.html) with descriptions of its main features.

### Public API Documentation [3.ii]

`foonathan_memory` has embedded API documentation that is generated using doxygen and is [hosted](https://foonathan.net/memory/index.html) alongside the feature documentation.
There is documentation for nearly all of the public API.

### License [3.iii]

The license for `foonathan_memory` is zlib, a summary statement is provided in each source file, and a full copy of the license is in the [LICENSE](https://raw.githubusercontent.com/foonathan/memory/master/LICENSE) file.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `foonathan_memory`.

## Testing [4]

### Feature Testing [4.i]

Many features in `foonathan_memory` have corresponding tests which simulate typical usage, and they are located in the `test` directory.

### Public API Testing [4.ii]

Many parts of the public API have tests.

### Coverage [4.iii]

`foonathan_memory` does not track testing coverage.

### Performance [4.iv]

`foonathan_memory` does not conduct performance tests.

### Linters and Static Analysis [4.v]

`foonathan_memory` does not conduct tests with linters on its public CI.

## Dependencies [5]

`foonathan_memory` does not have any dependencies outside of the C++ standard library.

## Platform Support [6]

`foonathan_memory` supports the following platforms and tests each one:
- Ubuntu 16.04 for gcc 4.9 and 5.5
- MacOS with XCode 9.4 with Apple LLVM version 9.1.0
- Windows with Microsoft Visual Studio 2015, MSVC 18.0

ARM 64 support is not directly evaluated by this package's CI process.

Regular CI results can be seen here:
* [Ubuntu and OS X](https://travis-ci.org/github/foonathan/memory).
* [Windows](https://ci.appveyor.com/project/foonathan/memory/branch/master).

## Vulnerability Disclosure Policy [7.i]

`foonathan_memory` does not have a Vulnerability Disclosure Policy


## Third-Party Package Quality Level Summary [8]

As `foonathan_memory` is a third-party package in maintenance mode, it is unlikely that it will be able to accomplish the remaining steps to become a Quality Level 1 type of package.
However, as part of the ROS 2 ecosystem it will be important that it can be validated to the equivalent quality level.

Even though `foonathan_memory` by itself will not likely reach the equivalent level of quality as Quality Level 1, there are steps that can be taken by ROS contributors to ensure that its incorporation into ROS packages can provide the equivalent level of quality.
ROS contributors will need to conduct coverage tests to identify the remaining API and features that are not currently covered by tests.
If features and portions of the public API that are used in ROS dependencies are found to be untested, then the appropriate tests will be required.
To solidify version stability, `foonathan_memory_vendor` shall declare a targeted version for each ROS 2 distribution release.
If a newer version is desired for a previously released ROS 2 distribution, then it will need to be verified that the API does not change between versions.
`foonathan_memory` tests will need to be conducted on all ROS 2 tier 1 platforms for each pinned version so regressions can be identified and ticketed upstream.
Performance tests should be written to ensure that `foonathan_memory` meets its performance targets and the ROS community can track performance regressions in newer pinned versions.
`foonathan_memory` does not have a rigorous change control policy, therefore the onus is on the ROS packages incorporating each pinned version to provide a suitable review of its quality as a third-party dependency. As `foonathan_memory` is not maintained by the ROS community, requirements for automatic linting can be waved.
