# rcpputils: ROS 2 C++ Utilities

`rcpputils` is a C++ API consisting of macros, functions, and data structures intended for use throughout the ROS 2 codebase

This package currently contains:
* Clang thread safety annotation macros
* Library discovery
* String helpers
* File system helpers
* Type traits helpers

## Clang Thread Safety Annotation Macros
the `rcpputils/thread_safety_annotations.hpp` header provides macros for Clang's [Thread Safety Analysis](https://clang.llvm.org/docs/ThreadSafetyAnalysis.html) feature.

The macros allow you to annotate your code, but expand to nothing when using a non-clang compiler, so they are safe for cross-platform use.

To use thread safety annotation in your package (in a Clang+libcxx build), enable the `-Wthread-safety` compiler flag.

For example usage, see [the documentation of this feature](https://clang.llvm.org/docs/ThreadSafetyAnalysis.html) and the tests in `test/test_basic.cpp`

## Library Discovery

In `rcpputils/find_library.hpp`:

*   `find_library(library_name)`: Namely used for dynamically loading RMW
    implementations.
    *   For dynamically loading user-defind plugins in C++, please use
        [`pluginlib`](https://github.com/ros/pluginlib) instead.
