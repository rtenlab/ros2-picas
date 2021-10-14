ament_clang_format
==================

Checks the code style of C / C++ source files using `ClangFormat
<http://clang.llvm.org/docs/ClangFormat.html>`_.
Files with the following extensions are being considered:
``.c``, ``.cc``, ``.cpp``, ``.cxx``, ``.h``, ``.hh``, ``.hpp``, ``.hxx``.


How to run the check from the command line?
-------------------------------------------

.. code:: sh

    ament_clang_format [<path> ...]

When using the option ``--reformat`` the proposed changes are applied in place.


How to run the check from within a CMake ament package as part of the tests?
----------------------------------------------------------------------------

The CMake integration is provided by the package `ament_cmake_clang_format
<https://github.com/ament/ament_lint>`_.
