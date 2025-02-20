cmake_minimum_required(VERSION 3.15...3.27)
project(pysyslink_base VERSION "0.0.1")


set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug")

include(litgen_cmake/litgen_cmake.cmake)


# Build our example C++ library (PySysLinkBase) that we want to bind to python
add_subdirectory(src/cpp_libraries/PySysLinkBase)


# Build a python module that provides bindings to the C++ library
# We will build it either with pybind11 or with nanobind:
# - By default we build with pybind11
# - If you set the environment variable LITGEN_USE_NANOBIND to "ON", we will build with nanobind.
#   With bash, you can set the variable with:
#       export LITGEN_USE_NANOBIND=ON
#
# (Feel free to simply remove the option and the if statement if you want to use only one of the two)
#########################################################################
option(LITGEN_USE_NANOBIND OFF)
if (DEFINED ENV{LITGEN_USE_NANOBIND})
    set(LITGEN_USE_NANOBIND $ENV{LITGEN_USE_NANOBIND})
else()
    set(LITGEN_USE_NANOBIND OFF)
endif()
message(STATUS "LITGEN_USE_NANOBIND: ${LITGEN_USE_NANOBIND}")

if (NOT LITGEN_USE_NANOBIND)
    # Case where we build with pybind11

    # litgen_find_pybind11 is defined in litgen_cmake/litgen_cmake.cmake
    # and helps to find pybind11, whenever building with CMake or with pip / skbuild.
    # When building with CMake, you may have to call cmake with -DPython_EXECUTABLE=/path/to/python
    litgen_find_pybind11()

    # The python module sources
    set(python_module_sources
        _pydef_pybind11/module.cpp                  # The python module entry point
        _pydef_pybind11/pybind_PySysLinkBase.cpp          # The pybind11 bindings to the library, which are mainly auto-generated by litgen
    )

    # _pysyslink_base is the native python module that will be built by calling pybind11_add_module
    # This will output a dynamic library called for example:
    #     _pysyslink_base.cpython-312-darwin.so on macOS
    #     _pysyslink_base.cpython-312-x86_64-linux-gnu.so on Linux
    #     _pysyslink_base.cp312-win_amd64.pyd on Windows)
    pybind11_add_module(_pysyslink_base ${python_module_sources})
    # Call litgen_setup_module to generate the python wrapper around the native module
    litgen_setup_module(
        # The C++ library for which we are building bindings
        PySysLinkBase
        # The native python module name
        _pysyslink_base
        # This is the python wrapper around the native module
        pysyslink_base
        # The path where the python module will be copied after build (for editable mode)
        ${CMAKE_CURRENT_SOURCE_DIR}/_stubs
    )
else()
    # Case where we build with nanobind

    litgen_find_nanobind()

    # The python module sources
    set(python_module_sources
        _pydef_nanobind/module.cpp                  # The python module entry point
        _pydef_nanobind/nanobind_PySysLinkBase.cpp        # The pybind11 bindings to the library, which are mainly auto-generated by litgen
    )

    # _pysyslink_base is the native python module that will be built by calling pybind11_add_module
    # This will output a dynamic library called for example:
    #     _pysyslink_base.cpython-312-darwin.so on macOS
    #     _pysyslink_base.cpython-312-x86_64-linux-gnu.so on Linux
    #     _pysyslink_base.cp312-win_amd64.pyd on Windows)
    nanobind_add_module(_pysyslink_base ${python_module_sources})
    # Call litgen_setup_module to generate the python wrapper around the native module
    litgen_setup_module(
        # The C++ library for which we are building bindings
        PySysLinkBase
        # The native python module name
        _pysyslink_base
        # This is the python wrapper around the native module
        pysyslink_base
        # The path where the python module will be copied after build (for editable mode)
        ${CMAKE_CURRENT_SOURCE_DIR}/_stubs
    )

endif()

    # Provide a tool to debug the python module
#########################################################################
if (NOT SKBUILD AND PROJECT_IS_TOP_LEVEL)
    add_subdirectory(src/pybind_native_debug)
endif()
