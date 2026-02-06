cmake_minimum_required(VERSION 3.20)

# 1. Project Definitions
# ----------------------
project(QuaternionLib
        VERSION 1.0.0
        LANGUAGES CXX
        DESCRIPTION "Continuity-Aware Quaternion Library"
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 2. Dependencies (FetchContent)
# ------------------------------
include(FetchContent)

# Reduce Eigen Noise: Disable Eigen's own tests and examples
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen Docs" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Disable Eigen PkgConfig" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Disable Eigen Testing" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Disable Eigen Internal Testing" FORCE) # Temporarily disable to suppress Eigen tests

# Eigen
FetchContent_Declare(
        Eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
)
FetchContent_MakeAvailable(Eigen)

# Re-enable testing for OUR project
set(BUILD_TESTING ON CACHE BOOL "Enable Project Testing" FORCE)

# 3. Main Library Target (Header-Only)
# ------------------------------------
add_library(quaternionlib INTERFACE)

target_include_directories(quaternionlib INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(quaternionlib INTERFACE Eigen3::Eigen)
add_library(QuaternionLib::QuaternionLib ALIAS quaternionlib)

# 4. Testing
# ---------------------
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# 5. Python Bindings (Pybind11)
# -----------------------------
FetchContent_Declare(
        pybind11
        URL https://github.com/pybind/pybind11/archive/refs/tags/v2.11.1.zip
)
FetchContent_MakeAvailable(pybind11)

# The actual module name in Python will be 'quaternionlib'
# Note: The target name is quaternionlib_py to avoid conflict with the C++ library target 'quaternionlib'
pybind11_add_module(quaternionlib_py bindings/bindings.cpp)

# Link against our C++ library
target_link_libraries(quaternionlib_py PRIVATE QuaternionLib::QuaternionLib)

# Ensure it finds the headers
target_include_directories(quaternionlib_py PRIVATE include)

# Set output name to just 'quaternionlib' so in python: import quaternionlib
set_target_properties(quaternionlib_py PROPERTIES
        OUTPUT_NAME "quaternionlib.so"
        PREFIX "${PYTHON_MODULE_PREFIX}"
        SUFFIX "${PYTHON_MODULE_SUFFIX}"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}" # Force output to build root for easier finding
)