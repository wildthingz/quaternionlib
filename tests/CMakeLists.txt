# tests/CMakeLists.txt

# 1. Fetch GoogleTest (Automatic Download)
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent GTest from overriding /MD with /MT
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# =========================================================
# 2. C++ Unit Tests (GTest)
# =========================================================
# Note: Source files are now in the 'cpp' subdirectory
add_executable(unit_tests
        cpp/test_continuity.cpp
        cpp/test_math.cpp
)

target_link_libraries(unit_tests PRIVATE
        QuaternionLib::QuaternionLib  # Link your library
        GTest::gtest_main             # Link GoogleTest
)

# Register C++ tests with CTest
include(GoogleTest)
gtest_discover_tests(unit_tests)

# =========================================================
# 3. Python Integration Tests
# =========================================================
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Helper function to register Python scripts
function(add_python_test TEST_NAME SCRIPT_PATH)
    add_test(NAME ${TEST_NAME} COMMAND Python3::Interpreter ${SCRIPT_PATH})

    # Inject PYTHONPATH so scripts can find the compiled library in the build dir
    if(WIN32)
        set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR};$ENV{PYTHONPATH}")
    else()
        set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}:$ENV{PYTHONPATH}")
    endif()
endfunction()

# Register scripts (using the standardized filenames)
add_python_test(py_fuzz_trajectories ${CMAKE_CURRENT_SOURCE_DIR}/python/test_fuzz_trajectories.py)
add_python_test(py_verify_truth ${CMAKE_CURRENT_SOURCE_DIR}/python/test_truth_table.py)
add_python_test(py_binding_basic ${CMAKE_CURRENT_SOURCE_DIR}/python/test_binding.py)