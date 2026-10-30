cmake_minimum_required(VERSION 3.22)
project(KalmanFilter CXX)

# ==============================================================================
# Configuration Toggles & Options
# ==============================================================================
option(ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" ON)
option(ENABLE_COVERAGE   "Enable code coverage instrumentation"                  OFF)

# ==============================================================================
# Dependencies
# ==============================================================================
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

# ==============================================================================
# Core Library Definition (Header-Only Interface)
# ==============================================================================
add_library(kalman_core INTERFACE)
target_include_directories(kalman_core INTERFACE src)
target_link_libraries(kalman_core INTERFACE Eigen3::Eigen)

# ==============================================================================
# Test Executable Definition
# ==============================================================================
add_executable(unit_tests
    tests/fixtures/ZeroAllocationTestFixtureTest.cpp
    tests/fixtures/ZeroAllocationTestFixture.cpp
    tests/KalmanFilterTest.cpp
)

target_link_libraries(unit_tests PRIVATE GTest::gtest_main GTest::gtest kalman_core)
target_include_directories(unit_tests PRIVATE tests/fixtures)

# ==============================================================================
# Compiler Warnings, Hardening, Sanitizers & Coverage (GCC / Clang)
# ==============================================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    # Strict Warnings as Errors
    target_compile_options(unit_tests PRIVATE
        -Wall
        -Wextra
        -Werror
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wformat=2
    )

    # Sanitizers
    if(ENABLE_SANITIZERS)
        target_compile_options(unit_tests PRIVATE
            -fsanitize=address,undefined
            -fno-sanitize-recover=all
            -fno-omit-frame-pointer
        )
        target_link_options(unit_tests PRIVATE
            -fsanitize=address,undefined
        )
    endif()

    # Code Coverage
    if(ENABLE_COVERAGE)
        target_compile_options(unit_tests PRIVATE
            -fprofile-arcs
            -ftest-coverage
            -O0
            -g
        )
        target_link_options(unit_tests PRIVATE
            --coverage
        )
    endif()
endif()

# ==============================================================================
# Static Analysis (Include-What-You-Use)
# ==============================================================================
find_program(IWYU_PATH NAMES include-what-you-use iwyu)
if(IWYU_PATH)
    set_target_properties(unit_tests PROPERTIES
        CXX_INCLUDE_WHAT_YOU_USE "${IWYU_PATH}"
    )
else()
    message(STATUS "IWYU not found; header hygiene checks skipped.")
endif()

# ==============================================================================
# Developer Workflow Custom Targets
# ==============================================================================
add_custom_target(run_tests
    COMMAND ./unit_tests
    DEPENDS unit_tests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running unit test suite..."
)

add_custom_target(debug_tests
    COMMAND gdb ./unit_tests
    DEPENDS unit_tests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Launching test binary in GDB..."
)

add_custom_target(test_coverage
    COMMAND ./unit_tests
    COMMAND lcov --capture --directory . --output-file coverage.info --rc branch_coverage=1 --ignore-errors mismatch
    COMMAND lcov --remove coverage.info '/usr/*' '${CMAKE_SOURCE_DIR}/tests/*' --output-file coverage_filtered.info --rc branch_coverage=1 --ignore-errors mismatch
    COMMAND genhtml coverage_filtered.info --output-directory coverage_html --demangle-cpp --rc branch_coverage=1
    DEPENDS unit_tests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Building, testing, and generating LCOV HTML report (coverage_html/index.html)..."
)