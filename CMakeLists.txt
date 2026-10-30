cmake_minimum_required(VERSION 3.22)
project(KalmanFilter CXX)

# ==============================================================================
# Configuration Toggles & Options
# ==============================================================================
option(ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==============================================================================
# Dependencies
# ==============================================================================
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

find_program(IWYU_PATH NAMES include-what-you-use iwyu)
find_program(IWYU_TOOL_PATH NAMES iwyu_tool iwyu_tool.py)

# ==============================================================================
# Core Library Definition (Header-Only Interface)
# ==============================================================================
add_library(kalman_core INTERFACE)
target_include_directories(kalman_core INTERFACE src)
target_link_libraries(kalman_core INTERFACE Eigen3::Eigen)

# ==============================================================================
# Test Executables (Standard vs. Instrumented Coverage Binary)
# ==============================================================================

# Common source files for tests
set(TEST_SOURCES
    tests/fixtures/ZeroAllocationTestFixtureTest.cpp
    tests/fixtures/ZeroAllocationTestFixture.cpp
    tests/KalmanFilterTest.cpp
)

# 1. Standard Unit Tests (with Sanitizers)
add_executable(unit_tests ${TEST_SOURCES})
target_link_libraries(unit_tests PRIVATE GTest::gtest_main GTest::gtest kalman_core)
target_include_directories(unit_tests PRIVATE tests/fixtures)

# 2. Dedicated Coverage Unit Tests (Instrumented)
add_executable(unit_tests_cov ${TEST_SOURCES})
target_link_libraries(unit_tests_cov PRIVATE GTest::gtest_main GTest::gtest kalman_core)
target_include_directories(unit_tests_cov PRIVATE tests/fixtures)
target_compile_options(unit_tests_cov PRIVATE -fprofile-arcs -ftest-coverage -O0 -g)
target_link_options(unit_tests_cov PRIVATE --coverage)

# ==============================================================================
# Compiler Warnings & Hardening (GCC / Clang)
# ==============================================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    foreach(target unit_tests unit_tests_cov)
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Werror
            -Wpedantic
            -Wconversion
            -Wsign-conversion
            -Wshadow
            -Wformat=2
        )
    endforeach()

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
    COMMAND ./unit_tests_cov
    COMMAND lcov --capture --directory . --output-file coverage.info --rc branch_coverage=1 --ignore-errors mismatch
    COMMAND lcov --remove coverage.info '/usr/*' '${CMAKE_SOURCE_DIR}/tests/*' --output-file coverage_filtered.info --rc branch_coverage=1 --ignore-errors mismatch
    COMMAND genhtml coverage_filtered.info --output-directory coverage_html --demangle-cpp --rc branch_coverage=1
    DEPENDS unit_tests_cov
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Building, testing, and generating LCOV HTML report (coverage_html/index.html)..."
)

if(IWYU_PATH AND IWYU_TOOL_PATH)
    add_custom_target(iwyu
        COMMAND ${IWYU_TOOL_PATH} -p ${CMAKE_BINARY_DIR} -- -Xiwyu --mapping_file=${CMAKE_SOURCE_DIR}/eigen.imp
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running Include-What-You-Use header analysis..."
    )
endif()