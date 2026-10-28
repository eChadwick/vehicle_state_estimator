cmake_minimum_required(VERSION 3.22)
project(KalmanFilter CXX)

# Enable strict compiler warnings and sanitizers for GCC and Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    # Compiler Hardening: Warnings as errors
    add_compile_options(
        -Wall
        -Wextra
        -Werror
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wformat=2
    )

    # Sanitizers (Toggleable via -DENABLE_SANITIZERS=OFF)
    option(ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" ON)
    if(ENABLE_SANITIZERS)
        add_compile_options(
            -fsanitize=address,undefined
            -fno-sanitize-recover=all
            -fno-omit-frame-pointer
        )
        add_link_options(
            -fsanitize=address,undefined
        )
    endif()

    find_package(Eigen3 REQUIRED)
    add_library(kalman_core INTERFACE)
    target_include_directories(kalman_core INTERFACE
        src
    )
    target_link_libraries(kalman_core INTERFACE Eigen3::Eigen)

    add_executable(unit_tests
        tests/fixtures/ZeroAllocationTestFixtureTest.cpp
        tests/fixtures/ZeroAllocationTestFixture.cpp
        tests/KalmanFilterTest.cpp
    )

    find_package(GTest REQUIRED)

    target_link_libraries(unit_tests PRIVATE GTest::gtest_main GTest::gtest kalman_core)
    target_include_directories(unit_tests PRIVATE tests/fixtures)

    find_program(IWYU_PATH NAMES include-what-you-use iwyu)
    if(IWYU_PATH)
        set_target_properties(unit_tests PROPERTIES
        CXX_INCLUDE_WHAT_YOU_USE "${IWYU_PATH}"
    )
    else()
        message(STATUS "IWYU not found; header hygiene checks skipped.")
    endif()
    
endif()