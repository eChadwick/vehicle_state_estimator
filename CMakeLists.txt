cmake_minimum_required(VERSION 3.22)
project(KalmanFilter CXX)

find_program(IWYU_PATH NAMES include-what-you-use iwyu)
if(IWYU_PATH)
    set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE "${IWYU_PATH}")
else()
    message(STATUS "IWYU not found; header hygiene checks skipped.")
endif()

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

    add_executable(unit_tests
        tests/ZeroAllocationTestFixtureTest.cpp
    )

    find_package(GTest REQUIRED)
    target_link_libraries(unit_tests PRIVATE GTest::gtest_main GTest::gtest)
endif()