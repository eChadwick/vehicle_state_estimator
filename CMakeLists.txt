cmake_minimum_required(VERSION 3.16)
project(vehicle_state_estimator CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict compiler warnings as errors for GCC and Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
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
endif()