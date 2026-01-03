#----------------------------------------------------------------------------
# TMS9900 Trace - Standalone TMS9900 CPU Simulator
#
# CMakeLists.txt - Build configuration
#
# Copyright (c) 2024
# Licensed under GPL v2 (due to ti99sim heritage)
#----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.14)

project(tms9900-trace
    VERSION 0.1.0
    DESCRIPTION "Standalone TMS9900 CPU Simulator with Trace Output"
    LANGUAGES CXX
)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)

# Source files
set(SOURCES
    src/main.cpp
    src/cpu.cpp
    src/disasm.cpp
)

# Executable
add_executable(tms9900-trace ${SOURCES})

# Installation
install(TARGETS tms9900-trace
    RUNTIME DESTINATION bin
)

# Optional: Build type defaults to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
