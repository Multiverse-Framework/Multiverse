cmake_minimum_required(VERSION 3.5.0)

project(Multiverse)

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(EXT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)

set(STDLIB "libstdc++" CACHE STRING "Type of stdlib to use (libc++ or libstdc++)")

set(MULTIVERSE_CLIENT_LIBRARY_TYPE "STATIC" CACHE STRING "Type of library to create (SHARED or STATIC)")

set(PYTHON_EXECUTABLE "python3.8" CACHE STRING "Python executable to use")

set(ISAAC_SIM_PATH "/home/$ENV{USER}/.local/share/ov/pkg/isaac-sim-4.2.0" CACHE STRING "Path to Isaac Sim")

option(BUILD_SRC "Build the src" ON)
option(BUILD_MODULES "Build the modules" ON)

option(BUILD_TESTS "Build the tests" ON)

if (WIN32)
    set(PYTHON_EXECUTABLE "python")
endif()

if(BUILD_SRC)
    add_subdirectory(src)
endif(BUILD_SRC)

if(BUILD_MODULES)
    add_subdirectory(modules)
endif(BUILD_MODULES)