cmake_minimum_required(VERSION 3.10.0)

project(Multiverse)

set(MULTIVERSE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(BIN_DIR ${MULTIVERSE_DIR}/bin)

set(BUILD_DIR ${MULTIVERSE_DIR}/build)

set(EXT_DIR ${MULTIVERSE_DIR}/external)

set(LIB_DIR ${MULTIVERSE_DIR}/lib)

set(SRC_DIR ${MULTIVERSE_DIR}/src)

set(TEST_DIR ${MULTIVERSE_DIR}/tests)

set(STDLIB "libstdc++" CACHE STRING "Type of stdlib to use (libc++ or libstdc++)")

set(MULTIVERSE_CLIENT_LIBRARY_TYPE "STATIC" CACHE STRING "Type of library to create (SHARED or STATIC)")

set(PYTHON_EXECUTABLE "python3" CACHE STRING "Python executable to use")

set(ISAAC_SIM_PATH "/home/$ENV{USER}/.local/share/ov/pkg/isaac-sim-4.2.0" CACHE STRING "Path to Isaac Sim")

option(BUILD_SRC "Build the src" ON)
option(BUILD_MODULES "Build the modules" ON)

option(BUILD_TESTS "Build the tests" ON)

if(BUILD_SRC)
    add_subdirectory(src)
endif(BUILD_SRC)

if(BUILD_MODULES)
    add_subdirectory(modules)
endif(BUILD_MODULES)