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

if(WIN32)
    include_directories(${BUILD_DIR}/vcpkg_installed/x64-windows/include)
endif()

option(BUILD_SRC "Build the src" ON)
option(BUILD_MODULES "Build the modules" ON)

if(BUILD_SRC)
    add_subdirectory(src)
endif(BUILD_SRC)

if(BUILD_MODULES)
    add_subdirectory(modules)
endif(BUILD_MODULES)