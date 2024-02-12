cmake_minimum_required(VERSION 3.5.0)

project(Multiverse)

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(EXT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)

if(UNIX)
    set(STDLIB "libstdc++" CACHE STRING "Type of stdlib to use (libc++ or libstdc++)")
endif()

add_subdirectory(src)

add_subdirectory(modules)