cmake_minimum_required(VERSION 3.5.0)

project(Multiverse)

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(EXT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)

add_subdirectory(src)

add_subdirectory(modules)