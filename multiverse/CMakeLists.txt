cmake_minimum_required(VERSION 3.5.0)

project(Multiverse)

set(EXT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)

add_subdirectory(src)

add_subdirectory(modules)