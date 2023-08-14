cmake_minimum_required(VERSION 3.5.0)

project(Multiverse)

set(EXT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)

set(ENV{PATH} "$ENV{PATH}:${CMAKE_CURRENT_SOURCE_DIR}/bin")

set(ENV{PYTHONPATH} "$ENV{PYTHONPATH}:${CMAKE_CURRENT_SOURCE_DIR}/lib/python")

set(ENV{LD_LIBRARY_PATH_TO_ADD} "$ENV{LD_LIBRARY_PATH_TO_ADD}:${CMAKE_CURRENT_SOURCE_DIR}/lib")

add_subdirectory(src)

add_subdirectory(modules)