cmake_minimum_required(VERSION 3.5.0)

project(Multiverse)

set(ENV{USD_SRC_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/external/USD)

set(ENV{USD_BUILD_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/build/USD)

add_subdirectory(src)

add_subdirectory(modules)