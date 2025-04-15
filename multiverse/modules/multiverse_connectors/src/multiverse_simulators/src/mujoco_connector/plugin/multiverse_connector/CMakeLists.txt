if (WIN32)
  find_library(JSONCPP_LIBRARY
    NAMES jsoncpp
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../../../build/vcpkg_installed/x64-windows/lib"
    NO_DEFAULT_PATH
    REQUIRED
  )
  find_library(ZMQ_LIBRARY
    NAMES libzmq-mt-4_3_5
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../../../build/vcpkg_installed/x64-windows/lib"
    NO_DEFAULT_PATH
    REQUIRED
  )
endif()

if (UNIX)
  set(MUJOCO_MULTIVERSE_CONNECTOR_INCLUDE 
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  )
elseif (WIN32)
  set(MUJOCO_MULTIVERSE_CONNECTOR_INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../build/vcpkg_installed/x64-windows/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  )
endif()

set(MUJOCO_MULTIVERSE_CONNECTOR_SRCS
  multiverse_connector.cc
  multiverse_connector.h
  register.cc
)

add_library(multiverse_connector SHARED)
target_sources(multiverse_connector PRIVATE ${MUJOCO_MULTIVERSE_CONNECTOR_SRCS})
target_include_directories(multiverse_connector PRIVATE ${MUJOCO_MULTIVERSE_CONNECTOR_INCLUDE})
if (UNIX)
  target_link_libraries(multiverse_connector PRIVATE 
    mujoco
    ${CMAKE_CURRENT_SOURCE_DIR}/libmultiverse_client_json.so
    ${CMAKE_CURRENT_SOURCE_DIR}/libmultiverse_client.a
  )
elseif (WIN32)
  target_link_libraries(multiverse_connector PRIVATE 
    mujoco
    ${CMAKE_CURRENT_SOURCE_DIR}/multiverse_client_json.lib
    ${CMAKE_CURRENT_SOURCE_DIR}/multiverse_client.lib
    ${JSONCPP_LIBRARY}
    ${ZMQ_LIBRARY}
  )
endif()
target_compile_options(
  multiverse_connector
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)
target_link_options(
  multiverse_connector
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)
