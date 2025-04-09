if(WIN32)
    find_package(cppzmq REQUIRED)
    find_package(jsoncpp REQUIRED)
endif()

set(MUJOCO_MULTIVERSE_CONNECTOR_INCLUDE 
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

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
