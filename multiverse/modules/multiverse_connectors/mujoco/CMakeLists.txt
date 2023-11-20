execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco_compile ${BIN_DIR}/mujoco_compile
)

include_directories(
	${EXT_DIR}/mujoco/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SRC_DIR}/multiverse_client/include
)

find_package(Threads REQUIRED)

add_executable(mujoco_headless ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco_headless.cpp)
add_library(mujoco_headless_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mj_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mj_multiverse_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mj_simulate.cpp
)
target_link_libraries(mujoco_headless
    mujoco_headless_lib
    multiverse_client_json
    Threads::Threads
    jsoncpp
    boost_filesystem
    ${BUILD_DIR}/mujoco/lib/libmujoco.so
)
set_target_properties(mujoco_headless PROPERTIES
    INSTALL_RPATH "${BUILD_DIR}/mujoco/lib")
install(TARGETS mujoco_headless DESTINATION ${BIN_DIR})

add_executable(mujoco ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco.cpp)
add_library(mujoco_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mj_visual.cpp
)
target_link_libraries(mujoco
    mujoco_lib
    mujoco_headless_lib
    multiverse_client_json
    glfw
    GL
    Threads::Threads
    jsoncpp
    boost_filesystem
    ${BUILD_DIR}/mujoco/lib/libmujoco.so
)
set_target_properties(mujoco PROPERTIES
    INSTALL_RPATH "${BUILD_DIR}/mujoco/lib")
install(TARGETS mujoco DESTINATION ${BIN_DIR})