include_directories(
	${EXT_DIR}/mujoco/include
)

find_package(Threads REQUIRED)

add_executable(mujoco ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco.cpp)
target_link_libraries(mujoco
    glfw
    Threads::Threads
    ${BUILD_DIR}/mujoco/lib/libmujoco.so
)
set_target_properties(mujoco PROPERTIES
    INSTALL_RPATH ${BUILD_DIR}/mujoco/lib)
install(TARGETS mujoco DESTINATION ${BIN_DIR})