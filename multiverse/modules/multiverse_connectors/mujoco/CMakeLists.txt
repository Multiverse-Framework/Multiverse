include_directories(
	${EXT_DIR}/mujoco/include
)

find_package(Threads REQUIRED)

add_executable(mujoco ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco.cpp)
target_link_libraries(mujoco
    glfw
    Threads::Threads
    jsoncpp
    ${BUILD_DIR}/mujoco/lib/libmujoco.so
)
set_target_properties(mujoco PROPERTIES
    INSTALL_RPATH ${BUILD_DIR}/mujoco/lib)
install(TARGETS mujoco DESTINATION ${BIN_DIR})

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco_compile ${BIN_DIR}/mujoco_compile
    RESULT_VARIABLE result
)

if(result EQUAL 0)
    message(STATUS "Symbolic link for mujoco_compile created successfully.")
else()
    message(WARNING "Failed to create symbolic link with error code: ${result}")
endif()