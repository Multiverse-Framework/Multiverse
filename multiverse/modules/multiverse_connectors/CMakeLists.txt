add_subdirectory(mujoco)

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/bin/multiverse_launch ${BIN_DIR}/multiverse_launch
)