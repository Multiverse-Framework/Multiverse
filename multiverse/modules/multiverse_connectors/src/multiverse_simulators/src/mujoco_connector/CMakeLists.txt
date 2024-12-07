if (UNIX)
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco_compile.py ${BIN_DIR}/mujoco_compile
    )
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/pymujoco.py ${BIN_DIR}/pymujoco
    )

    set(ENV{MUJOCO_SRC_DIR} ${EXT_DIR}/mujoco)
    set(ENV{MUJOCO_BUILD_DIR} ${BUILD_DIR}/mujoco)

    # Execute the shell file during CMake configuration
    execute_process(
        COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/install.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE INSTALL_RESULT
    )
elseif (WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco_compile.py ${BIN_DIR}/mujoco_compile.py
    )
endif()