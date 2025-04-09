if (UNIX)
    execute_process(
        COMMAND cp -rf ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco_connector/mujoco_compiler.py ${BIN_DIR}/mujoco_compiler
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    execute_process(
        COMMAND cp -rf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco.py ${BIN_DIR}/mujoco
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    set(ENV{MUJOCO_SRC_DIR} ${EXT_DIR}/mujoco)
    set(ENV{MUJOCO_BUILD_DIR} ${BUILD_DIR}/mujoco)

    # Execute the shell file during CMake configuration
    execute_process(
        COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/install.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE INSTALL_RESULT
    )

    # Check the result of the script execution
    if(INSTALL_RESULT EQUAL 0)
        message(VERBOSE "install.sh executed successfully.")
    else()
        message(FATAL_ERROR "Failed to execute install.sh.")
    endif()
elseif (WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco_connector/mujoco_compiler.py ${BIN_DIR}/mujoco_compiler.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco.py ${BIN_DIR}/mujoco.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    set(ENV{MUJOCO_SRC_DIR} ${EXT_DIR}/mujoco)
    set(ENV{MUJOCO_BUILD_DIR} ${BUILD_DIR}/mujoco)

    # Execute the batch file during CMake configuration
    execute_process(
        COMMAND cmd /c ${CMAKE_CURRENT_SOURCE_DIR}/install.bat
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE INSTALL_RESULT
    )

    # Check the result of the script execution
    if(INSTALL_RESULT EQUAL 0)
        message(VERBOSE "install.bat executed successfully.")
    else()
        message(FATAL_ERROR "Failed to execute install.bat.")
    endif()
endif()