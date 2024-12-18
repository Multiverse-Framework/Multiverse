execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE BUILD_RESULT
)

if(BUILD_RESULT EQUAL 0)
    message(VERBOSE "Build multiverse_ros_socket successfully.")
else()
    message(FATAL_ERROR "Failed to build multiverse_ros_socket.")
endif()

execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m pip install -U ${CMAKE_CURRENT_BINARY_DIR}/multiverse_ros_socket-0.0.1.tar.gz -t ${LIB_DIR}/dist-packages
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE INSTALL_RESULT
)

if(INSTALL_RESULT EQUAL 0)
    message(VERBOSE "Install multiverse_ros_socket successfully.")
else()
    message(FATAL_ERROR "Failed to install multiverse_ros_socket.")
endif()

if (UNIX)
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/multiverse_ros_run.py ${BIN_DIR}/multiverse_ros_run
    )
elseif(WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/multiverse_ros_run.py ${BIN_DIR}/multiverse_ros_run.py
    )
endif()
