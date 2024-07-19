add_custom_target(multiverse_ros_socket_build ALL
    COMMAND ${PYTHON_EXECUTABLE} -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_ros_socket package"
    VERBATIM
)

add_custom_target(multiverse_ros_socket_install ALL
    DEPENDS multiverse_ros_socket_build
    COMMAND ${PYTHON_EXECUTABLE} -m pip install -U ${CMAKE_CURRENT_BINARY_DIR}/multiverse_ros_socket-0.0.1.tar.gz -t ${LIB_DIR}/dist-packages
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Installing multiverse_ros_socket package"
    VERBATIM
)

if (UNIX)
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/multiverse_ros_run.py ${BIN_DIR}/multiverse_ros_run
        RESULT_VARIABLE result
    )
elseif(WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/multiverse_ros_run.py ${BIN_DIR}/multiverse_ros_run.py
        RESULT_VARIABLE result
    )
endif()


if(result EQUAL 0)
    message(STATUS "Symbolic link for multiverse_ros_socket created successfully.")
else()
    message(WARNING "Failed to create symbolic link with error code: ${result}")
endif()
