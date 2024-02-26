add_custom_target(multiverse_ros_socket_build ALL
    COMMAND python3 -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_ros_socket package"
    VERBATIM
)

add_custom_target(multiverse_ros_socket_install ALL
    DEPENDS multiverse_ros_socket_build
    COMMAND python3 -m pip install ${CMAKE_CURRENT_BINARY_DIR}/multiverse_ros_socket-0.0.1.tar.gz
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Installing multiverse_ros_socket package"
    VERBATIM
)

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/multiverse_ros_run.py ${BIN_DIR}/multiverse_ros_run
    RESULT_VARIABLE result
)

if(result EQUAL 0)
    message(STATUS "Symbolic link for multiverse_ros_socket created successfully.")
else()
    message(WARNING "Failed to create symbolic link with error code: ${result}")
endif()
