project(multiverse_connectors)

add_custom_target(multiverse_launch_build ALL
    COMMAND ${PYTHON_EXECUTABLE} -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_launch package"
    VERBATIM
)

add_custom_target(multiverse_launch_install ALL
    DEPENDS multiverse_launch_build
    COMMAND ${PYTHON_EXECUTABLE} -m pip install -U ${CMAKE_CURRENT_BINARY_DIR}/multiverse_launch-0.0.1.tar.gz -t ${LIB_DIR}/dist-packages
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Installing multiverse_launch package"
    VERBATIM
)

# add_subdirectory(src)

if (UNIX)
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/bin/multiverse_launch.sh ${BIN_DIR}/multiverse_launch
    )

    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/bin/multiverse_view.sh ${BIN_DIR}/multiverse_view
    )
elseif (WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/bin/multiverse_launch.bat ${BIN_DIR}/multiverse_launch.bat
    )
endif()