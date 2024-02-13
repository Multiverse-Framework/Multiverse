add_subdirectory(src)

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