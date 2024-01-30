add_subdirectory(src)

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/bin/multiverse_launch ${BIN_DIR}/multiverse_launch
)

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/bin/multiverse_view ${BIN_DIR}/multiverse_view
)