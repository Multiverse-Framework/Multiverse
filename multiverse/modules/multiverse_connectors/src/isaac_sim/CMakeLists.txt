if (UNIX)
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim_compile.py ${BIN_DIR}/isaac_sim_compile
    )
elseif (WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim_compile.py ${BIN_DIR}/isaac_sim_compile.py
    )
endif()