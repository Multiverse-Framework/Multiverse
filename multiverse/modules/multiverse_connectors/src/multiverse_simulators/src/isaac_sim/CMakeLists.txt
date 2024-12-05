if (UNIX)
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim_compile.py ${BIN_DIR}/isaac_sim_compile
    )
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim.sh ${BIN_DIR}/isaac_sim
    )
elseif (WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim_compile.py ${BIN_DIR}/isaac_sim_compile.py
    )
endif()