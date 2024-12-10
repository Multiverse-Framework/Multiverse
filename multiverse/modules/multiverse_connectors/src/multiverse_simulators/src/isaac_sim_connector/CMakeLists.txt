if (UNIX)
    execute_process(
        COMMAND cp -rf ${CMAKE_CURRENT_SOURCE_DIR}/src/isaac_sim_connector/isaac_sim_compiler.py ${BIN_DIR}/isaac_sim_compiler
    )
    execute_process(
        COMMAND cp -rf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim.py ${BIN_DIR}/isaac_sim
    )
    execute_process(
        COMMAND sed -i s|ISAAC_SIM_PATH|${ISAAC_SIM_PATH}|g ${BIN_DIR}/isaac_sim
    )
elseif (WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim_compiler.py ${BIN_DIR}/isaac_sim_compiler.py
    )
endif()