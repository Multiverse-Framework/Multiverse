if (UNIX)
    execute_process(
        COMMAND cp -rf ${CMAKE_CURRENT_SOURCE_DIR}/src/isaac_sim_connector/isaac_sim_compiler.py ${BIN_DIR}/isaac_sim_compiler
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    execute_process(
        COMMAND cp -rf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim.py ${BIN_DIR}/isaac_sim
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/extensions/multiverse_connector /home/$ENV{USER}/Documents/Kit/shared/exts
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
elseif (WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/isaac_sim_connector/isaac_sim_compiler.py ${BIN_DIR}/isaac_sim_compiler.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_sim.py ${BIN_DIR}/isaac_sim.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()