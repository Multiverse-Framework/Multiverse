project(multiverse_knowledge)

set(ENV{USD_SRC_DIR} ${EXT_DIR}/USD)
set(ENV{USD_BUILD_DIR} ${BUILD_DIR}/USD)
set(ENV{PYTHON_EXECUTABLE} ${PYTHON_EXECUTABLE})
set(ENV{PYTHONPATH} $ENV{PYTHONPATH}:$ENV{USD_BUILD_DIR}/lib/python)

# Execute the shell file during CMake configuration
execute_process(
  COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/install.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE INSTALL_RESULT
)

# Check the result of the script execution
if(INSTALL_RESULT EQUAL 0)
  message(VERBOSE "install.sh executed successfully.")
else()
  message(FATAL_ERROR "Failed to execute install.sh.")
endif()

INSTALL(CODE "execute_process( \
    COMMAND ${CMAKE_COMMAND} -E create_symlink \
    ${CMAKE_CURRENT_SOURCE_DIR}/Konclude/src/queries/queries.py \
    ${LIB_DIR}/dist-packages/multiverse_knowledge.py   \
    )"
)

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/Konclude/bin/Konclude ${BIN_DIR}/Konclude
    RESULT_VARIABLE result
)

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/TBox_to_usd.py ${BIN_DIR}/TBox_to_usd
    RESULT_VARIABLE result
)

if(result EQUAL 0)
    message(STATUS "Symbolic link for TBox_to_usd created successfully.")
else()
    message(WARNING "Failed to create symbolic link with error code: ${result}")
endif()

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/semantic_tagging.py ${BIN_DIR}/semantic_tagging
    RESULT_VARIABLE result
)

if(result EQUAL 0)
    message(STATUS "Symbolic link for semantic_tagging created successfully.")
else()
    message(WARNING "Failed to create symbolic link with error code: ${result}")
endif()

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/semantic_reporting.py ${BIN_DIR}/semantic_reporting
    RESULT_VARIABLE result
)

if(result EQUAL 0)
    message(STATUS "Symbolic link for semantic_reporting created successfully.")
else()
    message(WARNING "Failed to create symbolic link with error code: ${result}")
endif()

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/usd_to_ABox.py ${BIN_DIR}/usd_to_ABox
    RESULT_VARIABLE result
)

if(result EQUAL 0)
    message(STATUS "Symbolic link for usd_to_ABox created successfully.")
else()
    message(WARNING "Failed to create symbolic link with error code: ${result}")
endif()