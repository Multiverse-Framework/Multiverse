execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE BUILD_RESULT
)

if(BUILD_RESULT EQUAL 0)
    message(VERBOSE "Build multiverse_client_py successfully.")
else()
    message(FATAL_ERROR "Failed to build multiverse_client_py.")
endif()

execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m pip install -U ${CMAKE_CURRENT_BINARY_DIR}/multiverse_client_py-0.0.1.tar.gz -t ${LIB_DIR}/dist-packages
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE INSTALL_RESULT
)

if(INSTALL_RESULT EQUAL 0)
    message(VERBOSE "Install multiverse_client_py successfully.")
else()
    message(FATAL_ERROR "Failed to install multiverse_client_py.")
endif()