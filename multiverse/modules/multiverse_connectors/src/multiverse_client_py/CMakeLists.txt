add_custom_target(multiverse_client_py_build ALL
    COMMAND ${PYTHON_EXECUTABLE} -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_client_py package"
    VERBATIM
)

add_custom_target(multiverse_client_py_install ALL
    DEPENDS multiverse_client_py_build
    COMMAND ${PYTHON_EXECUTABLE} -m pip install -U ${CMAKE_CURRENT_BINARY_DIR}/multiverse_client_py-0.0.1.tar.gz -t ${LIB_DIR}/dist-packages
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Installing multiverse_client_py package"
    VERBATIM
)
