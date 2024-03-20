add_custom_target(multiverse_client_py_build ALL
    COMMAND python3 -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_client_py package"
    VERBATIM
)

add_custom_target(multiverse_client_py_install ALL
    DEPENDS multiverse_client_py_build
    COMMAND python3 -m pip install ${CMAKE_CURRENT_BINARY_DIR}/multiverse_client_py-0.0.1.tar.gz
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Installing multiverse_client_py package"
    VERBATIM
)
