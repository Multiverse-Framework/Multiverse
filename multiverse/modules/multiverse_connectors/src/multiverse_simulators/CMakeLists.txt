add_custom_target(multiverse_simulator_build ALL
    COMMAND ${PYTHON_EXECUTABLE} -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_simulator package"
    VERBATIM
)

add_custom_target(multiverse_simulator_install ALL
    DEPENDS multiverse_simulator_build
    COMMAND ${PYTHON_EXECUTABLE} -m pip install -U ${CMAKE_CURRENT_BINARY_DIR}/multiverse_simulator-0.0.1.tar.gz -t ${LIB_DIR}/dist-packages
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Installing multiverse_simulator package"
    VERBATIM
)

# if (UNIX)
#     if(MULTIVERSE_CLIENT_LIBRARY_TYPE STREQUAL "STATIC" AND STDLIB STREQUAL "libstdc++")
#         add_subdirectory(src/mujoco)
#     endif()
# elseif (WIN32)
#     add_subdirectory(src/mujoco)
# endif()

# add_subdirectory(src/isaac_sim)