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

include(FetchContent)
if (BUILD_TESTS)
    FetchContent_Populate(
      mujoco_menagerie
      GIT_REPOSITORY https://github.com/google-deepmind/mujoco_menagerie.git
      GIT_TAG        286cfe3563c963084df8dea781f39735d59c532e # Dec 6, 2024
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources/mjcf/mujoco_menagerie
    )
endif()

if (UNIX)
    add_subdirectory(src/mujoco_connector)
elseif (WIN32)
    add_subdirectory(src/mujoco_connector)
endif()

# add_subdirectory(src/isaac_sim)