execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE BUILD_RESULT
)

if(BUILD_RESULT EQUAL 0)
  message(VERBOSE "Build multiverse_simulator successfully.")
else()
  message(FATAL_ERROR "Failed to build multiverse_simulator.")
endif()

execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m pip install -U ${CMAKE_CURRENT_BINARY_DIR}/multiverse_simulator-0.0.1.tar.gz -t ${LIB_DIR}/dist-packages
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE INSTALL_RESULT
)

if(INSTALL_RESULT EQUAL 0)
  message(VERBOSE "Install multiverse_simulator successfully.")
else()
  message(FATAL_ERROR "Failed to install multiverse_simulator.")
endif()

include(FetchContent)
if (BUILD_TESTS)
    FetchContent_Populate(
      mujoco_menagerie
      GIT_REPOSITORY https://github.com/google-deepmind/mujoco_menagerie.git
      GIT_TAG        286cfe3563c963084df8dea781f39735d59c532e # Dec 6, 2024
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources/mjcf/mujoco_menagerie
    )
endif()

add_subdirectory(src/mujoco_connector)

if(EXISTS "${ISAAC_SIM_PATH}")
    add_subdirectory(src/isaac_sim_connector)
endif()