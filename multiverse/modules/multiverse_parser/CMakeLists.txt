set(ENV{USD_SRC_DIR} ${EXT_DIR}/USD)
set(ENV{USD_BUILD_DIR} ${BUILD_DIR}/USD)

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

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE BUILD_RESULT
)

if(BUILD_RESULT EQUAL 0)
  message(VERBOSE "Build multiverse_parser successfully.")
else()
  message(FATAL_ERROR "Failed to build multiverse_parser.")
endif()

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -m pip install -U ${CMAKE_CURRENT_BINARY_DIR}/multiverse_parser-0.0.1.tar.gz -t ${LIB_DIR}/dist-packages
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE INSTALL_RESULT
)

if(INSTALL_RESULT EQUAL 0)
  message(VERBOSE "Install multiverse_parser successfully.")
else()
  message(FATAL_ERROR "Failed to install multiverse_parser.")
endif()

execute_process(
  COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/multiverse_parser ${BIN_DIR}/multiverse_parser
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

set(MUJOCO_MENAGERIE_DIR ${MULTIVERSE_DIR}/resources/robots/mujoco_menagerie)
if(EXISTS "${MUJOCO_MENAGERIE_DIR}")
  execute_process(
    COMMAND ln -sf ${MUJOCO_MENAGERIE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/input/mujoco_menagerie
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
else()
  include(FetchContent)

  if(BUILD_TESTS)
    FetchContent_Populate(
      mujoco_menagerie
      GIT_REPOSITORY https://github.com/google-deepmind/mujoco_menagerie.git
      GIT_TAG 286cfe3563c963084df8dea781f39735d59c532e # Dec 6, 2024
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources/input/mujoco_menagerie
    )
  endif()
endif()