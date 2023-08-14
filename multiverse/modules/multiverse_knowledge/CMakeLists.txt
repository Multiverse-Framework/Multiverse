# Execute the shell file during CMake configuration
execute_process(
  COMMAND sh install.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE INSTALL_RESULT
)

# Check the result of the script execution
if(INSTALL_RESULT EQUAL 0)
  message(VERBOSE "install.sh executed successfully.")
else()
  message(FATAL_ERROR "Failed to execute install.sh.")
endif()