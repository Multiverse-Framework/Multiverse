add_custom_target(multiverse_parser_build ALL
    COMMAND python3.10 -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_parser package"
    VERBATIM
)

add_custom_target(multiverse_parser_install ALL
    DEPENDS multiverse_parser_build
    COMMAND python3.10 -m pip install ${CMAKE_CURRENT_BINARY_DIR}/multiverse_parser-0.0.1.tar.gz
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Installing multiverse_parser package"
    VERBATIM
)

execute_process(
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/multiverse_parser ${BIN_DIR}/multiverse_parser
    RESULT_VARIABLE result
)

if(result EQUAL 0)
    message(STATUS "Symbolic link for multiverse_parser created successfully.")
else()
    message(WARNING "Failed to create symbolic link with error code: ${result}")
endif()

enable_testing()

add_test(
  NAME mjcf_importer1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/mjcf_importer/usd/with_physics/apartment.usda
          --physics
)

add_test(
  NAME mjcf_importer2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/mjcf_importer/usd/no_physics/apartment.usda
          --no-physics
)