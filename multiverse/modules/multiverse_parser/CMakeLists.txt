add_custom_target(multiverse_parser_build ALL
    COMMAND ${BIN_DIR}/python3.10 -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_parser package"
    VERBATIM
)

add_custom_target(multiverse_parser_install ALL
    DEPENDS multiverse_parser_build
    COMMAND ${BIN_DIR}/python3.10 -m pip install ${CMAKE_CURRENT_BINARY_DIR}/multiverse_parser-0.0.1.tar.gz
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
  NAME mjcf_to_usd1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_usd/with_physics/apartment.usda
          --physics
)

add_test(
  NAME mjcf_to_usd2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_usd/no_physics/apartment.usda
          --no-physics
)

add_test(
  NAME urdf_to_usd1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_usd/with_physics/apartment.usda
          --physics
)

add_test(
  NAME urdf_to_usd2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_usd/no_physics/apartment.usda
          --no-physics
)

add_test(
  NAME mjcf_to_urdf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_urdf/with_physics/apartment.urdf
          --physics
)

add_test(
  NAME mjcf_to_urdf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_urdf/no_physics/apartment.urdf
          --no-physics
)

add_test(
  NAME urdf_to_urdf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_urdf/with_physics/apartment.urdf
          --physics
)

add_test(
  NAME urdf_to_urdf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_urdf/no_physics/apartment.urdf
          --no-physics
)

add_test(
  NAME mjcf_to_mjcf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_mjcf/with_physics/apartment.xml
          --physics
)

add_test(
  NAME mjcf_to_mjcf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_mjcf/no_physics/apartment.xml
          --no-physics
)

add_test(
  NAME urdf_to_mjcf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_mjcf/with_physics/apartment.xml
          --physics
)

add_test(
  NAME urdf_to_mjcf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_mjcf/no_physics/apartment.xml
          --no-physics
)

add_test(
  NAME usd_to_usd1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_usd/with_physics/apartment.usda
          --physics
)

add_test(
  NAME usd_to_usd2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_usd/no_physics/apartment.usda
          --no-physics
)

add_test(
  NAME usd_to_urdf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_urdf/with_physics/apartment.urdf
          --physics
)

add_test(
  NAME usd_to_urdf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_urdf/no_physics/apartment.urdf
          --no-physics
)

add_test(
  NAME usd_to_mjcf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_mjcf/with_physics/apartment.xml
          --physics
)

add_test(
  NAME usd_to_mjcf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_mjcf/no_physics/apartment.xml
          --no-physics
)