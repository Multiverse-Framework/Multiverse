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

add_custom_target(multiverse_parser_build ALL
    COMMAND python3 -m build ${CMAKE_CURRENT_SOURCE_DIR} --outdir ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building multiverse_parser package"
    VERBATIM
)

add_custom_target(multiverse_parser_install ALL
    DEPENDS multiverse_parser_build
    COMMAND python3 -m pip install ${CMAKE_CURRENT_BINARY_DIR}/multiverse_parser-0.0.1.tar.gz
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

###############
## apartment ##
###############

## With physics

add_test(
  NAME usd_to_usd1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_usd/with_physics/apartment.usda
          --physics
)

add_test(
  NAME usd_to_urdf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_urdf/with_physics/apartment.urdf
          --physics
)

add_test(
  NAME usd_to_mjcf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_mjcf/with_physics/apartment.xml
          --physics
)

add_test(
  NAME urdf_to_usd1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_usd/with_physics/apartment.usda
          --physics
)

add_test(
  NAME urdf_to_urdf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_urdf/with_physics/apartment.urdf
          --physics
)

add_test(
  NAME urdf_to_mjcf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_mjcf/with_physics/apartment.xml
          --physics
)

add_test(
  NAME mjcf_to_usd1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_usd/with_physics/apartment.usda
          --physics
)

add_test(
  NAME mjcf_to_urdf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_urdf/with_physics/apartment.urdf
          --physics
)


add_test(
  NAME mjcf_to_mjcf1
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_mjcf/with_physics/apartment.xml
          --physics
)
## No physics

add_test(
  NAME usd_to_usd2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_usd/no_physics/apartment.usda
          --no-physics
)

add_test(
  NAME usd_to_urdf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_urdf/no_physics/apartment.urdf
          --no-physics
)

add_test(
  NAME usd_to_mjcf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/usd/apartment.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_mjcf/no_physics/apartment.xml
          --no-physics
)

add_test(
  NAME urdf_to_usd2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_usd/no_physics/apartment.usda
          --no-physics
)

add_test(
  NAME urdf_to_urdf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_urdf/no_physics/apartment.urdf
          --no-physics
)

add_test(
  NAME urdf_to_mjcf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/urdf/apartment.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_mjcf/no_physics/apartment.xml
          --no-physics
)

add_test(
  NAME mjcf_to_usd2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_usd/no_physics/apartment.usda
          --no-physics
)

add_test(
  NAME mjcf_to_urdf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_urdf/no_physics/apartment.urdf
          --no-physics
)

add_test(
  NAME mjcf_to_mjcf2
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/apartment/mjcf/apartment.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_mjcf/no_physics/apartment.xml
          --no-physics
)

##############
## milk_box ##
##############

add_test(
  NAME usd_to_usd3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/usd/milk_box.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_usd/with_physics/milk_box.usda
          --physics
)

add_test(
  NAME usd_to_urdf3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/usd/milk_box.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_urdf/with_physics/milk_box.urdf
          --physics
)

add_test(
  NAME usd_to_mjcf3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/usd/milk_box.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_mjcf/with_physics/milk_box.xml
          --physics
)

add_test(
  NAME urdf_to_usd3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/urdf/milk_box.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_usd/with_physics/milk_box.usda
          --physics
)

add_test(
  NAME urdf_to_urdf3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/urdf/milk_box.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_urdf/with_physics/milk_box.urdf
          --physics
)

add_test(
  NAME urdf_to_mjcf3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/urdf/milk_box.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_mjcf/with_physics/milk_box.xml
          --physics
)

add_test(
  NAME mjcf_to_usd3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/mjcf/milk_box.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_usd/with_physics/milk_box.usda
          --physics
)

add_test(
  NAME mjcf_to_urdf3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/mjcf/milk_box.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_urdf/with_physics/milk_box.urdf
          --physics
)

add_test(
  NAME mjcf_to_mjcf3
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/milk_box/mjcf/milk_box.xml
          --output=${TEST_DIR}/multiverse_parser/output/from_mjcf/to_mjcf/with_physics/milk_box.xml
          --physics
)

###################
## subsea_system ##
###################

add_test(
  NAME usd_to_usd4
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/subsea_system/usd/subsea_system.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_usd/with_physics/subsea_system.usda
          --physics
)

add_test(
  NAME usd_to_urdf4
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/subsea_system/usd/subsea_system.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_urdf/with_physics/subsea_system.urdf
          --physics
)

add_test(
  NAME usd_to_mjcf4
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/subsea_system/usd/subsea_system.usda
          --output=${TEST_DIR}/multiverse_parser/output/from_usd/to_mjcf/with_physics/subsea_system.xml
          --physics
)

################
## tiago_dual ##
################

add_test(
  NAME urdf_to_usd4
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/tiago_dual/urdf/tiago_dual.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_usd/with_physics/tiago_dual.usda
          --physics
)

add_test(
  NAME urdf_to_urdf4
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/tiago_dual/urdf/tiago_dual.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_urdf/with_physics/tiago_dual.urdf
          --physics --no-collision
)

add_test(
  NAME urdf_to_mjcf4
  COMMAND multiverse_parser
          --input=${TEST_DIR}/multiverse_parser/input/tiago_dual/urdf/tiago_dual.urdf
          --output=${TEST_DIR}/multiverse_parser/output/from_urdf/to_mjcf/with_physics/tiago_dual.xml
          --physics --no-visual
)