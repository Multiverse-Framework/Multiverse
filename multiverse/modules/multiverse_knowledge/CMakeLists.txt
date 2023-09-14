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

enable_testing()

add_test(
  NAME TBox_to_usd_box
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/TBox_to_usd.py
          --in_owl=${TEST_DIR}/multiverse_knowledge/input/box/owl/box_TBox.owl
          --out_usd=${TEST_DIR}/multiverse_knowledge/output/box/usd/box_TBox.usda
)

add_test(
  NAME TBox_to_usd_box_with_copy
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/TBox_to_usd.py
          --in_owl=${TEST_DIR}/multiverse_knowledge/input/box/owl/box_TBox.owl
          --out_usd=${TEST_DIR}/multiverse_knowledge/output/box/usd/box_TBox.usda
          --out_owl=${TEST_DIR}/multiverse_knowledge/output/box/owl/box_TBox.owl
)

add_test(
  NAME auto_sem_tag_box
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/auto_sem_tag.py
          --in_usd=${TEST_DIR}/multiverse_knowledge/input/box/usd/box.usda
          --in_TBox_usd=${TEST_DIR}/multiverse_knowledge/output/box/usd/box_TBox.usda
          --out_ABox_usd=${TEST_DIR}/multiverse_knowledge/output/box/usd/box_ABox.usda
)

add_test(
  NAME usd_to_ABox_box
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/usd_to_ABox.py
          --in_usd=${TEST_DIR}/multiverse_knowledge/output/box/usd/box_ABox.usda
          --in_owl=${TEST_DIR}/multiverse_knowledge/input/box/owl/box_TBox.owl
          --out_owl=${TEST_DIR}/multiverse_knowledge/output/box/owl/box_ABox.owl
)

add_test(
  NAME TBox_to_usd_apartment
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/TBox_to_usd.py
          --in_owl=${TEST_DIR}/multiverse_knowledge/input/apartment/owl/SOMA-HOME.owl
          --out_usd=${TEST_DIR}/multiverse_knowledge/output/apartment/usd/apartment_TBox.usda
)

add_test(
  NAME auto_sem_tag_apartment
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/auto_sem_tag.py
          --in_usd=${TEST_DIR}/multiverse_knowledge/input/apartment/usd/apartment.usda
          --in_TBox_usd=${TEST_DIR}/multiverse_knowledge/output/apartment/usd/apartment_TBox.usda
          --out_ABox_usd=${TEST_DIR}/multiverse_knowledge/output/apartment/usd/apartment_ABox.usda
)

add_test(
  NAME usd_to_ABox_apartment
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/usd_to_ABox.py
          --in_usd=${TEST_DIR}/multiverse_knowledge/output/apartment/usd/apartment_ABox.usda
          --in_owl=${TEST_DIR}/multiverse_knowledge/input/apartment/owl/SOMA-HOME.owl
          --out_owl=${TEST_DIR}/multiverse_knowledge/output/apartment/owl/apartment_ABox.owl
)