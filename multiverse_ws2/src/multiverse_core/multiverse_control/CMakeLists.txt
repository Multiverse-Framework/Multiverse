cmake_minimum_required(VERSION 3.16)
project(multiverse_control LANGUAGES CXX)

execute_process(
  COMMAND lsb_release -rs
  OUTPUT_VARIABLE UBUNTU_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT ${UBUNTU_VERSION} STREQUAL "24.04")
  message(WARNING "multiverse_control is only supported on Ubuntu 24.04")
  return()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)

# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. ")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# # COMPILE
set(MULTIVERSE_DIR ${CMAKE_SOURCE_DIR}/../../../../multiverse)

include_directories(
  ${MULTIVERSE_DIR}/src/multiverse_client/include
)

add_library(
  multiverse_control
  SHARED
  src/multiverse_control/multiverse_hw_interface.cpp
)
target_link_libraries(
  multiverse_control
  PRIVATE
  ${MULTIVERSE_DIR}/lib/libstdc++/libmultiverse_client_json.so
)

target_compile_features(multiverse_control PUBLIC cxx_std_17)
target_include_directories(multiverse_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/multiverse_control>
)
ament_target_dependencies(
  multiverse_control PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface multiverse_control.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/multiverse_control
)

install(TARGETS multiverse_control
  EXPORT export_multiverse_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

execute_process(
  COMMAND cp ${MULTIVERSE_DIR}/lib/libstdc++/libmultiverse_client_json.so ${MULTIVERSE_DIR}/../multiverse_ws2/install/multiverse_control/lib/libmultiverse_client_json.so
)

# # EXPORTS
ament_export_targets(export_multiverse_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()
