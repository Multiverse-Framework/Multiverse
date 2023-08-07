cmake_minimum_required(VERSION 2.8...3.14)
project(multiverse_knowledge)

add_compile_options(-std=c++17)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

# ##########
# # Build ##
# ##########
if(IS_DIRECTORY ${CATKIN_DEVEL_PREFIX}/../../.private)
  set(ENV{CATKIN_DEVEL_DIR} ${CATKIN_DEVEL_PREFIX}/../../../devel)
  set(ENV{USD_SRC_DIR} $ENV{CATKIN_DEVEL_DIR}/../../src/USD)
  set(ENV{USD_BUILD_DIR} $ENV{CATKIN_DEVEL_DIR}/../../build/USD)
endif()

# Execute the shell file during CMake configuration
execute_process(
  COMMAND sh ${PROJECT_SOURCE_DIR}/install.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE INSTALL_RESULT
)

# Check the result of the script execution
if(INSTALL_RESULT EQUAL 0)
  message(VERBOSE "install.sh executed successfully.")
else()
  message(FATAL_ERROR "Failed to execute install.sh.")
endif()

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)