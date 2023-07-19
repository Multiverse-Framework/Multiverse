cmake_minimum_required(VERSION 2.8...3.14)
project(multiverse_parser)

add_compile_options(-std=c++17)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  mujoco_sim
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
catkin_package(
  LIBRARIES multiverse_parser
  CATKIN_DEPENDS rospy roscpp
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(URDF_TO_MUJOCO_NODE urdf_to_mjcf)
add_executable(${URDF_TO_MUJOCO_NODE} src/urdf_to_mjcf.cpp)
target_link_libraries(${URDF_TO_MUJOCO_NODE}
	${catkin_LIBRARIES}
	$ENV{ROS_ROOT}/../../lib/liburdf.so
  tinyxml2
)

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/mjcf_to_usd_no_physics.py
  scripts/mjcf_to_usd_with_physics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)