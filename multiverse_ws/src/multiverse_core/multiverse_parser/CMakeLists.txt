cmake_minimum_required(VERSION 2.8...3.14)
project(multiverse_parser)

add_compile_options(-std=c++17)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  mujoco_sim
)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
set(MULTIVERSE_DIR ${CATKIN_DEVEL_PREFIX}/../../../../multiverse)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${MULTIVERSE_DIR}/external/mujoco/include
)

set(URDF_TO_MJCF_NODE urdf_to_mjcf_node)
add_executable(${URDF_TO_MJCF_NODE} urdf_to_mjcf/src/urdf_to_mjcf_node.cpp)
target_link_libraries(${URDF_TO_MJCF_NODE}
  ${catkin_LIBRARIES}
  $ENV{ROS_ROOT}/../../lib/liburdf.so
  tinyxml2
  ${MULTIVERSE_DIR}/build/mujoco/lib/libmujoco.so
)

# ############
# # Install ##
# ############

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  urdf_to_fbx/scripts/urdf_to_fbx_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)