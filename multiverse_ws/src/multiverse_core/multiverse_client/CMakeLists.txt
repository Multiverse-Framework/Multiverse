cmake_minimum_required(VERSION 3.0.2)
project(multiverse_client)

add_compile_options(-std=c++17)

# set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fPIC -stdlib=libc++ -lc++abi")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  multiverse_msgs
  roscpp
  rospy
)

set(PYBIND11_PYTHON_VERSION "3.8")
include_directories(/usr/include/python3.8)
set(PYBIND11_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../../../../src/pybind11)
add_subdirectory(${PYBIND11_SOURCE_DIR} build)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

set(pybind11_INCLUDE_DIRS ${PYBIND11_SOURCE_DIR}/include)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_json
  CATKIN_DEPENDS multiverse_msgs roscpp rospy
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/
  ${pybind11_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)

add_library(${PROJECT_NAME}_json
  src/${PROJECT_NAME}/${PROJECT_NAME}_json.cpp
)

# # Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  zmq
  jsoncpp
)

target_link_libraries(${PROJECT_NAME}_json
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/multiverse_socket_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_json
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# # Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

pybind11_add_module(multiverse_socket MODULE src/${PROJECT_NAME}/multiverse_socket.cpp)
set_target_properties(multiverse_socket PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
)
target_link_libraries(multiverse_socket PRIVATE ${PROJECT_NAME})
