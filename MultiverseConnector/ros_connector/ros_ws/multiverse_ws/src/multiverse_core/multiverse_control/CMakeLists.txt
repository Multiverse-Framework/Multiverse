cmake_minimum_required(VERSION 2.8...3.14)
project(multiverse_control)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	controller_manager
	urdf
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

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
  	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/ext
)
link_directories($ENV{ROS_ROOT}/../../lib/)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
add_library(multiverse_hw_interface
	${PROJECT_SOURCE_DIR}/src/multiverse_control/multiverse_hw_interface.cpp
)
target_link_libraries(multiverse_hw_interface
	urdf
)
target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	multiverse_hw_interface
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libmultiverse_client_json.a
  	${CMAKE_CURRENT_SOURCE_DIR}/lib/libmultiverse_client.a
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libjsoncpp.a
  	${CMAKE_CURRENT_SOURCE_DIR}/lib/libzmq.a
	pthread
)