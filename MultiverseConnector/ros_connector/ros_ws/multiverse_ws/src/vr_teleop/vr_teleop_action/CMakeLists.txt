cmake_minimum_required(VERSION 2.8...3.14)
project(vr_teleop_action)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	genmsg
	vr_teleop_msgs
	actionlib
	std_msgs
	sensor_msgs
	tf2_ros
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

add_executable(${PROJECT_NAME}_server_node src/${PROJECT_NAME}_server_node.cpp)
add_dependencies(${PROJECT_NAME}_server_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server_node
	${catkin_LIBRARIES}
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libmultiverse_client_json_all.a
  	${CMAKE_CURRENT_SOURCE_DIR}/lib/libmultiverse_client_all.a
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libjsoncpp.a
  	${CMAKE_CURRENT_SOURCE_DIR}/lib/libzmq.a
	pthread
)