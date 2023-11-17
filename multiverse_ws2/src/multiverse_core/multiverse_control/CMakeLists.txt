cmake_minimum_required(VERSION 3.5)
project(multiverse_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

set(MULTIVERSE_DIR ${PROJECT_SOURCE_DIR}/../../../../multiverse)

add_library(multiverse_system_hardware SHARED
  src/multiverse_control/multiverse_system_hardware.cpp
)
target_include_directories(multiverse_system_hardware PRIVATE
  ${MULTIVERSE_DIR}/src/multiverse_client/include
  include
)
ament_target_dependencies(multiverse_system_hardware
  hardware_interface
  pluginlib
  rclcpp
)

install(
  TARGETS multiverse_system_hardware
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

add_executable(multiverse_control_node src/multiverse_control_node.cpp)
target_include_directories(multiverse_control_node PRIVATE
  ${MULTIVERSE_DIR}/src/multiverse_client/include
  include
)
target_link_libraries(multiverse_control_node 
  multiverse_system_hardware
  jsoncpp
  zmq
  ${MULTIVERSE_DIR}/lib/libstdc++/libmultiverse_client_json.so
)
ament_target_dependencies(multiverse_control_node
  controller_manager
  hardware_interface
  rclcpp
)
install(
  TARGETS multiverse_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
