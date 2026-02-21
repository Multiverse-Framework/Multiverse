cmake_minimum_required(VERSION 3.8)
project(vr_teleop_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(vr_teleop_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(action_server SHARED
  src/vr_teleop_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server PRIVATE 
  "VR_TELEOP_ACTION_BUILDING_DLL")
target_link_directories(action_server PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/lib)
target_link_libraries(action_server
  multiverse_client_json_all
  multiverse_client_all
  jsoncpp
  zmq
)
ament_target_dependencies(action_server
  "vr_teleop_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "trajectory_msgs"
  "tf2_ros")
rclcpp_components_register_node(action_server PLUGIN "vr_teleop_action::VrTeleopActionServer" EXECUTABLE vr_teleop_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
