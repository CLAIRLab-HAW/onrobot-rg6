cmake_minimum_required(VERSION 3.20)
project(rg6_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rg6_msgs REQUIRED)

add_executable(rg6_joint_state_broadcaster src/rg6_joint_state_broadcaster.cpp)
add_executable(rg6_control src/rg6_control.cpp)
add_executable(rg6_control_sim src/rg6_control_sim.cpp)
add_executable(joint_state_aggregator src/joint_state_aggregator.cpp)
add_executable(joint_state_relay src/joint_state_relay.cpp)

# Link libraries
target_link_libraries(rg6_joint_state_broadcaster PUBLIC
      ${sensor_msgs_TARGETS}
      ${ur_msgs_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
)


target_link_libraries(rg6_control
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  std_srvs::std_srvs__rosidl_typesupport_cpp
  ${std_msgs_TARGETS}
  ${ur_msgs_TARGETS}
  ${control_msgs_TARGETS}
  ${rg6_msgs_TARGETS}
)

# Sim-Node: gleiche ROS-Schnittstelle wie rg6_control, aber ohne UR-Hardware
# (publiziert joint_states selbst; kein ur_msgs noetig).
target_link_libraries(rg6_control_sim
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  std_srvs::std_srvs__rosidl_typesupport_cpp
  ${sensor_msgs_TARGETS}
  ${control_msgs_TARGETS}
  ${rg6_msgs_TARGETS}
)

target_link_libraries(joint_state_aggregator PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

target_link_libraries(joint_state_relay PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS rg6_joint_state_broadcaster rg6_control rg6_control_sim joint_state_aggregator joint_state_relay
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/rg6_moveit_patch
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
