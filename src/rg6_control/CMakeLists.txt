cmake_minimum_required(VERSION 3.20)
project(rg6_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(rg6_joint_state_broadcaster src/rg6_joint_state_broadcaster.cpp)
add_executable(rg6_joint_state_broadcaster_sim src/rg6_joint_state_broadcaster_sim.cpp)
add_executable(rg6_control src/rg6_control.cpp)
add_executable(rg6_control_sim src/rg6_control_sim.cpp)
add_executable(joint_state_aggregator src/joint_state_aggregator.cpp)

# Link libraries
target_link_libraries(rg6_joint_state_broadcaster PUBLIC
      ${sensor_msgs_TARGETS}
      ${ur_msgs_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
)


target_link_libraries(rg6_control
  rclcpp::rclcpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  ${ur_msgs_TARGETS}
)

target_link_libraries(rg6_control_sim
  rclcpp::rclcpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  ${ur_msgs_TARGETS}
)

target_link_libraries(rg6_joint_state_broadcaster_sim
  ${sensor_msgs_TARGETS}
  ${ur_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

target_link_libraries(joint_state_aggregator PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS rg6_joint_state_broadcaster rg6_control rg6_control_sim rg6_joint_state_broadcaster_sim joint_state_aggregator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
