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
find_package(control_msgs REQUIRED)

add_executable(rg6_control_sim src/rg6_control_sim.cpp)
add_executable(joint_state_aggregator src/joint_state_aggregator.cpp)
add_executable(joint_state_relay src/joint_state_relay.cpp)

# Link libraries
# The linkage table (joint angle <-> width) lives as a generated header in
# include/ -- produced from the URDF of the gripper model, so that the sim does
# not silently compute differently from the robot.
target_include_directories(rg6_control_sim PRIVATE include)


# Sim node: the same ROS surface the gripper has on the robot --
# GripperCommand action, driver joint on joint_states and rg6/bridge_state --
# only without hardware.  ur_msgs went away together with the real driver.
target_link_libraries(rg6_control_sim
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${control_msgs_TARGETS}
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


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS rg6_control_sim joint_state_aggregator joint_state_relay
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/rg6_moveit_patch scripts/rg6_grip_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
# The linkage table goes NEXT TO the bridge, not under share/: rg6_grip_bridge resolves it with
# Path(__file__).with_name(), so the two files have to stay adjacent in the install space as well -- and they do
# in the root-owned copy the robot's installer places under /usr/local/bin.  The parameter kinematics_file
# overrides it; the default is what the deployed pair relies on.
install(FILES scripts/rg6_finger_kinematics.json
  DESTINATION lib/${PROJECT_NAME}
)

# BUILD_TESTING: nothing stands here, and that is not the same as untested.
# What rg6_control_sim DOES is checked by an E2E against the running stack
# (plan-bridge/tests/test_offboard_gripper.py); the AI2 characteristic curve
# that a unit test could once have covered went away with the analogue model,
# when the RG6 moved onto the OnRobot URCap.
#
# What is left to check is the generated linkage table in
# include/rg6_control/finger_kinematics.hpp -- it exists three times over
# (here, the bridge's JSON on the robot, the robot_contract profile) and is
# interpolated three times.  That is pinned down by ../../tests/, plain pytest
# from the workspace root: it compiles this header and compares it against both
# Python copies, so it needs neither colcon nor a robot and does not belong in
# a BUILD_TESTING block.
#
# Deliberately NO ament_lint_auto: the existing sources have never been linted,
# and enforcing it after the fact would break the build for everyone.

ament_package()
