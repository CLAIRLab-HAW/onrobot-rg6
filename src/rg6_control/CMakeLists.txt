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
# Die Getriebetabelle (Gelenkwinkel <-> Weite) liegt als generierter Header in
# include/ -- erzeugt aus dem URDF des Greifermodells, damit der Sim nicht
# stillschweigend anders rechnet als der Roboter.
target_include_directories(rg6_control_sim PRIVATE include)


# Sim-Node: dieselbe ROS-Oberflaeche, die der Greifer am Roboter hat --
# GripperCommand-Action, Treibergelenk auf joint_states und rg6/bridge_state --
# nur ohne Hardware.  ur_msgs ist mit dem Realtreiber weggefallen.
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
install(PROGRAMS scripts/rg6_moveit_patch
  DESTINATION lib/${PROJECT_NAME}
)

# BUILD_TESTING: hier steht nichts mehr.  Der einzige Test war
# test_analog_model -- die Kennlinie Greifweite <-> Tool-AI2-Spannung, ein
# Kanal, den es seit dem 2026-08-19 nicht mehr gibt (der RG6 haengt an der
# OnRobot-URCap, nicht am Tool-Anschluss).  Er ist mit dem Modell entfallen.
# Was der Greifer im Container tut, prueft jetzt eine E2E gegen den laufenden
# Stack: plan-bridge/tests/test_offboard_gripper.py.
# Bewusst KEIN ament_lint_auto: die bestehenden Quellen sind nie gelintet
# worden, das nachtraeglich zu erzwingen wuerde den Build fuer alle brechen.

ament_package()
