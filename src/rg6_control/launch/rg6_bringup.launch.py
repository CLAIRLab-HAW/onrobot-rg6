#!/usr/bin/env python3
"""Bringup fuer den OnRobot RG6 auf dem Clearpath-UR5 (a200-0553).

Startet:
  1. rg6_control (Vollausbau-Treiber) im manipulators-Namespace:
     Services rg6_control/{open,close,open_gripper,close_gripper,grip,
     set_force_preset,set_tool_power}, Topic rg6/state und die
     GripperCommand-Action rg6_gripper_controller/gripper_cmd (MoveIt).
  2. rg6_joint_state_broadcaster -> mappt tool_data.analog_input2 auf das
     Gelenk 'rg6_finger_joint' (+5 Folgegelenke) und publiziert joint_states.
  3. optional urscript_interface (ur_robot_driver): nimmt URScript entgegen und
     schickt es an den Controller (Primary Interface :30001). Wird vom
     'grip'-Backend 'urscript' (Ziel-Weite/-Kraft, 17-Bit-Protokoll) benoetigt.
     ACHTUNG: Injektion ersetzt das ExternalControl-Programm; rg6_control ruft
     danach automatisch resend_robot_program.

Der 'io_and_status_controller' (set_io/tool_data/io_states) wird hier NICHT
gespawnt: er kommt ueber robot.yaml ros_parameters (Clearpath spawnt jeden
Top-Level-Key der control.yaml mit 'controller' im Namen aktiv). rg6_control
pollt beim Start auf dessen set_io-Service (Tool-Spannung 24 V).

Gedacht zum Einbinden ueber 'platform.extras.launch' bzw. rg6-bringup.service.
Voraussetzung: Workspace in robot.yaml unter 'system.ros2.workspaces'.

Argumente:
  js_topic   joint_states-Topic der Greifergelenke (Phase 2:
             /a200_0553/manipulators/endeffectors/joint_states; das Relay aus
             joint_states.launch.py spiegelt es auf platform/joint_states fuer
             RSP + move_group).
  params_file            Parameter-YAML fuer rg6_control (Kalibrierung).
  start_urscript_interface  'true': urscript_interface-Node mitstarten.
  robot_ip               UR-Controller-IP fuer urscript_interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Robotspezifisch (a200-0553):
NAMESPACE = "/a200_0553/manipulators"


def generate_launch_description():
    js_topic = LaunchConfiguration("js_topic")
    params_file = LaunchConfiguration("params_file")
    start_urscript = LaunchConfiguration("start_urscript_interface")
    robot_ip = LaunchConfiguration("robot_ip")

    declare_js_topic = DeclareLaunchArgument(
        "js_topic",
        default_value="/a200_0553/manipulators/endeffectors/joint_states",
        description="joint_states-Topic fuer die Greifer-Gelenke (siehe Docstring).",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rg6_control"), "config", "rg6_params.yaml"]),
        description="Parameter-YAML fuer rg6_control (Kalibrierwerte).",
    )
    declare_start_urscript = DeclareLaunchArgument(
        "start_urscript_interface",
        default_value="true",
        description="urscript_interface-Node mitstarten (fuer grip_backend=urscript).",
    )
    declare_robot_ip = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.131.40",
        description="IP des UR-Controllers (Primary Interface fuer urscript_interface).",
    )

    # 1) RG6-Treiber im manipulators-Namespace (relative Namen loesen dort auf).
    rg6_control = Node(
        package="rg6_control",
        executable="rg6_control",
        namespace=NAMESPACE,
        output="screen",
        parameters=[params_file],
    )

    # 2) tool_data -> joint_states (rg6_finger_joint + Folgegelenke).
    rg6_jsb = Node(
        package="rg6_control",
        executable="rg6_joint_state_broadcaster",
        namespace=NAMESPACE,
        output="screen",
        remappings=[("joint_states", js_topic)],
    )

    # 3) URScript-Injektionskanal (fuer Ziel-Weite/-Kraft).
    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace=NAMESPACE,
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
        condition=IfCondition(start_urscript),
    )

    return LaunchDescription([
        declare_js_topic,
        declare_params_file,
        declare_start_urscript,
        declare_robot_ip,
        rg6_control,
        rg6_jsb,
        urscript_interface,
    ])
