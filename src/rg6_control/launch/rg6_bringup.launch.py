#!/usr/bin/env python3
"""Bringup fuer den OnRobot RG6 auf dem Clearpath-UR5.

Startet:
  1. den rg6_control-Node (open/close) im manipulators-Namespace,
  2. den rg6_joint_state_broadcaster -> mappt tool_data.analog_input2 auf das
     Gelenk 'rg6-l_out_joint' und publiziert es als joint_states -> Greifer
     animiert im RViz/Foxglove (rg6_*-Gelenke sind revolute+mimic).

Der 'io_and_status_controller' (GPIOController, liefert set_io + tool_data) wird
hier NICHT gespawnt: clearpath_manipulators/control.launch.py spawnt jeden
Top-Level-Key der control.yaml, dessen Name 'controller' enthaelt (Schleife). Der
clearpath-custom-setup-Patcher injiziert genau so einen Top-Level-Block fuer
'io_and_status_controller' -> Clearpath spawnt ihn selbst (aktiv). rg6_control
wartet beim Start aktiv auf dessen set_io-Service.

Gedacht zum Einbinden ueber 'platform.extras.launch' in der robot.yaml.
Voraussetzung: der Workspace mit rg6_control ist in robot.yaml unter
'system.ros2.workspaces' eingetragen (sonst wird das Package nicht gefunden).

Argument 'js_topic' (default '/a200_0553/platform/joint_states'): Topic, auf dem
das Greifer-Gelenk publiziert wird. Es MUSS das joint_states-Topic sein, das der
Greifer-rendernde robot_state_publisher (der /a200_0553/robot_description
ausliefert) konsumiert. Auf a200-0553 ist das /a200_0553/platform/joint_states
(dort liegen auch die Arm-Gelenke). Per Argument ueberschreibbar.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Robotspezifisch (a200-0553):
NAMESPACE = "/a200_0553/manipulators"


def generate_launch_description():
    js_topic = LaunchConfiguration("js_topic")

    declare_js_topic = DeclareLaunchArgument(
        "js_topic",
        # Der /a200_0553/robot_state_publisher (rendert /a200_0553/robot_description
        # inkl. Greifer) abonniert /a200_0553/platform/joint_states -> dort liegen
        # auch schon die Arm-Gelenke. Das Greifer-Gelenk MUSS auf dasselbe Topic.
        default_value="/a200_0553/platform/joint_states",
        description="joint_states-Topic fuer das Greifer-Gelenk (siehe Docstring).",
    )

    # 1) RG6-Treiber im manipulators-Namespace (relative Namen loesen dann korrekt auf)
    rg6_control = Node(
        package="rg6_control",
        executable="rg6_control",
        namespace=NAMESPACE,
        output="screen",
    )

    # 2) tool_data -> joint_states (Gelenk 'rg6-l_out_joint') fuer die Animation.
    #    Relativer 'joint_states'-Output wird per Remap auf js_topic gelegt.
    rg6_jsb = Node(
        package="rg6_control",
        executable="rg6_joint_state_broadcaster",
        namespace=NAMESPACE,
        output="screen",
        remappings=[("joint_states", js_topic)],
    )

    return LaunchDescription([declare_js_topic, rg6_control, rg6_jsb])
