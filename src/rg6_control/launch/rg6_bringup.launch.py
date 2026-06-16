#!/usr/bin/env python3
"""Bringup fuer den OnRobot RG6 auf dem Clearpath-UR5.

Startet:
  1. den UR 'io_and_status_controller' (GPIOController) auf dem manipulators-
     controller_manager -> liefert set_io + tool_data (das spawnt Clearpath nicht),
  2. den rg6_control-Node (open_gripper/close_gripper) im manipulators-Namespace.

Gedacht zum Einbinden ueber 'platform.extras.launch' in der robot.yaml.
Voraussetzung: der Workspace mit rg6_control ist in robot.yaml unter
'system.ros2.workspaces' eingetragen (sonst wird das Package nicht gefunden).
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# Robotspezifisch (a200-0553):
NAMESPACE = "/a200_0553/manipulators"
CONTROLLER_MANAGER = "/a200_0553/manipulators/controller_manager"


def generate_launch_description():
    # 1) io_and_status_controller laden+aktivieren (wartet auf den controller_manager).
    #    Der Controller-TYP steht in der control.yaml (per clearpath-custom-setup
    #    gepatcht) -> hier reicht der Name.
    io_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "io_and_status_controller",
            "-c", CONTROLLER_MANAGER,
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    # 2) RG6-Treiber im manipulators-Namespace (relative Namen loesen dann korrekt auf)
    rg6_control = Node(
        package="rg6_control",
        executable="rg6_control",
        namespace=NAMESPACE,
        output="screen",
    )

    return LaunchDescription([io_spawner, rg6_control])
