#!/usr/bin/env python3
"""Robot-weite joint_states-Aggregation + Legacy-Bus-Relay fuer a200_0553 (Phase 2).

Hintergrund (Multi-CM-Clearpath): Raeder-, Arm- und Greifer-joint_states kommen aus
DREI getrennten Quellen und werden nicht automatisch zu einem vollstaendigen
/joint_states zusammengefuehrt. Dieses Launch stellt her:

  1. joint_state_aggregator: fasst die (partiellen) Quellen zu EINEM vollstaendigen
     /a200_0553/joint_states zusammen (mit velocity+effort) -> fuer rosbag/Foxglove.
     Bewusst NICHT als Live-TF-Feed (Aggregator im TF-Pfad = SPOF).

  2. joint_state_relay: spiegelt die sauberen Quell-Topics manipulators/joint_states
     und manipulators/endeffectors/joint_states zurueck auf
     /a200_0553/platform/joint_states, den Clearpaths robot_state_publisher (x2) und
     move_group per Stock-Launch abonnieren. So bleiben die Live-Consumer (TF/MoveIt)
     UNANGETASTET, waehrend Arm & Greifer im korrekten Namespace publizieren.

     WICHTIG: eigener Relay-Node (NICHT topic_tools relay), weil dieser mit EXPLIZIT
     RELIABLE Publisher-QoS publiziert. move_group abonniert platform/joint_states
     RELIABLE -> ein best-effort-Publisher (topic_tools-Default) wuerde dort NICHT
     ankommen -> Zustand zwar korrekt angezeigt (best-effort RSP), aber Planning
     schlaegt fehl. Siehe joint_state_relay.cpp.

Voraussetzung Phase 2:
  - Arm-JSB-Remap in clearpath_manipulators/control.launch.py ist per
    clearpath-custom-setup auf manipulators/joint_states umgestellt.
  - Greifer publiziert auf manipulators/endeffectors/joint_states
    (rg6_bringup.launch.py, Argument js_topic).
Die Raeder bleiben (korrekt) auf platform/joint_states.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

NAMESPACE = "a200_0553"


def generate_launch_description():
    # Aggregat fuer Beobachtung/Recording: /a200_0553/joint_states (vollstaendig).
    aggregator = Node(
        package="rg6_control",
        executable="joint_state_aggregator",
        namespace=NAMESPACE,
        name="joint_state_aggregator",
        output="screen",
        parameters=[{
            # relative Namen -> im /a200_0553-Namespace aufgeloest.
            "source_topics": [
                "platform/joint_states",                   # Raeder
                "manipulators/joint_states",               # Arm
                "manipulators/endeffectors/joint_states",  # Greifer
            ],
            "output_topic": "joint_states",  # -> /a200_0553/joint_states
            "publish_rate": 50.0,
        }],
    )

    # Legacy-Bus am Leben halten: Arm + Greifer zurueck auf platform/joint_states,
    # RELIABLE (fuer den reliable move_group-Subscriber; siehe joint_state_relay.cpp).
    # Die Subscriptions reconnecten automatisch, wenn clearpath-manipulators (und
    # damit der Arm-JSB) neu startet.
    relay = Node(
        package="rg6_control",
        executable="joint_state_relay",
        namespace=NAMESPACE,
        name="joint_state_relay",
        output="screen",
        parameters=[{
            "input_topics": [
                "manipulators/joint_states",
                "manipulators/endeffectors/joint_states",
            ],
            "output_topic": "platform/joint_states",  # -> /a200_0553/platform/joint_states
            "depth": 20,
        }],
    )

    return LaunchDescription([aggregator, relay])
