#!/usr/bin/env python3
"""Robot-weite joint_states-Aggregation + Legacy-Bus-Relays fuer a200_0553 (Phase 2).

Hintergrund (Multi-CM-Clearpath): Raeder-, Arm- und Greifer-joint_states kommen aus
DREI getrennten Quellen und werden nicht automatisch zu einem vollstaendigen
/joint_states zusammengefuehrt. Dieses Launch stellt her:

  1. joint_state_aggregator: fasst die (partiellen) Quellen zu EINEM vollstaendigen
     /a200_0553/joint_states zusammen (mit velocity+effort) -> fuer rosbag/Foxglove.
     Bewusst NICHT als Live-TF-Feed (Aggregator im TF-Pfad = SPOF).

  2. relay_arm / relay_endeffector: spiegeln die sauberen Quell-Topics
     manipulators/joint_states bzw. manipulators/endeffectors/joint_states zurueck
     auf /a200_0553/platform/joint_states, den Clearpaths robot_state_publisher (x2)
     und move_group per Stock-Launch abonnieren. So bleiben die Live-Consumer
     (TF/MoveIt) UNANGETASTET, waehrend Arm & Greifer im korrekten Namespace
     publizieren.

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
PLATFORM_BUS = "/a200_0553/platform/joint_states"
ARM_TOPIC = "/a200_0553/manipulators/joint_states"
EE_TOPIC = "/a200_0553/manipulators/endeffectors/joint_states"


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
                "platform/joint_states",                  # Raeder
                "manipulators/joint_states",              # Arm
                "manipulators/endeffectors/joint_states",  # Greifer
            ],
            "output_topic": "joint_states",  # -> /a200_0553/joint_states
            "publish_rate": 50.0,
        }],
    )

    # Legacy-Bus am Leben halten: Arm + Greifer zurueck auf platform/joint_states,
    # damit die Stock-Consumer (RSP x2, move_group) unveraendert weiterlaufen.
    # topic_tools relay: Per-Message-Forwarder; die Subscription reconnectet
    # automatisch, wenn clearpath-manipulators (und damit der Arm-JSB) neu startet.
    relay_arm = Node(
        package="topic_tools",
        executable="relay",
        name="relay_arm_joint_states",
        output="screen",
        parameters=[{"input_topic": ARM_TOPIC, "output_topic": PLATFORM_BUS}],
    )
    relay_endeffector = Node(
        package="topic_tools",
        executable="relay",
        name="relay_endeffector_joint_states",
        output="screen",
        parameters=[{"input_topic": EE_TOPIC, "output_topic": PLATFORM_BUS}],
    )

    return LaunchDescription([aggregator, relay_arm, relay_endeffector])
