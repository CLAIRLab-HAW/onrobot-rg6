#!/usr/bin/env python3
"""Generisches Launch fuer rg6_control (real oder Simulation).

Real (Default): rg6_control + rg6_joint_state_broadcaster. In den Namespace des
UR-io_and_status_controller launchen, z. B.:
    ros2 launch rg6_control rg6_control.launch.py \
        --ros-args ... (oder Node-Namespace via __ns setzen)

Simulation (gripper_sim:=true): rg6_control_sim - gleiche ROS-Schnittstelle
(Services, grip, GripperCommand-Action, rg6/state) ohne Hardware; publiziert
die 6 Greifergelenke selbst auf 'joint_states' (per js_topic remapbar).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper_sim = LaunchConfiguration('gripper_sim')
    params_file = LaunchConfiguration('params_file')
    js_topic = LaunchConfiguration('js_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gripper_sim',
            default_value='false',
            description='use simulated gripper'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('rg6_control'), 'config', 'rg6_params.yaml']),
            description='Parameter-YAML fuer rg6_control'
        ),
        DeclareLaunchArgument(
            'js_topic',
            default_value='joint_states',
            description='joint_states-Topic der Greifergelenke'
        ),
        Node(
            package='rg6_control',
            executable='rg6_joint_state_broadcaster',
            name='rg6_joint_state_broadcaster',
            output='screen',
            remappings=[('joint_states', js_topic)],
            condition=UnlessCondition(gripper_sim),
        ),
        Node(
            package='rg6_control',
            executable='rg6_control',
            output='screen',
            parameters=[params_file],
            condition=UnlessCondition(gripper_sim),
        ),
        Node(
            package='rg6_control',
            executable='rg6_control_sim',
            output='screen',
            remappings=[('joint_states', js_topic)],
            condition=IfCondition(gripper_sim),
        ),
    ])
