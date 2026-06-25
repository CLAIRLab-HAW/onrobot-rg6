from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    gripper_sim = LaunchConfiguration('gripper_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gripper_sim',
            default_value='false',
            description='use simulated gripper'
        ),
        Node(
            package='rg6_control',
            executable='rg6_joint_state_broadcaster',
            name='rg6_joint_state_broadcaster',
            output='screen',
            parameters=[],
            condition=UnlessCondition(gripper_sim),
        ),
        Node(
            package='rg6_control',
            executable='rg6_control',
            name='rg6_control',
            output='screen',
            condition=UnlessCondition(gripper_sim),
            parameters=[],
        ),
        Node(
            package='rg6_control',
            executable='rg6_control_sim',
            name='rg6_sim',
            output='screen',
            parameters=[],
            condition=IfCondition(gripper_sim),
        ),
        Node(
            package='rg6_control',
            executable='rg6_joint_state_broadcaster_sim',
            name='rg6_sim',
            output='screen',
            parameters=[],
            condition=IfCondition(gripper_sim),
        ),

    ])

