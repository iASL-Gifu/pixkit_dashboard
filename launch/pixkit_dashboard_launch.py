import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pixkit_dashboard',
            executable='pixkit_dashboard',
            name='pixkit_dashboard',
            output='screen',
            emulate_tty=True,
        ),
    ])