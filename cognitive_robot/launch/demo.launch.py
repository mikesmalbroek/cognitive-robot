"""
demo.launch.py

Launches all cognitive robot perception services for the REAL robot.

Run with:
    ros2 launch cognitive_robot demo.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='cognitive_robot',
            executable='detect_abacus_service',
            name='detect_abacus_service',
            output='screen',
        ),

        Node(
            package='cognitive_robot',
            executable='detect_station_service',
            name='detect_station_service',
            output='screen',
        ),

        Node(
            package='cognitive_robot',
            executable='read_time_service',
            name='read_time_service',
            output='screen',
        ),

        Node(
            package='cognitive_robot',
            executable='abacus_manipulation_node',
            name='abacus_manipulation_node',
            output='screen',
        ),

    ])
