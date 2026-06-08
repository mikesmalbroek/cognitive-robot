"""
demo_gazebo.launch.py

Launches all cognitive robot perception services for the GAZEBO simulation.

Camera topics in Gazebo differ from the real robot:
  camera_topic      : /camera/image_raw      (vs /camera/color/image_raw)
  camera_info_topic : /camera/camera_info    (vs /camera/color/camera_info)
  depth_topic       : /camera/depth/image_raw (same as real robot)
  cmd_vel_topic     : /mirte_base_controller/cmd_vel_unstamped
                      (vs /mirte_base_controller/cmd_vel)

Run with:
    ros2 launch cognitive_robot demo_gazebo.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

CAMERA_TOPIC      = '/camera/image_raw'
DEPTH_TOPIC       = '/camera/depth/image_raw'
CAMERA_INFO_TOPIC = '/camera/camera_info'
CMD_VEL_TOPIC     = '/mirte_base_controller/cmd_vel_unstamped'


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='cognitive_robot',
            executable='detect_abacus_service',
            name='detect_abacus_service',
            output='screen',
            parameters=[{
                'camera_topic':      CAMERA_TOPIC,
                'depth_topic':       DEPTH_TOPIC,
                'camera_info_topic': CAMERA_INFO_TOPIC,
            }],
        ),

        Node(
            package='cognitive_robot',
            executable='detect_station_service',
            name='detect_station_service',
            output='screen',
            parameters=[{
                'camera_topic':      CAMERA_TOPIC,
                'depth_topic':       DEPTH_TOPIC,
                'camera_info_topic': CAMERA_INFO_TOPIC,
            }],
        ),

        Node(
            package='cognitive_robot',
            executable='read_time_service',
            name='read_time_service',
            output='screen',
            parameters=[{
                'camera_topic': CAMERA_TOPIC,
                'cmd_vel_topic': CMD_VEL_TOPIC,
            }],
        ),

    ])
