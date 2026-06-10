"""
phase1_real.launch.py

Phase 1 — Real Robot: Manual map generation + station registration.

What this starts (all on the laptop):
  - SLAM Toolbox        : builds the live map from the robot's LiDAR
  - RViz                : visualise the map as you drive
  - CV perception nodes : detect_abacus, detect_station, read_time
  - trial_depth         : keyboard-driven teleop + station/map saver (OpenCV window)

Before running:
  1. Connect laptop WiFi to Mirte-XXXXXX  (password: mirte_mirte)
  2. Verify the robot is publishing topics:
       export ROS_DOMAIN_ID=4
       ros2 topic list   (expect /scan, /mirte_base_controller/cmd_vel, etc.)
  3. Build and source:
       cd ~/mirte_ws && colcon build && source install/setup.bash

Run with:
    ros2 launch cognitive_robot phase1_real.launch.py

Controls (click the OpenCV window first):
  w/s/a/d/q/e  move the robot
  b            detect + register the nearest station
  v            save map and quit
  ESC          quit without saving

Map and station YAML files are saved to:
    ~/mirte_ws/src/cognitive-robot/maps/
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MAPS_DIR   = os.path.expanduser('~/mirte_ws/src/cognitive-robot/maps')
CONFIG_DIR = os.path.expanduser('~/mirte_ws/src/cognitive-robot/config')
SLAM_PARAMS = os.path.join(CONFIG_DIR, 'mapper_params_online_async.yaml')
RVIZ_CONFIG = os.path.join(CONFIG_DIR, 'mirte_slam.rviz')


def generate_launch_description():
    slam_share = get_package_share_directory('slam_toolbox')

    return LaunchDescription([

        SetEnvironmentVariable('ROS_DOMAIN_ID', '4'),

        # SLAM Toolbox — builds the map from the robot's LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': SLAM_PARAMS,
                'use_sim_time':     'false',
            }.items(),
        ),

        # RViz — map visualisation
        ExecuteProcess(
            cmd=['rviz2', '-d', RVIZ_CONFIG],
            output='screen',
        ),

        # CV perception services (real robot topic defaults)
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

        # trial_depth — keyboard teleop + station/map saver (Phase 1 main node)
        Node(
            package='cognitive_robot',
            executable='trial_depth',
            name='trial_depth_mapper',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/mirte_base_controller/cmd_vel',
                'camera_topic':  '/camera/color/image_raw',
                'station_dir':   MAPS_DIR,
                'map_dir':       MAPS_DIR,
                'map_name':      'auto_map',
            }],
        ),

    ])
