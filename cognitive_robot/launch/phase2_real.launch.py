"""
phase2_real.launch.py

Phase 2 — Real Robot: Autonomous navigation to stations.

What this starts (all on the laptop):
  - Nav2 stack          : map server, AMCL localisation, planner, controller
  - RViz                : set the 2D pose estimate here before the mission starts
  - CV perception nodes : detect_abacus, detect_station, read_time
  - station_demo        : autonomous mission (Station A → read clock → Station B)

Note: the SLAM&NAV_README.md originally suggested running Nav2 on the robot
([ROBOT] terminal). We run it here on the laptop instead — Nav2 nodes
communicate with the robot over ROS_DOMAIN_ID=4, the same way SLAM does in
Phase 1. If this causes issues, fall back to: ssh into the robot and run
  ros2 launch mirte_navigation minimal_navigation_launch.py

Before running:
  1. Phase 1 must be complete — station_a_location.yaml and
     station_b_location.yaml must exist in:
       ~/mirte_ws/src/cognitive-robot/maps/
  2. Connect laptop WiFi to Mirte-XXXXXX  (password: mirte_mirte)
  3. Place the robot somewhere on the saved map.
  4. Build and source:
       cd ~/mirte_ws && colcon build && source install/setup.bash

Run with:
    ros2 launch cognitive_robot phase2_real.launch.py

After launch:
  - In RViz click "2D Pose Estimate", click where the robot is on the map,
    and drag in the direction it is facing.
  - The laser scan should align with the map walls.
  - station_demo will then drive to Station A, read the clock, and go to Station B.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MAPS_DIR    = os.path.expanduser('~/mirte_ws/src/cognitive-robot/maps')
RVIZ_CONFIG = os.path.expanduser('~/mirte_ws/src/cognitive-robot/config/mirte_slam.rviz')


def generate_launch_description():
    nav_share = get_package_share_directory('mirte_navigation')

    return LaunchDescription([

        SetEnvironmentVariable('ROS_DOMAIN_ID', '4'),

        # Nav2 stack — map server, AMCL, planner, controller, BT navigator
        # Uses ~/mirte_ws/src/mirte_navigation/params/minimal_nav2_params.yaml
        # (map path already configured in that file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'minimal_navigation_launch.py')
            ),
        ),

        # RViz — needed to set the 2D pose estimate before the mission starts
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

        # Autonomous mission — drive to Station A, read clock, drive to Station B
        Node(
            package='cognitive_robot',
            executable='station_demo',
            name='station_clock_mission',
            output='screen',
            parameters=[{
                'station_dir': MAPS_DIR,
            }],
        ),

    ])
