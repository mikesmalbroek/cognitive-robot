"""
phase2_real.launch.py

Phase 2 — Real Robot: Autonomous navigation to stations.

What this starts (all on the laptop):
  - Nav2                : AMCL localisation against the saved map + path planning
  - TF relays           : base_link→base_footprint and base_link→base_frame
  - Topic relays        : /cmd_vel→/mirte_base_controller/cmd_vel and
                          /mirte_base_controller/odom→/odom
  - RViz                : set the 2D pose estimate here before the mission starts
  - CV perception nodes : detect_abacus, detect_station, read_time
  - station_demo        : autonomous mission (Station A → read clock → Station B)
                          waits for the 2D pose estimate before driving

Before running:
  1. Phase 1 must be complete — auto_map.yaml, station_a_location.yaml,
     station_b_location.yaml (and optionally abacus_location.yaml) must exist in:
       ~/mirte_ws/src/cognitive-robot/maps/
  2. Connect laptop WiFi to Mirte-XXXXXX  (password: mirte_mirte)
  3. Place the robot somewhere on the saved map.
  4. Build and source:
       cd ~/mirte_ws && colcon build --packages-select cognitive_robot
       source install/setup.bash

Run with:
    ros2 launch cognitive_robot phase2_real.launch.py

To use a different map:
    ros2 launch cognitive_robot phase2_real.launch.py map:=/full/path/to/map.yaml

After launch:
  - In RViz click "2D Pose Estimate", click where the robot is on the map,
    and drag in the direction it is facing.
  - The laser scan should align with the map walls.
  - station_demo will then drive to Station A, read the clock, and go to Station B.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    SetEnvironmentVariable, ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MAPS_DIR    = os.path.expanduser('~/mirte_ws/src/cognitive-robot/maps')
DEFAULT_MAP = os.path.join(MAPS_DIR, 'auto_map.yaml')
CONFIG_DIR  = os.path.expanduser('~/mirte_ws/src/cognitive-robot/config')
NAV2_PARAMS = os.path.join(CONFIG_DIR, 'nav2_params_mirte_real.yaml')


def generate_launch_description():
    nav2_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        SetEnvironmentVariable('ROS_DOMAIN_ID', '4'),

        DeclareLaunchArgument(
            'map',
            default_value=DEFAULT_MAP,
            description='Full path to the saved map YAML file',
        ),

        # Nav2 — AMCL localisation against the saved map, no SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam':            'False',
                'map':             LaunchConfiguration('map'),
                'use_sim_time':    'false',
                'autostart':       'true',
                'params_file':     NAV2_PARAMS,
                'use_composition': 'False',
            }.items(),
        ),

        # TF: base_link → base_footprint (required by Nav2)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen',
        ),

        # TF: base_link → base_frame (required by some nodes)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_frame'],
            output='screen',
        ),

        # Relay /cmd_vel (Nav2 output) → /mirte_base_controller/cmd_vel (robot input)
        Node(
            package='topic_tools',
            executable='relay',
            arguments=['/cmd_vel', '/mirte_base_controller/cmd_vel'],
            output='screen',
        ),

        # Relay /mirte_base_controller/odom (robot) → /odom (Nav2 input)
        Node(
            package='topic_tools',
            executable='relay',
            arguments=['/mirte_base_controller/odom', '/odom'],
            output='screen',
        ),

        # RViz — Nav2 view with Map, costmaps, and 2D Pose Estimate tool
        ExecuteProcess(
            cmd=[
                'rviz2', '-d',
                os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz'),
            ],
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

        # Autonomous mission — waits for 2D pose estimate, then drives Station A → B
        Node(
            package='cognitive_robot',
            executable='station_demo',
            name='station_clock_mission',
            output='screen',
            parameters=[{
                'station_dir': MAPS_DIR,
                'camera_topic': '/camera/color/image_raw',
            }],
        ),

    ])
