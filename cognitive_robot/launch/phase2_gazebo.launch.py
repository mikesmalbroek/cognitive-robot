"""
phase2_gazebo.launch.py

Phase 2 — Gazebo: Autonomous navigation to stations.

What this starts (all on the laptop):
  - Gazebo              : same empty world as phase 1
  - Nav2                : localisation (AMCL) against the saved map + path planning
  - RViz                : Nav2 view — set the 2D Pose Estimate here before the robot moves
  - CV perception nodes : detect_abacus, detect_station, read_time
  - station_demo        : autonomous mission (Station A → read clock → Station B)
                          waits for the 2D pose estimate before driving

Before running:
  - Phase 1 Gazebo must be complete — station_a_location.yaml and
    station_b_location.yaml must exist in:
      ~/mirte_ws/src/cognitive-robot/maps/
  - The saved map (auto_map.yaml) must also be in that folder.

Run with:
    ros2 launch cognitive_robot phase2_gazebo.launch.py

To use a different map:
    ros2 launch cognitive_robot phase2_gazebo.launch.py map:=/full/path/to/map.yaml

After launch:
  - Wait for RViz and Gazebo to load.
  - In RViz click "2D Pose Estimate", click where the robot is on the map,
    and drag in the direction it is facing.
  - station_demo will then drive to Station A, read the clock, and go to Station B.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    SetEnvironmentVariable, ExecuteProcess,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MAPS_DIR    = os.path.expanduser('~/mirte_ws/src/cognitive-robot/maps')
DEFAULT_MAP = os.path.join(MAPS_DIR, 'auto_map.yaml')
MODELS_DIR  = os.path.expanduser('~/mirte_ws/src/cognitive-robot/gazebo_map_load')
CONFIG_DIR  = os.path.expanduser('~/mirte_ws/src/cognitive-robot/config')
NAV2_PARAMS = os.path.join(CONFIG_DIR, 'nav2_params_mirte.yaml')

CAMERA_TOPIC      = '/camera/image_raw'
DEPTH_TOPIC       = '/camera/depth/image_raw'
CAMERA_INFO_TOPIC = '/camera/camera_info'
CMD_VEL_TOPIC     = '/mirte_base_controller/cmd_vel_unstamped'


def generate_launch_description():
    gazebo_share = get_package_share_directory('mirte_gazebo')
    nav2_share   = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        # Make Gazebo find textures/materials inside gazebo_map_load/
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            MODELS_DIR + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
        ),

        DeclareLaunchArgument(
            'map',
            default_value=DEFAULT_MAP,
            description='Full path to the saved map YAML file',
        ),

        # Gazebo simulation — same empty world as phase 1
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gazebo_mirte_master_empty.launch.xml')
            ),
        ),

        # Nav2 — localisation (AMCL) against the saved map, no SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam':            'False',
                'map':             LaunchConfiguration('map'),
                'use_sim_time':    'true',
                'autostart':       'true',
                'params_file':     NAV2_PARAMS,
                'use_composition': 'False',
            }.items(),
        ),

        # RViz — Nav2 view with 2D Pose Estimate tool
        ExecuteProcess(
            cmd=[
                'rviz2', '-d',
                os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz'),
            ],
            output='screen',
        ),

        # CV perception services (Gazebo camera topics)
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
                'camera_topic':  CAMERA_TOPIC,
                'cmd_vel_topic': CMD_VEL_TOPIC,
            }],
        ),

        # Spawn map entities — waits 10 s for Gazebo to be ready, then spawns sequentially
        ExecuteProcess(
            cmd=['bash', '-c',
                'sleep 10 && '
                f'ros2 run gazebo_ros spawn_entity.py -entity station_a_laptop -file {MODELS_DIR}/station_a_laptop/model.sdf -x -1 -y 1.22 -z 0 -Y 1 && '
                'sleep 1 && '
                f'ros2 run gazebo_ros spawn_entity.py -entity station_b_box -file {MODELS_DIR}/station_b_box/model.sdf -x 1.0 -y -1.2 -z 0.0 -Y -1.5 && '
                'sleep 1 && '
                f'ros2 run gazebo_ros spawn_entity.py -entity rect_wall -file {MODELS_DIR}/rect_wall/model.sdf -x 0 -y 0 -z 0'
            ],
            output='screen',
        ),

        # Move arm out of camera view at startup
        ExecuteProcess(
            cmd=['bash', '-c',
                "sleep 8 && "
                "ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory "
                "trajectory_msgs/msg/JointTrajectory "
                "'{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint], "
                "points: [{positions: [0.0, -0.35, -1.20, 0.80], velocities: [0.0, 0.0, 0.0, 0.0], "
                "time_from_start: {sec: 3, nanosec: 0}}]}'"
            ],
            output='screen',
        ),

        # Autonomous mission — waits for 2D pose estimate, then drives to Station A → B
        Node(
            package='cognitive_robot',
            executable='station_demo',
            name='station_clock_mission',
            output='screen',
            parameters=[{
                'station_dir': MAPS_DIR,
                'camera_topic': CAMERA_TOPIC,
            }],
        ),

    ])
