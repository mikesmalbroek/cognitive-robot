"""
phase1_gazebo.launch.py

Phase 1 — Gazebo: Manual map generation + station registration.

What this starts (all on the laptop):
  - Gazebo              : simulated robot environment
  - SLAM Toolbox        : builds the live map (sim time)
  - RViz                : visualise the map as you drive
  - CV perception nodes : detect_abacus, detect_station, read_time
  - trial_depth         : keyboard-driven teleop + station/map saver (OpenCV window)

Run with:
    ros2 launch cognitive_robot phase1_gazebo.launch.py

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
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MAPS_DIR    = os.path.expanduser('~/mirte_ws/src/cognitive-robot/maps')
CONFIG_DIR  = os.path.expanduser('~/mirte_ws/src/cognitive-robot/config')
MODELS_DIR  = os.path.expanduser('~/mirte_ws/src/cognitive-robot/gazebo_map_load')
SLAM_PARAMS = os.path.join(CONFIG_DIR, 'mapper_params_online_async.yaml')
RVIZ_CONFIG = os.path.join(CONFIG_DIR, 'mirte_slam.rviz')

CAMERA_TOPIC      = '/camera/image_raw'
DEPTH_TOPIC       = '/camera/depth/image_raw'
CAMERA_INFO_TOPIC = '/camera/camera_info'
CMD_VEL_TOPIC     = '/mirte_base_controller/cmd_vel_unstamped'


def generate_launch_description():
    slam_share   = get_package_share_directory('slam_toolbox')
    gazebo_share = get_package_share_directory('mirte_gazebo')

    return LaunchDescription([

        # Make Gazebo find textures/materials inside gazebo_map_load/
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            MODELS_DIR + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
        ),

        # Gazebo simulation
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gazebo_mirte_master_empty.launch.xml')
            ),
        ),

        # SLAM Toolbox — sim time on
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': SLAM_PARAMS,
                'use_sim_time':     'true',
            }.items(),
        ),

        # RViz — map visualisation
        ExecuteProcess(
            cmd=['rviz2', '-d', RVIZ_CONFIG],
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

        # trial_depth — Gazebo keyboard teleop + station/map saver
        Node(
            package='cognitive_robot',
            executable='trial_depth',
            name='trial_depth_mapper',
            output='screen',
            parameters=[{
                'cmd_vel_topic': CMD_VEL_TOPIC,
                'camera_topic':  CAMERA_TOPIC,
                'station_dir':   MAPS_DIR,
                'map_dir':       MAPS_DIR,
                'map_name':      'auto_map',
            }],
        ),

    ])
