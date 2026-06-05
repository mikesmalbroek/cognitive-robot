from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='depth_image_proc',
            executable='register_node',
            name='depth_register',
            remappings=[
                ('depth/image_rect',            '/camera/depth/image_raw'),
                ('depth/camera_info',           '/camera/depth/camera_info'),
                ('rgb/camera_info',             '/camera/color/camera_info'),
                ('depth_registered/image_rect', '/camera/aligned_depth/image_raw'),
            ],
        ),

        Node(
            package='cognitive_robot',
            executable='detect_abacus_service',
            name='detect_abacus_service',
        ),

        Node(
            package='cognitive_robot',
            executable='detect_station_service',
            name='detect_station_service',
        ),

        Node(
            package='cognitive_robot',
            executable='read_time_service',
            name='read_time_service',
        ),

    ])
