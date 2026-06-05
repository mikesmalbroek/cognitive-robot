cmake_minimum_required(VERSION 3.8)
project(cognitive_robot_interfaces)

# Find ament_cmake (the ROS2 CMake build system)
find_package(ament_cmake REQUIRED)

# Find the tool that generates Python/C++ code from .msg/.srv files
find_package(rosidl_default_generators REQUIRED)

# Tell rosidl to generate code for our service definition.
# Add any future .msg or .srv files to this list.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ReadTime.srv"
  "srv/DetectAbacus.srv"
)

ament_package()
