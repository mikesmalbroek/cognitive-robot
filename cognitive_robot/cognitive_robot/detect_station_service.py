"""
detect_station_service.py

One-shot ROS2 node for ArUco station detection.

WHAT THIS SCRIPT DOES
----------------------
This script runs on the laptop. It connects via ROS2 to the front camera
of the MIRTE robot, captures one frame, saves it as a photo, and runs the
frame through ArUco detection.

The result is printed to the terminal:
  - Whether a marker was found
  - Which station it is (Station A or Station B)
  - Position in metres (x, y, z relative to the camera)
  - Orientation in radians (roll, pitch, yaw)

STRUCTURE (3 levels)
---------------------
  Level 1 — Pure ArUco functions (no ROS, also callable standalone)
    - save_photo(image, folder)   -> save an OpenCV image as .jpg
    - detect_aruco(image)         -> detect markers, return results

  Level 2 — ROS2 camera capture
    - grab_frame(node, topic)     -> wait for one camera frame via ROS2

  Level 3 — ROS2 entry point
    - main()                      -> connects all levels, starts and stops the node

HOW TO RUN
-----------
  ros2 run cognitive_robot detect_station_service

  Different camera topic (e.g. Gazebo):
    ros2 run cognitive_robot detect_station_service \
      --ros-args -p camera_topic:=/camera/image_raw

  Different save directory:
    ros2 run cognitive_robot detect_station_service \
      --ros-args -p save_dir:=~/my_folder

CAMERA CALIBRATION
-------------------
The values below are estimates for a generic webcam.
TODO: replace with real calibration values from the MIRTE camera.
      Use: ros2 run camera_calibration cameracalibrator

TOPICS
-------
  /camera/color/image_raw  (subscribe) — front camera of the robot
"""

import os
import threading
from datetime import datetime

import cv2
from cv2 import aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


# ===========================================================================
# CONFIGURATION — adjust these values per situation
# ===========================================================================

# Mapping from ArUco marker ID to station name
STATION_MAP = {
    0: 'Station A',
    1: 'Station B',
}

# ArUco dictionary — must match the one used in aruro_make_photos.py
ARUCO_DICT = aruco.DICT_6X6_250

# Physical size of the printed marker in metres
# If you printed a 20 cm marker, use 0.20
MARKER_LENGTH_METERS = 0.20

# Placeholder camera calibration (estimates for a generic webcam)
# TODO: replace with real calibration values from the MIRTE camera
FOCAL_LENGTH_PX = 800
IMAGE_CENTER = (320, 240)


# ===========================================================================
# LEVEL 1 — Pure functions: no ROS, callable directly with an OpenCV image
# ===========================================================================

def save_photo(image, folder):
    """
    Save an OpenCV image as a JPEG file.

    Parameters
    ----------
    image  : numpy.ndarray   BGR image in OpenCV format
    folder : str             Directory to save the photo in

    Returns
    -------
    str — full file path of the saved photo
    """
    os.makedirs(folder, exist_ok=True)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    filename  = f'aruco_{timestamp}.jpg'
    filepath  = os.path.join(folder, filename)

    cv2.imwrite(filepath, image)
    return filepath


def detect_aruco(image):
    """
    Detect ArUco markers in an OpenCV image.

    Uses the camera matrix and distortion coefficients defined in the
    configuration section at the top of this file. Estimates the 3D
    position and orientation of each detected marker.

    Also draws the detected markers and coordinate axes on a copy of the
    image, so the caller can save or display it.

    Parameters
    ----------
    image : numpy.ndarray   BGR image in OpenCV format

    Returns
    -------
    detections : list of dict, one entry per detected marker:
        {
            'id'           : int    — marker ID (0, 1, ...)
            'station_name' : str    — e.g. 'Station A', or 'Unknown (ID: 2)'
            'x'            : float  — left/right offset in metres
            'y'            : float  — up/down offset in metres
            'z'            : float  — forward distance in metres
            'yaw'          : float  — horizontal rotation in radians
                                      0 = facing camera squarely
                                      positive = rotated right, negative = left
        }
    annotated : numpy.ndarray
        Copy of the image with detected markers and coordinate axes drawn on it.
        Identical to the input image if no markers were found.
    """
    # Build the ArUco detector
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
    params     = aruco.DetectorParameters()
    detector   = aruco.ArucoDetector(aruco_dict, params)

    # Build the camera matrix from the configuration at the top of the file
    camera_matrix = np.array([
        [FOCAL_LENGTH_PX, 0,               IMAGE_CENTER[0]],
        [0,               FOCAL_LENGTH_PX, IMAGE_CENTER[1]],
        [0,               0,               1              ],
    ], dtype=float)

    # Assume no lens distortion (placeholder)
    dist_coeffs = np.zeros(5)

    # Work on a copy so the original image stays unmodified
    annotated = image.copy()

    # Search for markers in the image
    corners, ids, _ = detector.detectMarkers(image)

    if ids is None:
        return [], annotated

    # Draw green borders around every detected marker
    aruco.drawDetectedMarkers(annotated, corners, ids)

    # Estimate the 3D pose per marker.
    # estimatePoseSingleMarkers is deprecated in OpenCV 4.7+, but still works.
    # TODO: replace with solvePnP when upgrading to OpenCV 4.7+.
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        corners, MARKER_LENGTH_METERS, camera_matrix, dist_coeffs
    )

    results = []
    for i, marker_id in enumerate(ids):
        mid = int(marker_id[0])

        # Position from the translation vector
        x, y, z = tvecs[i][0]

        # Draw the 3D coordinate axes on the marker (R=X, G=Y, B=Z), 10 cm long
        cv2.drawFrameAxes(annotated, camera_matrix, dist_coeffs,
                          rvecs[i], tvecs[i], 0.1)

        # Compute yaw from the marker's normal vector (third column of R).
        # The marker's Z axis points outward from its face toward the camera.
        # Projecting that vector onto the horizontal plane and measuring its
        # angle gives the true horizontal rotation of the marker relative to
        # facing the camera squarely.
        #
        # Formula: yaw = atan2(normal_x, -normal_z)
        #   yaw =  0   → marker faces camera squarely
        #   yaw > 0    → marker is rotated to the right (from camera's view)
        #   yaw < 0    → marker is rotated to the left
        #
        # This avoids the Euler angle ambiguity that occurs when roll ≈ ±π,
        # which caused the standard ZYX decomposition to always return yaw ≈ 0.
        rotation_matrix, _ = cv2.Rodrigues(rvecs[i][0])
        marker_normal = rotation_matrix[:, 2]
        yaw = float(np.arctan2(marker_normal[0], -marker_normal[2]))

        station_name = STATION_MAP.get(mid, f'Unknown (ID: {mid})')

        # Draw station name, distance and yaw as text above the marker
        label = f'{station_name}  z={z:.2f}m  yaw={np.degrees(yaw):.1f}deg'
        top_left = tuple(corners[i][0][0].astype(int))
        cv2.putText(annotated, label, (top_left[0], top_left[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        results.append({
            'id':           mid,
            'station_name': station_name,
            'x':            float(x),
            'y':            float(y),
            'z':            float(z),
            'yaw':          yaw,
        })

    return results, annotated


# ===========================================================================
# LEVEL 2 — ROS2 camera capture: wait for one frame from the camera topic
# ===========================================================================

def grab_frame(node, topic, timeout_sec=10.0):
    """
    Wait for one camera frame via ROS2 and return it as an OpenCV image.

    Creates a temporary subscription internally, waits until a frame
    arrives, then destroys the subscription.

    Parameters
    ----------
    node        : rclpy.node.Node   Active ROS2 node
    topic       : str               Camera topic, e.g. '/camera/color/image_raw'
    timeout_sec : float             Maximum wait time in seconds (default: 10)

    Returns
    -------
    numpy.ndarray or None
        BGR OpenCV image if a frame arrived, otherwise None.
    """
    bridge         = CvBridge()
    received       = threading.Event()
    captured_frame = [None]  # list so the closure can write to it

    def _callback(msg):
        if not received.is_set():
            captured_frame[0] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            received.set()

    # Create a temporary subscription
    sub = node.create_subscription(Image, topic, _callback, 10)

    node.get_logger().info(f'Waiting for camera frame on: {topic}')

    # Spin the node in a separate thread so the callback can fire
    spin_thread = threading.Thread(
        target=rclpy.spin_once,
        args=(node,),
        kwargs={'timeout_sec': timeout_sec},
        daemon=True,
    )
    spin_thread.start()

    frame_arrived = received.wait(timeout=timeout_sec)

    # Additional spin_once calls to ensure the callback fires if the frame
    # was already queued before the thread started
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if received.is_set():
            break

    node.destroy_subscription(sub)

    if not frame_arrived:
        node.get_logger().error(
            f'No camera frame received within {timeout_sec}s on {topic}. '
            'Check that the robot is connected and the topic exists.'
        )
        return None

    node.get_logger().info('Camera frame received.')
    return captured_frame[0]


# ===========================================================================
# LEVEL 3 — ROS2 entry point: connects all levels and runs the full pipeline
# ===========================================================================

def main(args=None):
    """
    Start the ROS2 node, grab one frame, save it, detect ArUco markers,
    print the results, and shut down the node.

    Run with:
        ros2 run cognitive_robot detect_station_service
    """
    rclpy.init(args=args)
    node = rclpy.create_node('detect_station_service')

    # ------------------------------------------------------------------ #
    # ROS2 parameters                                                      #
    # ------------------------------------------------------------------ #
    node.declare_parameter('camera_topic', '/camera/color/image_raw')
    # Real robot : /camera/color/image_raw
    # Gazebo     : /camera/image_raw

    node.declare_parameter('save_dir', '~/aruco_photos')
    # Directory where captured photos are saved

    camera_topic = node.get_parameter('camera_topic').get_parameter_value().string_value
    save_dir     = os.path.expanduser(
        node.get_parameter('save_dir').get_parameter_value().string_value
    )

    node.get_logger().info('=== ArUco station detection started ===')
    node.get_logger().info(f'Camera topic : {camera_topic}')
    node.get_logger().info(f'Photo dir    : {save_dir}')

    # ------------------------------------------------------------------ #
    # Step 1 — Grab a frame from the front camera (Level 2)               #
    # ------------------------------------------------------------------ #
    frame = grab_frame(node, camera_topic)

    if frame is None:
        node.get_logger().error('Stopped: no camera frame available.')
        node.destroy_node()
        rclpy.shutdown()
        return

    # ------------------------------------------------------------------ #
    # Step 2 — Detect ArUco markers (Level 1)                             #
    # ------------------------------------------------------------------ #
    node.get_logger().info('Running ArUco detection...')
    detections, annotated = detect_aruco(frame)

    # ------------------------------------------------------------------ #
    # Step 3 — Save the annotated photo (Level 1)                         #
    # ------------------------------------------------------------------ #
    photo_path = save_photo(annotated, save_dir)
    node.get_logger().info(f'Photo saved: {photo_path}')

    # ------------------------------------------------------------------ #
    # Step 4 — Print results                                               #
    # ------------------------------------------------------------------ #
    print('\n' + '=' * 50)
    print('ARUCO DETECTION RESULT')
    print('=' * 50)
    print(f'Photo: {photo_path}')
    print()

    if not detections:
        print('No ArUco markers found in this frame.')
    else:
        print(f'{len(detections)} marker(s) found:\n')
        for det in detections:
            print(f'  Station      : {det["station_name"]}  (ID: {det["id"]})')
            print(f'  Position (m) :  x={det["x"]:+.3f}  y={det["y"]:+.3f}  z={det["z"]:+.3f}')
            print(f'  Yaw          :  {np.degrees(det["yaw"]):+.1f} deg  ({det["yaw"]:+.3f} rad)')
            print(f'  → Robot must rotate {np.degrees(det["yaw"]):+.1f} deg to face the marker squarely')
            print()

    print('=' * 50 + '\n')

    # ------------------------------------------------------------------ #
    # Shut down                                                            #
    # ------------------------------------------------------------------ #
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
