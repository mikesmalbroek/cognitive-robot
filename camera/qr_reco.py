#!/usr/bin/env python3

# ============================================================
# ArUco Marker Detection Node for ROS 2 + Gazebo
#
# This script subscribes to the Gazebo camera image topic:
#   /camera/image_raw
#
# It also subscribes to the camera calibration topic:
#   /camera/camera_info
#
# The camera image is converted into an OpenCV image.
# Then OpenCV searches for ArUco markers in the image.
# If a marker is found, the code estimates its 3D pose
# relative to the camera.
#
# The output tells you:
#   x = left/right offset from camera
#   y = up/down offset from camera
#   z = forward distance from camera
#
# The code also opens a window showing the camera image,
# detected marker boundary, and marker coordinate axes.
# ============================================================


# ROS 2 Python library
import rclpy
from rclpy.node import Node

# OpenCV library for image processing
import cv2

# ArUco module from OpenCV
# This is used for detecting ArUco markers
from cv2 import aruco

# NumPy is used for matrix operations
import numpy as np

# ROS 2 image message types
from sensor_msgs.msg import Image, CameraInfo

# cv_bridge converts ROS Image messages into OpenCV images
from cv_bridge import CvBridge


class ArucoCameraNode(Node):
    """
    This class defines a ROS 2 node.

    A node is basically a small ROS program.
    This node listens to camera topics from Gazebo,
    processes the camera image, detects ArUco markers,
    and estimates the marker pose.
    """

    def __init__(self):
        """
        This function runs once when the node starts.
        It sets up the camera subscribers, ArUco detector,
        and other variables.
        """

        # Create a ROS 2 node called 'aruco_camera_node'
        super().__init__('aruco_camera_node')

        # Create the cv_bridge object.
        # This is needed to convert ROS images into OpenCV images.
        self.bridge = CvBridge()

        # These will store the camera calibration parameters.
        # At the start, they are unknown.
        # They will be filled when /camera/camera_info publishes data.
        self.camera_matrix = None
        self.dist_coeffs = None

        # Physical size of your printed/simulated ArUco marker.
        # This must match the real marker size in Gazebo.
        # Example: 0.15 means the marker is 15 cm wide.
        self.marker_length = 0.15  # meters

        # Load the predefined ArUco dictionary.
        # DICT_6X6_250 means:
        #   - each marker has a 6x6 internal binary pattern
        #   - there are 250 possible marker IDs
        self.aruco_dict = aruco.getPredefinedDictionary(
            aruco.DICT_6X6_250
        )

        # Create detector parameters.
        # This older OpenCV API uses DetectorParameters_create().
        # These parameters control things like thresholding,
        # corner refinement, and marker filtering.
        self.params = aruco.DetectorParameters_create()

        # We do not create an ArucoDetector object here because
        # your OpenCV version uses the older API:
        #   aruco.detectMarkers(...)
        self.detector = None

        # Subscribe to the camera calibration topic.
        # /camera/camera_info gives:
        #   - focal lengths
        #   - image center
        #   - distortion coefficients
        #
        # These values are required for accurate 3D pose estimation.
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Subscribe to the RGB camera image topic.
        # Every time a new image arrives, image_callback() is called.
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Print message in terminal to confirm the node started.
        self.get_logger().info('ArUco camera node started.')

    def camera_info_callback(self, msg):
        """
        This function receives camera calibration data.

        The CameraInfo message contains:
          msg.k = 3x3 camera intrinsic matrix
          msg.d = distortion coefficients

        The camera matrix looks like:

            [ fx   0  cx ]
            [  0  fy  cy ]
            [  0   0   1 ]

        where:
          fx, fy = focal lengths in pixels
          cx, cy = image center in pixels

        These values allow OpenCV to convert 2D image marker
        corners into a 3D pose estimate.
        """

        # Convert msg.k, which is a flat list of 9 numbers,
        # into a 3x3 NumPy matrix.
        self.camera_matrix = np.array(
            msg.k,
            dtype=np.float64
        ).reshape(3, 3)

        # Convert distortion coefficients into a NumPy array.
        # In Gazebo this may often be close to zero,
        # but it is still best to use the published value.
        self.dist_coeffs = np.array(
            msg.d,
            dtype=np.float64
        )

    def image_callback(self, msg):
        """
        This function runs every time a camera image is received.

        It does the main work:
          1. Convert ROS image to OpenCV image
          2. Detect ArUco markers
          3. Estimate 3D position/orientation
          4. Draw marker boundary and axes
          5. Show the image in an OpenCV window
        """

        # Pose estimation requires the camera calibration.
        # If camera_info has not arrived yet, we cannot estimate pose.
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for /camera/camera_info...')
            return

        # Convert the ROS Image message into an OpenCV BGR image.
        # OpenCV uses BGR color order by default, not RGB.
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Detect ArUco markers in the current camera frame.
        #
        # corners:
        #   list of detected marker corner coordinates.
        #   Each marker has 4 corner points in image pixels.
        #
        # ids:
        #   marker IDs, for example 0, 1, 2, etc.
        #
        # _:
        #   rejected marker candidates, not used here.
        corners, ids, _ = aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.params
        )

        # Check if any marker was detected.
        if ids is not None:

            # Draw green boxes around detected markers.
            aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            # Estimate pose for each detected marker.
            #
            # rvecs:
            #   rotation vectors.
            #   These describe the marker's orientation
            #   relative to the camera.
            #
            # tvecs:
            #   translation vectors.
            #   These describe the marker's position
            #   relative to the camera.
            #
            # The result is in meters because marker_length is in meters.
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )

            # Loop through all detected markers.
            for i in range(len(ids)):

                # Draw the marker coordinate frame on the image.
                #
                # Red axis   = marker X direction
                # Green axis = marker Y direction
                # Blue axis  = marker Z direction
                #
                # The last value, 0.1, is the length of the axes in meters.
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    0.1
                )

                # Extract translation vector.
                # This is the marker position relative to the camera.
                x, y, z = tvecs[i][0]

                # Extract rotation vector.
                # This is the marker orientation relative to the camera.
                rx, ry, rz = rvecs[i][0]

                # Extract the marker ID.
                marker_id = ids[i][0]

                # Print marker pose information to terminal.
                #
                # Coordinate convention:
                #   x = left/right in the camera image
                #   y = up/down in the camera image
                #   z = forward distance from the camera
                #
                # For navigation, z is usually the most useful first value.
                self.get_logger().info(
                    f'Marker {marker_id}: '
                    f'x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, '
                    f'rx={rx:.3f}, ry={ry:.3f}, rz={rz:.3f}'
                )

                # Create short text to display on the image.
                text = f'ID:{marker_id} Dist:{z:.2f}m'

                # Draw the text on the top-left area of the image.
                cv2.putText(
                    frame,
                    text,
                    (30, 40 + i * 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

        # Show the camera frame in a window.
        # This window updates continuously as new camera images arrive.
        cv2.imshow(
            'Gazebo ArUco Detection',
            frame
        )

        # waitKey(1) is required for OpenCV windows to refresh.
        # The value 1 means wait 1 millisecond.
        cv2.waitKey(1)


def main(args=None):
    """
    Main function.

    This initializes ROS 2, creates the node,
    and keeps the node alive until Ctrl+C is pressed.
    """

    # Start ROS 2 Python system.
    rclpy.init(args=args)

    # Create the ArUco camera node.
    node = ArucoCameraNode()

    try:
        # Keep the node running.
        # This allows callbacks to keep receiving camera images.
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Allows clean exit using Ctrl+C.
        pass

    # Destroy the ROS node.
    node.destroy_node()

    # Close OpenCV image windows.
    cv2.destroyAllWindows()

    # Shutdown ROS 2.
    rclpy.shutdown()


if __name__ == '__main__':
    main()
