"""
detect_station_service.py

ROS2 service node that detects an ArUco station marker in the robot's front
camera image and returns its identity, orientation, and 3D position using
the depth camera.

HOW IT FITS IN THE SYSTEM
--------------------------
The task planner calls the /detect_station service (empty request). This node:

  1. Grabs the latest frame from the robot's front camera.
  2. Runs ArUco marker detection on the frame.
  3. For each detected marker, computes:
       - yaw : horizontal rotation of the marker (from solvePnP, ArUco only)
       - x_m, y_m, z_m / distance_m : real 3D position (from depth camera)
  4. Returns the marker identity, orientation, and 3D position.

WHY POSITION COMES FROM THE DEPTH CAMERA (NOT ARUCO)
------------------------------------------------------
ArUco's solvePnP estimates position from corner geometry, but it requires
accurate camera intrinsics. Using placeholder intrinsics (as we would before
calibration) leads to large position errors at typical station distances.
The depth camera measures distance directly and is unaffected by intrinsic
errors, so we use it for all position data (x_m, y_m, z_m).

Yaw (rotation of the marker) cannot be measured by the depth camera, so
solvePnP is still used for that single value. Its accuracy for rotation
is acceptable even with approximate intrinsics.

WHY THIS RUNS ON THE LAPTOP (NOT THE ROBOT)
--------------------------------------------
Depth measurement uses image_geometry (PinholeCameraModel) which requires
more compute than the robot's CPU is set up to run for this task.

STRUCTURE
---------
  Level 1 — Pure functions (no ROS, callable standalone with any OpenCV image)
    - detect_aruco(image, camera_matrix, dist_coeffs) → detections, annotated
    - save_photo(image, folder)                        → file path

  Level 2 — ROS2 service class
    - DetectStationService(Node, DepthCameraMixin)

  Level 3 — Entry point
    - main()

TOPICS USED
-----------
  /camera/color/image_raw      (subscribe) — colour camera from the robot
  /camera/depth/image_raw      (subscribe) — depth camera (via DepthCameraMixin)
  /camera/color/camera_info    (subscribe) — intrinsics  (via DepthCameraMixin)

SERVICE PROVIDED
----------------
  /detect_station  (cognitive_robot_interfaces/srv/DetectStation)
      Request  : (empty)
      Response : detected     (bool)     False when no marker found
                 marker_id    (int32)    ArUco marker ID
                 station_name (string)   e.g. 'Station A'
                 distance_m   (float32)  depth-camera distance in metres
                 x_m          (float32)  right offset in metres
                 y_m          (float32)  down  offset in metres
                 z_m          (float32)  forward distance (= distance_m)
                 yaw          (float32)  horizontal rotation in radians
"""

import os
import threading
from datetime import datetime

import cv2
from cv2 import aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cognitive_robot_interfaces.srv import DetectStation
from cognitive_robot.depth_utils import DepthCameraMixin


# ===========================================================================
# CONFIGURATION
# ===========================================================================

# Mapping from ArUco marker ID to station name.
STATION_MAP = {
    0: 'Station A',
    1: 'Station B',
}

# ArUco dictionary — must match the one used when the markers were printed.
ARUCO_DICT = aruco.DICT_6X6_250

# Physical size of the printed marker in metres.
MARKER_LENGTH_METERS = 0.20

# Fallback camera matrix used for solvePnP (yaw only) when the real
# camera_info has not arrived yet. These are rough estimates for a typical
# webcam and are only used as a last resort.
_FALLBACK_FOCAL_PX = 800
_FALLBACK_CENTER   = (320, 240)


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


def detect_aruco(image, camera_matrix=None, dist_coeffs=None):
    """
    Detect ArUco markers in an OpenCV image.

    Returns the pixel centre and bounding box of each marker (used by the
    caller to sample depth) and the yaw from solvePnP rotation (the only
    geometric value that the depth camera cannot provide).

    Position (x, y, z in metres) is intentionally NOT returned here.
    The caller must obtain that from the depth camera instead, because
    solvePnP position is inaccurate without a precisely calibrated camera
    matrix.

    Also draws the detected markers and coordinate axes on a copy of the
    image, so the caller can save or display it.

    Parameters
    ----------
    image         : numpy.ndarray   BGR image in OpenCV format
    camera_matrix : numpy.ndarray or None
        3x3 camera intrinsic matrix. Pass the real matrix from
        PinholeCameraModel when available. Falls back to a generic
        estimate when None.
    dist_coeffs   : numpy.ndarray or None
        Distortion coefficients. Pass real values when available,
        otherwise zero distortion is assumed.

    Returns
    -------
    detections : list of dict, one entry per detected marker:
        {
            'id'           : int    — ArUco marker ID
            'station_name' : str    — e.g. 'Station A', or 'Unknown (ID: 2)'
            'pixel_x'      : int    — pixel X of the marker centre
            'pixel_y'      : int    — pixel Y of the marker centre
            'bbox_width'   : int    — marker bounding box width  in pixels
            'bbox_height'  : int    — marker bounding box height in pixels
            'yaw'          : float  — horizontal rotation in radians
                                      0 = facing camera squarely
                                      positive = rotated right, negative = left
        }
    annotated : numpy.ndarray
        Copy of the image with detected markers and coordinate axes drawn.
        Identical to the input image when no markers were found.
    """
    # Use the real camera matrix if provided, otherwise fall back to estimates.
    if camera_matrix is None:
        camera_matrix = np.array([
            [_FALLBACK_FOCAL_PX, 0,                  _FALLBACK_CENTER[0]],
            [0,                  _FALLBACK_FOCAL_PX, _FALLBACK_CENTER[1]],
            [0,                  0,                  1                  ],
        ], dtype=float)

    if dist_coeffs is None:
        dist_coeffs = np.zeros(5)

    if hasattr(aruco, 'ArucoDetector'):
        aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
        detector   = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())
        annotated  = image.copy()
        corners, ids, _ = detector.detectMarkers(image)
    else:
        aruco_dict = aruco.Dictionary_get(ARUCO_DICT)
        params     = aruco.DetectorParameters_create()
        annotated  = image.copy()
        corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=params)

    if ids is None:
        return [], annotated

    aruco.drawDetectedMarkers(annotated, corners, ids)

    # 3D corner positions of a flat marker centred at the origin (marker frame).
    half = MARKER_LENGTH_METERS / 2.0
    obj_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)

    detections = []
    for i, marker_id in enumerate(ids):
        mid = int(marker_id[0])

        # Pixel centre of the marker (mean of the four corner points).
        x_coords = corners[i][0][:, 0]
        y_coords = corners[i][0][:, 1]
        pixel_x = int(np.mean(x_coords))
        pixel_y = int(np.mean(y_coords))

        # Bounding box in pixels.
        bbox_width  = int(np.max(x_coords) - np.min(x_coords))
        bbox_height = int(np.max(y_coords) - np.min(y_coords))

        # solvePnP is used only to get the rotation vector for yaw estimation.
        # The translation vector (position) is discarded — the depth camera
        # provides a more accurate position measurement.
        img_points = corners[i][0].astype(np.float32)
        _, rvec, _ = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

        # Draw the 3D coordinate axes on the marker (R=X, G=Y, B=Z), 10 cm long.
        cv2.drawFrameAxes(annotated, camera_matrix, dist_coeffs, rvec,
                          np.zeros((3, 1)), 0.1)

        # Compute yaw from the marker's normal vector (third column of R).
        # The marker's Z axis points outward from its face toward the camera.
        # Projecting that vector onto the horizontal plane and measuring its
        # angle gives the true horizontal rotation of the marker.
        #
        # Formula: yaw = atan2(normal_x, -normal_z)
        #   yaw =  0   → marker faces camera squarely
        #   yaw > 0    → marker is rotated to the right (from camera's view)
        #   yaw < 0    → marker is rotated to the left
        #
        # This avoids the Euler angle ambiguity that occurs when roll ≈ ±π.
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        marker_normal = rotation_matrix[:, 2]
        yaw = float(np.arctan2(marker_normal[0], -marker_normal[2]))

        station_name = STATION_MAP.get(mid, f'Unknown (ID: {mid})')

        # Draw station name and yaw above the marker (position comes from depth).
        label = f'{station_name}  yaw={np.degrees(yaw):.1f}deg'
        top_left = tuple(corners[i][0][0].astype(int))
        cv2.putText(annotated, label, (top_left[0], top_left[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        detections.append({
            'id':           mid,
            'station_name': station_name,
            'pixel_x':      pixel_x,
            'pixel_y':      pixel_y,
            'bbox_width':   bbox_width,
            'bbox_height':  bbox_height,
            'yaw':          yaw,
        })

    return detections, annotated


# ===========================================================================
# LEVEL 2 — ROS2 service class
# ===========================================================================

class DetectStationService(Node, DepthCameraMixin):
    """
    ROS2 node that exposes the /detect_station service.

    On startup it:
      - Subscribes to the robot's front camera and caches every incoming frame.
      - Sets up depth camera subscriptions via DepthCameraMixin.
      - Declares ROS2 parameters for easy tuning at launch time.

    When the service is called it:
      1. Captures the latest colour frame.
      2. Builds the camera matrix from real intrinsics (if camera_info arrived).
      3. Runs ArUco detection to get pixel centre, bounding box, and yaw.
      4. Samples depth at the pixel centre (DepthCameraMixin).
      5. Projects pixel + depth to a real-world 3D position (DepthCameraMixin).
      6. Saves the annotated colour photo and depth debug images.
      7. Fills in and returns the response.
    """

    def __init__(self):
        """Initialise the node and set up subscribers and service."""
        super().__init__('detect_station_service')

        # ------------------------------------------------------------------ #
        # ROS2 parameters                                                      #
        # ------------------------------------------------------------------ #
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        # Real robot : /camera/color/image_raw
        # Gazebo     : /camera/image_raw

        self.declare_parameter('save_dir', '~/aruco_photos')
        # Directory where annotated colour photos are saved.

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        # Raw depth topic (uint16, values in mm).

        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        # Camera info topic used to initialise the PinholeCameraModel.
        # Real robot : /camera/color/camera_info
        # Gazebo     : /camera/camera_info

        # ------------------------------------------------------------------ #
        # Callback group                                                       #
        # ------------------------------------------------------------------ #
        # ReentrantCallbackGroup lets the camera callback keep running while
        # the service callback is blocked.
        self._cb_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------ #
        # Colour camera subscription                                           #
        # ------------------------------------------------------------------ #
        self.bridge = CvBridge()

        self.latest_frame = None
        # The most recent colour camera frame, updated on every incoming message.

        self._frame_lock = threading.Lock()
        # Protects latest_frame against concurrent access from camera and
        # service callbacks running in different threads.

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.create_subscription(
            Image,
            camera_topic,
            self._camera_callback,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f'Subscribing to camera on: {camera_topic}')

        # ------------------------------------------------------------------ #
        # Depth camera subscriptions (via DepthCameraMixin)                   #
        # ------------------------------------------------------------------ #
        # Sets up depth image and camera_info subscriptions.
        depth_topic       = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self._setup_depth_subscriptions(self._cb_group, depth_topic, camera_info_topic)

        # ------------------------------------------------------------------ #
        # Service server                                                       #
        # ------------------------------------------------------------------ #
        self.srv = self.create_service(
            DetectStation,
            '/detect_station',
            self._handle_detect_station,
            callback_group=self._cb_group,
        )
        self.get_logger().info('Service /detect_station is ready.')

    # ---------------------------------------------------------------------- #
    # Camera callback                                                          #
    # ---------------------------------------------------------------------- #

    def _camera_callback(self, msg):
        """
        Called automatically by ROS every time a new colour camera frame arrives.

        Parameters
        ----------
        msg : sensor_msgs.msg.Image
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._frame_lock:
            self.latest_frame = frame

    # ---------------------------------------------------------------------- #
    # Base functions                                                           #
    # ---------------------------------------------------------------------- #

    def _capture_frame(self):
        """
        Return a thread-safe copy of the most recent colour camera frame.

        Returns
        -------
        numpy.ndarray or None
            A BGR OpenCV image, or None if no frame has arrived yet.
        """
        with self._frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def _build_camera_matrix(self):
        """
        Build a camera matrix and distortion coefficients for solvePnP.

        Uses the real calibration values from PinholeCameraModel (received
        via camera_info) when available. Falls back to placeholder values
        if camera_info has not arrived yet.

        The camera matrix is only needed for yaw estimation. Position comes
        from the depth camera and does not depend on this matrix.

        Returns
        -------
        tuple (numpy.ndarray, numpy.ndarray)
            (camera_matrix 3x3, dist_coeffs)
            Returns (None, None) to signal detect_aruco to use its own fallback.
        """
        if not self._camera_model_ready or self._camera_info_msg is None:
            return None, None

        fx = self._camera_model.fx()
        fy = self._camera_model.fy()
        cx = self._camera_model.cx()
        cy = self._camera_model.cy()

        camera_matrix = np.array(
            [[fx, 0, cx],
             [0, fy, cy],
             [0,  0,  1]],
            dtype=float,
        )
        dist_coeffs = np.array(self._camera_info_msg.d, dtype=float)
        return camera_matrix, dist_coeffs

    # ---------------------------------------------------------------------- #
    # Service callback                                                         #
    # ---------------------------------------------------------------------- #

    def _handle_detect_station(self, request, response):
        """
        Main service callback — called when the task planner calls /detect_station.

        Execution order
        ---------------
        1. _capture_frame()                    → get the latest colour image
        2. _build_camera_matrix()              → real or fallback intrinsics
        3. detect_aruco(frame, ...)            → pixel centre, bbox, yaw per marker
        4. save_photo(annotated, save_dir)     → save annotated colour image
        5. _capture_depth_frame()              → get the latest depth image
        6. _sample_depth(depth_frame, px, py)  → distance in metres
        7. _project_to_3d(px, py, distance_m)  → x_m, y_m, z_m
        8. _save_depth_image(...)             → save debug depth images

        When multiple markers are visible only the first detection is returned.
        A warning is logged when more than one marker is found.

        Parameters
        ----------
        request  : DetectStation.Request   (empty — no fields)
        response : DetectStation.Response

        Returns
        -------
        DetectStation.Response
        """
        self.get_logger().info('Received /detect_station request.')

        # Helper to fill a zero/false response in one call.
        def _empty_response():
            response.detected     = False
            response.marker_id    = 0
            response.station_name = ''
            response.distance_m   = 0.0
            response.x_m          = 0.0
            response.y_m          = 0.0
            response.z_m          = 0.0
            response.yaw          = 0.0
            return response

        # Step 1: grab the latest colour frame.
        frame = self._capture_frame()
        if frame is None:
            self.get_logger().warn('No camera frame available yet.')
            return _empty_response()

        # Step 2: build the camera matrix for solvePnP (yaw only).
        camera_matrix, dist_coeffs = self._build_camera_matrix()

        # Step 3: detect ArUco markers.
        save_dir = os.path.expanduser(
            self.get_parameter('save_dir').get_parameter_value().string_value
        )
        detections, annotated = detect_aruco(frame, camera_matrix, dist_coeffs)

        # Step 4: save the annotated colour photo.
        photo_path = save_photo(annotated, save_dir)
        self.get_logger().info(f'Photo saved: {photo_path}')

        if not detections:
            self.get_logger().info('No ArUco markers found.')
            return _empty_response()

        if len(detections) > 1:
            self.get_logger().warn(
                f'{len(detections)} markers found — using the first one '
                f'(ID={detections[0]["id"]}).'
            )

        det = detections[0]

        # Step 5 & 6: measure depth at the pixel centre of the marker.
        depth_frame = self._capture_depth_frame()
        if depth_frame is None:
            self.get_logger().warn('No depth frame available — position not measured.')
            response.detected     = True
            response.marker_id    = det['id']
            response.station_name = det['station_name']
            response.distance_m   = 0.0
            response.x_m          = 0.0
            response.y_m          = 0.0
            response.z_m          = 0.0
            response.yaw          = det['yaw']
            return response

        distance_m = self._sample_depth(depth_frame, det['pixel_x'], det['pixel_y'])

        # Step 7: project pixel + depth to a real-world 3D position.
        x_m, y_m, z_m = self._project_to_3d(det['pixel_x'], det['pixel_y'], distance_m)

        # Step 8: save depth debug images.
        self._save_depth_image(
            depth_frame,
            det['pixel_x'], det['pixel_y'],
            det['bbox_width'], det['bbox_height'],
            label='aruco',
        )

        response.detected     = True
        response.marker_id    = det['id']
        response.station_name = det['station_name']
        response.distance_m   = distance_m
        response.x_m          = x_m
        response.y_m          = y_m
        response.z_m          = z_m
        response.yaw          = det['yaw']

        self.get_logger().info(
            f'Result — station={det["station_name"]}, '
            f'distance={distance_m:.2f}m, '
            f'position=({x_m:.2f}m, {y_m:.2f}m, {z_m:.2f}m), '
            f'yaw={np.degrees(det["yaw"]):.1f}deg'
        )
        return response


# ===========================================================================
# LEVEL 3 — Entry point
# ===========================================================================

def main(args=None):
    """
    Start the ROS2 node with a MultiThreadedExecutor.

    A MultiThreadedExecutor runs callbacks in a thread pool. Combined with
    ReentrantCallbackGroup (declared in __init__), this means the camera
    callback keeps updating latest_frame even while the service callback is
    blocked waiting for the depth measurement.
    """
    rclpy.init(args=args)
    node = DetectStationService()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
