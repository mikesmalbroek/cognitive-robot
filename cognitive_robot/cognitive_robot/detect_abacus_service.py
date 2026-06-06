"""
detect_abacus_service.py

ROS2 service node that detects an abacus in the robot's front camera image
using the Roboflow serverless inference API, and measures its distance using
the depth camera.

HOW IT FITS IN THE SYSTEM
--------------------------
The task planner calls the /detect_abacus service (empty request). This node:

  1. Grabs the latest frame from the robot's front camera.
  2. Saves it as a temporary JPEG file on disk.
  3. Sends the image to the Roboflow serverless inference API.
  4. Picks the prediction with the highest confidence score.
  5. If the abacus is detected, reads the depth frame and measures distance.
  6. Returns the confidence, pixel coordinates, bounding box, and distance.

WHY THIS RUNS ON THE LAPTOP (NOT THE ROBOT)
--------------------------------------------
The Roboflow inference SDK sends images over HTTP to a cloud API.
The robot itself does not need to run the model — only the laptop needs
an internet connection and the inference_sdk package installed.

TOPICS USED
-----------
  /camera/color/image_raw  (subscribe) — colour camera from the robot
  /camera/depth/image_raw  (subscribe) — depth camera from the robot

SERVICE PROVIDED
----------------
  /detect_abacus  (cognitive_robot_interfaces/srv/DetectAbacus)
      Request  : (empty)
      Response : confidence  (float32)  0.0 means nothing detected
                 x           (int32)    pixel X of bounding box centre
                 y           (int32)    pixel Y of bounding box centre
                 bbox_width  (int32)    bounding box width in pixels
                 bbox_height (int32)    bounding box height in pixels
                 distance_m  (float32)  distance to abacus in metres
"""

import os
import tempfile
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image

from inference_sdk import InferenceHTTPClient

from cognitive_robot_interfaces.srv import DetectAbacus


class DetectAbacusService(Node):
    """
    ROS2 node that exposes the /detect_abacus service.

    On startup it:
      - Subscribes to the robot's front camera and caches every incoming frame.
      - Creates a Roboflow InferenceHTTPClient once (reused on every call).
      - Declares ROS2 parameters so behaviour can be tuned at launch time
        without editing this file.

    When the service is called it:
      1. Captures the latest camera frame.
      2. Saves it to a temporary JPEG file.
      3. Sends it to the Roboflow API.
      4. Extracts the best detection and fills in the response.
    """

    def __init__(self):
        """Initialise the node, create the inference client, set up subscribers/services."""
        super().__init__('detect_abacus_service')

        # ------------------------------------------------------------------ #
        # ROS2 parameters — override at launch time with e.g.:               #
        #   --ros-args -p confidence_threshold:=0.7                           #
        # ------------------------------------------------------------------ #
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        # Camera topic to subscribe to.
        # Real robot : /camera/color/image_raw
        # Gazebo     : /camera/image_raw

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        # Raw depth topic from the depth sensor (16UC1, values in mm).
        # The depth and colour cameras are a few cm apart, causing a small
        # pixel offset (~20-30 px at 1-2 m). This is acceptable for distance
        # and orientation estimation — no alignment node needed.

        self.declare_parameter('confidence_threshold', 0.7)
        # Minimum Roboflow confidence (0.0–1.0) to count a detection as valid.
        # Detections below this score are ignored and detected=False is returned.

        self.declare_parameter('api_url', 'https://serverless.roboflow.com')
        # Base URL for the Roboflow serverless inference API.

        self.declare_parameter('api_key', '8U4Olre0d5v9lWGCeHHT')
        # Your Roboflow API key. Keep this secret in a production environment.

        self.declare_parameter('model_id', 'abacus_recognition_v1/2')
        # Roboflow model ID in the format  <project-slug>/<version-number>.

        # ------------------------------------------------------------------ #
        # Callback group                                                       #
        # ------------------------------------------------------------------ #
        # ReentrantCallbackGroup lets the camera callback keep running while
        # the service callback is blocked waiting for the Roboflow HTTP response.
        # Without this, a SingleThreadedExecutor would freeze the camera during
        # every API call and latest_frame would go stale.
        self._cb_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------ #
        # Camera subscription                                                  #
        # ------------------------------------------------------------------ #
        self.bridge = CvBridge()
        # CvBridge converts ROS Image messages to OpenCV (NumPy) BGR arrays.

        self.latest_frame = None
        # The most recent camera frame. Updated by _camera_callback on every
        # incoming message. None until the first frame has arrived.

        self._frame_lock = threading.Lock()
        # Protects latest_frame: the camera callback and the service callback
        # run in different threads, so we need a lock to prevent data races.

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.create_subscription(
            Image,
            camera_topic,
            self._camera_callback,
            10,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f'Subscribing to camera on: {camera_topic}')

        # ------------------------------------------------------------------ #
        # Depth camera subscription                                            #
        # ------------------------------------------------------------------ #
        self.latest_depth_frame = None
        self._depth_frame_lock = threading.Lock()

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            10,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f'Subscribing to depth on: {depth_topic}')

        # ------------------------------------------------------------------ #
        # Camera model (image_geometry)                                        #
        # ------------------------------------------------------------------ #
        # PinholeCameraModel converts pixel coordinates + depth into real-world
        # 3D positions. It reads the camera intrinsics (focal length, principal
        # point, distortion) from the camera_info topic and is initialised once
        # on the first incoming message.
        self._camera_model = PinholeCameraModel()
        self._camera_model_ready = False
        # Flag so we only log "model ready" once instead of on every message.

        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self._camera_info_callback,
            10,
            callback_group=self._cb_group,
        )

        # ------------------------------------------------------------------ #
        # Roboflow inference client                                            #
        # ------------------------------------------------------------------ #
        # We create the client once here so it can be reused on every service
        # call. Creating a new client every call would re-negotiate the
        # connection each time and add unnecessary overhead.
        api_url = self.get_parameter('api_url').get_parameter_value().string_value
        api_key = self.get_parameter('api_key').get_parameter_value().string_value
        self._inference_client = InferenceHTTPClient(api_url=api_url, api_key=api_key)
        self.get_logger().info(f'Roboflow inference client created (api_url={api_url})')

        # ------------------------------------------------------------------ #
        # Service server                                                       #
        # ------------------------------------------------------------------ #
        self.srv = self.create_service(
            DetectAbacus,
            '/detect_abacus',
            self._handle_detect_abacus,
            callback_group=self._cb_group,
        )
        self.get_logger().info('Service /detect_abacus is ready.')

    # ---------------------------------------------------------------------- #
    # Camera callback                                                          #
    # ---------------------------------------------------------------------- #

    def _camera_callback(self, msg):
        """
        Called automatically by ROS every time a new camera frame arrives.

        Converts the ROS Image message to a BGR OpenCV image and stores it
        in self.latest_frame so the service callback can pick it up later.

        Parameters
        ----------
        msg : sensor_msgs.msg.Image
            The raw image message published by the robot's front camera.
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._frame_lock:
            self.latest_frame = frame

    def _depth_callback(self, msg):
        """Cache the latest depth frame (16UC1, values in mm)."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self._depth_frame_lock:
            self.latest_depth_frame = frame

    def _camera_info_callback(self, msg):
        """
        Initialise the PinholeCameraModel from the colour camera_info.

        Called automatically on every camera_info message, but the model only
        needs to be set up once — the intrinsics don't change during a session.
        After the first message the flag _camera_model_ready is set to True so
        _project_to_3d knows it can safely use the model.
        """
        if not self._camera_model_ready:
            self._camera_model.fromCameraInfo(msg)
            self._camera_model_ready = True
            self.get_logger().info(
                f'Camera model ready — '
                f'fx={self._camera_model.fx():.1f}, '
                f'fy={self._camera_model.fy():.1f}, '
                f'cx={self._camera_model.cx():.1f}, '
                f'cy={self._camera_model.cy():.1f}'
            )

    # ---------------------------------------------------------------------- #
    # Base functions                                                           #
    # ---------------------------------------------------------------------- #

    def _capture_frame(self):
        """
        Return a thread-safe copy of the most recent camera frame.

        We copy the frame while holding the lock so that _camera_callback
        cannot overwrite it in another thread while we are processing it.

        Returns
        -------
        numpy.ndarray or None
            A BGR OpenCV image, or None if no frame has arrived yet.
        """
        with self._frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def _save_temp_image(self, frame):
        """
        Save a camera frame as a temporary JPEG file on disk.

        The Roboflow inference SDK accepts a file path as input.
        We write to a fixed path in the system temp directory and overwrite it
        on every call, so we never accumulate old frames on disk.

        Parameters
        ----------
        frame : numpy.ndarray
            BGR image in OpenCV format (as returned by _capture_frame).

        Returns
        -------
        str
            Absolute path to the saved JPEG file.
        """
        path = os.path.join(tempfile.gettempdir(), 'detect_abacus_temp.jpg')
        cv2.imwrite(path, frame)
        self.get_logger().debug(f'Temp image saved to: {path}')
        return path

    def _run_inference(self, image_path):
        """
        Send the image to the Roboflow API and return the raw prediction list.

        The Roboflow response looks like:
          {
            "predictions": [
              {
                "x": 320,          <- centre X of bounding box (pixels)
                "y": 240,          <- centre Y of bounding box (pixels)
                "width": 100,
                "height": 80,
                "confidence": 0.91,
                "class": "abacus"
              },
              ...
            ]
          }

        Parameters
        ----------
        image_path : str
            Absolute path to the JPEG file to send (as returned by
            _save_temp_image).

        Returns
        -------
        list[dict]
            List of prediction dicts from Roboflow, or an empty list when the
            API call fails. An empty list means detected=False downstream.
        """
        model_id = self.get_parameter('model_id').get_parameter_value().string_value

        try:
            result = self._inference_client.infer(image_path, model_id=model_id)
            predictions = result.get('predictions', [])
            self.get_logger().debug(f'Roboflow returned {len(predictions)} prediction(s).')
            return predictions
        except Exception as exc:
            # Log the error but do not crash the node — return an empty list
            # so the service can still send a valid detected=False response.
            self.get_logger().error(f'Roboflow API call failed: {exc}')
            return []

    def _extract_best_detection(self, predictions):
        """
        Select the prediction with the highest confidence score.

        A detection is only accepted when its confidence is at or above the
        configured threshold parameter. If no predictions arrive, or the best
        one is below the threshold, this function returns detected=False.

        Parameters
        ----------
        predictions : list[dict]
            Raw prediction dicts as returned by _run_inference.

        Returns
        -------
        tuple : (float, int, int, int, int)
            (confidence, x, y, bbox_width, bbox_height)

            confidence  : Score of the best prediction (0.0 if nothing found).
            x           : Pixel X of the bounding box centre (0 if confidence 0.0).
            y           : Pixel Y of the bounding box centre (0 if confidence 0.0).
            bbox_width  : Bounding box width in pixels (0 if confidence 0.0).
            bbox_height : Bounding box height in pixels (0 if confidence 0.0).
        """
        if not predictions:
            self.get_logger().info('No predictions returned by the API.')
            return 0.0, 0, 0, 0, 0

        # Pick the prediction the model is most certain about.
        best = max(predictions, key=lambda p: p['confidence'])

        threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        if best['confidence'] < threshold:
            self.get_logger().info(
                f'Best confidence {best["confidence"]:.2f} is below '
                f'threshold {threshold:.2f} — returning 0.0.'
            )
            return 0.0, 0, 0, 0, 0

        return (
            float(best['confidence']),
            int(best['x']),
            int(best['y']),
            int(best['width']),
            int(best['height']),
        )

    def _save_depth_image(self, depth_frame, x, y, bbox_width, bbox_height):
        """
        Save the depth frame as two images in ~/depth_photos/ when an abacus
        is detected. Also draws the bounding box so you can see where the
        abacus was found.

          depth_HHMMSS_raw.png   — grayscale (brighter = further away)
          depth_HHMMSS_color.png — false colour (blue = close, red = far)
        """
        save_dir = os.path.expanduser('~/depth_photos')
        os.makedirs(save_dir, exist_ok=True)

        # Normalize uint16 (mm) to uint8 (0-255) for saving.
        normalized = np.zeros(depth_frame.shape, dtype=np.uint8)
        valid_mask = depth_frame > 0
        if valid_mask.any():
            normalized[valid_mask] = cv2.normalize(
                depth_frame[valid_mask], None, 0, 255, cv2.NORM_MINMAX
            ).flatten()

        # Draw bounding box on both images so you can see where the abacus is.
        half_w = bbox_width // 2
        half_h = bbox_height // 2
        x1, y1 = x - half_w, y - half_h
        x2, y2 = x + half_w, y + half_h

        timestamp = time.strftime('%H%M%S')

        raw_img = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(raw_img, (x1, y1), (x2, y2), (255, 255, 255), 2)
        raw_path = os.path.join(save_dir, f'depth_{timestamp}_raw.png')
        cv2.imwrite(raw_path, raw_img)

        color_img = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        color_img[~valid_mask] = [0, 0, 0]
        cv2.rectangle(color_img, (x1, y1), (x2, y2), (255, 255, 255), 2)
        color_path = os.path.join(save_dir, f'depth_{timestamp}_color.png')
        cv2.imwrite(color_path, color_img)

        self.get_logger().info(f'Depth images saved: {raw_path}, {color_path}')

    def _capture_depth_frame(self):
        """Return a thread-safe copy of the latest aligned depth frame, or None."""
        with self._depth_frame_lock:
            if self.latest_depth_frame is None:
                return None
            return self.latest_depth_frame.copy()

    def _sample_depth(self, depth_frame, x, y, radius=5):
        """
        Return the median depth in metres around pixel (x, y).

        Uses a small window to filter out invalid (zero) pixels that the
        depth sensor could not measure. Returns 0.0 when no valid pixels
        are found in the window.

        Parameters
        ----------
        depth_frame : numpy.ndarray  (uint16, values in mm)
        x, y        : int  pixel coordinates
        radius      : int  half-size of the sampling window

        Returns
        -------
        float  distance in metres, or 0.0 if unmeasurable.
        """
        h, w = depth_frame.shape
        x_min = max(0, x - radius)
        x_max = min(w, x + radius + 1)
        y_min = max(0, y - radius)
        y_max = min(h, y + radius + 1)
        region = depth_frame[y_min:y_max, x_min:x_max].astype(np.float32)
        valid = region[region > 0]
        if len(valid) == 0:
            return 0.0
        return float(np.median(valid)) / 1000.0

    def _project_to_3d(self, pixel_x, pixel_y, distance_m):
        """
        Convert a pixel coordinate + depth into a real-world 3D position.

        Uses image_geometry.PinholeCameraModel which applies the full pinhole
        camera model including lens distortion. The model must be initialised
        first via _camera_info_callback.

        How it works
        ------------
        1. projectPixelTo3dRay() shoots a ray from the camera origin through
           the given pixel. The ray is a unit direction vector (dx, dy, dz).
        2. Multiplying that ray by the measured depth gives the actual 3D
           position of the point in the camera's coordinate frame.

        Coordinate frame (camera optical frame)
        ----------------------------------------
          x_m : positive = to the right of the camera centre
          y_m : positive = below the camera centre
          z_m = distance_m (depth, already measured separately)

        Parameters
        ----------
        pixel_x, pixel_y : int   pixel coordinates from Roboflow detection
        distance_m       : float depth in metres from _sample_depth()

        Returns
        -------
        tuple (float, float) — (x_m, y_m), or (0.0, 0.0) if model not ready.
        """
        if not self._camera_model_ready or distance_m == 0.0:
            return 0.0, 0.0

        # projectPixelTo3dRay returns a normalised direction vector.
        # Scaling it by depth gives the real-world position.
        ray = self._camera_model.projectPixelTo3dRay((pixel_x, pixel_y))
        x_m = ray[0] * distance_m
        y_m = ray[1] * distance_m
        return float(x_m), float(y_m)

    # ---------------------------------------------------------------------- #
    # Service callback                                                         #
    # ---------------------------------------------------------------------- #

    def _handle_detect_abacus(self, request, response):
        """
        Main service callback — called when the task planner calls /detect_abacus.

        This function is the 'director': it calls the base functions in order
        and assembles the final response. It does not contain any heavy logic
        itself; all real work is delegated to the base functions above.

        Execution order
        ---------------
        1. _capture_frame()          → get the latest camera image
        2. _save_temp_image(frame)   → write it to a temp JPEG file
        3. _run_inference(path)      → send to Roboflow, get predictions
        4. _extract_best_detection() → pick best result above threshold
        5. Fill in response fields and return.

        Parameters
        ----------
        request  : DetectAbacus.Request   (empty — no fields)
        response : DetectAbacus.Response
            confidence  : float32  (0.0 means nothing detected)
            x           : int32    (bounding box centre X, pixels)
            y           : int32    (bounding box centre Y, pixels)
            bbox_width  : int32    (bounding box width, pixels)
            bbox_height : int32    (bounding box height, pixels)
            distance_m  : float32  (distance to abacus in metres)

        Returns
        -------
        DetectAbacus.Response
        """
        self.get_logger().info('Received /detect_abacus request.')

        # Step 1: grab the latest camera frame.
        frame = self._capture_frame()
        if frame is None:
            self.get_logger().warn('No camera frame available yet — returning confidence=0.0.')
            response.confidence  = 0.0
            response.x           = 0
            response.y           = 0
            response.bbox_width  = 0
            response.bbox_height = 0
            response.distance_m  = 0.0
            response.x_m         = 0.0
            response.y_m         = 0.0
            return response

        # Step 2: save the frame as a temporary JPEG so the API can read it.
        image_path = self._save_temp_image(frame)

        # Step 3: send the image to the Roboflow inference API.
        predictions = self._run_inference(image_path)

        # Step 4: extract the highest-confidence detection.
        confidence, x, y, bbox_width, bbox_height = self._extract_best_detection(predictions)

        # Step 5: fill detection fields (depth fields default to 0.0).
        response.confidence  = confidence
        response.x           = x
        response.y           = y
        response.bbox_width  = bbox_width
        response.bbox_height = bbox_height
        response.distance_m  = 0.0
        response.x_m         = 0.0
        response.y_m         = 0.0

        if confidence == 0.0:
            return response

        # Step 6: measure distance from the depth frame.
        depth_frame = self._capture_depth_frame()
        if depth_frame is not None:
            response.distance_m = self._sample_depth(depth_frame, x, y)
            self._save_depth_image(depth_frame, x, y, bbox_width, bbox_height)
        else:
            self.get_logger().warn('No depth frame available — distance not measured.')

        # Step 7: calculate real-world X/Y position using image_geometry.
        response.x_m, response.y_m = self._project_to_3d(x, y, response.distance_m)

        self.get_logger().info(
            f'Result — confidence={confidence:.2f}, pixel=({x},{y}), '
            f'distance={response.distance_m:.2f}m, '
            f'position=({response.x_m:.2f}m, {response.y_m:.2f}m)'
        )
        return response


# --------------------------------------------------------------------------- #
# Entry point                                                                   #
# --------------------------------------------------------------------------- #

def main(args=None):
    """
    Start the ROS2 node with a MultiThreadedExecutor.

    A MultiThreadedExecutor runs callbacks in a thread pool. Combined with
    ReentrantCallbackGroup (declared in __init__), this means the camera
    callback keeps updating latest_frame even while the service callback is
    blocked waiting for the Roboflow HTTP response.

    A SingleThreadedExecutor (the rclpy.spin default) would block on the
    HTTP call and camera frames would stop arriving during inference.
    """
    rclpy.init(args=args)
    node = DetectAbacusService()

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
