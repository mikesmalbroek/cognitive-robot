"""
detect_abacus_service.py

ROS2 service node that detects an abacus in the robot's front camera image
using the Roboflow serverless inference API, and measures its distance and
3D position using the depth camera.

HOW IT FITS IN THE SYSTEM
--------------------------
The task planner calls the /detect_abacus service (empty request). This node:

  1. Grabs the latest frame from the robot's front camera.
  2. Saves it as a temporary JPEG file on disk.
  3. Sends the image to the Roboflow serverless inference API.
  4. Picks the prediction with the highest confidence score.
  5. If the abacus is detected, uses DepthCameraMixin to measure:
       - distance_m : depth to the abacus in metres
       - x_m, y_m   : real-world lateral and vertical offset in metres
  6. Returns confidence, pixel coordinates, bounding box, and 3D position.

Depth camera logic lives in DepthCameraMixin (depth_utils.py) and is shared
with DetectStationService so it only needs to be maintained in one place.

WHY THIS RUNS ON THE LAPTOP (NOT THE ROBOT)
--------------------------------------------
The Roboflow inference SDK sends images over HTTP to a cloud API.
The robot itself does not need to run the model — only the laptop needs
an internet connection and the inference_sdk package installed.

TOPICS USED
-----------
  /camera/color/image_raw      (subscribe) — colour camera from the robot
  /camera/depth/image_raw      (subscribe) — depth camera  (via DepthCameraMixin)
  /camera/color/camera_info    (subscribe) — intrinsics    (via DepthCameraMixin)

SERVICE PROVIDED
----------------
  /detect_abacus  (cognitive_robot_interfaces/srv/DetectAbacus)
      Request  : (empty)
      Response : confidence  (float32)  0.0 means nothing detected
                 x           (int32)    pixel X of bounding box centre
                 y           (int32)    pixel Y of bounding box centre
                 bbox_width  (int32)    bounding box width in pixels
                 bbox_height (int32)    bounding box height in pixels
                 distance_m  (float32)  distance to abacus in metres (= z_m)
                 x_m         (float32)  real-world right offset in metres
                 y_m         (float32)  real-world down  offset in metres
"""

import os
import tempfile
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from inference_sdk import InferenceHTTPClient

from cognitive_robot_interfaces.srv import DetectAbacus
from cognitive_robot.depth_utils import DepthCameraMixin


class DetectAbacusService(Node, DepthCameraMixin):
    """
    ROS2 node that exposes the /detect_abacus service.

    On startup it:
      - Subscribes to the robot's front camera and caches every incoming frame.
      - Sets up depth camera subscriptions via DepthCameraMixin.
      - Creates a Roboflow InferenceHTTPClient once (reused on every call).
      - Declares ROS2 parameters so behaviour can be tuned at launch time
        without editing this file.

    When the service is called it:
      1. Captures the latest colour frame.
      2. Saves it to a temporary JPEG file.
      3. Sends it to the Roboflow API.
      4. Extracts the best detection.
      5. Measures depth and calculates 3D position via DepthCameraMixin.
      6. Fills in and returns the response.
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

        self.declare_parameter('confidence_threshold', 0.7)
        # Minimum Roboflow confidence (0.0–1.0) to count a detection as valid.

        self.declare_parameter('api_url', 'https://serverless.roboflow.com')
        # Base URL for the Roboflow serverless inference API.

        self.declare_parameter('api_key', '8U4Olre0d5v9lWGCeHHT')
        # Your Roboflow API key. Keep this secret in a production environment.

        self.declare_parameter('model_id', 'abacus_recognition_v1/2')
        # Roboflow model ID in the format <project-slug>/<version-number>.

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
        # the service callback is blocked waiting for the Roboflow HTTP response.
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
        # Roboflow inference client                                            #
        # ------------------------------------------------------------------ #
        # Created once here so the TCP connection is reused on every call.
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
        Called automatically by ROS every time a new colour camera frame arrives.

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
        # Resize to 320x240 before saving — reduces upload size by 4x while
        # keeping enough resolution for the Roboflow model to detect the abacus.
        small = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
        path = os.path.join(tempfile.gettempdir(), 'detect_abacus_temp.jpg')
        cv2.imwrite(path, small)
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
            Absolute path to the JPEG file to send.

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
            # Log but do not crash — return empty list so the service can
            # still send a valid detected=False response.
            self.get_logger().error(f'Roboflow API call failed: {exc}')
            return []

    def _extract_best_detection(self, predictions):
        """
        Select the prediction with the highest confidence score.

        A detection is only accepted when its confidence is at or above the
        configured threshold. If no predictions arrive, or the best one is
        below the threshold, this function returns confidence=0.0.

        Parameters
        ----------
        predictions : list[dict]
            Raw prediction dicts as returned by _run_inference.

        Returns
        -------
        tuple : (float, int, int, int, int)
            (confidence, x, y, bbox_width, bbox_height)
            All zeros when nothing is detected.
        """
        if not predictions:
            self.get_logger().info('No predictions returned by the API.')
            return 0.0, 0, 0, 0, 0

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

    # ---------------------------------------------------------------------- #
    # Service callback                                                         #
    # ---------------------------------------------------------------------- #

    def _handle_detect_abacus(self, request, response):
        """
        Main service callback — called when the task planner calls /detect_abacus.

        This function is the 'director': it calls the base functions in order
        and assembles the final response. All heavy logic is delegated to the
        base functions above and to DepthCameraMixin.

        Execution order
        ---------------
        1. _capture_frame()                       → get the latest colour image
        2. _save_temp_image(frame)                → write it to a temp JPEG file
        3. _run_inference(path)                   → send to Roboflow, get predictions
        4. _extract_best_detection(predictions)   → pick best result above threshold
        5. _capture_depth_frame()                 → get the latest depth image
        6. _sample_depth(depth_frame, x, y)       → median depth in metres
        7. _project_to_3d(x, y, distance_m)       → real-world x_m, y_m, z_m
        8. _save_depth_image(...)                 → save debug depth images

        Parameters
        ----------
        request  : DetectAbacus.Request   (empty — no fields)
        response : DetectAbacus.Response
            confidence  : float32  (0.0 means nothing detected)
            x           : int32    (bounding box centre X, pixels)
            y           : int32    (bounding box centre Y, pixels)
            bbox_width  : int32    (bounding box width, pixels)
            bbox_height : int32    (bounding box height, pixels)
            distance_m  : float32  (distance to abacus in metres, = z_m)
            x_m         : float32  (real-world right offset in metres)
            y_m         : float32  (real-world down  offset in metres)

        Returns
        -------
        DetectAbacus.Response
        """
        self.get_logger().info('Received /detect_abacus request.')

        # Step 1: grab the latest colour camera frame.
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

        # Fill detection fields; depth fields default to 0.0.
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

        # Step 5 & 6: measure depth at the detection centre (via DepthCameraMixin).
        depth_frame = self._capture_depth_frame()
        if depth_frame is None:
            self.get_logger().warn('No depth frame available — distance not measured.')
            return response

        # Roboflow ran on a 320x240 resized image, so its pixel coordinates are in
        # that reduced space. The depth image is the original camera resolution.
        # Scale back so depth sampling and 3D projection use the correct pixel.
        scale_x = frame.shape[1] / 320.0
        scale_y = frame.shape[0] / 240.0
        depth_x = int(x * scale_x)
        depth_y = int(y * scale_y)

        response.distance_m = self._sample_depth(depth_frame, depth_x, depth_y, radius=30)

        # Step 7: calculate real-world 3D position (via DepthCameraMixin).
        response.x_m, response.y_m, _ = self._project_to_3d(depth_x, depth_y, response.distance_m)

        # Step 8: save debug depth images.
        self._save_depth_image(
            depth_frame, depth_x, depth_y,
            int(bbox_width * scale_x), int(bbox_height * scale_y),
            label='abacus',
        )

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
