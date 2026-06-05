"""
detect_abacus_service.py

ROS2 service node that detects an abacus in the robot's front camera image
using the Roboflow serverless inference API.

HOW IT FITS IN THE SYSTEM
--------------------------
The task planner calls the /detect_abacus service (empty request). This node:

  1. Grabs the latest frame from the robot's front camera.
  2. Saves it as a temporary JPEG file on disk.
  3. Sends the image to the Roboflow serverless inference API.
  4. Picks the prediction with the highest confidence score.
  5. Returns detected=True/False, the confidence score, and the pixel
     coordinates (x, y) of the bounding box centre.

WHY THIS RUNS ON THE LAPTOP (NOT THE ROBOT)
--------------------------------------------
The Roboflow inference SDK sends images over HTTP to a cloud API.
The robot itself does not need to run the model — only the laptop needs
an internet connection and the inference_sdk package installed.

TOPICS USED
-----------
  /camera/color/image_raw  (subscribe) — front camera from the robot

SERVICE PROVIDED
----------------
  /detect_abacus  (cognitive_robot_interfaces/srv/DetectAbacus)
      Request  : (empty)
      Response : detected   (bool)
                 confidence (float32)
                 x          (int32)   pixel X of bounding box centre
                 y          (int32)   pixel Y of bounding box centre
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
from sensor_msgs.msg import Image

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

        self.declare_parameter('confidence_threshold', 0.5)
        # Minimum Roboflow confidence (0.0–1.0) to count a detection as valid.
        # Detections below this score are ignored and detected=False is returned.

        self.declare_parameter('api_url', 'https://serverless.roboflow.com')
        # Base URL for the Roboflow serverless inference API.

        self.declare_parameter('api_key', '8U4Olre0d5v9lWGCeHHT')
        # Your Roboflow API key. Keep this secret in a production environment.

        self.declare_parameter('model_id', 'abacus_recognition_v1/1')
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
        tuple : (bool, float, int, int)
            (detected, confidence, x, y)

            detected   : True if a valid detection was found above threshold.
            confidence : Confidence of the best prediction (0.0 if none found).
            x          : Pixel X of the bounding box centre (0 if not detected).
            y          : Pixel Y of the bounding box centre (0 if not detected).
        """
        if not predictions:
            self.get_logger().info('No predictions returned by the API.')
            return False, 0.0, 0, 0

        # Pick the prediction the model is most certain about.
        best = max(predictions, key=lambda p: p['confidence'])

        threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        if best['confidence'] < threshold:
            self.get_logger().info(
                f'Best confidence {best["confidence"]:.2f} is below '
                f'threshold {threshold:.2f} — treating as not detected.'
            )
            return False, float(best['confidence']), 0, 0

        return True, float(best['confidence']), int(best['x']), int(best['y'])

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
            detected   : bool
            confidence : float32
            x          : int32  (bounding box centre X, pixels)
            y          : int32  (bounding box centre Y, pixels)

        Returns
        -------
        DetectAbacus.Response
        """
        self.get_logger().info('Received /detect_abacus request.')

        # Step 1: grab the latest camera frame.
        frame = self._capture_frame()
        if frame is None:
            self.get_logger().warn('No camera frame available yet — returning detected=False.')
            response.detected   = False
            response.confidence = 0.0
            response.x          = 0
            response.y          = 0
            return response

        # Step 2: save the frame as a temporary JPEG so the API can read it.
        image_path = self._save_temp_image(frame)

        # Step 3: send the image to the Roboflow inference API.
        predictions = self._run_inference(image_path)

        # Step 4: extract the highest-confidence detection.
        detected, confidence, x, y = self._extract_best_detection(predictions)

        # Step 5: fill in the service response.
        response.detected   = detected
        response.confidence = confidence
        response.x          = x
        response.y          = y

        self.get_logger().info(
            f'Result — detected={detected}, confidence={confidence:.2f}, x={x}, y={y}'
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
