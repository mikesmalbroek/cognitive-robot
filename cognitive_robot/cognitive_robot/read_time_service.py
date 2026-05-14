"""
read_time_service.py

ROS2 service server that reads the time from a digital clock using OCR.

HOW IT FITS IN THE SYSTEM
--------------------------
The task planner navigates the robot to a station. Once there, it calls
the /read_time service (empty request). This node:

  1. Looks straight ahead and tries to read the clock with OCR.
  2. If that fails, it rotates slightly left or right and tries again.
  3. It keeps alternating, with increasing rotation angles, until it
     either finds a valid time or exhausts all attempts.
  4. Returns found=True + the four time digits, or found=False.

WHY THIS RUNS ON THE LAPTOP (NOT THE ROBOT)
--------------------------------------------
EasyOCR loads a neural-network model (~200 MB) and is too slow for the
robot's CPU. The laptop subscribes to the robot's camera stream over
the network and does all the heavy processing locally.

TOPICS USED
-----------
  /camera/color/image_raw  (subscribe) — front camera from the robot
  /cmd_vel                 (publish)   — rotation commands to the robot
"""

import math
import os
import re
import threading
import time
from datetime import datetime

import cv2
import easyocr
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from cognitive_robot_interfaces.srv import ReadTime


class ReadTimeService(Node):
    """
    ROS2 node that exposes the /read_time service.

    On startup it:
      - Loads the EasyOCR model (slow, so once only).
      - Subscribes to the front camera and caches the latest frame.
      - Creates a publisher for velocity commands (used to rotate the robot).
      - Declares ROS2 parameters so behaviour can be tuned without editing code.

    When the service is called it runs the scan-and-OCR loop and returns
    the result.
    """

    def __init__(self):
        """Initialise the node, load OCR model, set up publishers/subscribers."""
        super().__init__('read_time_service')

        # ------------------------------------------------------------------ #
        # ROS2 parameters — change these at launch time with:                 #
        #   --ros-args -p confidence_threshold:=0.8                           #
        # ------------------------------------------------------------------ #
        self.declare_parameter('step_degrees', 10)
        # How many degrees to add with each iteration (e.g. 0°, ±10°, ±20°…)

        self.declare_parameter('max_iterations', 10)
        # Maximum number of scan attempts before giving up.

        self.declare_parameter('rotation_speed', 0.5)
        # Angular speed in rad/s used when rotating the robot.

        self.declare_parameter('confidence_threshold', 0.7)
        # Minimum EasyOCR confidence (0.0–1.0) to accept a detection.

        self.declare_parameter('debug_save_dir', '~/ocr_debug')
        # Directory on the laptop where debug images are saved.

        self.declare_parameter('cmd_vel_topic', '/mirte_base_controller/cmd_vel')
        # Topic to publish rotation commands on.
        # Real robot : /mirte_base_controller/cmd_vel
        # Gazebo     : /cmd_vel

        # ------------------------------------------------------------------ #
        # Callback group                                                        #
        # ------------------------------------------------------------------ #
        # ReentrantCallbackGroup allows multiple callbacks to run at the same
        # time in the MultiThreadedExecutor.  This is essential: the service
        # callback calls time.sleep() while waiting for the robot to rotate,
        # and the camera callback must keep running during that sleep so that
        # self.latest_frame stays up to date.  Without this, both callbacks
        # would share a single thread and the camera would freeze during rotation.
        self._cb_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------ #
        # Camera subscription                                                  #
        # ------------------------------------------------------------------ #
        self.bridge = CvBridge()
        # CvBridge converts ROS Image messages to OpenCV (NumPy) arrays.

        self.latest_frame = None
        # Stores the most recent camera frame. Updated by _camera_callback.

        self._frame_lock = threading.Lock()
        # Protects latest_frame: the camera callback and service callback run
        # in different threads — we need a lock so they don't corrupt each other.

        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._camera_callback,
            10,
            callback_group=self._cb_group,
        )

        # ------------------------------------------------------------------ #
        # Velocity publisher (used to rotate the robot)                        #
        # ------------------------------------------------------------------ #
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.get_logger().info(f'Publishing rotation commands on: {cmd_vel_topic}')

        # ------------------------------------------------------------------ #
        # EasyOCR reader — loading the model takes ~10 seconds, so we do it   #
        # once here rather than on every service call.                          #
        # ------------------------------------------------------------------ #
        self.get_logger().info('Loading EasyOCR model (this takes ~10 s)…')
        self.reader = easyocr.Reader(['en'], gpu=False, verbose=False)
        self.get_logger().info('EasyOCR model loaded.')

        # ------------------------------------------------------------------ #
        # Debug output directory                                               #
        # ------------------------------------------------------------------ #
        raw_dir = self.get_parameter('debug_save_dir').get_parameter_value().string_value
        self.debug_dir = os.path.expanduser(raw_dir)
        os.makedirs(self.debug_dir, exist_ok=True)
        self.get_logger().info(f'Debug images will be saved to: {self.debug_dir}')

        # ------------------------------------------------------------------ #
        # Service server                                                        #
        # ------------------------------------------------------------------ #
        self.srv = self.create_service(
            ReadTime, '/read_time', self._handle_read_time,
            callback_group=self._cb_group,
        )
        self.get_logger().info('Service /read_time is ready.')

    # ---------------------------------------------------------------------- #
    # Camera callback                                                          #
    # ---------------------------------------------------------------------- #

    def _camera_callback(self, msg):
        """
        Called automatically by ROS every time a new camera frame arrives.

        Converts the ROS Image message to a BGR OpenCV image and stores it
        in self.latest_frame so the service callback can use it.

        Parameters
        ----------
        msg : sensor_msgs.msg.Image
            The raw image message published by the robot's front camera.
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._frame_lock:
            self.latest_frame = frame

    # ---------------------------------------------------------------------- #
    # Service callback                                                         #
    # ---------------------------------------------------------------------- #

    def _handle_read_time(self, request, response):
        """
        Main service callback — called when the task planner calls /read_time.

        Runs a zigzag scan: look straight → small rotation left → larger
        rotation right → etc., running OCR on each camera frame.  Stops as
        soon as a valid NN:NN time is found or max_iterations is reached.

        Parameters
        ----------
        request  : ReadTime.Request   (empty — no fields)
        response : ReadTime.Response  (found: bool, time_digits: int32[])

        Returns
        -------
        ReadTime.Response
            found=True  + time_digits=[d0,d1,d2,d3]  on success.
            found=False + time_digits=[]              on failure.
        """
        self.get_logger().info('Received /read_time request — starting scan.')

        # Read parameters (allows runtime tuning without restarting the node)
        step_deg   = self.get_parameter('step_degrees').get_parameter_value().integer_value
        max_iter   = self.get_parameter('max_iterations').get_parameter_value().integer_value
        threshold  = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # ------------------------------------------------------------------ #
        # State for this service call                                          #
        # ------------------------------------------------------------------ #
        num_iterations  = 0
        found           = False
        result_digits   = []
        left_or_right   = 1     # +1 = clockwise (right), flips each iteration
        current_heading = 0.0   # degrees we have rotated from the start

        # ------------------------------------------------------------------ #
        # Scan loop                                                            #
        # ------------------------------------------------------------------ #
        while num_iterations < max_iter and not found:

            # -- Step a: compute the target heading for this iteration ------
            # Pattern (with step_deg=10):
            #   iter 0 → target =   0° (look straight)
            #   iter 1 → target = −10° (small rotation)
            #   iter 2 → target = +20°
            #   iter 3 → target = −30°  … etc.
            target_heading = float(left_or_right * num_iterations * step_deg)

            # -- Step b: rotate by the delta from where we currently are ----
            # We track current_heading so we rotate the RIGHT amount.
            # E.g. if we are at −10° and the target is +20°, we must rotate
            # +30°, not just +20°.
            delta = target_heading - current_heading

            if abs(delta) > 0.01:  # skip if the rotation is negligible
                self.get_logger().info(
                    f'Iter {num_iterations}: rotating {delta:+.1f}° '
                    f'(target heading {target_heading:+.1f}°)'
                )
                self._rotate_robot(delta)

            current_heading = target_heading

            # -- Step c: wait for the camera to settle ----------------------
            # After rotation the image is briefly blurry/shaky.
            # 0.5 s is usually enough for the robot and camera to stabilise.
            time.sleep(0.5)

            # -- Step d: grab the latest camera frame -----------------------
            with self._frame_lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None

            if frame is None:
                self.get_logger().warn('No camera frame available yet — skipping iteration.')
                num_iterations += 1
                left_or_right  *= -1
                continue

            # -- Step e: run OCR --------------------------------------------
            # allowlist restricts recognition to digits and colons, which
            # reduces false positives from other characters.
            results = self.reader.readtext(frame, allowlist='0123456789:')

            # -- Step f: save debug image + sidecar text file ---------------
            self._save_debug(frame, results, num_iterations)

            # -- Step g: validate OCR results --------------------------------
            valid, digits = self._validate_ocr(results, threshold)

            if valid:
                found         = True
                result_digits = digits
                self.get_logger().info(f'Time found: {digits}')
            else:
                self.get_logger().info(f'Iter {num_iterations}: no valid time detected.')
                num_iterations += 1
                left_or_right  *= -1   # flip direction for the next attempt

        # ------------------------------------------------------------------ #
        # Build and return the response                                        #
        # ------------------------------------------------------------------ #
        response.found       = found
        response.time_digits = result_digits
        self.get_logger().info(
            f'Scan complete — found={found}, digits={result_digits}'
        )
        return response

    # ---------------------------------------------------------------------- #
    # Robot rotation helper                                                    #
    # ---------------------------------------------------------------------- #

    def _rotate_robot(self, degrees):
        """
        Rotate the robot in place by the given number of degrees.

        Publishes a Twist message with only angular.z set (no linear motion),
        waits for the estimated time to complete the rotation, then stops.

        This is open-loop (time-based), so it is approximate — exact accuracy
        is not required for the OCR scan.

        Parameters
        ----------
        degrees : float
            Positive = rotate clockwise (right) when viewed from above.
            Negative = rotate counter-clockwise (left).
        """
        speed = self.get_parameter('rotation_speed').get_parameter_value().double_value

        radians  = math.radians(degrees)
        duration = abs(radians) / speed  # time (seconds) = angle / angular_speed

        twist = Twist()
        # angular.z: positive = counter-clockwise in ROS convention (left),
        # negative = clockwise (right).  We negate so that positive degrees
        # means "turn right" (more intuitive for this application).
        twist.angular.z = -speed if degrees > 0 else speed

        self.get_logger().debug(
            f'Rotating {degrees:+.1f}° at {speed} rad/s for {duration:.2f} s'
        )

        rate_hz = 10.0
        dt = 1.0 / rate_hz
        end_time = time.time() + duration

        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(dt)

        # Send stop command multiple times to make sure robot stops
        stop = Twist()
        for _ in range(5):
        self.cmd_vel_pub.publish(stop)
        time.sleep(0.05)

    # ---------------------------------------------------------------------- #
    # OCR validation                                                           #
    # ---------------------------------------------------------------------- #

    def _validate_ocr(self, results, threshold):
        """
        Check whether the EasyOCR results contain exactly one valid time string.

        A detection is accepted only if:
          1. The text matches the pattern NN:NN  (exactly two digits, colon,
             two digits — e.g. '14:32').
          2. The confidence score is >= threshold.

        If there is exactly one accepted detection, extract the four digits.
        Zero or two+ accepted detections both count as failure (ambiguous).

        Parameters
        ----------
        results   : list of (bbox, str, float)
            Raw output from easyocr.Reader.readtext().
        threshold : float
            Minimum confidence to accept a detection (e.g. 0.7).

        Returns
        -------
        (bool, list[int])
            (True,  [d0, d1, d2, d3])  — exactly one valid match found.
            (False, [])                — zero or multiple matches.
        """
        time_pattern = re.compile(r'^\d{2}:\d{2}$')
        accepted = []

        for _bbox, text, confidence in results:
            text = text.strip()
            if time_pattern.match(text) and confidence >= threshold:
                accepted.append(text)

        if len(accepted) == 1:
            # Extract only the digit characters (skip the colon)
            digits = [int(c) for c in accepted[0] if c.isdigit()]
            return True, digits

        # 0 detections → nothing matched; 2+ → ambiguous
        return False, []

    # ---------------------------------------------------------------------- #
    # Debug image saving                                                       #
    # ---------------------------------------------------------------------- #

    def _save_debug(self, frame, results, iteration):
        """
        Save a debug image and a sidecar text file for one scan attempt.

        Files are named:
          attempt_<timestamp>_iter<N>.jpg   — the raw camera frame
          attempt_<timestamp>_iter<N>.txt   — OCR results + validation notes

        This lets you inspect failures offline by looking at what the camera
        saw and what EasyOCR returned.

        Parameters
        ----------
        frame     : numpy.ndarray   BGR image (OpenCV format).
        results   : list of (bbox, str, float)  Raw EasyOCR output.
        iteration : int             Which iteration this attempt belongs to.
        """
        threshold  = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        timestamp  = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        base_name  = f'attempt_{timestamp}_iter{iteration}'

        img_path   = os.path.join(self.debug_dir, base_name + '.jpg')
        txt_path   = os.path.join(self.debug_dir, base_name + '.txt')

        # Save the camera frame as JPEG.
        cv2.imwrite(img_path, frame)

        # Write a human-readable summary of the OCR results.
        time_pattern = re.compile(r'^\d{2}:\d{2}$')
        lines = [
            f'Iteration : {iteration}',
            f'Timestamp : {timestamp}',
            f'Threshold : {threshold}',
            '',
            '--- OCR detections ---',
        ]
        for i, (_bbox, text, confidence) in enumerate(results):
            text = text.strip()
            format_ok  = bool(time_pattern.match(text))
            conf_ok    = confidence >= threshold
            verdict    = 'ACCEPTED' if (format_ok and conf_ok) else 'rejected'
            lines.append(
                f'  [{i}] text="{text}"  confidence={confidence:.3f}  '
                f'format_ok={format_ok}  conf_ok={conf_ok}  → {verdict}'
            )

        if not results:
            lines.append('  (no detections)')

        with open(txt_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        self.get_logger().debug(f'Debug files saved: {base_name}')


# --------------------------------------------------------------------------- #
# Entry point                                                                   #
# --------------------------------------------------------------------------- #

def main(args=None):
    """
    Start the ROS2 node with a MultiThreadedExecutor.

    A MultiThreadedExecutor runs callbacks in a thread pool.  Combined with
    ReentrantCallbackGroup (set in __init__), this means the camera callback
    keeps receiving frames even while the service callback is sleeping during
    a rotation.  A SingleThreadedExecutor (the rclpy.spin default) would block
    on the first time.sleep() and camera frames would stop arriving.
    """
    rclpy.init(args=args)
    node = ReadTimeService()

    # The executor manages the thread pool.  By default it creates as many
    # threads as there are CPU cores, which is more than enough here.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # destroy_node and shutdown are safe to call here; if the executor
        # already triggered shutdown internally, the second call is a no-op.
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
