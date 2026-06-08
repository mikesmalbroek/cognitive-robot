"""
depth_utils.py

Shared depth camera mixin for ROS2 nodes that need depth measurements.

HOW IT FITS IN THE SYSTEM
--------------------------
Both DetectAbacusService and DetectStationService need to measure the
distance to a detected object and project its pixel position to a real-world
3D coordinate. This mixin centralises that logic so it only lives in one place.

HOW TO USE
----------
  1. Inherit from both Node and DepthCameraMixin.
  2. In __init__, call self._setup_depth_subscriptions(cb_group) after
     creating your ReentrantCallbackGroup.

  Example:

      class MyService(Node, DepthCameraMixin):
          def __init__(self):
              super().__init__('my_service')
              cb_group = ReentrantCallbackGroup()
              self._setup_depth_subscriptions(cb_group)

TOPICS SUBSCRIBED
-----------------
  /camera/depth/image_raw     — raw depth frames  (uint16, values in mm)
  /camera/color/camera_info   — camera intrinsics (focal length, principal
                                point, distortion) used for 3D projection

OUTPUT OF _project_to_3d
-------------------------
  x_m : offset to the right of the camera centre  (positive = right)
  y_m : offset below the camera centre             (positive = down)
  z_m : forward distance                           (equals distance_m)
"""

import os
import threading
import time

import cv2
import numpy as np
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class DepthCameraMixin:
    """
    Mixin that adds depth camera capabilities to a ROS2 Node.

    Not a Node itself — designed for multiple inheritance:
        class MyService(Node, DepthCameraMixin): ...

    After calling _setup_depth_subscriptions() in __init__, this mixin
    provides:
      - Continuously cached depth frames via a ROS2 subscription.
      - PinholeCameraModel initialised from real camera_info intrinsics.
      - Helper methods for depth sampling, 3D projection, and debug saving.
    """

    def _setup_depth_subscriptions(self, cb_group,
                                   depth_topic='/camera/depth/image_raw',
                                   camera_info_topic='/camera/color/camera_info'):
        """
        Subscribe to the depth image and colour camera_info topics.

        Must be called once from __init__ of the inheriting class, after
        the Node super().__init__() and after creating the callback group.

        Parameters
        ----------
        cb_group          : ReentrantCallbackGroup
            The callback group of the inheriting Node so the depth callback
            keeps running while the service callback is blocked.
        depth_topic       : str
            ROS2 topic for raw depth frames (uint16, values in mm).
            Default: /camera/depth/image_raw
        camera_info_topic : str
            ROS2 topic for camera intrinsics.
            Default: /camera/color/camera_info
        """
        self._depth_bridge = CvBridge()

        # Most recent depth frame, continuously updated by _depth_callback.
        self._latest_depth_frame = None
        self._depth_frame_lock = threading.Lock()

        # PinholeCameraModel for accurate pixel → 3D projection.
        # Initialised once when the first camera_info message arrives.
        self._camera_model = PinholeCameraModel()
        self._camera_model_ready = False
        self._camera_info_msg = None  # stored so distortion coefficients are accessible

        self.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
            callback_group=cb_group,
        )
        self.get_logger().info(f'[DepthMixin] Subscribing to depth on: {depth_topic}')

        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
            callback_group=cb_group,
        )
        self.get_logger().info(f'[DepthMixin] Subscribing to camera_info on: {camera_info_topic}')

    # ---------------------------------------------------------------------- #
    # ROS callbacks                                                            #
    # ---------------------------------------------------------------------- #

    def _depth_callback(self, msg):
        """Cache the latest depth frame (16UC1, values in mm)."""
        frame = self._depth_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self._depth_frame_lock:
            self._latest_depth_frame = frame

    def _camera_info_callback(self, msg):
        """
        Initialise the PinholeCameraModel from the colour camera_info.

        Only runs once — the camera intrinsics do not change during a session.
        After the first message, _camera_model_ready is True and _project_to_3d
        can safely use the model.
        """
        if not self._camera_model_ready:
            self._camera_model.fromCameraInfo(msg)
            self._camera_info_msg = msg  # stored for distortion coefficient access
            self._camera_model_ready = True
            self.get_logger().info(
                f'[DepthMixin] Camera model ready — '
                f'fx={self._camera_model.fx():.1f}, '
                f'fy={self._camera_model.fy():.1f}, '
                f'cx={self._camera_model.cx():.1f}, '
                f'cy={self._camera_model.cy():.1f}'
            )

    # ---------------------------------------------------------------------- #
    # Public helpers                                                           #
    # ---------------------------------------------------------------------- #

    def _capture_depth_frame(self):
        """Return a thread-safe copy of the latest depth frame, or None."""
        with self._depth_frame_lock:
            if self._latest_depth_frame is None:
                return None
            return self._latest_depth_frame.copy()

    def _sample_depth(self, depth_frame, x, y, radius=5):
        """
        Return the median depth in metres around pixel (x, y).

        Uses a small window to filter out invalid (zero) pixels that the
        depth sensor could not measure. Returns 0.0 when no valid pixels
        are found in the window.

        Parameters
        ----------
        depth_frame : numpy.ndarray  (uint16, values in mm)
        x, y        : int  pixel coordinates of the point to measure
        radius      : int  half-size of the sampling window (default 5)

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
            # Log what the depth image actually looks like to diagnose the problem.
            all_valid = depth_frame[depth_frame > 0]
            if len(all_valid) > 0:
                self.get_logger().warn(
                    f'[DepthMixin] No depth at ({x},{y}) radius={radius} '
                    f'— depth frame has valid pixels elsewhere '
                    f'(range {int(all_valid.min())}–{int(all_valid.max())} mm, '
                    f'depth size={depth_frame.shape})'
                )
            else:
                self.get_logger().warn(
                    f'[DepthMixin] No depth at ({x},{y}) — depth frame is entirely zero '
                    f'(size={depth_frame.shape})'
                )
            return 0.0
        return float(np.median(valid)) / 1000.0

    def _project_to_3d(self, pixel_x, pixel_y, distance_m):
        """
        Convert a pixel coordinate + depth into a real-world 3D position.

        Uses image_geometry.PinholeCameraModel with real camera intrinsics
        (including lens distortion) for accurate results.

        How it works
        ------------
        projectPixelTo3dRay() shoots a unit direction ray from the camera
        origin through the given pixel. Multiplying by distance_m gives
        the real-world 3D position in the camera optical frame.

        Coordinate frame (camera optical frame)
        ----------------------------------------
          x_m : positive = to the right of the camera centre
          y_m : positive = below the camera centre
          z_m : positive = forward (equals distance_m)

        Parameters
        ----------
        pixel_x, pixel_y : int    pixel coordinates of the detected object
        distance_m       : float  depth in metres from _sample_depth()

        Returns
        -------
        tuple (float, float, float) — (x_m, y_m, z_m)
            Returns (0.0, 0.0, 0.0) if the camera model is not ready
            or distance_m is 0.0.
        """
        if not self._camera_model_ready or distance_m == 0.0:
            return 0.0, 0.0, 0.0

        # projectPixelTo3dRay returns a normalised direction vector.
        # Scaling it by depth gives the real-world 3D position.
        ray = self._camera_model.projectPixelTo3dRay((pixel_x, pixel_y))
        x_m = float(ray[0] * distance_m)
        y_m = float(ray[1] * distance_m)
        z_m = float(distance_m)  # forward distance in the camera frame
        return x_m, y_m, z_m

    def _save_depth_image(self, depth_frame, x, y, bbox_width, bbox_height, label='object'):
        """
        Save the depth frame as two debug images when an object is detected.

        Files are saved to ~/depth_photos/:
          depth_HHMMSS_<label>_raw.png   — greyscale  (brighter = further away)
          depth_HHMMSS_<label>_color.png — false colour (blue = close, red = far)

        The bounding box is drawn on both images so you can verify the
        position where depth was sampled.

        Parameters
        ----------
        depth_frame : numpy.ndarray  uint16 depth frame (values in mm)
        x, y        : int            pixel coordinates of the detection centre
        bbox_width  : int            bounding box width  in pixels
        bbox_height : int            bounding box height in pixels
        label       : str            short name added to the filename (default 'object')
        """
        save_dir = os.path.expanduser('~/depth_photos')
        os.makedirs(save_dir, exist_ok=True)

        # Normalise uint16 (mm) to uint8 (0–255) for saving.
        normalized = np.zeros(depth_frame.shape, dtype=np.uint8)
        valid_mask = depth_frame > 0
        if valid_mask.any():
            normalized[valid_mask] = cv2.normalize(
                depth_frame[valid_mask], None, 0, 255, cv2.NORM_MINMAX
            ).flatten()

        half_w = bbox_width // 2
        half_h = bbox_height // 2
        x1, y1 = x - half_w, y - half_h
        x2, y2 = x + half_w, y + half_h

        timestamp = time.strftime('%H%M%S')

        raw_img = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(raw_img, (x1, y1), (x2, y2), (255, 255, 255), 2)
        raw_path = os.path.join(save_dir, f'depth_{timestamp}_{label}_raw.png')
        cv2.imwrite(raw_path, raw_img)

        color_img = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        color_img[~valid_mask] = [0, 0, 0]
        cv2.rectangle(color_img, (x1, y1), (x2, y2), (255, 255, 255), 2)
        color_path = os.path.join(save_dir, f'depth_{timestamp}_{label}_color.png')
        cv2.imwrite(color_path, color_img)

        self.get_logger().info(f'[DepthMixin] Depth images saved: {raw_path}, {color_path}')
