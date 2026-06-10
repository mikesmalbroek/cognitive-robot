#!/usr/bin/env python3

import os
import math
import time
import subprocess
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

import tf2_ros

from cognitive_robot_interfaces.srv import DetectStation, DetectAbacus


# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_NAME = "MIRTE Depth Camera Station Mapper"

STATION_FILE_MAP = {
    "Station A": "station_a_location.yaml",
    "Station B": "station_b_location.yaml",
}

ABACUS_FILE = "abacus_location.yaml"

STATION_STANDOFF_DISTANCE = {
    "Station A": 0.80,
    "Station B": 0.20,
    "Abacus": 0.20,
}

DEFAULT_CAMERA_FRAME = "camera_depth_optical_frame"


# =============================================================================
# MATH HELPERS
# =============================================================================

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)

    if norm < 1e-12:
        return np.eye(3)

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ])


def transform_point_with_tf(point_xyz, transform_stamped):
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation

    rotation_matrix = quaternion_to_rotation_matrix(
        q.x,
        q.y,
        q.z,
        q.w
    )

    p_source = np.array(point_xyz, dtype=float)
    p_target = rotation_matrix @ p_source + np.array([t.x, t.y, t.z], dtype=float)

    return p_target


def yaw_from_xy(dx, dy):
    return math.atan2(dy, dx)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_destination_pose(station_point, camera_origin_in_map, station_name):
    stand_off_distance = STATION_STANDOFF_DISTANCE.get(station_name, 0.50)

    dx = station_point[0] - camera_origin_in_map[0]
    dy = station_point[1] - camera_origin_in_map[1]

    distance = math.sqrt(dx * dx + dy * dy)

    if distance < 1e-6:
        raise ValueError("Camera and station position are too close.")

    ux = dx / distance
    uy = dy / distance

    destination_x = station_point[0] - stand_off_distance * ux
    destination_y = station_point[1] - stand_off_distance * uy
    destination_z = 0.0

    destination_yaw = math.atan2(
        station_point[1] - destination_y,
        station_point[0] - destination_x
    )

    destination_yaw = normalize_angle(destination_yaw)

    return {
        "x": float(destination_x),
        "y": float(destination_y),
        "z": float(destination_z),
        "yaw": float(destination_yaw),
        "stand_off_distance": float(stand_off_distance),
        "distance_camera_to_station_when_saved": float(distance),
    }


# =============================================================================
# MAIN NODE
# =============================================================================

class TrialDepthMapper(Node):
    def __init__(self):
        super().__init__("trial_depth_mapper")

        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", DEFAULT_CAMERA_FRAME)
        self.declare_parameter("station_dir", os.path.dirname(os.path.abspath(__file__)))
        self.declare_parameter("map_name", "auto_map")
        self.declare_parameter("map_dir", os.path.dirname(os.path.abspath(__file__)))

        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("strafe_speed", 0.20)
        self.declare_parameter("angular_speed", 0.50)

        self.cmd_vel_topic = self.get_parameter(
            "cmd_vel_topic"
        ).get_parameter_value().string_value

        self.camera_topic = self.get_parameter(
            "camera_topic"
        ).get_parameter_value().string_value

        self.map_frame = self.get_parameter(
            "map_frame"
        ).get_parameter_value().string_value

        self.camera_frame = self.get_parameter(
            "camera_frame"
        ).get_parameter_value().string_value

        self.station_dir = os.path.expanduser(
            self.get_parameter(
                "station_dir"
            ).get_parameter_value().string_value
        )

        self.map_name = self.get_parameter(
            "map_name"
        ).get_parameter_value().string_value

        self.map_dir = os.path.expanduser(
            self.get_parameter(
                "map_dir"
            ).get_parameter_value().string_value
        )

        self.linear_speed = self.get_parameter(
            "linear_speed"
        ).get_parameter_value().double_value

        self.strafe_speed = self.get_parameter(
            "strafe_speed"
        ).get_parameter_value().double_value

        self.angular_speed = self.get_parameter(
            "angular_speed"
        ).get_parameter_value().double_value

        os.makedirs(self.station_dir, exist_ok=True)
        os.makedirs(self.map_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_display_frame = None

        self.status_text = "Ready. B: station  N: abacus"
        self.last_detection_text = "No detection yet."
        self.last_abacus_text = "No abacus detection yet."

        self.pending_detection_future = None
        self.detection_request_active = False

        self.pending_abacus_future = None
        self.abacus_request_active = False

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.detect_station_client = self.create_client(
            DetectStation,
            "/detect_station"
        )

        self.detect_abacus_client = self.create_client(
            DetectAbacus,
            "/detect_abacus"
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.current_twist = Twist()
        self.last_key_time = 0.0
        self.key_timeout = 0.15

        self.cmd_timer = self.create_timer(0.05, self.publish_cmd_loop)
        self.display_timer = self.create_timer(0.03, self.display_loop)
        self.service_timer = self.create_timer(0.05, self.check_detection_future)
        self.abacus_timer = self.create_timer(0.05, self.check_abacus_future)

        self.print_startup_info()

    # -------------------------------------------------------------------------
    # CAMERA DISPLAY
    # -------------------------------------------------------------------------

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def display_loop(self):
        if self.latest_frame is None:
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                placeholder,
                "Waiting for camera...",
                (160, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
            )
            cv2.imshow(WINDOW_NAME, placeholder)
            cv2.waitKey(1)
            return

        frame = self.latest_frame.copy()
        self.draw_overlay(frame)

        self.latest_display_frame = frame

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF

        if key != 255:
            self.handle_key(key)

    def draw_overlay(self, frame):
        h, w = frame.shape[:2]

        lines = [
            "W/S: forward/back",
            "A/D: left/right",
            "Q/E: rotate",
            "X: stop",
            "B: detect + register station",
            "N: detect abacus",
            "V: save map + quit",
            "ESC: quit no save",
        ]

        x = 15
        y = h - 175

        for i, line in enumerate(lines):
            cv2.putText(
                frame,
                line,
                (x, y + i * 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                2
            )

        cv2.putText(
            frame,
            self.status_text,
            (15, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            self.last_detection_text,
            (15, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            self.last_abacus_text,
            (15, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 200, 255),
            2
        )

        cv2.putText(
            frame,
            f"camera topic: {self.camera_topic}",
            (15, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1
        )

        cv2.putText(
            frame,
            f"camera frame: {self.camera_frame}",
            (15, 118),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1
        )

    # -------------------------------------------------------------------------
    # KEYBOARD CONTROL
    # -------------------------------------------------------------------------

    def handle_key(self, key):
        twist = Twist()

        if key == ord("w"):
            twist.linear.x = self.linear_speed
            self.set_twist(twist)

        elif key == ord("s"):
            twist.linear.x = -self.linear_speed
            self.set_twist(twist)

        elif key == ord("a"):
            twist.linear.y = self.strafe_speed
            self.set_twist(twist)

        elif key == ord("d"):
            twist.linear.y = -self.strafe_speed
            self.set_twist(twist)

        elif key == ord("q"):
            twist.angular.z = self.angular_speed
            self.set_twist(twist)

        elif key == ord("e"):
            twist.angular.z = -self.angular_speed
            self.set_twist(twist)

        elif key == ord("x"):
            self.stop_robot()

        elif key == ord("b"):
            self.stop_robot()
            self.start_detect_station_request()

        elif key == ord("n"):
            self.stop_robot()
            self.start_detect_abacus_request()

        elif key == ord("v"):
            self.stop_robot()
            self.save_map()
            self.get_logger().info("Map saved. Shutting down.")
            rclpy.shutdown()

        elif key == 27:
            self.stop_robot()
            self.get_logger().info("ESC pressed. Quitting without saving map.")
            rclpy.shutdown()

    # -------------------------------------------------------------------------
    # MOVEMENT
    # -------------------------------------------------------------------------

    def set_twist(self, twist):
        self.current_twist = twist
        self.last_key_time = time.time()

    def publish_cmd_loop(self):
        now = time.time()

        if now - self.last_key_time > self.key_timeout:
            self.current_twist = Twist()

        self.cmd_pub.publish(self.current_twist)

    def stop_robot(self):
        self.current_twist = Twist()
        self.cmd_pub.publish(self.current_twist)
        self.status_text = "Robot stopped."

    # -------------------------------------------------------------------------
    # DETECTION SERVICE
    # -------------------------------------------------------------------------

    def start_detect_station_request(self):
        print()
        print("=" * 80)
        print("CALL /detect_station AND REGISTER STATION")
        print("=" * 80)

        if self.detection_request_active:
            print("Detection request already active. Wait for it to finish.")
            self.status_text = "Detection already running..."
            return

        if not self.detect_station_client.wait_for_service(timeout_sec=4.0):
            print("ERROR: /detect_station service is not available.")
            print("Start it with:")
            print("  ros2 launch cognitive_robot demo_gazebo.launch.py")
            print("=" * 80)
            self.status_text = "/detect_station not available."
            return

        request = DetectStation.Request()

        self.pending_detection_future = self.detect_station_client.call_async(request)
        self.detection_request_active = True
        self.status_text = "Calling /detect_station..."

    def check_detection_future(self):
        if not self.detection_request_active:
            return

        if self.pending_detection_future is None:
            self.detection_request_active = False
            return

        if not self.pending_detection_future.done():
            return

        future = self.pending_detection_future
        self.pending_detection_future = None
        self.detection_request_active = False

        if future.result() is None:
            print("ERROR: /detect_station service call failed.")
            print("=" * 80)
            self.status_text = "Detection service call failed."
            return

        response = future.result()
        self.handle_detect_station_response(response)

    def handle_detect_station_response(self, response):
        if not response.detected:
            print("No station detected by /detect_station.")
            print("=" * 80)
            self.status_text = "No station detected."
            self.last_detection_text = "No station detected."
            return

        station_name = response.station_name

        if station_name not in STATION_FILE_MAP:
            print(f"Detected station is not Station A/B: {station_name}")
            print(f"marker_id: {response.marker_id}")
            print("=" * 80)
            self.status_text = "Unknown station detected."
            self.last_detection_text = f"Unknown: {station_name}"
            return

        if response.distance_m <= 0.0 or response.z_m <= 0.0:
            print("Station detected, but depth measurement is invalid.")
            print(f"distance_m: {response.distance_m}")
            print(f"x_m: {response.x_m}, y_m: {response.y_m}, z_m: {response.z_m}")
            print("=" * 80)
            self.status_text = "Invalid depth measurement."
            self.last_detection_text = f"{station_name}: invalid depth"
            return

        print(f"Detected station : {station_name}")
        print(f"Marker ID        : {response.marker_id}")
        print(f"Depth distance   : {response.distance_m:.3f} m")
        print(
            f"Camera-frame pos : "
            f"x={response.x_m:+.3f}, "
            f"y={response.y_m:+.3f}, "
            f"z={response.z_m:+.3f}"
        )
        print(f"Marker yaw       : {math.degrees(response.yaw):+.1f} deg")

        self.last_detection_text = (
            f"{station_name}: "
            f"x={response.x_m:+.2f}, "
            f"y={response.y_m:+.2f}, "
            f"z={response.z_m:+.2f}, "
            f"d={response.distance_m:.2f}m"
        )

        self.register_station_from_service_response(response)

    # -------------------------------------------------------------------------
    # ABACUS DETECTION SERVICE
    # -------------------------------------------------------------------------

    def start_detect_abacus_request(self):
        print()
        print("=" * 80)
        print("CALL /detect_abacus")
        print("=" * 80)

        if self.abacus_request_active:
            print("Abacus request already active. Wait for it to finish.")
            self.status_text = "Abacus detection already running..."
            return

        if not self.detect_abacus_client.wait_for_service(timeout_sec=1.0):
            print("ERROR: /detect_abacus service is not available.")
            print("=" * 80)
            self.status_text = "/detect_abacus not available."
            return

        self.pending_abacus_future = self.detect_abacus_client.call_async(
            DetectAbacus.Request()
        )
        self.abacus_request_active = True
        self.status_text = "Calling /detect_abacus..."

    def check_abacus_future(self):
        if not self.abacus_request_active:
            return

        if self.pending_abacus_future is None:
            self.abacus_request_active = False
            return

        if not self.pending_abacus_future.done():
            return

        future = self.pending_abacus_future
        self.pending_abacus_future = None
        self.abacus_request_active = False

        if future.result() is None:
            print("ERROR: /detect_abacus service call failed.")
            print("=" * 80)
            self.status_text = "Abacus service call failed."
            return

        self.handle_detect_abacus_response(future.result())

    def handle_detect_abacus_response(self, response):
        print()
        if response.confidence <= 0.0:
            print("No abacus detected by /detect_abacus.")
            print("=" * 80)
            self.status_text = "No abacus detected."
            self.last_abacus_text = "No abacus detected."
            return

        print(f"Abacus detected!")
        print(f"  Confidence : {response.confidence:.2f}")
        print(f"  Pixel pos  : x={response.x}, y={response.y}")
        print(f"  Bbox       : {response.bbox_width}x{response.bbox_height} px")
        print(f"  Distance   : {response.distance_m:.3f} m")
        print(f"  Camera pos : x={response.x_m:+.3f} m, y={response.y_m:+.3f} m")
        print("=" * 80)

        self.status_text = f"Abacus: conf={response.confidence:.2f} d={response.distance_m:.2f}m"
        self.last_abacus_text = (
            f"Abacus: conf={response.confidence:.2f} "
            f"x={response.x_m:+.2f}m y={response.y_m:+.2f}m "
            f"d={response.distance_m:.2f}m"
        )

        self.register_abacus_from_service_response(response)

    def register_abacus_from_service_response(self, response):
        camera_point = np.array([
            response.x_m,
            response.y_m,
            response.distance_m,
        ], dtype=float)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                self.get_clock().now(),
                timeout=Duration(seconds=2.0)
            )
        except Exception as e:
            print("Failed to get TF transform for abacus:")
            print(f"  {e}")
            print("=" * 80)
            self.status_text = "Abacus TF transform failed."
            return

        abacus_point_map = transform_point_with_tf(camera_point, transform)
        camera_origin_in_map = transform_point_with_tf(
            np.array([0.0, 0.0, 0.0]), transform
        )

        abacus_path = os.path.join(self.station_dir, ABACUS_FILE)
        timestamp = datetime.now().isoformat()

        try:
            destination_pose = compute_destination_pose(
                station_point=abacus_point_map,
                camera_origin_in_map=camera_origin_in_map,
                station_name="Abacus"
            )
        except Exception as e:
            print(f"Failed to compute abacus destination pose: {e}")
            print("=" * 80)
            self.status_text = "Abacus destination pose failed."
            return

        yaml_content = f"""# Auto-generated by trial_depth.py
# Detection comes from /detect_abacus service.
# This file is overwritten every time abacus is registered.

station_name: "Abacus"
created_time: "{timestamp}"

detection:
  confidence: {response.confidence:.6f}
  pixel_x: {response.x}
  pixel_y: {response.y}
  bbox_width: {response.bbox_width}
  bbox_height: {response.bbox_height}

map_pose:
  frame_id: "{self.map_frame}"
  x: {abacus_point_map[0]:.6f}
  y: {abacus_point_map[1]:.6f}
  z: {abacus_point_map[2]:.6f}

destination_pose:
  frame_id: "{self.map_frame}"
  description: "Robot navigation goal in front of the abacus, facing it"
  x: {destination_pose["x"]:.6f}
  y: {destination_pose["y"]:.6f}
  z: {destination_pose["z"]:.6f}
  yaw_rad: {destination_pose["yaw"]:.6f}
  yaw_deg: {math.degrees(destination_pose["yaw"]):.6f}
  stand_off_distance_m: {destination_pose["stand_off_distance"]:.6f}

camera_pose_in_map_when_saved:
  frame_id: "{self.map_frame}"
  x: {camera_origin_in_map[0]:.6f}
  y: {camera_origin_in_map[1]:.6f}
  z: {camera_origin_in_map[2]:.6f}

raw_detection_in_camera_frame:
  frame_id: "{self.camera_frame}"
  x: {response.x_m:.6f}
  y: {response.y_m:.6f}
  distance_m: {response.distance_m:.6f}
"""

        with open(abacus_path, "w") as f:
            f.write(yaml_content)

        print()
        print("SLAM map-frame abacus position:")
        print(f"  frame : {self.map_frame}")
        print(f"  x     : {abacus_point_map[0]:+.3f} m")
        print(f"  y     : {abacus_point_map[1]:+.3f} m")
        print(f"  z     : {abacus_point_map[2]:+.3f} m")
        print()
        print("Saved/overwritten:")
        print(f"  {abacus_path}")
        print("=" * 80)

        self.status_text = "Saved abacus location."

    # -------------------------------------------------------------------------
    # STATION REGISTRATION
    # -------------------------------------------------------------------------

    def register_station_from_service_response(self, response):
        station_name = response.station_name

        camera_point = np.array([
            response.x_m,
            response.y_m,
            response.z_m,
        ], dtype=float)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                self.get_clock().now(),
                timeout=Duration(seconds=2.0)
            )

        except Exception as e:
            print("Failed to get TF transform:")
            print(f"  target frame: {self.map_frame}")
            print(f"  source frame: {self.camera_frame}")
            print("Error:")
            print(f"  {e}")
            print()
            print("Check:")
            print(f"  ros2 run tf2_ros tf2_echo {self.map_frame} {self.camera_frame}")
            print("=" * 80)

            self.status_text = "TF transform failed."
            return

        station_point_map = transform_point_with_tf(camera_point, transform)

        camera_origin_in_map = transform_point_with_tf(
            np.array([0.0, 0.0, 0.0]),
            transform
        )

        dx = station_point_map[0] - camera_origin_in_map[0]
        dy = station_point_map[1] - camera_origin_in_map[1]
        yaw_from_camera_to_station_map = yaw_from_xy(dx, dy)

        try:
            destination_pose = compute_destination_pose(
                station_point=station_point_map,
                camera_origin_in_map=camera_origin_in_map,
                station_name=station_name
            )
        except Exception as e:
            print(f"Failed to compute destination pose: {e}")
            print("=" * 80)
            self.status_text = "Destination pose failed."
            return

        station_filename = STATION_FILE_MAP[station_name]
        station_path = os.path.join(self.station_dir, station_filename)

        station_yaml = self.make_station_yaml(
            station_name=station_name,
            station_path=station_path,
            response=response,
            station_point_map=station_point_map,
            camera_origin_in_map=camera_origin_in_map,
            yaw_from_camera_to_station_map=yaw_from_camera_to_station_map,
            destination_pose=destination_pose
        )

        with open(station_path, "w") as f:
            f.write(station_yaml)

        print()
        print("SLAM map-frame station position:")
        print(f"  frame : {self.map_frame}")
        print(f"  x     : {station_point_map[0]:+.3f} m")
        print(f"  y     : {station_point_map[1]:+.3f} m")
        print(f"  z     : {station_point_map[2]:+.3f} m")
        print()
        print("Computed robot destination:")
        print(f"  x     : {destination_pose['x']:+.3f} m")
        print(f"  y     : {destination_pose['y']:+.3f} m")
        print(f"  yaw   : {math.degrees(destination_pose['yaw']):+.1f} deg")
        print(f"  stand-off: {destination_pose['stand_off_distance']:.2f} m")
        print()
        print("Saved/overwritten:")
        print(f"  {station_path}")
        print("=" * 80)

        self.status_text = f"Saved {station_name} location."

    def make_station_yaml(
        self,
        station_name,
        station_path,
        response,
        station_point_map,
        camera_origin_in_map,
        yaw_from_camera_to_station_map,
        destination_pose
    ):
        timestamp = datetime.now().isoformat()

        return f"""# Auto-generated by trial_depth.py
# Detection comes from /detect_station service using depth camera.
# This file is overwritten every time this station is registered.

station_name: "{station_name}"
station_file: "{station_path}"

created_time: "{timestamp}"

marker:
  type: "aruco_depth_service"
  id: {response.marker_id}
  source_service: "/detect_station"

map_pose:
  frame_id: "{self.map_frame}"
  x: {station_point_map[0]:.6f}
  y: {station_point_map[1]:.6f}
  z: {station_point_map[2]:.6f}
  yaw_from_camera_to_station_rad: {yaw_from_camera_to_station_map:.6f}
  yaw_from_camera_to_station_deg: {math.degrees(yaw_from_camera_to_station_map):.6f}

destination_pose:
  frame_id: "{self.map_frame}"
  description: "Robot navigation goal in front of the station, facing the station"
  x: {destination_pose["x"]:.6f}
  y: {destination_pose["y"]:.6f}
  z: {destination_pose["z"]:.6f}
  yaw_rad: {destination_pose["yaw"]:.6f}
  yaw_deg: {math.degrees(destination_pose["yaw"]):.6f}
  stand_off_distance_m: {destination_pose["stand_off_distance"]:.6f}
  distance_camera_to_station_when_saved_m: {destination_pose["distance_camera_to_station_when_saved"]:.6f}

camera_pose_in_map_when_saved:
  frame_id: "{self.map_frame}"
  x: {camera_origin_in_map[0]:.6f}
  y: {camera_origin_in_map[1]:.6f}
  z: {camera_origin_in_map[2]:.6f}

raw_detection_in_camera_frame:
  frame_id: "{self.camera_frame}"
  x: {response.x_m:.6f}
  y: {response.y_m:.6f}
  z: {response.z_m:.6f}
  distance_m: {response.distance_m:.6f}
  yaw_rad: {response.yaw:.6f}
  yaw_deg: {math.degrees(response.yaw):.6f}
"""

    # -------------------------------------------------------------------------
    # MAP SAVE
    # -------------------------------------------------------------------------

    def save_map(self):
        map_base = os.path.join(self.map_dir, self.map_name)

        self.get_logger().info("Saving SLAM map...")
        self.get_logger().info(f"Map output base: {map_base}")

        cmd = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            map_base
        ]

        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=20
            )

            if result.stdout:
                self.get_logger().info(result.stdout)

            if result.stderr:
                self.get_logger().warn(result.stderr)

            if result.returncode == 0:
                self.get_logger().info("Map saver finished successfully.")
                self.get_logger().info(f"Saved YAML: {map_base}.yaml")
                self.get_logger().info(f"Saved image: {map_base}.pgm")
                self.status_text = "Map saved."
            else:
                self.get_logger().error(
                    f"map_saver_cli failed with return code {result.returncode}"
                )
                self.status_text = "Map save failed."

        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")
            self.status_text = "Map save failed."

    # -------------------------------------------------------------------------
    # INFO
    # -------------------------------------------------------------------------

    def print_startup_info(self):
        self.get_logger().info("=== Trial Depth Mapper Started ===")
        self.get_logger().info("")
        self.get_logger().info("Required terminals:")
        self.get_logger().info("  1) Cartographer / SLAM")
        self.get_logger().info("  2) cognitive_robot perception service")
        self.get_logger().info("  3) this mapper")
        self.get_logger().info("")
        self.get_logger().info("For Gazebo perception:")
        self.get_logger().info("  ros2 launch cognitive_robot demo_gazebo.launch.py")
        self.get_logger().info("")
        self.get_logger().info(f"cmd_vel topic : {self.cmd_vel_topic}")
        self.get_logger().info(f"camera topic  : {self.camera_topic}")
        self.get_logger().info(f"map frame     : {self.map_frame}")
        self.get_logger().info(f"camera frame  : {self.camera_frame}")
        self.get_logger().info(f"station dir   : {self.station_dir}")
        self.get_logger().info(f"map output    : {os.path.join(self.map_dir, self.map_name)}")
        self.get_logger().info("")
        self.get_logger().info("Click the camera window, then use:")
        self.get_logger().info("  w/s/a/d/q/e/x for movement")
        self.get_logger().info("  b to detect and register station")
        self.get_logger().info("  v to save map and quit")
        self.get_logger().info("  ESC to quit without saving")


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):
    rclpy.init(args=args)

    node = TrialDepthMapper()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()