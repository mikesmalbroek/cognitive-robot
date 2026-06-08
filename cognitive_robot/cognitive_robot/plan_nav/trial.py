#!/usr/bin/env python3

import os
import math
import subprocess
import time
from datetime import datetime

import cv2
from cv2 import aruco
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import tf2_ros


# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_NAME = 'MIRTE Camera - QR / ArUco SLAM Mapper'

STATION_MAP = {
    0: 'Station A',
    1: 'Station B',
}

STATION_FILE_MAP = {
    'Station A': 'station_a_location.yaml',
    'Station B': 'station_b_location.yaml',
}

# Navigation stand-off distance in front of each station
# Station A: robot stops 0.8 m in front
# Station B: robot stops 0.2 m in front
STATION_STANDOFF_DISTANCE = {
    'Station A': 0.80,
    'Station B': 0.20,
}

ARUCO_DICT = aruco.DICT_6X6_250
MARKER_LENGTH_METERS = 0.20

FALLBACK_CAMERA_MATRIX = np.array([
    [554.254691191187, 0.0, 320.5],
    [0.0, 554.254691191187, 240.5],
    [0.0, 0.0, 1.0],
], dtype=float)

FALLBACK_DIST_COEFFS = np.zeros(5)


# =============================================================================
# BASIC MATH / TF HELPERS
# =============================================================================

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    Convert quaternion to 3x3 rotation matrix.
    """
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
    """
    Transform a 3D point using a ROS TransformStamped.
    point_xyz should be in the source frame of transform_stamped.
    Output is in the target frame of transform_stamped.
    """
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
    """
    Direction angle in map frame.
    Useful if later you want the robot to face the station.
    """
    return math.atan2(dy, dx)


def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_destination_pose(station_point, camera_origin_in_map, station_name):
    """
    Compute a navigation destination in front of the station.

    The direction is estimated from the camera position to the station position
    at the moment the station is registered.

    destination = station position - stand_off_distance * unit_vector(camera -> station)

    Robot yaw at destination points from destination toward station.
    """

    stand_off_distance = STATION_STANDOFF_DISTANCE.get(station_name, 0.50)

    dx = station_point[0] - camera_origin_in_map[0]
    dy = station_point[1] - camera_origin_in_map[1]

    distance = math.sqrt(dx * dx + dy * dy)

    if distance < 1e-6:
        raise ValueError(
            'Camera and station position are almost identical. '
            'Cannot compute destination point.'
        )

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
        'x': float(destination_x),
        'y': float(destination_y),
        'z': float(destination_z),
        'yaw': float(destination_yaw),
        'stand_off_distance': float(stand_off_distance),
        'distance_camera_to_station_when_saved': float(distance),
    }


def save_photo(image, folder, prefix='camera_capture'):
    os.makedirs(folder, exist_ok=True)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    filename = f'{prefix}_{timestamp}.jpg'
    filepath = os.path.join(folder, filename)

    cv2.imwrite(filepath, image)
    return filepath


# =============================================================================
# ARUCO / QR DETECTION HELPERS
# =============================================================================

def create_aruco_detector():
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)

    if hasattr(aruco, 'DetectorParameters_create'):
        params = aruco.DetectorParameters_create()
        detector = None
    else:
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, params)

    return aruco_dict, params, detector


def detect_aruco_markers(
    image,
    aruco_dict,
    aruco_params,
    aruco_detector,
    camera_matrix,
    dist_coeffs
):
    annotated = image.copy()
    detections = []

    if aruco_detector is not None:
        corners, ids, _ = aruco_detector.detectMarkers(image)
    else:
        corners, ids, _ = aruco.detectMarkers(
            image,
            aruco_dict,
            parameters=aruco_params
        )

    if ids is None:
        return detections, annotated

    aruco.drawDetectedMarkers(annotated, corners, ids)

    half = MARKER_LENGTH_METERS / 2.0

    obj_points = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)

    for i, marker_id in enumerate(ids):
        mid = int(marker_id[0])
        img_points = corners[i][0].astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs
        )

        if not success:
            continue

        x, y, z = tvec.flatten()

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        marker_normal = rotation_matrix[:, 2]
        yaw_camera = float(np.arctan2(marker_normal[0], -marker_normal[2]))

        station_name = STATION_MAP.get(mid, f'Unknown ArUco ID {mid}')

        try:
            cv2.drawFrameAxes(
                annotated,
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
                0.10
            )
        except Exception:
            pass

        top_left = tuple(corners[i][0][0].astype(int))
        text_x = max(top_left[0], 10)
        text_y = max(top_left[1] - 70, 30)

        cv2.putText(
            annotated,
            f'{station_name} | ArUco ID={mid}',
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 0),
            2
        )

        cv2.putText(
            annotated,
            f'camera x={x:+.2f} y={y:+.2f} z={z:+.2f}',
            (text_x, text_y + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 0),
            2
        )

        cv2.putText(
            annotated,
            f'yaw={np.degrees(yaw_camera):+.1f} deg',
            (text_x, text_y + 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 0),
            2
        )

        detections.append({
            'type': 'aruco',
            'id': mid,
            'station_name': station_name,
            'camera_x': float(x),
            'camera_y': float(y),
            'camera_z': float(z),
            'camera_yaw': float(yaw_camera),
        })

    return detections, annotated


def detect_qr_codes(image, qr_detector):
    annotated = image.copy()
    qr_results = []

    try:
        retval, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(image)

        if retval and points is not None:
            for text, pts in zip(decoded_info, points):
                if text is None or text.strip() == '':
                    continue

                pts = pts.astype(int)

                cv2.polylines(
                    annotated,
                    [pts],
                    isClosed=True,
                    color=(255, 0, 255),
                    thickness=2
                )

                x = int(pts[0][0])
                y = int(pts[0][1])

                cv2.putText(
                    annotated,
                    f'QR: {text}',
                    (x, max(y - 10, 30)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 0, 255),
                    2
                )

                qr_results.append({
                    'type': 'qr',
                    'data': text.strip(),
                    'points': pts.tolist(),
                })

    except Exception:
        text, points, _ = qr_detector.detectAndDecode(image)

        if text is not None and text.strip() != '' and points is not None:
            pts = points[0].astype(int)

            cv2.polylines(
                annotated,
                [pts],
                isClosed=True,
                color=(255, 0, 255),
                thickness=2
            )

            x = int(pts[0][0])
            y = int(pts[0][1])

            cv2.putText(
                annotated,
                f'QR: {text}',
                (x, max(y - 10, 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 0, 255),
                2
            )

            qr_results.append({
                'type': 'qr',
                'data': text.strip(),
                'points': pts.tolist(),
            })

    return qr_results, annotated


def station_name_from_qr_text(text):
    normalized = text.strip().upper().replace(' ', '').replace('-', '_')

    if normalized in ['STATION_A', 'A', 'STATIONA']:
        return 'Station A'

    if normalized in ['STATION_B', 'B', 'STATIONB']:
        return 'Station B'

    return None


# =============================================================================
# MAIN ROS2 NODE
# =============================================================================

class ManualSlamQrMapper(Node):
    def __init__(self):
        super().__init__('manual_slam_qr_mapper')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('strafe_speed', 0.20)
        self.declare_parameter('angular_speed', 0.50)

        self.declare_parameter('photo_dir', '~/slam_station_photos')
        self.declare_parameter('map_name', 'auto_map')
        self.declare_parameter('map_dir', os.getcwd())

        self.declare_parameter('station_dir', os.getcwd())
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('fallback_camera_frame', 'camera_depth_optical_frame')

        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).get_parameter_value().string_value

        self.camera_topic = self.get_parameter(
            'camera_topic'
        ).get_parameter_value().string_value

        self.camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).get_parameter_value().string_value

        self.linear_speed = self.get_parameter(
            'linear_speed'
        ).get_parameter_value().double_value

        self.strafe_speed = self.get_parameter(
            'strafe_speed'
        ).get_parameter_value().double_value

        self.angular_speed = self.get_parameter(
            'angular_speed'
        ).get_parameter_value().double_value

        self.photo_dir = os.path.expanduser(
            self.get_parameter(
                'photo_dir'
            ).get_parameter_value().string_value
        )

        self.map_name = self.get_parameter(
            'map_name'
        ).get_parameter_value().string_value

        self.map_dir = os.path.expanduser(
            self.get_parameter(
                'map_dir'
            ).get_parameter_value().string_value
        )

        self.station_dir = os.path.expanduser(
            self.get_parameter(
                'station_dir'
            ).get_parameter_value().string_value
        )

        self.map_frame = self.get_parameter(
            'map_frame'
        ).get_parameter_value().string_value

        self.fallback_camera_frame = self.get_parameter(
            'fallback_camera_frame'
        ).get_parameter_value().string_value

        os.makedirs(self.photo_dir, exist_ok=True)
        os.makedirs(self.map_dir, exist_ok=True)
        os.makedirs(self.station_dir, exist_ok=True)

        self.bridge = CvBridge()

        self.latest_frame = None
        self.latest_annotated = None

        self.latest_image_stamp = None
        self.latest_camera_frame = self.fallback_camera_frame

        self.latest_qr_results = []
        self.latest_aruco_results = []

        self.camera_matrix = FALLBACK_CAMERA_MATRIX.copy()
        self.dist_coeffs = FALLBACK_DIST_COEFFS.copy()
        self.got_camera_info = False

        self.qr_detector = cv2.QRCodeDetector()
        self.aruco_dict, self.aruco_params, self.aruco_detector = create_aruco_detector()

        self.current_twist = Twist()
        self.last_key_time = 0.0
        self.key_timeout = 0.1

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.display_timer = self.create_timer(0.03, self.display_loop)
        self.cmd_timer = self.create_timer(0.05, self.publish_cmd_loop)

        self.print_startup_info()

    # -------------------------------------------------------------------------
    # ROS CALLBACKS
    # -------------------------------------------------------------------------

    def camera_info_callback(self, msg):
        if self.got_camera_info:
            return

        if msg.header.frame_id:
            self.latest_camera_frame = msg.header.frame_id

        k = np.array(msg.k, dtype=float).reshape(3, 3)

        if k[0, 0] > 0.0 and k[1, 1] > 0.0:
            self.camera_matrix = k

            if len(msg.d) > 0:
                self.dist_coeffs = np.array(msg.d, dtype=float)
            else:
                self.dist_coeffs = np.zeros(5)

            self.got_camera_info = True
            self.get_logger().info(
                f'Received camera calibration from {self.camera_info_topic}. '
                f'Camera frame: {self.latest_camera_frame}'
            )

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            self.latest_image_stamp = msg.header.stamp

            if msg.header.frame_id:
                self.latest_camera_frame = msg.header.frame_id

        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    # -------------------------------------------------------------------------
    # DISPLAY LOOP
    # -------------------------------------------------------------------------

    def display_loop(self):
        if self.latest_frame is None:
            return

        frame = self.latest_frame.copy()

        qr_results, qr_annotated = detect_qr_codes(
            frame,
            self.qr_detector
        )

        aruco_results, aruco_annotated = detect_aruco_markers(
            qr_annotated,
            self.aruco_dict,
            self.aruco_params,
            self.aruco_detector,
            self.camera_matrix,
            self.dist_coeffs
        )

        self.latest_qr_results = qr_results
        self.latest_aruco_results = aruco_results
        self.latest_annotated = aruco_annotated

        self.draw_help_overlay(self.latest_annotated)

        cv2.imshow(WINDOW_NAME, self.latest_annotated)

        key = cv2.waitKey(1) & 0xFF

        if key != 255:
            self.handle_key(key)

    def draw_help_overlay(self, image):
        lines = [
            'W/S: forward/back',
            'A/D: left/right',
            'Q/E: rotate',
            'X: stop',
            'C: capture image',
            'B: register Station A/B + destination in map',
            'V: save auto_map.yaml + exit',
            'ESC: quit no save',
        ]

        x = 15
        y = image.shape[0] - 165

        for i, line in enumerate(lines):
            cv2.putText(
                image,
                line,
                (x, y + i * 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                2
            )

        if self.latest_qr_results:
            status = 'QR detected'
            color = (255, 0, 255)
        elif self.latest_aruco_results:
            status = 'ArUco detected'
            color = (0, 255, 0)
        else:
            status = 'No station code detected'
            color = (0, 0, 255)

        cv2.putText(
            image,
            status,
            (15, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            color,
            2
        )

        cv2.putText(
            image,
            f'camera frame: {self.latest_camera_frame}',
            (15, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            2
        )

    # -------------------------------------------------------------------------
    # KEYBOARD HANDLING
    # -------------------------------------------------------------------------

    def handle_key(self, key):
        twist = Twist()

        if key == ord('w'):
            twist.linear.x = self.linear_speed
            self.set_twist(twist)

        elif key == ord('s'):
            twist.linear.x = -self.linear_speed
            self.set_twist(twist)

        elif key == ord('a'):
            twist.linear.y = self.strafe_speed
            self.set_twist(twist)

        elif key == ord('d'):
            twist.linear.y = -self.strafe_speed
            self.set_twist(twist)

        elif key == ord('q'):
            twist.angular.z = self.angular_speed
            self.set_twist(twist)

        elif key == ord('e'):
            twist.angular.z = -self.angular_speed
            self.set_twist(twist)

        elif key == ord('x'):
            self.stop_robot()
            self.get_logger().info('Stop command sent.')

        elif key == ord('c'):
            self.capture_image()

        elif key == ord('b'):
            self.register_current_station()

        elif key == ord('v'):
            self.stop_robot()
            self.save_map()
            self.get_logger().info('Map saved. Shutting down.')
            rclpy.shutdown()

        elif key == 27:
            self.stop_robot()
            self.get_logger().info('ESC pressed. Quitting without saving map.')
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

    # -------------------------------------------------------------------------
    # STATION REGISTRATION
    # -------------------------------------------------------------------------

    def get_current_station_detection(self):
        """
        Priority:
        1. ArUco station detection with pose, because it gives 3D position.
        2. QR text detection, but QR text alone does not give accurate 3D pose.
        """

        if self.latest_aruco_results:
            for det in self.latest_aruco_results:
                if det['station_name'] in ['Station A', 'Station B']:
                    return det

        if self.latest_qr_results:
            for qr in self.latest_qr_results:
                station_name = station_name_from_qr_text(qr['data'])
                if station_name is not None:
                    return {
                        'type': 'qr',
                        'station_name': station_name,
                        'qr_data': qr['data'],
                    }

        return None

    def register_current_station(self):
        print('\n' + '=' * 80)
        print('REGISTER STATION AND DESTINATION IN CURRENT SLAM MAP')
        print('=' * 80)

        detection = self.get_current_station_detection()

        if detection is None:
            print('No Station A or Station B detected.')
            print('Move the robot/camera so the marker is visible, then press b again.')
            print('=' * 80 + '\n')
            return

        station_name = detection['station_name']

        if station_name not in STATION_FILE_MAP:
            print(f'Detected marker is not Station A or Station B: {station_name}')
            print('=' * 80 + '\n')
            return

        if detection['type'] == 'qr':
            print(f'QR text detected: {detection["qr_data"]}')
            print('But this QR detection does not include 3D pose.')
            print('Use the ArUco marker detection for map-frame station location.')
            print('=' * 80 + '\n')
            return

        camera_point = np.array([
            detection['camera_x'],
            detection['camera_y'],
            detection['camera_z'],
        ], dtype=float)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.latest_camera_frame,
                Time()
            )

        except Exception as e:
            print(f'Failed to get TF transform:')
            print(f'  target frame: {self.map_frame}')
            print(f'  source frame: {self.latest_camera_frame}')
            print(f'Error:')
            print(f'  {e}')
            print()
            print('Check TF tree with:')
            print(f'  ros2 run tf2_ros tf2_echo {self.map_frame} {self.latest_camera_frame}')
            print('=' * 80 + '\n')
            return

        map_point = transform_point_with_tf(camera_point, transform)

        camera_origin_in_map = transform_point_with_tf(
            np.array([0.0, 0.0, 0.0]),
            transform
        )

        dx = map_point[0] - camera_origin_in_map[0]
        dy = map_point[1] - camera_origin_in_map[1]
        yaw_from_camera_to_station_map = yaw_from_xy(dx, dy)

        try:
            destination_pose = compute_destination_pose(
                station_point=map_point,
                camera_origin_in_map=camera_origin_in_map,
                station_name=station_name
            )
        except Exception as e:
            print(f'Failed to compute destination pose: {e}')
            print('=' * 80 + '\n')
            return

        station_filename = STATION_FILE_MAP[station_name]
        station_path = os.path.join(self.station_dir, station_filename)

        yaml_text = self.make_station_yaml(
            station_name=station_name,
            detection=detection,
            map_point=map_point,
            camera_origin_in_map=camera_origin_in_map,
            yaw_from_camera_to_station_map=yaw_from_camera_to_station_map,
            destination_pose=destination_pose,
            station_path=station_path
        )

        with open(station_path, 'w') as f:
            f.write(yaml_text)

        print(f'Detected station : {station_name}')
        print(f'ArUco ID         : {detection["id"]}')
        print()
        print('Camera-frame marker position:')
        print(f'  frame : {self.latest_camera_frame}')
        print(f'  x     : {detection["camera_x"]:+.3f} m')
        print(f'  y     : {detection["camera_y"]:+.3f} m')
        print(f'  z     : {detection["camera_z"]:+.3f} m')
        print()
        print('SLAM map-frame station position:')
        print(f'  frame : {self.map_frame}')
        print(f'  x     : {map_point[0]:+.3f} m')
        print(f'  y     : {map_point[1]:+.3f} m')
        print(f'  z     : {map_point[2]:+.3f} m')
        print(f'  yaw from camera to station: {math.degrees(yaw_from_camera_to_station_map):+.1f} deg')
        print()
        print('Computed navigation destination:')
        print(f'  frame : {self.map_frame}')
        print(f'  x     : {destination_pose["x"]:+.3f} m')
        print(f'  y     : {destination_pose["y"]:+.3f} m')
        print(f'  z     : {destination_pose["z"]:+.3f} m')
        print(f'  yaw   : {math.degrees(destination_pose["yaw"]):+.1f} deg')
        print(f'  stand-off distance: {destination_pose["stand_off_distance"]:.2f} m')
        print()
        print(f'Saved/overwritten station file:')
        print(f'  {station_path}')
        print('=' * 80 + '\n')

    def make_station_yaml(
        self,
        station_name,
        detection,
        map_point,
        camera_origin_in_map,
        yaw_from_camera_to_station_map,
        destination_pose,
        station_path
    ):
        timestamp = datetime.now().isoformat()

        return f"""# Auto-generated by manual_slam_qr_mapper.py
# This file is overwritten every time you press b while detecting this station.

station_name: "{station_name}"
station_file: "{station_path}"

created_time: "{timestamp}"

marker:
  type: "aruco"
  id: {detection["id"]}
  physical_size_m: {MARKER_LENGTH_METERS}

map_pose:
  frame_id: "{self.map_frame}"
  x: {map_point[0]:.6f}
  y: {map_point[1]:.6f}
  z: {map_point[2]:.6f}
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
  frame_id: "{self.latest_camera_frame}"
  x: {detection["camera_x"]:.6f}
  y: {detection["camera_y"]:.6f}
  z: {detection["camera_z"]:.6f}
  yaw_rad: {detection["camera_yaw"]:.6f}
  yaw_deg: {math.degrees(detection["camera_yaw"]):.6f}
"""

    # -------------------------------------------------------------------------
    # PHOTO + MAP SAVE
    # -------------------------------------------------------------------------

    def capture_image(self):
        if self.latest_annotated is None:
            self.get_logger().warn('No camera frame available yet.')
            return

        photo_path = save_photo(
            self.latest_annotated,
            self.photo_dir,
            prefix='slam_qr_capture'
        )

        self.get_logger().info(f'Saved camera image: {photo_path}')

    def save_map(self):
        map_base = os.path.join(self.map_dir, self.map_name)

        self.get_logger().info('Saving SLAM map...')
        self.get_logger().info(f'Map output base: {map_base}')

        cmd = [
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
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

            yaml_path = map_base + '.yaml'
            pgm_path = map_base + '.pgm'

            if result.returncode == 0:
                self.get_logger().info('Map saver finished successfully.')
                self.get_logger().info(f'Saved YAML: {yaml_path}')
                self.get_logger().info(f'Saved image: {pgm_path}')
            else:
                self.get_logger().error(
                    f'map_saver_cli failed with return code {result.returncode}'
                )

        except subprocess.TimeoutExpired:
            self.get_logger().error(
                'Map saving timed out. Is /map being published by Cartographer?'
            )

        except FileNotFoundError:
            self.get_logger().error(
                'Could not run ros2 command. Make sure ROS2 is sourced in this terminal.'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')

    # -------------------------------------------------------------------------
    # STARTUP INFO
    # -------------------------------------------------------------------------

    def print_startup_info(self):
        self.get_logger().info('=== Manual SLAM QR / ArUco Mapper Started ===')
        self.get_logger().info('')
        self.get_logger().info('Start Cartographer first:')
        self.get_logger().info('  ros2 launch spatial_ai_navigation cartographer.py')
        self.get_logger().info('')
        self.get_logger().info('Keyboard controls inside camera window:')
        self.get_logger().info('  w : forward')
        self.get_logger().info('  s : backward')
        self.get_logger().info('  a : move left / strafe left')
        self.get_logger().info('  d : move right / strafe right')
        self.get_logger().info('  q : rotate left')
        self.get_logger().info('  e : rotate right')
        self.get_logger().info('  x : stop')
        self.get_logger().info('  c : capture camera image')
        self.get_logger().info('  b : save Station A/B + destination in current SLAM map frame')
        self.get_logger().info('  v : save map as auto_map.yaml and exit')
        self.get_logger().info('  ESC : quit without saving map')
        self.get_logger().info('')
        self.get_logger().info(f'cmd_vel topic     : {self.cmd_vel_topic}')
        self.get_logger().info(f'camera topic      : {self.camera_topic}')
        self.get_logger().info(f'camera info topic : {self.camera_info_topic}')
        self.get_logger().info(f'map frame         : {self.map_frame}')
        self.get_logger().info(f'camera frame      : {self.latest_camera_frame}')
        self.get_logger().info(f'photo dir         : {self.photo_dir}')
        self.get_logger().info(f'station dir       : {self.station_dir}')
        self.get_logger().info(f'map output base   : {os.path.join(self.map_dir, self.map_name)}')


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):
    rclpy.init(args=args)

    node = ManualSlamQrMapper()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received.')

    finally:
        node.stop_robot()
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()