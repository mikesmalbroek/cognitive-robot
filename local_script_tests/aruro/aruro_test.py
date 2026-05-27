"""
ArUco Marker Detection + Pose Estimation
Shows the 3D position and orientation of the marker relative to the camera.
The RGB axes show the marker's coordinate frame.
"""

import cv2
from cv2 import aruco
import numpy as np

# ===== SETUP =====
# Load the ArUco dictionary (DICT_6X6_250 contains 250 different marker patterns)
dict_ = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Create detector parameters (default settings work fine)
params = aruco.DetectorParameters()

# Create the ArUco detector
detector = aruco.ArucoDetector(dict_, params)

# ===== CAMERA CALIBRATION =====
# These are EXAMPLE values - you need to calibrate your own camera for accurate results!
# For now, we use rough estimates for a typical webcam
focal_length = 800  # approximate focal length in pixels
center = (320, 240)  # approximate image center (half of 640x480)

# Camera matrix - describes the camera's intrinsic properties
camera_matrix = np.array([
    [focal_length, 0, center[0]],
    [0, focal_length, center[1]],
    [0, 0, 1]
], dtype=float)

# Distortion coefficients - we assume zero distortion for now
dist_coeffs = np.zeros(5)

# Marker size in meters (if you printed 15cm marker, use 0.15)
marker_length = 0.15

# ===== LOAD PHOTO =====
frame = cv2.imread('src/cognitive_robot/local_script_tests/aruco/photos_for_aruco/stationA_skewed_02.png')

# Check if the image loaded successfully
if frame is None:
    print("Cannot load photo!")
    exit()

# ===== DETECTION =====
# Search for ArUco markers in the image
# corners = list of 4 corner points per detected marker
# ids = list of marker IDs (0, 1, 2, etc.)
corners, ids, _ = detector.detectMarkers(frame)

# ===== POSE ESTIMATION =====
# Check if any markers were found
if ids is not None:
    # Draw green lines around all detected markers
    aruco.drawDetectedMarkers(frame, corners, ids)
    
    # Estimate the 3D pose (position + orientation) for each marker
    # rvecs = rotation vectors (orientation in 3D space)
    # tvecs = translation vectors (position in 3D space)
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        corners,           # corner points from detection
        marker_length,     # physical size of marker in meters
        camera_matrix,     # camera intrinsic parameters
        dist_coeffs        # lens distortion parameters
    )
    
    # Loop through each detected marker
    for i in range(len(ids)):
        # Draw 3D coordinate axes on the marker
        # Red = X axis, Green = Y axis, Blue = Z axis
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                         rvecs[i], tvecs[i], 0.1)  # 0.1m = 10cm axis length
        
        # Extract position components (in meters)
        x, y, z = tvecs[i][0]
        
        # Extract rotation components (in radians)
        rx, ry, rz = rvecs[i][0]
        
        # Print detailed information to terminal
        print(f"\n--- Marker ID {ids[i][0]} ---")
        print(f"Position (meters):")
        print(f"  X (left/right offset): {x:.3f}")
        print(f"  Y (up/down offset):    {y:.3f}")
        print(f"  Z (forward distance):  {z:.3f}")
        print(f"Rotation (radians):")
        print(f"  RX (pitch - tilting up/down): {rx:.3f}")
        print(f"  RY (yaw - turning left/right): {ry:.3f}")
        print(f"  RZ (roll - rotating in plane): {rz:.3f}")
        
        # Display summary text on the image
        text = f"ID:{ids[i][0]} Dist:{z:.2f}m Yaw:{ry:.2f}rad"
        cv2.putText(frame, text, (50, 50 + i*40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
else:
    print("No markers found in the photo")

# ===== SHOW RESULT =====
# Display the image with detected markers and coordinate frames
cv2.imshow('Detection + Pose Estimation - Press any key to close', frame)
cv2.waitKey(0)  # Wait until you press a key
cv2.destroyAllWindows()