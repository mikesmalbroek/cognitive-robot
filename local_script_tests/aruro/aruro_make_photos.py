import cv2
from cv2 import aruco

dict_ = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Marker ID 0 voor Station A
marker0 = aruco.generateImageMarker(dict_, id=0, sidePixels=200)
cv2.imwrite('local_script_tests/aruco/aruco_codes/station_A_marker.png', marker0)

# Marker ID 1 voor Station B
marker1 = aruco.generateImageMarker(dict_, id=1, sidePixels=200)
cv2.imwrite('local_script_tests/aruco/aruco_codes/station_B_marker.png', marker1)