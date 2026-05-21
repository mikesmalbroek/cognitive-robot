import cv2
from cv2 import aruco

# Setup
dict_ = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dict_, params)

cap = cv2.VideoCapture(0)  # of /dev/video0 op Linux

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detectie
    corners, ids, _ = detector.detectMarkers(frame)
    
    # Teken markers op het beeld
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        for id_ in ids.flatten():
            if id_ == 0:
                cv2.putText(frame, "STATION A", (50, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            elif id_ == 1:
                cv2.putText(frame, "STATION B", (50, 100), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    cv2.imshow('ArUco Test', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()