import cv2
from matplotlib import pyplot as plt

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Connect to webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)
    print("corners of aruco tag:", corners)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow('Detected markers', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release
cv2.destroyAllWindows()

# sphero ar tracker object 
#   start/stop visualizer
#   get image position    
#   get all tags
#   get position of specific tag
#   start/stop recording image coordinates
#   wrap some basic opencv functions: getpixelcolor/getaveragecolor/blob detector (remind Elaine to look for this code)