from aruco_detector import ArucoDetector
from aruco_obj import Aruco
import cv2

detector = ArucoDetector()
detector.begin_visualization()
# while True:
# if cv2.waitKey(1) & 0xFF == ord('q'):
#     detector.end_visualization()
    # break