import cv2
from matplotlib import pyplot as plt
from aruco_obj import Aruco
import asyncio


class ArucoDetector:
    
    def __init__(self):
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.aruco_tag = Aruco()
        
    async def visualize(self):
        # Connect to webcam
        self.cap = cv2.VideoCapture(0)

        while self.cap.isOpened():
            ret, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = self.detector.detectMarkers(gray)
            # print("corners of aruco tag:", corners)
            if not len(corners) == 0:
                self.aruco_tag.update_corners(corners)
                print("detected")

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            cv2.imshow('Detected markers', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        self.cap.release
        cv2.destroyAllWindows()
        
    def begin_visualization(self):
        asyncio.run(self.visualize())
        
    def end_visualization(self):
        pass