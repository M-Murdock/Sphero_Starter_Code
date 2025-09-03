from aruco_detector import ArucoDetector
from aruco_obj import Aruco
import asyncio
import cv2
import numpy as np
import image_process_utils


async def main():
    detector = ArucoDetector()
    await detector.begin_visualization()

    # run for 10 seconds
    await asyncio.sleep(10)

    await detector.end_visualization()
    
    # get the dictionary of Aruco markers
    # print(detector.get_tags())
    
    # get the coordinates of the centers of each aruco tag at their most recent location
    # print(detector.get_last_tag_centers())
    
    # get the coordinates of the corners of each aruco tag at their most recent location
    # print(detector.get_last_tag_corners())
    
    # get all the past coordinates of the centers and corners for each aruco tag 
    # print(detector.get_all_tag_centers())
    # print(detector.get_all_tag_corners())
    
    # take a picture and get the color at a given point in the image
    # detector.take_pic()
    # print(image_process_utils.get_color_at_point(300, 500, "captured_image.jpg"))
    # print(image_process_utils.get_avg_color_at_point(300, 500, radius=5, img_path="captured_image.jpg"))
    

    

asyncio.run(main())

