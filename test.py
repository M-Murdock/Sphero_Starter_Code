from aruco_detector import ArucoDetector
from aruco_obj import Aruco
import cv2
import asyncio

# detector = ArucoDetector()
# detector.begin_visualization()
async def main():
    detector = ArucoDetector()
    await detector.begin_visualization()

    # run for 10 seconds
    await asyncio.sleep(20)

    # then stop
    await detector.end_visualization()
    
    print(detector.get_tags())

asyncio.run(main())

# detector.end_visualization()
# while Tru# The commented out code block you provided is a common pattern used in OpenCV applications
# for handling user input.
# e:
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         detector.end_visualization()
#         break
    