import cv2
import numpy as np
        
    
def get_color_at_point(x, y, img_path):
    img = cv2.imread(img_path)  # BGR format
    
    # Example coordinate (x, y)
    x, y = 100, 150  

    # Get exact pixel color
    color = img[y, x]   # (B, G, R)
    print("Exact pixel color (BGR):", color)

    # Get average color in a region around the coordinate
    radius = 5
    roi = img[y-radius:y+radius+1, x-radius:x+radius+1]
    avg_color = roi.mean(axis=(0,1))
    print("Average color around point (BGR):", avg_color)