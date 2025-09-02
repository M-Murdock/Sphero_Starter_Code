import numpy as np
# Keeps track of the positions of an aruco tag
class Aruco:
    
    def __init__(self):
        self.corners = []
        self.past_corners = []
        # self.center = None 
        self.past_centers = []
        
    def update_corners(self, corners):
        self.past_corners.append(self.corners)
        self.corners = corners
        self.find_center()
        
        
    def find_center(self):
        # self.corners is [[[x, y], [x,y], [x,y], [x,y]]]
        self.center = np.mean(self.corners[0], axis=1)
        self.past_centers.append(self.center)
                
    def get_last_corners(self):
        return self.corners
    
    def get_last_centers(self):
        return self.center
    
    # Get -1 index of the aruco
    # get movement based on color, 