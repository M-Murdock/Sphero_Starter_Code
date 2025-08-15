class Aruco:
    
    def __init__(self):
        self.corners = []
        
    def update_corners(self, corners):
        self.corners = corners
        
    def get_last_corners(self):
        return self.corners