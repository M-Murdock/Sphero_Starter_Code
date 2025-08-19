# Sphero Starter Code

This repo has some utility functions that will be useful for working with the spheros and using ArUco markers. The most relevant ones include:

### ``marker_generator.py``
To start, you'll need to generate the ArUco tags that you'll be using. Modify the code in the main function to create as many as you'd like. You'll need to specify: 

- The dictionary size (i.e. the number of unique markers available). By default, this code generates a 4x4 dictionary with 50 markers.
- The marker ID (this can be whatever you like, but it must be less than the dictionary size). 
- The marker size in pixels
- Output file name

### ``aruco_detector.py``
This contains the code you'll need to track your ArUco tags. The functions include:

- ``visualize`` - the main code for starting the camera and look for ArUco markers. When one is identified, it will be highlighted in green and the tag's ID will visible in the center of the marker. The code automatically records the position of the tag (coordinates of each corner) within your camera frame. 
- ``begin_visualization`` and ``end_visualization`` - start and stop the visualization.
- ``get_tags`` - returns a dictionary of the markers which have been detected. The key represents the unique ID of the marker and the value is the ArUco tag object.
- ``get_last_tag_corners`` - returns a dictionary of the last coordinates where each marker was identified.


### ``sphero.py``
This is basic code for connecting to a sphero and controlling it via keyboard input. Note that you will need to replace line 9 with your sphero's serial number (you can find this on the side of your sphero).