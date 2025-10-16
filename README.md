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

- ``visualize`` - the main code for starting the camera and look for ArUco markers. When a tag is identified, it will be highlighted in green and the tag's ID will visible in the center of the marker. The code automatically updates the position of that tag within your camera frame.
- ``begin_visualization`` and ``end_visualization`` - start and stop the visualization.
- ``take_pic`` - takes a picture using the connected camera and saves it to a file.
- ``get_tags`` - returns a dictionary of the markers which have been detected. The key represents the unique ID of the marker and the value is the ArUco tag object.
- ``get_last_tag_corners`` - returns a dictionary of the last corner coordinates where each marker was identified.
- ``get_last_tag_centers`` - returns a dictionary of the last center coordinates where each marker was identified.
- ``get_all_tag_corners`` - returns a dictionary of all the corner coordinates where each marker was previously identified
- ``get_all_tag_centers`` - returns a dictionary of all the center coordinates where each marker was previously identified


### ``sphero.py``
This is basic code for connecting to a sphero and controlling it via keyboard input. Note that you will need to replace line 9 with your sphero's serial number (you can find this on the side of your sphero).

### ``sphero_nav.py``
This is basic code for having the sphero navigate a pre-specified path. Note: this code has not been tested, so use with caution.

### ``image_process_utils.py``
This contains basic utility functions for working with images. It includes functions for grabbing the precise color at a given coordinate, and the average color in a region.

### ``example.py``
This has example usages of the aruco detector.

## Getting Started with SpheroV2
- To get started using this code, you'll need to have the SpheroV2 library installed: ``pip install spherov2``. For additional information, see the docs [here](https://github.com/artificial-intelligence-class/spherov2.py/tree/main).
- You will also need to use Python 3.7-3.12, since some actions (such as displaying animations or colors on the Sphero) are not supported with the latest Python versions. 