# Sphero_Starter_Code

This repo has some utility functions that will be useful for working with the spheros and using ArUco markers.

### ``marker_generator.py``
To start, you'll need to generate the ArUco tags that you'll be using. Modify the code in the main function to create as many as you'd like. You'll need to specify: 

- The dictionary size (i.e. the number of unique markers available). By default, this code generates a 4x4 dictionary with 50 markers.
- The marker ID (this can be whatever you like, but it must be less than the dictionary size). 
- The marker size in pixels
- Output file name