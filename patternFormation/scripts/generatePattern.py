import cv2
import numpy as np
import json
from PatternClass import *
import os

if __name__ == "__main__":

    img = cv2.imread("/home/mohit/workspace/drone_ws/src/drone_swarming/patternFormation/image_input/circle.png")
    no_of_drones = 12
    grid_unit_meter = 5
    hover_height = 3

    path = "/home/mohit/workspace/drone_ws/src/drone_swarming/patternFormation/pattern_data_output"

    # Create object of Pattern class with above parameters
    pattern = Pattern(img, no_of_drones, grid_unit_meter, hover_height, path)

    # Generating the Pattern points
    pattern.generate_points()

    # Saving the pattern points data (in meters) to the above mentioned path
    pattern.savePattern_data()

    # Highlighting the Pattern with points
    pattern.show_IMG()
