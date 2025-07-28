import cv2
import numpy as np
import matplotlib.pyplot as plt

# Define the dictionary we want to use
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)



for i in range (0,20):
    marker_size = 150  # Size in pixels
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, i, marker_size)
    cv2.imwrite('marker' + str(i) + '.jpg', marker_image)

