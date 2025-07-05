# import cv2 as cv
from cv2 import destroyAllWindows
import numpy as np
import cv2
import cv2.aruco as aruco

# Load the predefined dictionary
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Generate the marker
markerImage = np.zeros((200, 200), dtype=np.uint8)
markerImage = cv2.aruco.drawMarker(dictionary, 33, 200, markerImage, 1)

cv2.imshow("sh",markerImage)
k=cv2.waitKey(0)
if  k==27:
    cv2.destroyAllWindows()