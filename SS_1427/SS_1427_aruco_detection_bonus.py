#!/usr/bin/env python3

############## Task1.1 - ArUco Detection ##############
### YOU CAN EDIT THIS FILE FOR DEBUGGING PURPOSEs, SO THAT YOU CAN TEST YOUR ArUco_library.py AGAINST THE VIDEO Undetected ArUco markers.avi###
### BUT MAKE SURE THAT YOU UNDO ALL THE CHANGES YOU HAVE MADE FOR DEBUGGING PURPOSES BEFORE TESTING AGAINST THE TEST IMAGES ###

import numpy as np
import cv2
import cv2.aruco as aruco
import time
from aruco_library import *


cap=cv2.VideoCapture(0)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,  480))

while cap.isOpened():

	ret, img= cap.read()
	if not ret:
		print("cannnot receive frame")
		break
	
	try: 

		Detected_ArUco_markers = detect_ArUco(img)
		angle = Calculate_orientation_in_degree(Detected_ArUco_markers)				## finding orientation of aruco with respective to the menitoned scale in problem statement
		img = mark_ArUco(img,Detected_ArUco_markers,angle)	
		out.write(img)

		cv2.imshow('gray',img)
		if cv2.waitKey(1)==ord("q"):
			break
	except:
		cv2.imshow('gray',img)
		if cv2.waitKey(1)==ord("q"):
			break	

cap.release()
out.release()
cv2.destroyAllWindows()   