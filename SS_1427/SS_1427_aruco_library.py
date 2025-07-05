#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

	Detected_ArUco_markers = {}
	## enter your code here ##

	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)  #

	for i in range(len(ids)):
		Detected_ArUco_markers[ids[i][0]]=corners[i][0]

	return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	## enter your code here ##

	for id in Detected_ArUco_markers.keys():

		center_x=(Detected_ArUco_markers[id][0][0]+Detected_ArUco_markers[id][2][0]+Detected_ArUco_markers[id][1][0]+Detected_ArUco_markers[id][3][0])/4
		center_y=(Detected_ArUco_markers[id][0][1]+Detected_ArUco_markers[id][2][1]+Detected_ArUco_markers[id][3][1]+Detected_ArUco_markers[id][1][1])/4

		top_center_x=(Detected_ArUco_markers[id][0][0]+Detected_ArUco_markers[id][1][0])/2
		top_center_y=(Detected_ArUco_markers[id][0][1]+Detected_ArUco_markers[id][1][1])/2

		x2 = top_center_x-center_x
		y2 = top_center_y-center_y

		if top_center_y<center_y:
			theta = math.atan2(-y2, x2)
			angle = math.degrees(theta) 
		else:
			theta = math.atan2(-y2, x2)
			angle = 360+math.degrees(theta)

		if top_center_y == center_y and center_x > top_center_x:
			angle = 180.0
		elif top_center_y == center_y and center_x < top_center_x:
			angle = 0.0	
		elif top_center_x == center_x and top_center_y < center_y:
			angle = 90.0
		elif top_center_x == center_x and top_center_y > center_y:
			angle = 270.0	

		ArUco_marker_angles[id]= int(angle)

	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

	## enter your code here ##

	for id in Detected_ArUco_markers.keys():
		
		center_x=int((Detected_ArUco_markers[id][0][0]+Detected_ArUco_markers[id][2][0]+Detected_ArUco_markers[id][1][0]+Detected_ArUco_markers[id][3][0])/4)
		center_y=int((Detected_ArUco_markers[id][0][1]+Detected_ArUco_markers[id][2][1]+Detected_ArUco_markers[id][3][1]+Detected_ArUco_markers[id][1][1])/4)
		top_center_x=int((Detected_ArUco_markers[id][0][0]+Detected_ArUco_markers[id][1][0])/2)
		top_center_y=int((Detected_ArUco_markers[id][0][1]+Detected_ArUco_markers[id][1][1])/2)

		cv2.circle(img,(center_x,center_y),5,(0,0,255),-1)
		cv2.circle(img,(Detected_ArUco_markers[id][0][0],Detected_ArUco_markers[id][0][1]), 5, (125,125,125), -1)
		cv2.circle(img,(Detected_ArUco_markers[id][1][0],Detected_ArUco_markers[id][1][1]), 5, (0,255,0), -1)
		cv2.circle(img,(Detected_ArUco_markers[id][2][0],Detected_ArUco_markers[id][2][1]), 5, (180,105,255), -1)
		cv2.circle(img,(Detected_ArUco_markers[id][3][0],Detected_ArUco_markers[id][3][1]), 5, (255,255,255), -1)
		cv2.line(img,(center_x,center_y),(top_center_x,top_center_y),(255,0,0),3)

		cv2.putText(img,str(id),(center_x+20,center_y), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),3,cv2.LINE_AA)
		cv2.putText(img,str(ArUco_marker_angles[id]),(center_x-90,center_y), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,255,0),3,cv2.LINE_AA)

	return img






