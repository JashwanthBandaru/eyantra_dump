#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
import math


class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		self.Image=0
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw
		
		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		 # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.img = np.empty([])
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker
		
		
		

	# Callback function of amera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			self.process(self.img)

		except CvBridgeError as e:
			print(e)
			return

	def process(self,data):
		gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
		aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250)
		parameters = aruco.DetectorParameters_create()
		corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) 
		rate = rospy.Rate(10)  # 10Hz Frequency

		Detected_ArUco_markers = {}
	
		for i in range(len(ids)):
			Detected_ArUco_markers[ids[i][0]]=corners[i]
		
		for ids, corner in Detected_ArUco_markers.items():
			corner = corner[0]
			# Since angle is atan2(-y,x), then converting that to degrees
			top_right_angle = (math.degrees(math.atan2(-corner[1][1] + corner[3][1], corner[1][0] - corner[3][0]))) % 360
			angle = (top_right_angle + 45) % 360

			center_x=int((corner[0][0]+corner[2][0]+corner[1][0]+corner[3][0])/4)
			center_y=int((corner[0][1]+corner[2][1]+corner[3][1]+corner[1][1])/4)
			self.marker_msg.id=ids
			self.marker_msg.x = center_x
			self.marker_msg.y = center_y
			self.marker_msg.yaw= int(angle)
			self.marker_pub.publish(self.marker_msg)
			rate.sleep()

			
	# def publish_data(self):
	# 	self.marker_pub.publish(self.marker_msg)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()