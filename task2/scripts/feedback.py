#!/usr/bin/env python

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		HB-1570
# Author List:	Pratik, Romala
# Filename:		feedback.py
# Functions:    __init__(), callback(), aruco_detection() 
#			
# Nodes:		Publishing node: detected_aruco
# 				Subscribing node: overhead_cam/image_raw


######################## IMPORT MODULES ##########################
		
import rospy 				
from sensor_msgs.msg import Image 	      # Image is the message type for images in ROS
from cv_bridge import CvBridge	          # Package to convert between ROS and OpenCV Images
import cv2				                  # OpenCV Library
import math			
from geometry_msgs.msg import Pose2D	  # Required to publish ARUCO's detected position & orientation


class Feedback:

	def __init__(self):

		# initialising node named "aruco_feedback_node"
		rospy.init_node('aruco_feedback_node')  

		# initialising publisher and subscriber of detected_aruco and overhead_cam/image_raw respectively
		self.aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)  
		rospy.Subscriber('overhead_cam/image_raw', Image, self.callback)

		# Declaring a Pose2D message
		self.aruco_msg = Pose2D()

		# Declaring CvBridge to convert between ROS and OpenCV image
		self.br = CvBridge()


		# Initialising the required variables
		self.cX = 0.0
		self.cY = 0.0
		self.angle = 0.0
		self.current_frame = None


		# defining aruco code dictionary and parameters
		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		# For maintaining control loop rate.
		self.rate = rospy.Rate(10)


		# control loop
		while not rospy.is_shutdown():

			if self.current_frame is None: 
				continue

			self.aruco_detection()

			# updating x,y,theta values
			self.aruco_msg.x = self.cX
			self.aruco_msg.y = self.cY
			self.aruco_msg.theta = self.angle

			#display the position of the bot
			print("position of the bot initially is", (self.aruco_msg.x, self.aruco_msg.y, self.aruco_msg.theta))
		

			# to display the image frame
			cv2.imshow("frame", self.current_frame)
			if cv2.waitKey(30) & 0xFF == ord('q'):
					rospy.spin()

			#publishing the node Pose2D
			self.aruco_publisher.publish(self.aruco_msg)
			self.rate.sleep() 		


	def callback(self,data):
		# Bridge to Convert ROS Image message to OpenCV current_frame
		
		rospy.loginfo("receiving camera frame")

		self.get_frame = self.br.imgmsg_to_cv2(data, desired_encoding="mono8")
		self.current_frame = cv2.resize(self.get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)


	def aruco_detection(self):

		print("detecting arucomark..")

		#detecting arucomark
		corners, ids, _ = cv2.aruco.detectMarkers(self.current_frame, self.arucoDict, parameters=self.arucoParams)

		if len(corners) > 0:

		# flatten the Aruco IDs list
			ids = ids.flatten()

			# loop over the detected Aruco corners
			for (markerCorner, _) in zip(corners, ids):

				# extracting the marker corners 
				self.corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = self.corners

				# converting each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))			

				# computing the center (x, y)-coordinates of the Aruco marker
				self.cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				self.cY = int((topLeft[1] + bottomRight[1]) / 2.0)

				# for calculating the orientation of aruco marker
				(midx, midy) = int((topRight[0] + topLeft[0]) / 2), int((topRight[1] + topLeft[1]) / 2)
				robo_or = math.atan2(midy-self.cY, midx-self.cX ) 						
				self.angle = robo_or 
		
	
if __name__=='__main__':	
	aruco = Feedback()
	try:
		if not rospy.is_shutdown():
			Feedback()
	except rospy.ROSInterruptException as e:
			print(e)

	
