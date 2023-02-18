#!/usr/bin/env python

# Team ID:		    [ eYRC#HB#1570 ]
# Author List:		[ Pratik, Romala ]

import numpy				                            # If you find it required
import rospy 				
from sensor_msgs.msg import Image 	                    # Image is the message type for images in ROS
from cv_bridge import CvBridge  				        # Package to convert between ROS and OpenCV Images
import cv2				                                # OpenCV Library
import math				                                # If you find it required
from geometry_msgs.msg import Pose2D	                # Required to publish ARUCO's detected position & orientation

class ArucoFeedback():

	def __init__(self):

		# initialising node named "aruco_feedback_node"
		rospy.init_node('aruco_feedback_node') 

		# initialising publisher and subscriber of /detected_aruco and /overhead_cam/image_raw respectively
		self.aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)
		rospy.Subscriber('usb_cam/image_rect', Image, self.callback)

		# declaring a Pose2D message
		self.aruco_msg = Pose2D()

		# initialising the required variables
		self.current_frame = None
		self.zoom = None
		self.final_frame = None
		self.x = 0.0
		self.y = 0.0
		self.angle = 0.0

		# defining aruco dictionary and parameters
		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		while not rospy.is_shutdown():

			# skipping empty frames
			if self.final_frame is None: 
				continue

			cv2.imshow('image',self.final_frame)
			
			# calling function for aruco detection
			self.aruco_detection()

			print("sending odom: ", self.x, self.y, self.angle)

			# updating and publishing pose values
			self.aruco_msg.x = self.x
			self.aruco_msg.y = self.y 
			self.aruco_msg.theta = self.angle

			self.aruco_publisher.publish(self.aruco_msg)

			cv2.waitKey(1)

	def callback(self, data):

		# Bridge is Used to Convert ROS Image message to OpenCV image
		br = CvBridge()

		# Receiving raw image in a "grayscale" format and resizing image
		self.get_frame = br.imgmsg_to_cv2(data, desired_encoding="rgb8")

		cy, cx = [ i/2 for i in self.get_frame.shape[:-1] ] 
    
		rot_mat = cv2.getRotationMatrix2D((cx,cy), 0, 1.5)
		self.zoom = cv2.warpAffine(self.get_frame, rot_mat, self.get_frame.shape[1::-1], flags=cv2.INTER_LINEAR)

		self.current_frame = cv2.resize(self.zoom, (720, 500), interpolation = cv2.INTER_LINEAR)
		self.final_frame = self.current_frame[0:500,0:500]

	def aruco_detection(self):		
		
		# finding corners and aruco ids
		corner, ids, _ = cv2.aruco.detectMarkers(self.final_frame, self.arucoDict, parameters = self.arucoParams)	
		if len(corner) > 0:
			# flatten the Aruco IDs list
			ids = ids.flatten()

			for (markerCorner, markerID) in zip(corner, ids):

				if(markerID == 15):

					# extracting the marker corner (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
					corner = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corner
			
					# converting each of the (x, y)-coordinate pairs to integers
					topRight = (int(topRight[0]), int(topRight[1]))
					bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
					bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
					topLeft = (int(topLeft[0]), int(topLeft[1]))

					# computing the centre of the Aruco marker
					self.x = int((topLeft[0] + bottomRight[0]) / 2.0)
					self.y = int((topLeft[1] + bottomRight[1]) / 2.0)

					# finding midpoint of rightside of aruco marker
					(midx, midy) = int((topRight[0] + bottomRight[0]) / 2), int((topRight[1] + bottomRight[1]) / 2)
					
					# finding orientation
					self.angle = math.atan2((self.y - midy),(midx - self.x)) 
					
					cv2.putText(img = self.final_frame, text = str(self.x), org = (10, 24), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (255, 255, 255), thickness = 2)
					cv2.putText(img = self.final_frame, text = str(self.y), org = (90, 24), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (255, 255, 255), thickness = 2)
					cv2.putText(img = self.final_frame, text = str(round(self.angle,3)), org = (10, 60), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (255, 255, 255), thickness = 2)										

if __name__ == '__main__':

	aruco = ArucoFeedback()
	
	try:
		if not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)
