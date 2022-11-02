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
# Filename:		controller.py
# Functions:	__init__(), task2_goals_Cb(), aruco_feedback_Cb(), global_error(), body_error(), PID(), inverse_kinematics()
# Nodes:		Publishing node:Wrench, Subscribing nodes:Pose2D, PoseArray

################### IMPORT MODULES #######################

import rospy
import math		                           
import numpy as np
from geometry_msgs.msg import Wrench     	# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray		# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
from tf.transformations import euler_from_quaternion	# Convert angles


class controller:

	def __init__(self):
		
		#initialising node named "controller_node"
		rospy.init_node('controller_node')


		# Initialise the required variables
		self.hola_x = 0.0
		self.hola_y = 0.0 
		self.hola_theta = 0.0
		self.x = 0
		self.y = 0
		self.error_th = 0
		self.error_d = 0
		self.index = 0

		# defining variables required for control loop
		self.dist_thresh = 0.05
		self.angle_thresh = 0.1
	
		# Initialising Kp values for the P Controller
		self.kp_linear = 2.0
		self.kp_angular = 2.0

		#declaring goal points array
		self.x_goals = [50,350,50,250,250]
		self.y_goals = [350,50,50,350,50]
		self.theta_goals = [0, 0, 0, 0, 0]     

		self.right_wheel_pub = None
		self.left_wheel_pub = None
		self.front_wheel_pub  = None


		# declaring wrench node
		self.vector_f = Wrench()
		self.vector_r = Wrench()
		self.vector_l = Wrench()


		#initialising publisher of /right_wheel_force, /front_wheel_force and /left_wheel_force respectively
		self.right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
		self.front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
		self.left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

		#initialising subscriber of detected_aruco and task2_goals respectively
		rospy.Subscriber('detected_aruco',Pose2D,self.aruco_feedback_Cb)
		rospy.Subscriber('task2_goals',PoseArray,self.task2_goals_Cb)

		# For maintaining control loop rate.
		self.rate = rospy.Rate(100)

                                                                                                                                                                                                                                                                                                       
			
		while not rospy.is_shutdown():

			while self.index < len(self.x_goals):

				# Calculate Error from feedback
				self.global_error()

				# Change the frame by using Rotation Matrix (If you find it required)
				self.body_error()

				# Calculate the required velocity of bot for the next iteration(s)
				self.PID()
				
				# Find the required force vectors for individual wheels from it.(Inverse Kinematics)
				self.inverse_kinematics()

				# print(self.hola_x, self.hola_y, self.hola_theta)

			 	
				# Moving and Orienting till goal is reached
				if(self.error_d < self.dist_thresh and abs(self.error_th) < self.angle_thresh):
					
					# Apply appropriate force vectors
					# Stopping

					self.vector_r.force.x = 0.0
	
					self.vector_f.force.x = 0.0
			
					self.vector_l.force.x = 0.0
			
					
					# print("stop")

					self.right_wheel_pub.publish(self.vector_r)
					self.front_wheel_pub.publish(self.vector_f)
					self.left_wheel_pub.publish(self.vector_l)

					# Updating goals				
					self.index += 1

				else:
					
					# Apply appropriate force vectors
					self.vector_r.force.x = self.vr

					self.vector_f.force.x = self.vf

					self.vector_l.force.x = self.vl


					# print("moving")
	

					self.right_wheel_pub.publish(self.vector_r)
					self.front_wheel_pub.publish(self.vector_f)					
					self.left_wheel_pub.publish(self.vector_l)

			

			self.rate.sleep()

	def task2_goals_Cb(self, msg):
		
		self.x_goals.clear()
		self.y_goals.clear()
		self.theta_goals.clear()

		for waypoint_pose in msg.poses:
			self.x_goals.append(waypoint_pose.position.x)
			self.y_goals.append(waypoint_pose.position.y)

			orientation_q = waypoint_pose.orientation
			orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
			theta_goal = euler_from_quaternion (orientation_list)[2]
			self.theta_goals.append(theta_goal)

	def aruco_feedback_Cb(self, msg):

		# code to take the msg and update the three variables
		self.hola_x = msg.x
		self.hola_y = msg.y
		self.hola_theta = msg.theta


	def global_error(self):

		# calculating error in global frame
		self.error_x = self.x_goals[self.index] - self.hola_x
		self.error_y = self.y_goals[self.index] - self.hola_y
		self.error_th = self.theta_goals[self.index] - self.hola_theta
		self.error_d = np.linalg.norm(np.array((self.error_x, self.error_y)) - np.array((0,0)))
	
	def body_error(self):

		# Calculating error in body frame		
		self.shifted_x = self.x_goals[self.index] - self.hola_x
		self.shifted_y = self.y_goals[self.index] - self.hola_y
		self.x = self.shifted_x * math.cos(self.hola_theta) + self.shifted_y * math.sin(self.hola_theta)
		self.y = -self.shifted_x * math.sin(self.hola_theta) + self.shifted_y * math.cos(self.hola_theta)

	def PID(self):

		# Finally implementing a P controller to react to the error with velocities in self.x, self.y and theta
		prop_x = self.x
		prop_y = self.y
		prop_th = self.error_th
		balance_x = (self.kp_linear*prop_x) 
		balance_y = (self.kp_linear*prop_y)
		balance_th = (self.kp_angular*prop_th)

		# Updating balanced speed
		self.vel_x = balance_x
		self.vel_y = balance_y
		self.vel_z = balance_th		

	def inverse_kinematics(self):

		mat1 = ([0.667, 0, -0.333],
		   		[-0.333, 0.577, -0.333],
				[-0.333, -0.577, -0.333])

		mat2 = ([self.vel_x],
				[self.vel_y],
				[self.vel_z])	

		pro = np.dot(mat1,mat2)

		self.vf, self.vl, self.vr = float(pro[0]), float(pro[1]), float(pro[2])

		# print(self.vf, self.vl, self.vr)

if __name__ == "__main__":
	try:
		controller()
	except rospy.ROSInterruptException:
		pass
