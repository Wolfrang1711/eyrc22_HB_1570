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

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik, Romala ]
# Filename:		controller.py
# Functions:	task2_goals_Cb(), aruco_feedback_Cb(), global_error(), body_error(), PID(), inverse_kinematics()
# Nodes:		Publishing nodes: /right_wheel_force, /front_wheel_force, /left_wheel_force
#    			Subscribing nodes: /detected_aruco, /task2_goals

################### IMPORT MODULES #######################

import rospy

from geometry_msgs.msg import Wrench     	# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray		# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import math		                            # If you find it useful
import numpy as np

from tf.transformations import euler_from_quaternion	# Convert angles

class controller:

	def __init__(self):

		# initialising node named "controller_node"
		rospy.init_node('controller_node')

		# initialising publisher of /right_wheel_force, /front_wheel_force, /left_wheel_force 
		self.right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
		self.front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
		self.left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

		# initialising subscriber of /detected_aruco and /task2_goals
		rospy.Subscriber('detected_aruco',Pose2D,self.aruco_feedback_Cb)
		rospy.Subscriber('task2_goals',PoseArray,self.task2_goals_Cb)
		
		# initialising goalpoints array
		self.x_goals = []
		self.y_goals = []
		self.theta_goals = []                                                                                                                                                                                                                                                                                                            

		# initialising required variables
		self.hola_x = 0.0
		self.hola_y = 0.0 
		self.hola_theta = 0.0
		self.x = 0
		self.y = 0
		self.error_th = 0
		self.error_d = 0
		self.index = 0

		# declaring thresholds
		self.dist_thresh = 0.05
		self.angle_thresh = 5 * (math.pi/180)

		# Initialising Kp values for the P Controller
		self.kp_linear = 5.0
		self.kp_angular = 35.0

		# declaring wrench message for 3 wheels 
		self.vector_f = Wrench()
		self.vector_r = Wrench()
		self.vector_l = Wrench()

		# For maintaining control loop rate.
		rate = rospy.Rate(100)
		
		# control loop
		while not rospy.is_shutdown():

			while self.index < len(self.theta_goals):

				# Calculating Global Error from feedback
				self.global_error()

				# Changing to robot frame by using Rotation Matrix 
				self.body_error()

				# Calculating the required velocity of bot 
				self.PID()
				
				# Finding the required force vectors for individual wheels from it
				self.inverse_kinematics()

				# Moving and Orienting till goal is reached
				if(self.error_d < self.dist_thresh and abs(self.error_th) < self.angle_thresh):
					
				# Applying appropriate force vectors

					# Stopping
					self.vector_r.force.x = 0.0
					self.vector_f.force.x = 0.0
					self.vector_l.force.x = 0.0

					self.right_wheel_pub.publish(self.vector_r)
					self.front_wheel_pub.publish(self.vector_f)
					self.left_wheel_pub.publish(self.vector_l)
					
					# Stopping for 1 sec
					rospy.sleep(1)

					# Updating goals				
					self.index += 1

				else:

					self.vector_r.force.x = self.vr
					self.vector_f.force.x = self.vf
					self.vector_l.force.x = self.vl

					self.right_wheel_pub.publish(self.vector_r)
					self.front_wheel_pub.publish(self.vector_f)
					self.left_wheel_pub.publish(self.vector_l)

			rate.sleep()
		
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

		# taking the msg and updating the three variables
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
		self.shifted_y = self.hola_y - self.y_goals[self.index] 
		self.x = self.shifted_x * math.cos(self.hola_theta) + self.shifted_y * math.sin(self.hola_theta)
		self.y = -self.shifted_x * math.sin(self.hola_theta) + self.shifted_y * math.cos(self.hola_theta)

	def PID(self):

		# implementing a P controller to react to the error with velocities in self.x, self.y and theta
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

		# inverse kinematics matrix derived through calculation
		mat1 = ([0.667, 0, -0.333],
		   		[-0.333, 0.577, -0.333],
				[-0.333, -0.577, -0.333])

		# velocity matrix
		mat2 = ([self.vel_x],
				[self.vel_y],
				[self.vel_z])	

		# wheel force matrix
		res = np.dot(mat1,mat2)

		# assigning force to respective wheels
		self.vf = float(res[0])
		self.vl = float(res[1])
		self.vr = float(res[2])

if __name__ == "__main__":
	try:
		controller()
	except rospy.ROSInterruptException:
		pass