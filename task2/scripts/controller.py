#!/usr/bin/env python3

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

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys			# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray		# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful
import numpy as np

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################
class controller:

	def __init__(self):
		
		self.PI = 3.14

		self.x_goals = []
		self.y_goals = []
		self.theta_goals = []

		self.hola_x = 0.0
		self.hola_y = 0.0 
		self.hola_theta = 0.0
		self.x = 0
		self.y = 0
		self.error_th = 0
		self.error_d = 0

		# Initialising Kp values for the P Controller
		self.kp_linear = 0.8
		self.kp_angular = 4.0

		self.right_wheel_pub = None
		self.left_wheel_pub = None
		self.front_wheel_pub = None

		rospy.init_node('controller_node')

		signal.signal(signal.SIGINT, self.signal_handler)

		# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
		#	Use the below given topics to generate motion for the robot.
		self.right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
		self.front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
		self.left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

		rospy.Subscriber('detected_aruco',Pose2D,self.aruco_feedback_Cb)
		rospy.Subscriber('task2_goals',PoseArray,self.task2_goals_Cb)
		
		rate = rospy.Rate(100)
		

		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 
		#	-> Make use of the logic you have developed in previous task to go-to-goal.
		#	-> Extend your logic to handle the feedback that is in terms of pixels.
		#	-> Tune your controller accordingly.
		# 	-> In this task you have to further implement (Inverse Kinematics!)
		#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
		#      given velocity of the chassis (Vx, Vy, W)
		#	   

			
		while not rospy.is_shutdown():
			
			# Calculate Error from feedback
			self.global_error()

			# Change the frame by using Rotation Matrix (If you find it required)
			self.body_error()

			# Calculate the required velocity of bot for the next iteration(s)
			self.PID()
			
			# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

			# Apply appropriate force vectors

			# Modify the condition to Switch to Next goal (given position in pixels instead of meters)

			rate.sleep()

		############################################


	##################### FUNCTION DEFINITIONS #######################

	# NOTE :  You may define multiple helper functions here and use in your code

	def signal_handler(sig, frame):
		
		# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
		print('Clean-up !')
		#cleanup()
		sys.exit(0)

	#def cleanup():
		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 
		#	-> Not mandatory - but it is recommended to do some cleanup over here,
		#	   to make sure that your logic and the robot model behaves predictably in the next run.

		############################################

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
		############ ADD YOUR CODE HERE ############

		# code to take the msg and update the three variables
		self.hola_x = msg.x
		self.hola_y = msg.y
		self.hola_theta = msg.theta

		# INSTRUCTIONS & HELP : 
		#	-> Receive & store the feedback / coordinates found by aruco detection logic.
		#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

		############################################

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

	# def inverse_kinematics():
		############ ADD YOUR CODE HERE ############

		# INSTRUCTIONS & HELP : 
		#	-> Use the target velocity you calculated for the robot in previous task, and
		#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
		#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
		############################################

if __name__ == "__main__":
	try:
		controller()
	except rospy.ROSInterruptException:
		pass

