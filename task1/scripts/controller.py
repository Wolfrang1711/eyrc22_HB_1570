#!/usr/bin/env python3

"""
Author : Pratik, Romala
"""

from cv2 import sqrt
import rospy
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf.transformations import euler_from_quaternion

class controller:

	def __init__(self):
		
		# initialising node named "controller"
		rospy.init_node("controller", anonymous=True)
		
		# initialising publisher and subscriber of cmd_vel and odom respectively
		self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		rospy.Subscriber("/odom", Odometry, self.odometryCb)

		# declaring that the node subscribes to task1_goals along with the other declarations of publishing and subscribing
		rospy.Subscriber('task1_goals', PoseArray, self.task1_goals_Cb)

		# Declaring a Twist message
		self.vel = Twist()

		# For maintaining control loop rate.
		rate = rospy.Rate(100)

		# Initialise the required variables
		self.hola_x = 0.0
		self.hola_y = 0.0
		self.hola_theta = 0.0
		self.index = 0

		# Initialising variables that may be needed for the control loop for defining desired goal-pose and also Kp values for the P Controller
		self.dist_thresh = 0.05
		self.angle_thresh = math.pi/180
		self.x = 0
		self.y = 0
		self.error_th = 0
		self.error_d = 0

		# Initialising Kp values for the P Controller
		self.kp_linear = 0.8
		self.kp_angular = 4.0

		# desired goal points defined
		self.x_goals = []
		self.y_goals = []
		self.theta_goals = []

		# Control Loop goes here
		while not rospy.is_shutdown():

			while self.index < len(self.x_goals):

				self.global_error()
				self.body_error()

				self.PID()
				
				# Moving and Orienting till goal is reached
				if(self.error_d < self.dist_thresh and abs(self.error_th) < self.angle_thresh):
					
					# Stopping
					self.vel.linear.x = 0.0
					self.vel.linear.y = 0.0
					self.vel.angular.z = 0.0
					self.publisher.publish(self.vel)

					# Stopping for 1 sec
					rospy.sleep(1)

					# Updating goals				
					self.index += 1

				else:

					self.vel.linear.x =  self.vel_x 
					self.vel.linear.y =  self.vel_y 
					self.vel.angular.z = self.vel_z
					self.publisher.publish(self.vel)
				
			rate.sleep()

	def odometryCb(self, msg):

		# code to take the msg and update the three variables
		self.hola_x = msg.pose.pose.position.x
		self.hola_y = msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation

		_, _, self.hola_theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
		
	def task1_goals_Cb(self, msg):

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
	
if __name__ == "__main__":

	try:
		controller()
	except rospy.ROSInterruptException:
		pass
