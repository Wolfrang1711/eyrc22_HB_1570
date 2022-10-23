#!/usr/bin/env python3

"""
Author : Pratik, Romala
"""

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
		rospy.Subscriber("/odom", Odometry, odometryCb)

		# declaring that the node subscribes to task1_goals along with the other declarations of publishing and subscribing
		rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)

		# Declaring a Twist message
		self.vel = Twist()

		# For maintaining control loop rate.
		rate = rospy.Rate(100)

		# Initialise the required variables
		self.hola_x = 0.0
		self.hola_y = 0.0
		self.hola_theta = 0.0
		self.index = 0

		# desired goal points defined
		self.x_goals = []
		self.y_goals = []
		self.theta_goals = []

		# Control Loop goes here
		while not rospy.is_shutdown():

			while index < len(self.x_goals):

				

				
				
				# Finally implementing a P controller to react to the error with velocities in x, y and theta
				prop_x = x
				prop_y = y
				prop_th = error_th
				balance_x = (kp_l*prop_x) 
				balance_y = (kp_l*prop_y)
				balance_th = (kp_a*prop_th)

				# Updating balanced speed
				vel_x = balance_x
				vel_y = balance_y
				vel_z = balance_th
				
				# Moving and Orienting till goal is reached
				if(error_d < dist_thresh and abs(error_th) < angle_thresh):
					
					# Stopping
					vel.linear.x = 0.0
					vel.linear.y = 0.0
					vel.angular.z = 0.0
					pub.publish(vel)

					# Stopping for 1 sec
					rospy.sleep(1)

					# Updating goals				
					index += 1

				else:

					vel.linear.x =  vel_x 
					vel.linear.y =  vel_y 
					vel.angular.z = vel_z
					pub.publish(vel)
				
			rate.sleep()

`
		





def odometryCb(msg):
	global hola_x, hola_y, hola_theta

	# code to take the msg and update the three variables
	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation

	_, _, hola_theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	
def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:

		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def global_error(self):

	# calculating error in global frame
	self.error_x = self.x_goals[self.index] - self.hola_x
	self.error_y = self.y_goals[self.index] - self.hola_y
	self.error_th = self.theta_goals[self.index] - self.hola_theta
	self.error_d = np.linalg.norm(np.array((self.error_x, self.error_y)) - np.array((0,0)))

	# self.error_d = math.sqrt((pow(error_x,2) + pow(error_y,2)))

def body_error(self):

	# Calculating error in body frame		
	self.shifted_x = self.x_goals[self.index] - self.hola_x
	self.shifted_y = self.y_goals[self.index] - self.hola_y
	self.x = self.shifted_x * math.cos(self.hola_theta) + self.shifted_y * math.sin(self.hola_theta)
	self.y = - self.shifted_x * math.sin(self.hola_theta) + self.shifted_y * math.cos(self.hola_theta)


def main():
	global hola_x, hola_y, hola_theta, x_goals, y_goals, theta_goals

	

	

	
	
	# For maintaining control loop rate.
	rate = rospy.Rate(100)

	# Initialising variables that may be needed for the control loop for defining desired goal-pose and also Kp values for the P Controller
	dist_thresh = 0.05
	angle_thresh = math.pi/180
	kp_l = 0.8
	kp_a = 4.0

	
if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
