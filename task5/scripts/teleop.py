#!/usr/bin/env python

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik ]

import rospy

from geometry_msgs.msg import Twist		# Message type used for receiving feedback
from std_msgs.msg import String

import math		                            # If you find it useful
import numpy as np

from tf.transformations import euler_from_quaternion	# Convert angles

class controller:

	def __init__(self):

		# initialising node named "controller_node"
		rospy.init_node('teleop_node')

		self.move = rospy.Publisher('velocity_array', String, queue_size=10)

		rospy.Subscriber('cmd_vel', Twist, self.velocity_feedback)                                                                                                                                                                                                                                                                                                           

		# initialising required variables
		self.velocity = []
		self.vel_x = 0
		self.vel_y = 0
		self.vel_z = 0

		# declaring wrench message for 3 wheels 
		self.data_to_send = String()

		# For maintaining control loop rate.
		rate = rospy.Rate(100)
		
		# control loop
		while not rospy.is_shutdown():

			d = 0.165
			r = 0.029

			# inverse kinematics matrix derived through calculation
			mat1 = ([d, -1, 0],
					[d, 1/2, math.sqrt(3)/2],
					[d, 1/2, -math.sqrt(3)/2])

			# velocity matrix
			mat2 = ([self.vel_z],
					[self.vel_y],
					[self.vel_x])	

			# wheel force matrix
			res = (1/r) * np.dot(mat1,mat2)

			# assigning force to respective wheels
			self.vf = float(res[0])
			self.vr = float(res[1])
			self.vl = float(res[2])

			self.velocity = [self.vf, self.vl, self.vr]
			self.data_to_send = ','.join([str(e) for e in self.velocity])

			self.move.publish(self.data_to_send)

			print(self.data_to_send)

			rate.sleep()
		
	def velocity_feedback(self, msg):

		# taking the msg and updating the three variables
		self.vel_x = msg.linear.x
		self.vel_y = msg.linear.y
		self.vel_z = msg.angular.z

if __name__ == "__main__":
	try:
		controller()
	except rospy.ROSInterruptException:
		pass