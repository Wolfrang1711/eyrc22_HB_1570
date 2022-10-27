#!/usr/bin/env python3

from xml.sax.handler import property_lexical_handler
import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

hola_x = 0
hola_y = 0
hola_theta = 0

def odometryCb(msg):
	global hola_x, hola_y, hola_theta

	# Write your code to take the msg and update the three variables

	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation

	(hola_roll, hola_pitch, hola_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	
	# print(hola_x,hola_y,hola_theta)

def main():
	global hola_x, hola_y, hola_theta

	# Initialze Node
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
	rospy.init_node("controller", anonymous=True)
	
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	rospy.Subscriber("/odom", Odometry, odometryCb)

	# Declare a Twist message
	vel = Twist()
	# Initialise the required variables to 0
	# <This is explained below>
	vel_x = 0.1
	vel_y = 0.1
	vel_z = 0.1
	
	# For maintaining control loop rate.
	rate = rospy.Rate(100)

	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller
	x_d = -1.0
	y_d = 2.0
	theta_d = math.pi/4
	kp = 0.7

	#
	# 
	# Control Loop goes here
	#
	#
	while not rospy.is_shutdown():

		# Find error (in x, y and theta) in global frame
		# the /odom topic is giving pose of the robot in global frame
		# the desired pose is declared above and defined by you in global frame
		# therefore calculate error in global frame
		error_x = x_d - hola_x
		error_y = y_d - hola_y
		error_th = theta_d - hola_theta
		# print(error_x,error_y,error_th)
		

		# (Calculate error in body frame)
		# But for Controller outputs robot velocity in robot_body frame, 
		# i.e. velocity are define is in x, y of the robot frame, 
		# Notice: the direction of z axis says the same in global and body frame
		# therefore the errors will have have to be calculated in body frame.
		error_d = math.sqrt((pow(error_x,2) + pow(error_y,2)))
		error_th_b = math.atan2(error_y,error_x)
		print(error_d,error_th)

		# pose = Pose()
		# x = pose.position.x
		# y = pose.position.y
		# rot_q = pose.orientation
		# (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
		# print(x,y,theta)

		# 
		# This is probably the crux of Task 1, figure this out and rest should be fine.

		# Finally implement a P controller 
		# to react to the error with velocities in x, y and theta.
		prop_x = error_x
		prop_y = error_y
		prop_th = error_th
		balance_x = (kp*prop_x) 
		balance_y = (kp*prop_y)
		balance_th = (kp*prop_th)
		
		if (error_d > 0.1):
			
			# if (abs(error_th_b - hola_theta) > 0.1):

			# 	vel.linear.x = 0.0
			# 	vel.linear.y = 0.0 
			# 	vel.angular.z = 0.5

			# else:

			# 	vel.linear.x = 0.5
			# 	vel.linear.y = 0.0
			# 	vel.angular.z = 0.0

				vel.linear.x = vel_x + balance_x
				vel.linear.y = vel_y + balance_y
				vel.angular.z = vel_z + balance_th

		else:

			if (abs(error_th) > 0.1):

				vel.linear.x = 0.0
				vel.linear.y = 0.0 
				vel.angular.z = vel_z + balance_th

			else:

				vel.linear.x = 0.0
				vel.linear.y = 0.0
				vel.angular.z = 0.0


		# Safety Check
		# make sure the velocities are within a range.
		# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
		# we may get away with skipping this step. But it will be very necessary in the long run.

		# vel.linear.x = vel_x
		# vel.linear.y = vel_y
		# vel.angular.z = vel_z

		pub.publish(vel)
		rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
