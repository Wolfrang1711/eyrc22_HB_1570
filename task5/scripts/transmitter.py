#!/usr/bin/env python

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik ]

import rospy
from std_msgs.msg import String, Int32      # Message type used for receiving feedback
import socket                               # For UDP communication

class transmitter:

    def __init__(self):
        
        # initialising node named "transmitter_node"
        rospy.init_node('transmitter_node')

        # initialising subscriber of velocity_data
        rospy.Subscriber('velocity_data', String, self.data_feedback)

        # initialising subscriber of pen status
        rospy.Subscriber('penStatus', Int32, self.pen_feedback)
        
        # initialising required variables
        self.velocity_data = ''
        self.pen_data = 0
        self.recieved_data = ''
        self.data_packet = ''

        # IP address and port of UDP network 
        self.UDP_IP = "192.168.50.1"
        self.UDP_port = 44444

        # For maintaining control loop rate.
        rate = rospy.Rate(75)

        # creating a datagram socket for UDP commmunication
        self.UDP_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        # control loop
        while not rospy.is_shutdown():
            
            # convertion and appending pen status to send data
            self.pen_data = str(self.pen_data)
            self.recieved_data = ",".join([self.velocity_data, self.pen_data])

            # encoding the recieved data
            self.data_packet = str.encode(self.recieved_data)

            # sending data packets to ESP32 via UDP communication
            self.UDP_socket.sendto(self.data_packet, (self.UDP_IP,self.UDP_port))
            print("Sending Data: ", self.data_packet)

            rate.sleep()

    # callback to recieve and update the variable to send data
    def data_feedback(self, msg):

        # taking the msg and updating the vaariable
        self.velocity_data = msg.data  

    # callback to recieve and update the pen status to send data
    def pen_feedback(self, msg):

        # taking the msg and updating the vaariable
        self.pen_data = msg.data    

if __name__ == "__main__":
	try:
		transmitter()
	except rospy.ROSInterruptException as e:
		print(e)


