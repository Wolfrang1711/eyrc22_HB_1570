#!/usr/bin/env python

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik ]

import rospy
from std_msgs.msg import String
import socket

class transmitter:

    def __init__(self):
        
        # initialising node named "controller_node"
        rospy.init_node('transmitter_node')

        # initialising publisher of /right_wheel_force, /front_wheel_force, /left_wheel_force 
        rospy.Subscriber('velocity_array', String, self.velocity_feedback)

        self.recieved_data = ''
        self.dataToSend = ''

        #Enter IP address of laptop after connecting it to WIFI hotspot
        self.localIP     = "192.168.59.1"
        self.localPort   = 44444
        self.bufferSize  = 1024

        # Create a datagram socket
        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        # Bind to address and ip
        # self.UDPServerSocket.bind((self.localIP, self.localPort))
        # print("UDP server up and listening")

        while not rospy.is_shutdown():

            # Listen for incoming datagrams
            self.msgFromServer = self.recieved_data
            self.dataToSend = str.encode(self.msgFromServer)

            # Sending a reply to client
            self.UDPServerSocket.sendto(self.dataToSend, (self.localIP,self.localPort))
            print("Sending Data ", self.dataToSend)


    def velocity_feedback(self, msg):

        # taking the msg and updating the three variables
        self.recieved_data = msg.data  

if __name__ == "__main__":
	try:
		transmitter()
	except rospy.ROSInterruptException as e:
		print(e)


