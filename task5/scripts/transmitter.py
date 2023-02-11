#!/usr/bin/env python

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik ]

import rospy
import numpy as np
from std_msgs.msg import String
import socket
from time import sleep
import signal		
import sys		

class transmitter:

    def __init__(self):
        
        # initialising node named "controller_node"
        rospy.init_node('transmitter_node')

        # initialising publisher of /right_wheel_force, /front_wheel_force, /left_wheel_force 
        rospy.Subscriber('velocity_array', String, self.velocity_feedback)

        self.recieved_data = ''

        #Enter IP address of laptop after connecting it to WIFI hotspot
        self.localIP     = "192.168.66.232"
        self.localPort   = 8002
        self.bufferSize  = 1024

        # Create a datagram socket
        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        # Bind to address and ip
        self.UDPServerSocket.bind((self.localIP, self.localPort))
        print("UDP server up and listening")

        rate = rospy.Rate(100)


        while not rospy.is_shutdown():

            self.msgFromServer = self.recieved_data
            self.dataToSend = str.encode(self.msgFromServer)

            # Listen for incoming datagrams
            while(True):

                bytesAddressPair = self.UDPServerSocket.recvfrom(self.bufferSize)
                message = bytesAddressPair[0]
                address = bytesAddressPair[1]

                clientMsg = "Message from Client:{}".format(message)
                clientIP  = "Client IP Address:{}".format(address)
                
                print(clientMsg)
                print(clientIP)

                # Sending a reply to client
                self.UDPServerSocket.sendto(self.dataToSend, address)

            # rate.sleep()
            # rospy.spin()

    def velocity_feedback(self, msg):

        # taking the msg and updating the three variables
        self.recieved_data = msg.data   

    def signal_handler(self, sig, frame):
        print('Clean-up !')
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        self.UDPServerSocket.close()
        print("cleanup done")

if __name__ == "__main__":
	try:
		transmitter()
	except rospy.ROSInterruptException:
		pass


