#!/usr/bin/env python

'''
* Team Id : < HB_1570 >
* Author List : < Romala >
* Filename: < contours.py >
* Theme: < HB -- Specific to eYRC / eYRCPlus >
* Functions: < __init__(self), image_mode(self), function_mode(self) >
* Global Variables: < None >
'''

import cv2
import rospy 
import numpy as np
import math	                                       
import argparse		
from std_msgs.msg import String	

class contours:

    '''
    * Function Name: < __init__ >
    * Input: < None >
    * Output: < None >
    * Logic: < >
    * Example Call: < called automatically by Operating System >
    '''

    def __init__(self):

        rospy.init_node('contours_node')

        self.contourPub = rospy.Publisher('contours', String, queue_size=10)

        self.cData = String()  

        # initialzing argparse to switch between modes and provide resolution to the images
        parser = argparse.ArgumentParser(description="Select Mode: Function Mode or Image Mode")
        parser.add_argument("-f", "--mode", choices=["1", "2"], help="Select which mode to run")
        parser.add_argument("-r", "--resolution", help="Select resolution of image")

        # parsing the arguments
        self.args = parser.parse_args()

        # initialising the required variables
        self.frame = None
        self.contour_index = []
        self.cnt_extraction_list, self.final_contours_list = [], []
        self.contour_grp_last_x, self.contour_grp_last_y, self.contour_grp_last_th = [], [], []
        self.xList , self.yList , self.thList, self.xListFinal , self.yListFinal , self.thListFinal, self.x, self.y, self.th= [] , [] , [] , [] , [] , [] , [] , [] , []      

        self.resolution = int(self.args.resolution)

        rate = rospy.Rate(75)

        # selecting mode based on argparse
        if self.args.mode == "1":

            self.function_mode()
            print('starting function mode')

        elif self.args.mode == "2":

            self.image_mode()
            print('starting image mode')

        else:
            exit()
    
        while not rospy.is_shutdown():
            
            # sending the data in str format
            self.cData.data = str([self.xListFinal,self.yListFinal,self.thListFinal])

            rospy.loginfo("Publishing contours")

            # cv2.imshow('img', self.frame_copy)
            # cv2.waitKey(1)
            
            # img = np.zeros((500,500,3), dtype=np.uint8)   

            # pts = np.array([(x,y) for x,y in zip(self.xListFinal[0],self.yListFinal[0])], np.int32)
            # pts = pts.reshape((-1,1,2))
            # print(pts)

            # cv2.polylines(img, [pts], True, (0,255,0), thickness=2)

            # cv2.imshow('image', img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            # publishing the data to /contours topic
            self.contourPub.publish(self.cData) 

            rate.sleep() 

    '''
    * Function Name: < image_mode >
    * Input: < data - image path received>
    * Output: < None >
    * Logic: < finds contours of the img provided and extracts the contour points as coordinates in a list >
    * Example Call: < self.image_mode('/home/pratik/eyrc_ws/src/eyrc22_HB_1570/task6/scripts/robotFinal.png') >
    '''                  
        
    def image_mode(self):

        # read and do processing on the image
        self.image = cv2.imread('/home/pratik/eyrc_ws/src/eyrc22_HB_1570/task5/scripts/robotFinal.png')
        self.frame = cv2.resize(self.image, (500,500), interpolation=cv2.INTER_LINEAR)
        self.frame_copy = self.frame.copy()
        self.img_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.thresh = cv2.threshold(self.img_gray, 120, 255, cv2.THRESH_BINARY)[1]
        self.thresh = cv2.erode(self.thresh, None, iterations=2)
        self.thresh = cv2.dilate(self.thresh, None, iterations=2)

        # finding the contours of the image
        contours, hierarchy = cv2.findContours(image=self.thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

        # converting the hierarchy data to a list
        self.hierachy_list = hierarchy.tolist()

        #  appending the hierachy_list elements to cnt_extraction_list to get the list of list format
        for i in range(len(self.hierachy_list[0])):

            self.cnt_extraction_list.append(self.hierachy_list[0][i])   

        # generating list of interested contours
        list_of_interest = [item[3] for item in self.cnt_extraction_list]  
  
        # appending the corresponding index values of the list of interest
        for i in range(1,len(list_of_interest)):

            if list_of_interest[i]>1 or list_of_interest[i]<1:
                self.contour_index.append(i) 

        # storing in a list
        for i in range(len(self.contour_index)):

            # extracting contour groups corresponding to the list_of_interest 
            cnt = contours[self.contour_index[i]]

            # appending the contour groups to a list
            self.final_contours_list.append(cnt)
            
            # converting the list to a tuple to draw the contours on the frame
            tuple(self.final_contours_list) 

            # to draw the contours on the frame 
            cv2.drawContours(self.frame_copy, self.final_contours_list, -1, (0,255,0), 2)
        
        #  append contours as coordinates in a list
        for i in self.final_contours_list: 

            self.xList.clear()    
            self.yList.clear()
            self.thList.clear()
            
            for j in i:

                    self.xList.append(j[0][0])
                    self.yList.append(j[0][1])
                    self.thList.append(0)

            self.x.append(str(self.xList))
            self.y.append(str(self.yList))   
            self.th.append(str(self.thList))   

        # to change the str data type of the appended list into int 
        for i in range(len(self.x)):
            
            self.x[i] = eval(self.x[i])
            self.y[i] = eval(self.y[i])
            self.th[i] = eval(self.th[i])  

        # finding the last element of each contour group
        for i in range(len(self.x)):
            
            self.contour_grp_last_x.append(self.x[i][len(self.x[i])-1])
            self.contour_grp_last_y.append(self.y[i][len(self.y[i])-1])
            self.contour_grp_last_th.append(self.th[i][len(self.th[i])-1])
        
        # to choose selective coordinates according to resolution and appending it to respective lists 
        self.xListFinal = [sublist[::self.resolution] for sublist in self.x]
        self.yListFinal = [sublist[::self.resolution] for sublist in self.y]
        self.thListFinal = [sublist[::self.resolution] for sublist in self.th]

        for i in range(len(self.contour_grp_last_x)):
            
            # appending the last element extracted to the respective lists 
            self.xListFinal[i].append(self.contour_grp_last_x[i])
            self.yListFinal[i].append(self.contour_grp_last_y[i])
            self.thListFinal[i].append(self.contour_grp_last_th[i])    

    '''
    * Function Name: < function_mode >
    * Input: < None >
    * Output: < None >
    * Logic: < extracts the coordinates of the function equation over a time to a list>
    * Example Call: < self.function_mode() >
    '''              
    def function_mode(self):
        
        # clearing lists
        self.xListFinal.clear()    
        self.yListFinal.clear()
        self.thListFinal.clear()

        # to scale down the function
        scale_factor = 0.05

        for count in range(self.resolution):
            
            # time factor
            t =  count*((4*math.pi)/self.resolution)

            # defining the function eqn
            x_eqn = 4*abs(math.sin(2*t))*math.cos(t) 
            y_eqn = 4*abs(math.sin(2*t))*math.sin(t) 
            theta_eqn = (math.pi/4)*math.sin(t)

            # converting the cartesian frame coordinates to image frame coordinates
            x_eqn = int(x_eqn/scale_factor + 250)
            y_eqn = int(250 - y_eqn/scale_factor)

            # appending the coordinates into respective lists
            self.xList.append(x_eqn)
            self.yList.append(y_eqn)
            self.thList.append(round(theta_eqn,3))  

        for count in range(self.resolution):
            
            # time factor
            t =  count*((4*math.pi)/self.resolution) - count*((2*math.pi)/self.resolution)

            # defining the function eqn
            x_eqn = 3*abs(math.sin(2*t))*math.cos(t) 
            y_eqn = 3*abs(math.sin(2*t))*math.sin(t) 
            theta_eqn = (math.pi/4)*math.sin(t)

            # converting the cartesian frame coordinates to image frame coordinates
            x_eqn = int(x_eqn/scale_factor + 250)
            y_eqn = int(250 - y_eqn/scale_factor)

            # appending the coordinates into respective lists
            self.xList.append(x_eqn)
            self.yList.append(y_eqn)
            self.thList.append(round(theta_eqn,3))  

        # appending the lists to the respective final lists in list of list format
        self.xListFinal.append(self.xList)
        self.yListFinal.append(self.yList)
        self.thListFinal.append(self.thList)       

if __name__ == '__main__':

	try:
		contours()
	except rospy.ROSInterruptException as e:
		print(e)
  









