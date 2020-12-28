#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Taken from ITROB1 - Author: Mads Dyrmann
Rettet til af: Emil Warmdahl og Nicolai Nielsen
"""

import cv2
import time
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def findimagecoord():
    #read image
    frame = cv2.imread('/home/ros/Desktop/ITROB1-Final-Project-Gruppe13/billede.png')

    #show image on screen
    #cv2.imshow('frame', frame)

    #Extract the three color channels
    red = frame[:,:,2]
    green = frame[:,:,1]
    blue = frame[:,:,0]

    #Calculate excess red - Highlighting the red colors
    exr = 3*red - blue - green
    #cv2.imshow('exr', exr)

    #Find suitable threshold
    ret, thresholded = cv2.threshold(exr, 40, 200, cv2.THRESH_BINARY_INV)
    #cv2.imshow('inverse', thresholded)

    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv', hsv)
    #lines = cv2.HoughLines(exr, rho=1, theta=1, threshold=20)
    #cv2.imshow('Line', lines)
    kernel = np.ones((3,3),np.uint8)
    img_dil = cv2.dilate(thresholded, kernel)
    img_erode = cv2.erode(img_dil, kernel)
    #img_dil2 = cv2.dilate(img_erode, kernel)
    #cv2.imshow('dialte', img_erode)

    #New processed picture
    #thresholded = 

    #show the areas
    #cv2.imshow('exr', thresholded)

    # #find connected components
    image, contours, hierarchy = cv2.findContours(img_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #image2, contours2, hierarchy2 = cv2.findContours(img_dil2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_contours = np.zeros(frame.shape)
    cv2.drawContours(img_contours, contours, -1, (0,255,0), 3)
    #img_contours2 = np.zeros(frame.shape)
    #cv2.drawContours(img_contours2, contours2, -1, (0,255,0), 3)
    #cv2.imshow('contours', img_contours)
    #cv2.imshow('contours2', img_contours2)

    font = cv2.FONT_HERSHEY_SIMPLEX

    #loop through all components, one at the time
    for cnt in contours:
        #whats the are of the component
        areal = cv2.contourArea(cnt)
        #print(areal)

        #Do something if the areal is bigger than 1 (might has to be changed)
        if(areal > 4000):
            #get the center of the mass
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print(cx, cy)
            cv2.putText(frame, str(cx) + ',' + str(cy), (cx,cy), font, 1, (255,0,0),2)

    
    #cv2.putText(frame, str(cx) + ',' + str(cy), (cx,cy), font, 1, (255,0,0),2)

    cv2.imshow('frame annotated', frame)

    cv2.waitKey(0)

    data_to_send = Float64MultiArray()
    data_to_send.data = [cx,cy]

    return data_to_send


def cameracallback(image):
    objectcoords = findimagecoord()
    print(objectcoords)

    #Sending out the processed cameradata (x, y) coords
    pub.publish(objectcoords)


if __name__ == '__main__':
    rospy.init_node('Camera_node')

    #Subscriber - receiving camera data - how to implement? 
    #rospy.Subcriber('camera_scan', camera_data, cameracallback)

    #Publisher - Sending out the processed camera data
    pub = rospy.Publisher('camera_data', Float64MultiArray, queue_size=10)

    rospy.spin()