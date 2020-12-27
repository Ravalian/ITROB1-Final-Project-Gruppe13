#! /usr/bin/env python 3
# -*- coding: utf-8 -*-

"""
Taken from ITROB1 - Author: Mads Dyrmann
Rettet til af: Emil Warmdahl og Nicolai Nielsen
"""

import cv2
import time
import rospy

def findimagecoord():
    #read image
    frame = cv2.imread('billede.png')

    #show image on screen
    cv2.imshow('frame', frame)

    #Extract the three color channels
    red = frame[:,:,2]
    green = frame[:,:,1]
    blue = frame[:,:,0]

    #Calculate excess red - Highlighting the red colors
    exr = 2*red - blue - green

    #Find suitable threshold
    thresholded = 

    #show the areas
    cv2.imshow('exr', thresholded)

    #find connected components
    contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #loop through all components, one at the time
    for cnt in contours:
        #whats the are of the component
        areal = cv2.contourArea(cnt)

        #Do something if the areal is bigger than 1 (might has to be changed)
        if(areal > 1):
            #get the center of the mass
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print(cx, cy)
            return [cx, cy]
    
    cv2.waitKey(0)


def cameracallback(image):
    objectcoords = findimagecoord()

    #Sending out the processed cameradata (x, y) coords
    pub.publish(objectcoords)


if __name__ == '__main__':
    rospy.init_node('Camera_node')

    #Subscriber - receiving camera data - how to implement? 
    #rospy.Subcriber('camera_scan', camera_data, cameracallback)

    #Publisher - Sending out the processed camera data
    pub = rospy.Publisher('camera_data', )

    rospy.spin()