#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Taken from ITROB1 - Author: Mads Dyrmann
Rettet til af: Alexander Hoffmann, Emil Warmdahl, Jonathan Junker og Nicolai Nielsen
"""

import cv2
import time
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image

def findimagecoord():
    #read image, as we don't have a camera we read the image from a folder.
    frame = cv2.imread('/home/ros/ITROB1-Final-Project-Gruppe13/billede.png')

    #show image on screen
    #cv2.imshow('frame', frame)

    #Extract the three color channels
    red = frame[:,:,2]
    green = frame[:,:,1]
    blue = frame[:,:,0]

    #Calculate excess red - Highlighting the red colors
    #Using RGB channel weighting, we can exclude som of the color channels in the image
    #In this case we want to highlight the red color as it is the color of the bricks we are looking for.
    exr = 3*red - blue - green
    #cv2.imshow('exr', exr)

    #Find suitable threshold
    #We want to create a grey-scaled image and we want to remove background to have clear view of the main objects
    ret, thresholded = cv2.threshold(exr, 40, 200, cv2.THRESH_BINARY_INV)
    #cv2.imshow('inverse', thresholded)

    #Morphology
    #We want to clear out noise in the image and improve the quality of the objects in the image
    #We use the "Closing" strategy, dilation first then erosion.
    kernel = np.ones((3,3),np.uint8)
    img_dil = cv2.dilate(thresholded, kernel)   #Dilation: Changes background pixel to foreground if it has a foreground pixel in its structure element
    img_erode = cv2.erode(img_dil, kernel)      #Erosio: Changes a foreground pixel to background if it has a background pixel in its structure element
    #cv2.imshow('dialte', img_erode)

    #find connected components
    #We want to highlight each component found in the image
    #For this we use openCV's contours
    image, contours, hierarchy = cv2.findContours(img_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_contours = np.zeros(frame.shape)
    cv2.drawContours(img_contours, contours, -1, (0,255,0), 3)

    #Font setup for text on img (used later)
    font = cv2.FONT_HERSHEY_SIMPLEX

    #loop through all components, one at the time
    #If there are more than one object which has been highligted by the contours, we want to check each of them
    for cnt in contours:
        #whats the are of the component
        areal = cv2.contourArea(cnt)
        #print(areal)

        #Do something if the areal is bigger than 4000 
        #In this case the objects we want to grab is the biggest of the obejcts to be placed in the image at any point, by process of elimination we found a cutoff value for the objects.
        if(areal > 4000):
            #get the center of the mass
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print(cx, cy)
            cv2.putText(frame, str(cx) + ',' + str(cy), (cx,cy), font, 1, (255,0,0),2)

    #Want to show the picture with the marked object.
    #cv2.imshow('frame annotated', frame)
    #cv2.waitKey(0)

    #Creating a variable to store the x and y coordiantes
    data_to_send = Float64MultiArray()
    data_to_send.data = [cx,cy]

    #Returning the data
    return data_to_send


def cameracallback(image):
    #Run the image processing function
    objectcoords = findimagecoord()
    #print(objectcoords)

    #Publishing the processed cameradata (x, y) coords
    pub.publish(objectcoords)


if __name__ == '__main__':
    rospy.init_node('Camera_node')

    #Subscriber - receiving camera data - Should be replaced with a true camera note
    rospy.Subscriber('base_scan', Float64, cameracallback)

    #Publisher - Sending out the processed camera data
    pub = rospy.Publisher('camera_data', Float64MultiArray, queue_size=1)

    rospy.spin()