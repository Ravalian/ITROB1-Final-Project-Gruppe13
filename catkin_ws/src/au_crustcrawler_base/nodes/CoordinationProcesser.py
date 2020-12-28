#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

def processcallback(data):
    #recieved data from camera
    print("hello emil!")


if __name__ == '__main__':
    rospy.init_node('CoordinationProcesser')

    #Subcriber - receiving data from camera & sonar
    rospy.Subscriber('camera_data', Float64MultiArray, processcallback)
    rospy.Subscriber('sonar_data', Twist, processcallback)

    #Publisher - sending data to joint states
    rospy.Publisher('processed_data', Float64MultiArray)

    #rospy spin
    rospy.spin()