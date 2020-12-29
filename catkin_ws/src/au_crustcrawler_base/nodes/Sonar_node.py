#! /usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import  Float64

#def processSonarData ():


#Method for sennding data
def sonarcallback ( sensor_data ):
    
    t = sensor_data.data
    v = 343.0
    d = t*v/2
    data = d

    

    #sending data out with a rospy topic
    pub.publish(data)


if __name__ == '__main__':
    rospy.init_node('Sonar_node')

    #Receiving data from somewhere? 
    rospy.Subscriber('base_scan', Float64, sonarcallback)


    pub = rospy.Publisher('sonar_data', Float64,  queue_size=10)
    rospy.spin() #spin() simply keeps python from exiting until this node is closed