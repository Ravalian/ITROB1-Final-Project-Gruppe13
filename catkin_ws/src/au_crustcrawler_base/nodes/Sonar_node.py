#! /usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#def processSonarData ():


#Method for sennding data
def sonarcallback ( sensor_data ):
    base_data = Twist()
    base_data.angular.z = 0.5

    #sending data out with a rospy topic
    pub.publish(base_data)


if __name__ == '__main__':
    rospy.init_node('Sonar_node')

    #Receiving data from somewhere? 
    rospy.Subscriber('base_scan', LaserScan, sonarcallback)


    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.spin() #spin() simply keeps python from exiting until this node is closed