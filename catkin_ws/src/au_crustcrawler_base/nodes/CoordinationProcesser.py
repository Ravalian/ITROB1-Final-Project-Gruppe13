#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64, Header
from sensor_msgs.msg import JointState

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

class cp:

    x = 0.0
    y = 0.0
    z = 0.0
    pub = 0

    def __init__(self):

        #Subcriber - receiving data from camera & sonar
        rospy.Subscriber('camera_data', Float64MultiArray, self.cameracallback)
        rospy.Subscriber('sonar_data', Float64, self.sonarcallback)

        #Publisher - sending data to joint states
        self.pub = rospy.Publisher('joint_states', JointState,  queue_size=1)
        
    def invkin(self, xyz):
        d1 = 0.08 #H paa led 2 (cm)
        a1 = 0.0 #Distance langs y til led 2 (cm)
        a2 = 0.17 #Distance mellem led 2 og led 3(cm)
        d4 = 0.17 #Distance mellem led 3 og gribecenter (cm)

        x_c = xyz[0]
        y_c = xyz[2]
        z_c = xyz[1]
        # x_c=0
        # y_c=0
        # z_c= 0

        print("x=",x_c,"y=",y_c,"z=",z_c)

        #Calculate q1
        q1 = math.atan2(y_c, x_c)

        #Calculatae radius and height s as x and y coordinates between joint 2 and the end factor
        r2 = math.pow((x_c - a1 * math.cos(q1)), 2) + math.pow((y_c - a1 * math.sin(q1)), 2)
        s = (z_c - d1)

        #calculate D
        D = (r2 + math.pow(s,2) - math.pow(a2, 2) - math.pow(d4,2))/(2*a2*d4)
        print(D)

        #calculate q3
        q3 = math.atan2((math.sqrt(1-math.pow(D,2))),D)

        #calculateq2
        q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4*math.sin(q3), a2+d4*math.cos(q3))

        #calculate q4
        q4 = 0

        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append("joint1")
        js.position.append(q1)
        js.velocity.append(0.0)
        js.name.append("joint2")
        js.position.append(q2)
        js.velocity.append(0.0)
        js.name.append("joint3")
        js.position.append(q3)
        js.velocity.append(0.0)
        js.name.append("joint4")
        js.position.append(q4)
        js.velocity.append(0.0)
        
        return js

    def sonarcallback(self, sensor_data):
        self.z = sensor_data.data

        q = self.invkin([self.x,self.y,self.z])


        self.pub.publish(q)

    def cameracallback(self, camera_data):
        x_c = camera_data.data[0]
        y_c = camera_data.data[1]

        robot_orego = [320, 480]

        dist_x = robot_orego[0]-x_c #165
        dist_y = robot_orego[1]-y_c #174

        pix_to_m = 0.00353/112 
        self.x = dist_x*pix_to_m
        self.y = dist_y*pix_to_m


        coordinates = self.invkin([self.x,self.y,self.z])
        
        self.pub.publish(coordinates)

    # def coordinates(x,y,z):
    #     q = self.invkin([self.x,self.y,self.z])
    #     self.pub.publish(q)

if __name__ == '__main__':
    rospy.init_node('CoordinationProcesser')

    CoordinationProcesser = cp()
    
    #rospy spin
    rospy.spin()