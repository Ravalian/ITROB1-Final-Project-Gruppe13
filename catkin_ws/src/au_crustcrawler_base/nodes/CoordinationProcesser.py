#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

def invkin(xyz):
    d1 = 10.0
    a1 = 0.0
    a2 = 20.0
    d4 = 20.0

    x_c = xyz[0]
    y_c = xyz[1]
    z_c = xyz[2]

    #Calculate q1
    q1 = math.atan2(y_c, x_c)

    #Calculatae radius and height s as x and y coordinates between joint 2 and the end factor
    r2 = math.pow((x_c - a1 * math.cos(q1)), 2) + math.pow((y_c - a1 * math.sin(q1)), 2)
    s = (z_c - d1)

    #calculate D
    D = (r2 + math.pow(s,2) - math.pow(a2, 2) - math.pow(d4,2))/(2*a2*d4)

    #calculate q3
    q3 = math.atan2((-math.sqrt(1-math.pow(D,2))),D)

    #calculateq2
    q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4*math.sin(q3), a2+d4*math.cos(q3))

    #calculate q4
    q4 = 0.0
    
    return q1,q2,q3,q4

def sonarcallback(sensor_data):
    #recieved data from camera
    z = sensor_data.data

    dur = rospy.Duration(1)
    
	jtp = JointTrajectoryPoint(positions=invkin([x,y,z]),velocities=[0.5]*4 ,time_from_start=dur)
	dur += rospy.Duration(2)
	self.joint_positions.append(jtp)

	self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
	self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )
    
    data.data = invkin([x,y,z])
    
    pub.publish(data)

def cameracallback(camera_data):
    x = camera_data.data[0]
    y = camera_data.data[1]

    data.data = [x,y,z]

    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('CoordinationProcesser')
    x = 0.0
    y = 0.0
    z = 0.0

    data = Float64MultiArray()

    #Subcriber - receiving data from camera & sonar
    rospy.Subscriber('camera_data', Float64MultiArray, cameracallback)
    rospy.Subscriber('sonar_data', Float64, sonarcallback)

    #Publisher - sending data to joint states
    pub = rospy.Publisher('joint_state', Float64MultiArray,  queue_size=10)

    #rospy spin
    rospy.spin()