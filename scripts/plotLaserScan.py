#! /usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

#Declare auto move message for autonomous driving
auto_move_cmd = Twist()

global dists
dists = []
distance = 0

def callback(msg):
    global distance
    global dists
    dists = msg.ranges;
    distance = dists_to_dist(dists)
    print(distance)

def dists_to_dist(sensor):
    return sensor[256]

def drive_to_dist(howFar):
    global distance
    linearx = 0
    angularz = 0
    if(distance > howFar):
        linearx = 1
    else:
        linearx = 0
    auto_move_cmd.linear.x = linearx
    auto_move_cmd.angular.z = angularz

def avoid_obstacles():
    global dists
    linearx = 0
    angularz = 0
    objInPath = False
    if(dists):
        for i in range(200, 312):
            if(dists[i] < 1.5):
                objInPath = True
        if(not objInPath):
            linearx = 1
            angularz = 0
        else:
            linearx = 0
            angularz = 1
    auto_move_cmd.linear.x = linearx
    auto_move_cmd.angular.z = angularz

def controller():
    rospy.init_node('depthController', anonymous=True)
    
    #Create subscribers
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    
    #Create publishers
    velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    
    plt.show()
    while not rospy.is_shutdown():
        #Publish data
        #drive_to_dist(2)
        avoid_obstacles()
	velocity_pub.publish(auto_move_cmd)
        rate.sleep();

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInitException:
        pass
