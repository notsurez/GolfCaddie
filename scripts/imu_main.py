#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String, Int16
from geographiclib.geodesic import Geodesic
import math
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Imu

import re

qx = 0
qy = 0
qz = 0
qw = 0
yawDeg = 0
pitchDeg = 0
rollDeg = 0

def imu_data_cb(data):
	global qx
	global qy
	global qz
	global qw
	imu_data = data
	qx = imu_data.orientation.x
	qy = imu_data.orientation.y
	qz = imu_data.orientation.z
	qw = imu_data.orientation.w
	print("qx: " + "," + str(qx) + "," + "qy: " + "," + str(qy) + "," + "qz: " + "," + str(qz) + "," + "qw: "+ "," + str(qw))
	quat_to_ypr()

def map_vals(x, in_min, in_max, out_min, out_max):

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def quat_to_ypr():
	global qx
	global qy
	global qz
	global qw
	global rollDeg
	global yawDeg
	global pitchDeg
	
	# Create Roll Pitch Yaw Angles from Quaternions 
	yy = qy * qy; # 2 Uses below

	yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
	pitch = -math.asin(2.0*(qx*qz - qw*qy));
	roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);

	#  Convert Radians to Degrees 
	rollDeg  = 57.2958 * roll
	pitchDeg = 57.2958 * pitch
	yawDeg   = 57.2958 * yaw

def controller():
	rospy.init_node('controller', anonymous=True)
	sub = rospy.Subscriber('imu/data', Imu, imu_data_cb)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		#Create the string to be published here
		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInitException:
		pass
