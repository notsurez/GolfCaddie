#!/usr/bin/env python

import serial
import time
import string 
import pynmea2
import rospy
from std_msgs.msg import String
from geographiclib.geodesic import Geodesic

def talker():
    pub = rospy.Publisher('gps', String, queue_size=20)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	dataout = pynmea2.NMEAStreamReader()
	newdata=ser.readline()
	if (newdata[0:6]=="$GPRMC"):
		newmsg=pynmea2.parse(newdata)
		lat=round(newmsg.latitude,6)
		lon=round(newmsg.longitude,6) 
		gps= str(lat) + "," + str(lon)
		rospy.loginfo(gps)
		pub.publish(gps)
        	rate.sleep()
	else:
		pass

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
