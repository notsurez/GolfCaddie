#!/usr/bin/env python

import serial
import time
import string 
import pynmea2
import rospy
from std_msgs.msg import String

port ="/dev/ttyTHS1"
ser=serial.Serial(port,baudrate=9600,timeout=0.5)

def talker():
	heading = 0.0
	true_north = ""
	gps = ""
	lat = 0.0
	lon = 0.0
	pub = rospy.Publisher('gps', String, queue_size=1)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(5) # 10hz
	while not rospy.is_shutdown():

		dataout = pynmea2.NMEAStreamReader()
		newdata=ser.readline()
		#print(newdata)
		
		if (newdata[0:6]=="$GPRMC"):
			newmsg=pynmea2.parse(newdata)
			lat=round(newmsg.latitude,6)
			lon=round(newmsg.longitude,6) 
			gps= str(lat) + "," + str(lon)
			rospy.loginfo(gps)
			pub.publish(gps)	
			
		#print("lat long: " + gps)
		#print("heading: " + str(heading) + ", " + "reference: " + str(true_north))
		
		rate.sleep()
		
	#else:
	#pass
	#print("trying to connect\n")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
