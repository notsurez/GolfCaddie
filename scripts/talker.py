#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import UInt16

def talker():
    pub = rospy.Publisher('leftServo', UInt16, queue_size=100)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    leftMotorValue = 0

    while not rospy.is_shutdown():
        pub.publish(leftMotorValue)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
