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
        rospy.loginfo(leftMotorValue)
        pub.publish(leftMotorValue)
        leftMotorValue = 180
        time.sleep(1)
        rospy.loginfo(leftMotorValue)
        pub.publish(leftMotorValue)
        leftMotorValue = 0
        time.sleep(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
