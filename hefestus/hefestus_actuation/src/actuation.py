#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Accel

def actuation_node():
    rospy.init_node('actuation', anonymous=False)
    rospy.Subscriber("command", Accel, callback)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

def callback(data):
    pass

if __name__ == "__main__":
    try:
        actuation_node()
    except rospy.ROSInterruptException:
        pass