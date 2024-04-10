#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Accel

def control_node():
    rospy.init_node('control', anonymous=False)
    rospy.Subscriber("status", String, callback)
    
    pubCommand = rospy.Publisher('command',Accel,queue_size=10)
    
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        pubCommand.publish()
        rate.sleep()

def callback(data):
    pass

if __name__ == "__main__":
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass