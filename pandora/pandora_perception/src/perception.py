#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu, LaserScan

def perception_node():
    rospy.init_node('perception', anonymous=False)
    rospy.Subscriber("obstacle", String, callback)
    rospy.Subscriber("state", Imu, callback)
    rospy.Subscriber("distance", String, callback)
    rospy.Subscriber("lidar", LaserScan, callback)
    rospy.Subscriber("image", Image, callback)
    
    pubState = rospy.Publisher('status',String,queue_size=10)
    
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        pubState.publish()
        rate.sleep()

def callback(data):
    pass

if __name__ == "__main__":
    try:
        perception_node()
    except rospy.ROSInterruptException:
        pass