#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu, LaserScan 

def sensing_node():
    pubUlt = rospy.Publisher('obstacle', String, queue_size=10)
    pubImu = rospy.Publisher('state', Imu, queue_size=10)
    pubDist = rospy.Publisher('distance', String, queue_size=10)
    pubLid = rospy.Publisher('lidar', LaserScan, queue_size=10)
    pubImg = rospy.Publisher('image', Image, queue_size=10)
    rospy.init_node('sensing', anonymous=False)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        pubUlt.publish()
        pubImu.publish()
        pubDist.publish()
        pubLid.publish()
        pubImg.publish()
        rate.sleep()

if __name__ == "__main__":
    try:
        sensing_node()
    except rospy.ROSInterruptException:
        pass