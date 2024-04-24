#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu, LaserScan 



def sensing_node(image):
    bridge = CvBridge()
    pubUlt = rospy.Publisher('obstacle', String, queue_size=10)
    pubImu = rospy.Publisher('state', Imu, queue_size=10)
    pubDist = rospy.Publisher('distance', String, queue_size=10)
    pubLid = rospy.Publisher('lidar', LaserScan, queue_size=10)
    pubImg = rospy.Publisher('image', Image, queue_size=10)
    rospy.init_node('sensing', anonymous=False)
    rate = rospy.Rate(1) # 10hz
    ros_image = bridge.cv2_to_imgmsg(image, "bgr8")
    while not rospy.is_shutdown():
        pubUlt.publish()
        pubImu.publish()
        pubDist.publish()
        pubLid.publish()
        pubImg.publish(ros_image)
        rate.sleep()

if __name__ == "__main__":    
    imagefile="/home/pedro/catkin_ws/src/Automec-AD---ADC-VDI/hefestus/hefestus_sensing/src/lane_example.png"
    image=cv2.imread(imagefile)
    try:
        sensing_node(image)
    except rospy.ROSInterruptException:
        pass