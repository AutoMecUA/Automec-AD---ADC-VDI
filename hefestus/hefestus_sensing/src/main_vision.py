#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def mainVisionNode():
    img_Pub = rospy.Publisher('main_image', Image, queue_size=10)
    img_bridge = CvBridge()

    cam = cv2.VideoCapture(2)
    
    rate = rospy.Rate(1) # 10hz
    
    while not rospy.is_shutdown():
        _, img_raw = cam.read()  

        img_message = img_bridge.cv2_to_imgmsg(img_raw, "bgr8")

        img_Pub.publish(img_message)
        rate.sleep()

if __name__ == "__main__":    
    try:
        rospy.init_node('main_vision', anonymous=False)
        mainVisionNode()
    except rospy.ROSInterruptException:
        pass