#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def mainVisionNode():
    img_Pub = rospy.Publisher('main_image', Image, queue_size=10)
    img_bridge = CvBridge()

    cam = cv2.VideoCapture(2)

    if not cam.isOpened():
        rospy.logerr("Camera could not be opened.")
        return
    
    rate = rospy.Rate(1) # 1hz
    
    try:
        while not rospy.is_shutdown():
            ret, img_raw = cam.read()  

            if not ret:
                rospy.logerr("Failed to capture image")
                break
            
            img_message = img_bridge.cv2_to_imgmsg(img_raw, "bgr8")
            img_Pub.publish(img_message)
            rate.sleep()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception occurred.")
    
    finally:
        cam.release()
        rospy.loginfo("Camera released.")

if __name__ == "__main__":    
    try:
        rospy.init_node('main_vision', anonymous=False)
        mainVisionNode()
    except rospy.ROSInterruptException:
        pass