#!/usr/bin/env python3

PKG = 'hefestus_perception'
import roslib; roslib.load_manifest(PKG)

import cv2
import numpy as np
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from hefestus_perception.msg import MiddleLine
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

image_pub = rospy.Publisher("/status/lane_lines_image", Image, queue_size=10)
polyfit_pub = rospy.Publisher("/status/lane_polyfit", numpy_msg(Floats), queue_size=10)

def laneDetection_node():
    rospy.init_node('laneDetection', anonymous=False)
    rospy.Subscriber("/main_image", Image, callback)
        
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

def callback(data):	
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    processed_image, polyfits  = frame_processor(cv_image)
    msg_to_send = bridge.cv2_to_imgmsg(processed_image, "bgr8")
    image_pub.publish(msg_to_send)

    if polyfits is not None:
        polyfit_pub.publish(polyfits)

def frame_processor(image):
    """
    Process the input frame to detect lane points and then fit a polynomial
    to the left and right lanes.
    """
    # Convert the RGB image to grayscale and invert it (if desired)
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grayscale = 255 - grayscale
    
    # Apply Gaussian Blur to remove noise and focus on our region of interest.
    kernel_size = 5
    blur = cv2.GaussianBlur(grayscale, (kernel_size, kernel_size), 0)
    
    # Use Canny edge detection
    low_t = 50
    high_t = 150
    edges = cv2.Canny(blur, low_t, high_t)
    
    # Apply a mask to focus on the road region.
    region = region_selection(edges)
    
    # Instead of the Hough transform, extract nonzero edge points.
    left_points, right_points = get_lane_points(region)

    # Fit a polynomial (e.g., quadratic) to each lane's points.
    left_fit = polyfit_lane(left_points, image.shape[0])
    right_fit = polyfit_lane(right_points, image.shape[0])
    
    middle_fit = compute_middle_lane(left_fit, right_fit)

    polyfits = np.concatenate([
        middle_fit if middle_fit is not None else [0, 0, 0]
    ]).astype(np.float32)

    # Draw the polynomial curves onto the image.
    result = draw_lane_polyfit(image, left_fit, right_fit, middle_fit)
    
    return result, polyfits

def compute_middle_lane(left_fit, right_fit):
    """
    Compute the middle lane polynomial by averaging the left and right lane fits.
    """
    if left_fit is None or right_fit is None:
        return None  # Middle lane cannot be computed without both lanes

    return (left_fit + right_fit) / 2

def region_selection(image):
    """
    Determine and cut the region of interest in the input image.
    """
    mask = np.zeros_like(image)
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    rows, cols = image.shape[:2]
    bottom_left = [cols * 0.1, rows * 0.95]
    top_left = [cols * 0.4, rows * 0.6]
    top_right = [cols * 0.6, rows * 0.6]
    bottom_right = [cols * 0.9, rows * 0.95]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image 

def get_lane_points(edge_image):
    """
    Extract (x,y) points from the edge image and split them into left and right lanes
    based on the image center.
    """
    # Find non-zero points in the edge image.
    pts = cv2.findNonZero(edge_image)
    left_points = []
    right_points = []
    if pts is not None:
        pts = pts.reshape(-1, 2)
        mid_x = edge_image.shape[1] // 2  # vertical center of the image
        for (x, y) in pts:
            if x < mid_x:
                left_points.append((x, y))
            else:
                right_points.append((x, y))
    left_points = np.array(left_points) if len(left_points) > 0 else None
    right_points = np.array(right_points) if len(right_points) > 0 else None
    return left_points, right_points

def polyfit_lane(points, image_height, poly_order=2):
    """
    Fit a polynomial of given order to the lane points.
    The fit is done in the image coordinate system, with y as the independent variable.
    Returns polynomial coefficients [a, b, c] such that x = a*y^2 + b*y + c.
    """
    if points is None or len(points) < poly_order + 1:
        return None
    # Separate the coordinates.
    xs = points[:, 0]
    ys = points[:, 1]
    # Fit polynomial: note we fit x as a function of y.
    fit = np.polyfit(ys, xs, poly_order)
    return fit

def draw_lane_polyfit(image, left_fit, right_fit, middle_fit = None, color_left=(0, 255, 0), color_right=(0, 0, 255), color_middle=(255, 255, 0), thickness=5):
    """
    Draw the polynomial curves for left and right lanes onto the image.
    """
    overlay = image.copy()
    y_vals = np.linspace(int(image.shape[0]*0.6), image.shape[0]-1, num=50, dtype=np.int32)
    
    # For each lane, calculate corresponding x values from the polynomial.
    if left_fit is not None:
        left_points = [(int(np.polyval(left_fit, y)), y) for y in y_vals]
        for i in range(len(left_points) - 1):
            cv2.line(overlay, left_points[i], left_points[i + 1], color_left, thickness)

    if right_fit is not None:
        right_points = [(int(np.polyval(right_fit, y)), y) for y in y_vals]
        for i in range(len(right_points) - 1):
            cv2.line(overlay, right_points[i], right_points[i + 1], color_right, thickness)    
    
    if middle_fit is not None:
        middle_points = [(int(np.polyval(middle_fit, y)), y) for y in y_vals]
        for i in range(len(middle_points) - 1):
            cv2.line(overlay, middle_points[i], middle_points[i + 1], color_middle, thickness)

 
    # Combine the overlay with the original image.
    result = cv2.addWeighted(image, 1.0, overlay, 1.0, 0.0)
    return result

if __name__ == "__main__":
    try:
        laneDetection_node()
    except rospy.ROSInterruptException:
        pass
