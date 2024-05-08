#!/usr/bin/env python3

import cv2
import numpy as np
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image

def laneDetection_node():
    rospy.init_node('VisualLaneDetection', anonymous=False)
    rospy.Subscriber("/image", Image, callback)
        
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()

def callback(data):
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
	testimage1, testimage2 =frame_processor(cv_image)
	msgimg1=bridge.cv2_to_imgmsg(testimage1,"bgr8")
	pubState = rospy.Publisher('LaneImage',Image,queue_size=10)
	pubState.publish(msgimg1)
	msgimg2=bridge.cv2_to_imgmsg(testimage2,"bgr8")
	pubState = rospy.Publisher('LaneImage2',Image,queue_size=10)
	pubState.publish(msgimg2)


def frame_processor(image):
	"""
	Process the input frame to detect lane lines.
	Parameters:
		image: image of a road where one wants to detect lane lines
		(we will be passing frames of video to this function)
	"""
	# convert the RGB image to Gray scale
	grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# applying gaussian Blur which removes noise from the image 
	# and focuses on our region of interest
	# size of gaussian kernel
	kernel_size = 5
	# Applying gaussian blur to remove noise from the frames
	blur = cv2.GaussianBlur(grayscale, (kernel_size, kernel_size), 0)
	# first threshold for the hysteresis procedure
	low_t = 50
	# second threshold for the hysteresis procedure 
	high_t = 150
	# applying canny edge detection and save edges in a variable
	edges = cv2.Canny(blur, low_t, high_t)
	# since we are getting too many edges from our image, we apply 
	# a mask polygon to only focus on the road
	# Will explain Region selection in detail in further steps
	region1, region2 = region_selection(edges)
	# Applying hough transform to get straight lines from our image 
	# and find the lane lines
	hough1 = hough_transform(region1)
	hough2 = hough_transform(region2)
	#lastly we draw the lines on our resulting frame and return it as output 
	result1 = draw_lane_lines(image, lane_lines1(image, hough1))
	result2 = draw_lane_lines(image, lane_lines2(image, hough2))
	return result1, result2

def region_selection(image):
	"""
	Determine and cut the region of interest in the input image.
	Parameters:
		image: we pass here the output from canny where we have 
		identified edges in the frame
	"""
	# create an array of the same size as of the input image 
	mask_low = np.zeros_like(image)
	mask_high = np.zeros_like(image) 
	# if you pass an image with more then one channel
	if len(image.shape) > 2:
		channel_count = image.shape[2]
		ignore_mask_color = (255,) * channel_count
	# our image only has one channel so it will go under "else"
	else:
		# color of the mask polygon (white)
		ignore_mask_color = 255
	# creating a polygon to focus only on the road in the picture
	# we have created this polygon in accordance to how the camera was placed
	rows, cols = image.shape[:2]
	bottom_left = [cols * 0.1, rows * 0.95]
	middle_left = [cols * 0.25, rows * 0.80]
	top_left	 = [cols * 0.4, rows * 0.6]
	bottom_right = [cols * 0.9, rows * 0.95]
	middle_right = [cols * 0.75, rows * 0.80]
	top_right = [cols * 0.6, rows * 0.6]
	vertices_low = np.array([[bottom_left, middle_left, middle_right, bottom_right]], dtype=np.int32)
	vertices_high = np.array([[middle_left, top_left, top_right, middle_right]], dtype=np.int32)
	# filling the polygon with white color and generating the final mask
	cv2.fillPoly(mask_low, vertices_low, ignore_mask_color)
	cv2.fillPoly(mask_high, vertices_high, ignore_mask_color)
	# performing Bitwise AND on the input image and mask to get only the edges on the road
	masked_image_1 = cv2.bitwise_and(image, mask_low)
	masked_image_2 = cv2.bitwise_and(image, mask_high)
	return masked_image_1, masked_image_2

def hough_transform(image):
	"""
	Determine and cut the region of interest in the input image.
	Parameter:
		image: grayscale image which should be an output from the edge detector
	"""
	# Distance resolution of the accumulator in pixels.
	rho = 1			
	# Angle resolution of the accumulator in radians.
	theta = np.pi/180
	# Only lines that are greater than threshold will be returned.
	threshold = 20	
	# Line segments shorter than that are rejected.
	minLineLength = 20
	# Maximum allowed gap between points on the same line to link them
	maxLineGap = 500	
	# function returns an array containing dimensions of straight lines 
	# appearing in the input image
	return cv2.HoughLinesP(image, rho = rho, theta = theta, threshold = threshold,
						minLineLength = minLineLength, maxLineGap = maxLineGap)

def average_slope_intercept(lines):
	"""
	Find the slope and intercept of the left and right lanes of each image.
	Parameters:
		lines: output from Hough Transform
	"""
	left_lines = [] #(slope, intercept)
	left_weights = [] #(length,)
	right_lines = [] #(slope, intercept)
	right_weights = [] #(length,)
	
	for line in lines:
		for x1, y1, x2, y2 in line:
			if x1 == x2:
				continue
			# calculating slope of a line
			slope = (y2 - y1) / (x2 - x1)
			# calculating intercept of a line
			intercept = y1 - (slope * x1)
			# calculating length of a line
			length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
			# slope of left lane is negative and for right lane slope is positive
			if slope < 0:
				left_lines.append((slope, intercept))
				left_weights.append((length))
			else:
				right_lines.append((slope, intercept))
				right_weights.append((length))
	# 
	left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
	right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
	return left_lane, right_lane

def lane_lines1(image, lines):
	"""
	Create full lenght lines from pixel points.
		Parameters:
			image: The input test image.
			lines: The output lines from Hough Transform.
	"""
	left_lane, right_lane = average_slope_intercept(lines)
	y1 = image.shape[0]
	y2 = y1 * 0.8
	left_line = pixel_points(y1, y2, left_lane)
	right_line = pixel_points(y1, y2, right_lane)
	middle_line = ((int((left_line[0][0]+right_line[0][0])/2),int(y1)),(int((left_line[1][0]+right_line[1][0])/2),int(y2)))
	return left_line, right_line, middle_line

def lane_lines2(image, lines):
	"""
	Create full lenght lines from pixel points.
		Parameters:
			image: The input test image.
			lines: The output lines from Hough Transform.
	"""
	left_lane, right_lane = average_slope_intercept(lines)
	yt = image.shape[0]
	y1 = yt * 0.8
	y2 = yt * 0.6
	left_line = pixel_points(y1, y2, left_lane)
	right_line = pixel_points(y1, y2, right_lane)
	middle_line = ((int((left_line[0][0]+right_line[0][0])/2),int(y1)),(int((left_line[1][0]+right_line[1][0])/2),int(y2)))
	return left_line, right_line, middle_line

def pixel_points(y1, y2, line):
	"""
	Converts the slope and intercept of each line into pixel points.
		Parameters:
			y1: y-value of the line's starting point.
			y2: y-value of the line's end point.
			line: The slope and intercept of the line.
	"""
	if line is None:
		return None
	slope, intercept = line
	x1 = int((y1 - intercept)/slope)
	x2 = int((y2 - intercept)/slope)
	y1 = int(y1)
	y2 = int(y2)
	return ((x1, y1), (x2, y2))


def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=12):
	"""
	Draw lines onto the input image.
		Parameters:
			image: The input test image (video frame in our case).
			lines: The output lines from Hough Transform.
			color (Default = red): Line color.
			thickness (Default = 12): Line thickness. 
	"""
	line_image = np.zeros_like(image)
	for line in lines:
		if line is not None:
			cv2.line(line_image, *line, color, thickness)
	return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)

if __name__ == "__main__":
    try:
        laneDetection_node()
    except rospy.ROSInterruptException:
        pass