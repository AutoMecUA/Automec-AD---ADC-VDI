#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from hefestus_perception.msg import MiddleLine

def laneDetection_node():
	rospy.init_node('laneDetection', anonymous=False)
	rospy.Subscriber("/image", Image, callback)
		
	rate = rospy.Rate(1) # 10hz

	while not rospy.is_shutdown():
		rate.sleep()

def callback(data):	
	pub = rospy.Publisher("/status/middleLine",MiddleLine,queue_size=10)
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
	middle_line_pos1, middle_line_pos2 = frame_processor(cv_image)
	offset_px1 = int(middle_line_pos1[0][0]-cv_image.shape[1]/2)
	offset_px2 = int(middle_line_pos2[0][0]-cv_image.shape[1]/2)
	offset_pc1 = abs(offset_px1/cv_image.shape[1])
	offset_pc2 = abs(offset_px2/cv_image.shape[1])
	slope1 = (middle_line_pos1[0][1]-middle_line_pos1[1][1])/(middle_line_pos1[0][0]-middle_line_pos1[1][0])
	slope2 = (middle_line_pos2[0][1]-middle_line_pos2[1][1])/(middle_line_pos2[0][0]-middle_line_pos2[1][0])
	msg_to_send = MiddleLine()
	msg_to_send.offset_px_bottom = offset_px1
	msg_to_send.offset_px_top = offset_px2
	msg_to_send.offset_pc_bottom = offset_pc1
	msg_to_send.offset_pc_top = offset_pc2
	msg_to_send.slope_bottom = slope1
	msg_to_send.slope_top = slope2
	pub.publish(msg_to_send)


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
	all_lines1 = lane_lines(image, hough1)
	all_lines2 = lane_lines(image, hough2)
	return all_lines1[2], all_lines2[2] #only the middle one

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
	top_left = [cols * 0.4, rows * 0.6]
	middle_right = [cols * 0.75, rows * 0.80]
	bottom_right = [cols * 0.9, rows * 0.95]
	top_right = [cols * 0.6, rows * 0.6]
	vertices_low = np.array([[bottom_left, middle_left, middle_right, bottom_right]], dtype=np.int32)
	vertices_high = np.array([[middle_left, top_left, top_right, middle_right]], dtype=np.int32)
	# filling the polygon with white color and generating the final mask
	cv2.fillPoly(mask_low, vertices_low, ignore_mask_color)
	cv2.fillPoly(mask_high, vertices_high, ignore_mask_color)
	# performing Bitwise AND on the input image and mask to get only the edges on the road
	masked_image1 = cv2.bitwise_and(image, mask_low)
	masked_image2 = cv2.bitwise_and(image, mask_high)
	return masked_image1, masked_image2

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

def lane_lines(image, lines):
	"""
	Create full lenght lines from pixel points.
		Parameters:
			image: The input test image.
			lines: The output lines from Hough Transform.
	"""
	left_lane, right_lane = average_slope_intercept(lines)
	y1 = image.shape[0]
	y2 = y1 * 0.6
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

if __name__ == "__main__":
    try:
        laneDetection_node()
    except rospy.ROSInterruptException:
        pass