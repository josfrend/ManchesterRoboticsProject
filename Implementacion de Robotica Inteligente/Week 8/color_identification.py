#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
import cv2 as cv
import numpy as np
import argparse


class colorIdentificator():
    def __init__(self):
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.light_pub = rospy.Publisher('/light_color', String, queue_size=10)
        self.light_color = 'n'
        self.bridge = CvBridge()
        rospy.on_shutdown(self.stop)

        #Define threshold values for traffic light detection
        self.cv_image = None
        self.red_low_H = 160
        self.red_low_S = 60
        self.red_low_V = 130
        self.red_high_H = 179
        self.red_high_S = 255
        self.red_high_V = 255

        self.red2_low_H = 0
        self.red2_low_S = 60
        self.red2_low_V = 130
        self.red2_high_H = 9
        self.red2_high_S = 255
        self.red2_high_V = 255

        self.yellow_low_H = 20
        self.yellow_low_S = 100
        self.yellow_low_V = 100
        self.yellow_high_H = 30
        self.yellow_high_S = 255
        self.yellow_high_V = 255

        self.green_low_H = 50
        self.green_low_S = 50
        self.green_low_V = 100
        self.green_high_H = 89
        self.green_high_S = 255
        self.green_high_V = 255

        # Define erosion and dilation parameters for blob detection
        self.erosion_size = 7
        self.dilatation_size = 0
        self.max_elem = 2
        self.max_kernel_size = 21

        # Define SimpleBlobDetector parameters
        self.params = cv.SimpleBlobDetector_Params()
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.8
        self.params.filterByArea = True
        self.params.minArea = 50
        self.params.maxArea = 500000
        self.params.filterByInertia = True
        self.params.minInertiaRatio = 0.6

        # Create SimpleBlobDetector object
        self.detector = cv.SimpleBlobDetector_create(self.params)

        # Counters
        self.red_count = 0
        self.yellow_count = 0
        self.green_count = 0

    def blobs(self, val):
        # Apply erosion and dilatation to the image 
        element = cv.getStructuringElement(cv.MORPH_RECT, (2 * self.erosion_size + 1, 2 * self.erosion_size + 1),(self.erosion_size, self.erosion_size))
        erosion_dst = cv.erode(val, element)
        element = cv.getStructuringElement(cv.MORPH_RECT, (2 * self.dilatation_size + 1, 2 * self.dilatation_size + 1),(self.dilatation_size, self.dilatation_size))
        dilatation_dst = cv.dilate(erosion_dst, element)

        # Invert the image to adjust for the blob detector
        dilatation_dst = 255-dilatation_dst
        # Use blob detector
        keypoints = self.detector.detect(dilatation_dst)
        
        return len(keypoints)

    def image_callback(self,data):
        # Open image through Cv Bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        # Turn image from BGR to HSV and apply the color masks
        frame_HSV = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        red_HSV = cv.inRange(frame_HSV, (self.red_low_H, self.red_low_S, self.red_low_V), (self.red_high_H, self.red_high_S, self.red_high_V))
        red2_HSV = cv.inRange(frame_HSV, (self.red2_low_H, self.red2_low_S, self.red2_low_V), (self.red2_high_H, self.red2_high_S, self.red2_high_V))
        red = cv.add(red_HSV, red2_HSV)
        yellow_HSV = cv.inRange(frame_HSV, (self.yellow_low_H, self.yellow_low_S, self.yellow_low_V), (self.yellow_high_H, self.yellow_high_S, self.yellow_high_V))
        green_HSV = cv.inRange(frame_HSV, (self.green_low_H, self.green_low_S, self.green_low_V), (self.green_high_H, self.green_high_S, self.green_high_V))
       
        # Detect blobs in each color image
        self.red_count = self.blobs(red)
        self.yellow_count = self.blobs(yellow_HSV)
        self.green_count = self.blobs(green_HSV)

    
    def run(self):    
        
        # Assign the value of the traffic light color the camera sees, with the hierarchy such as red, yellow and green 
        if(self.red_count > 0):
            self.light_color = 'r'
        elif(self.yellow_count > 0):
            self.light_color = "y"
        elif(self.green_count > 0):
            self.light_color = 'g'
        else:
            self.light_color = 'n'

        self.light_pub.publish(self.light_color)



    def stop(self):
        print("Stopping")

    

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("light_detector")
    color = colorIdentificator()
    rate = rospy.Rate(100)
    rospy.on_shutdown(color.stop)

    print("processing")
    try:
        while not rospy.is_shutdown():
            color.run()
            rate.sleep()

    except rospy.ROSInterruptException():
        pass
