#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np
import argparse

class lineDetector():
    def __init__(self):
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.alignment_pub = rospy.Publisher('/alignment', Int32, queue_size=5)
        self.processed_image = rospy.Publisher('/line_detection', Image, queue_size=10)
        self.bridge = CvBridge()
        rospy.on_shutdown(self.stop)
        # Define resized image desired dimensions
        self.desired_width = 640
        self.desired_height = 480
        
        # Variable to store the offset from the line
        self.offset = 0
        
        # Define erosion and dilation parameters for blob detection
        self.erosion_size = 7
        self.dilatation_size = 5
        self.max_elem = 2
        self.max_kernel_size = 21
        


    def image_callback(self,data):
        # Read the image from the Cv Bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
       # Turn to image to monocrome and re-scale it 
        frame = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        original_resized = cv.resize(frame, (self.desired_width, self.desired_height))
        
        # Set up and apply the mask to filter black shades from the image
        lower_black = np.array([0,0,0])
        upper_black = np.array([110,110,110])
        mask = cv.inRange(cv_image, lower_white, upper_white)
        # Invert the image
        mask = 255 - mask
            
        # Resize, apply filters, erosion and dilatation to the image to exalt certain features in the image
        resized = cv.resize(mask, (self.desired_width, self.desired_height))
        gaussian = cv.GaussianBlur(resized, (7,7), 1)
        element = cv.getStructuringElement(cv.MORPH_RECT, (2 * self.erosion_size + 1, 2 * self.erosion_size + 1),(self.erosion_size, self.erosion_size))
        erosion_dst = cv.erode(gaussian, element)
        element = cv.getStructuringElement(cv.MORPH_RECT, (2 * self.dilatation_size + 1, 2 * self.dilatation_size + 1),(self.dilatation_size, self.dilatation_size))
        dilatation_dst = cv.dilate(erosion_dst, element)
        
        # Sum vertically through the image matrix in a lower and delimited region
        sum = np.sum(dilatation_dst[350:479,15:585], axis=0)
        
        # Obtain the index from the minim value in the array
        minim = np.argmin(sum)
        
        # Calculate the offset based on the previous image crop
        self.offset = minim + 50 - self.desired_width//2 
        
        
        image = original_resized
        # Draw a rectangle corresponding to the minim value
        cv.rectangle(image, (int(minim)+49,350), (int(minim)+51,479),(255,0,0))
        
        # Publish both the processed image and the value of the offset
        self.processed_image.publish(self.bridge.cv2_to_imgmsg(image, "mono8"))
        self.alignment_pub.publish(self.offset)



    def stop(self):
        print("Stopping")

    

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("line_detector")
    lines = lineDetector()
    rate = rospy.Rate(100)
    rospy.on_shutdown(lines.stop)

    print("processing")
    try:
        while not rospy.is_shutdown():
            #lines.run()
            rate.sleep()

    except rospy.ROSInterruptException():
        pass
        
