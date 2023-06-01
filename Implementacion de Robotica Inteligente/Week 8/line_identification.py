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
        self.last_offset = 300

        # Define erosion and dilation parameters for blob detection
        self.erosion_size = 7
        self.dilatation_size = 5
        self.max_elem = 2
        self.max_kernel_size = 21


        self.offset = 0
        


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

        
        borders = cv.Canny(dilatation_dst[350:479,:], 50, 50)
        lines = cv.HoughLinesP(borders, 1, np.pi/180, threshold=50, maxLineGap=10)

        
        if lines is not None:
            grouped_lines = []
            grouped_angles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2-y1,x2-x1)*180 /np.pi
                grouped = False

                for i, grouped_angle in enumerate(grouped_angles):
                    if(abs(angle - grouped_angle) < 2):
                        x1_grouped, y1_grouped, x2_grouped, y2_grouped = grouped_lines[i]
                        x1_grouped = min(x1,x1_grouped)
                        y1_grouped = min(y1,y1_grouped)
                        x2_grouped = min(x2,x2_grouped)
                        y2_grouped = min(y2,y2_grouped)

                        grouped_lines[i] = [x1_grouped, y1_grouped, x2_grouped, y2_grouped]

                        grouped = True
                        break
                if not grouped and ((angle > 30 and angle < 150) or (angle < -30 and angle > -150)):
                    grouped_lines.append([x1,y1,x2,y2])
                    grouped_angles.append(angle)
                    #cv.line(original_resized[350:479, :], (x1, y1), (x2, y2), (0, 255, 0), 2)



        minim = np.argmin(sum)
        min_values = []
        
        threshold = 15000
        for i in range(0,3):
            left = 0
            right = 560
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_min = min(x1,x2)
                x_max = max(x1,x2)
                if(x_min < minim and abs(minim - x_min) < abs(minim-left)):
                    left = x_min
                elif(x_max > minim and abs(x_max - minim) < abs(right - minim)):
                    right = x_max
            min_values.append(minim)
            if left > 150: left = left - 140
            if right < 400: right = right + 140
            elif right > 570: right = 569
            for j in range(left,right):
                sum[j] = 100000
            minim = np.argmin(sum)
    
        
        if abs(min_values[0]-min_values[1]) < 150:
            min_values = [min_values[0], min_values[2]]
        elif abs(min_values[0]-min_values[2]) < 150:
            min_values = [min_values[0], min_values[1]]
        elif abs(min_values[1]-min_values[2]) < 150:
            min_values = [min_values[0], min_values[1]]
        
        
        self.offset = minim + 15 - self.desired_width//2 
        
        min_values.sort()
        if(len(min_values) == 3):
            minim = min_values[1]
        else:
            if abs(self.last_offset - min_values[0]) < abs(self.last_offset - min_values[1]):
                minim = min_values[0]
            else:
                minim = min_values[1]

        #print(len(sum))
       

        image = original_resized
        cv.rectangle(image, (int(minim)+14,350), (int(minim)+16,479),(255,0,0))
        
        
    
        

        self.processed_image.publish(self.bridge.cv2_to_imgmsg(image, "mono8"))
        self.alignment_pub.publish(self.offset)
        self.last_offset = minim



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
