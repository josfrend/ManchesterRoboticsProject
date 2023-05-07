from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np

red_low_H = 0
red_low_S = 130
red_low_V = 179
red_high_H = 33
red_high_S = 255
red_high_V = 255

yellow_low_H = 20
yellow_low_S = 100
yellow_low_V = 100
yellow_high_H = 30
yellow_high_S = 255
yellow_high_V = 255

green_low_H = 49
green_low_S = 39
green_low_V = 130
green_high_H = 98
green_high_S = 255
green_high_V = 255
red = "Red Detector"
yellow = "Yellow Detector"
green = "Green Detector"


erosion_size = 7
dilatation_size = 0
max_elem = 2
max_kernel_size = 21


parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
cap = cv.VideoCapture(args.camera)
cv.namedWindow(red)
# cv.namedWindow(yellow)
# cv.namedWindow(green)

def blobs(val):
    element = cv.getStructuringElement(cv.MORPH_RECT, (2 * erosion_size + 1, 2 * erosion_size + 1),(erosion_size, erosion_size))
    erosion_dst = cv.erode(val, element)
    element = cv.getStructuringElement(cv.MORPH_RECT, (2 * dilatation_size + 1, 2 * dilatation_size + 1),(dilatation_size, dilatation_size))
    dilatation_dst = cv.dilate(erosion_dst, element)

    params = cv.SimpleBlobDetector_Params()
    params.filterByCircularity = True
    params.minCircularity = 1.0
    params.filterByArea = True
    params.minArea = 150
    params.maxArea = 400
    params.filterByInertia = True
    params.minInertiaRatio = 0.6

    detector = cv.SimpleBlobDetector_create()

    dilatation_dst = 255-dilatation_dst
    keypoints = detector.detect(dilatation_dst)

    #print("Number of matches found: ", len(keypoints))
    blank = np.zeros((1, 1))
    blobs_d = cv.drawKeypoints(dilatation_dst, keypoints, blank, (255, 0, 0),cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv.putText(blobs_d, str(len(keypoints)), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    return blobs_d



while True:
    
    ret, frame = cap.read()
    if frame is None:
        break
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    red_HSV = cv.inRange(frame_HSV, (red_low_H, red_low_S, red_low_V), (red_high_H, red_high_S, red_high_V))
    yellow_HSV = cv.inRange(frame_HSV, (yellow_low_H, yellow_low_S, yellow_low_V), (yellow_high_H, yellow_high_S, yellow_high_V))
    green_HSV = cv.inRange(frame_HSV, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))

    

    cv.imshow(red, blobs(red_HSV))
    cv.imshow(yellow, blobs(yellow_HSV))
    cv.imshow(green, blobs(green_HSV))
    
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
