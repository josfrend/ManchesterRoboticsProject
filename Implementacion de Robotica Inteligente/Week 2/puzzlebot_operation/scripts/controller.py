#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

wr = 0.
wl = 0.
r = 0.05
l = 0.18
pointSelf = [0,0]
direction = 0
distance = 0
angle = 0

def coordinates(x, y):
  xdf = pointSelf[0] - x 
  ydf = pointSelf[1] - y
  magnitude = np.sqrt(xdf ** 2 + ydf ** 2)
  v1_theta = math.atan2(pointSelf[1], pointSelf[0])
  v2_theta = math.atan2(y, x)
  angle = (v2_theta - v1_theta) * (180.0 / math.pi)
  #angle = np.rad2deg(np.atan2(ydf, xdf))
  movement = [magnitude, angle]
  return movement


def callbackWr(msg):
  global wr
  wr = msg.data

def callbackWl(msg):
  global wl
  wl = msg.data

def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
    pub2 = rospy.Publisher("distance", Vector3, queue_size = 10)
    rospy.Subscriber("/wr", Float32, callbackWr)
    rospy.Subscriber("/wl", Float32, callbackWl)
    mySetpoint = Twist()
    debug = Vector3()
    mySetpoint.linear.x = 0.
    mySetpoint.linear.y = 0.
    mySetpoint.linear.z = 0.
    mySetpoint.angular.x = 0.
    mySetpoint.angular.y = 0.
    mySetpoint.angular.z = 0.
    
    movement = [0,0]#rospy.get_param("/distance")

    print("Controller Node is Running")
    start_time = rospy.get_time()
    total_distance = 0.0
    total_angle = 0.0
    check = 0
    #Run the node
    while not rospy.is_shutdown():

      dt = rospy.get_time() - start_time
      if(check < 10):
        dt = 0.0
      distance = mySetpoint.linear.x * dt 
      angle = (mySetpoint.angular.z * r * (wr-wl)/l)*180/np.pi
      input_coordinates = [2,2]
      movement = coordinates(input_coordinates[0],input_coordinates[1])
      
      if(movement[1]-total_angle > 0):
        if(total_angle < movement[1]):
          mySetpoint.linear.x = 0.
          mySetpoint.angular.z = 0.5
        total_angle = total_angle + angle
      
      elif(movement[0]-total_distance > 0 and mySetpoint.angular.z == 0.):
        if(total_distance < movement[0] -0.1):
          mySetpoint.linear.x = 0.5
          mySetpoint.angular.z = 0.
        elif(total_distance >= movement[0] -0.1 and total_distance < movement[0]):
          mySetpoint.linear.x = 0.10
          mySetpoint.angular.z = 0.
        total_distance = total_distance + distance
      
      else:
        #direction += angle

        #if(angle < 360):
        #  angle = angle - (angle%360 * 360)
        mySetpoint.linear.x = 0.
        mySetpoint.angular.z = 0.
        #pointSelf = input_coordinates


      
      
      debug.x = total_angle
      debug.y = movement[1]
      debug.z = total_distance
      #Write your code here
      pub.publish(mySetpoint)
      pub2.publish(debug)
      start_time = rospy.get_time()
      check+=1

      rate.sleep()