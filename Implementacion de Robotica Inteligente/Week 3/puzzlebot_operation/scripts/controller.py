#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from puzzlebot_operation.msg import pose

wr = 0.
wl = 0.
r = 0.05
l = 0.18
pointSelf = [0,0]
direction = 0
distance = 0
angle = 0
total_distance = 0.0
total_angle = 0.0
pause = [0,0]
movement = [0,0]
speed = Twist()
speed.linear.x = 0.
speed.linear.y = 0.
speed.linear.z = 0.
speed.angular.x = 0.
speed.angular.y = 0.
speed.angular.z = 0.
input_path = []
previous_path = []
path = []


def coordinates(origin, coordinate):
  xdf = coordinate[0] - origin[0]  
  ydf = coordinate[1] - origin[1]
  magnitude = np.sqrt(xdf ** 2 + ydf ** 2)
  #v1_theta = math.atan2(ydf, xdf)
  #v2_theta = math.atan2(coordinate[1], coordinate[0])
  #angle = (v2_theta - v1_theta) * (180.0 / math.pi)
  angle = (np.arctan2(ydf, xdf))
  if(angle < 0):
    angle = math.pi - angle
  movement = [magnitude, angle]
  return movement

def calculate_speeds(desired_time, path):
  total_magnitude = 0
  total_angle = 0
  entire_path = [pointSelf] + path
  for i in range(0,len(entire_path)-1):
    step_speeds = coordinates(entire_path[i], entire_path[i+1])
    total_magnitude += step_speeds[0]
    total_angle += step_speeds[1]
  angular_speed = total_angle/(desired_time/2)
  linear_speed = total_magnitude/(desired_time/2)
  return [linear_speed,angular_speed]


def callbackpath(msg):
  global input_path
  input_path = msg.pose
  
  


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
    debug = Vector3()
    pointSelf = [0,0]
    
    movement = [0,0]

    print("Controller Node is Running")
    start_time = rospy.get_time()
    step = 0
    check = 0
    

    
    #Run the node
    while not rospy.is_shutdown():

      #input_path = rospy.get_param("/path")
      rospy.Subscriber("/pose", pose, callbackpath)

      
      desired_time = rospy.get_param("/time")

      
      dt = rospy.get_time() - start_time

      if(check < 5 or path != previous_path):
        dt = 0.0
        path = []
        for i in range(0,len(input_path)/2):
          path.append([input_path[i*2],input_path[i*2+1]])
        velocities = calculate_speeds(desired_time, path)
        #if(velocities[0]>1.0 or velocities[1]>2.0):
         # path = []
        step = 0
      
      previous_path = path
     
 
      
      distance = speed.linear.x * dt 
      angle = (speed.angular.z * dt)
      #path = [[2,0], [2,2], [0,2], [0,0]]
      

      if(path != []):
        movement = coordinates(pointSelf,path[step])
        if(pointSelf == path[step] and len(path)-1>step):
          step += 1
      

      if(movement[1]-total_angle > 0.):
        speed.linear.x = 0.
        speed.angular.z = velocities[1]
        total_angle = total_angle + angle
        """elif(movement[1]-total_angle > 0.0 and movement[1]-total_angle <= 5):
        speed.linear.x = 0.
        speed.angular.z = 0.25
        total_angle = total_angle + angle"""

      elif(pause[0] < 50 and total_distance == 0):
        pause[0] += 1
        speed.linear.x = 0.
        speed.angular.z = 0.

      elif(movement[0]-total_distance > 0.0005 and speed.angular.z == 0.):
        speed.linear.x = velocities[0]
        speed.angular.z = 0.
        """if(movement[0] - total_distance > 0.1):
        elif(movement[0] - total_distance <= 0.1 and total_distance >= movement[0]):
          speed.linear.x = 0.25
          speed.angular.z = 0."""
        total_distance = total_distance + distance
      
      elif(pause[1] < 50):
        pause[1] += 1
        speed.linear.x = 0.
        speed.angular.z = 0.

      else:
        pause = [0,0]
        if(angle > math.pi):
          angle = angle%math.pi
        speed.linear.x = 0.
        speed.angular.z = 0.
        total_angle = movement[1]
        total_distance = 0
        if(path != []):
          pointSelf = path[step]

      debug.x = velocities[0]
      debug.y = velocities[1]
      debug.z = total_angle
      #Write your code here
      pub.publish(speed)
      pub2.publish(debug)
      start_time = rospy.get_time()
      check+=1

      rate.sleep()