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
l = 0.19
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
theta_k = 0
x_k = 0
y_k = 0
dt = 0
kp_angular = 0.5
ki_angular = 0.4
kd_angular = 0

kp_linear = 0.5
ki_linear = 0.4
kd_linear = 0

integral_angular = 0.0
last_error_angular = 0.0
integral_linear = 0.0
last_error_linear = 0.0


def PID_Angular(feedback, setpoint):
  global dt, integral_angular, last_error_angular

  error = setpoint - feedback
  derivative = (error - last_error_angular)
  integral_angular += error * dt

  output = kp_angular * error + ki_angular * integral_angular + kd_angular * derivative
  last_error_angular = error

  return output

def PID_Linear(feedback, setpoint):
  global dt, integral_linear, last_error_linear

  error = setpoint - feedback
  derivative = (error - last_error_linear)
  integral_linear += error * dt

  output = kp_linear * error + ki_linear * integral_linear + kd_linear * derivative
  last_error_linear = error

  return output

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
      #rospy.Subscriber("/pose", pose, callbackpath)
      input_path = [2,0, 2,2, 0,2, 0,0]

      

      
      dt = rospy.get_time() - start_time

      if(check < 5 or path != previous_path):
        dt = 0.0
        path = []
        for i in range(0,len(input_path)/2):
          path.append([input_path[i*2],input_path[i*2+1]])

        step = 0
      
      previous_path = path
     
 
      
      distance = (r*(wr + wl) /2) * dt 
      angle = (r*((wr - wl) /l)) * dt
      

      if(path != []):
        movement = coordinates(pointSelf,path[step])
        if(pointSelf == path[step] and len(path)-1>step):
          step += 1
 
      angle_correction = PID_Angular(total_angle, movement[1])
      distance_correction = PID_Linear(total_distance, movement[0])
      
      
      
      
      if(angle_correction > 0.001) or angle_correction < -0.001:
        speed.angular.z = angle_correction
        speed.linear.x = 0.0
        total_angle = total_angle + angle
      elif(distance_correction > 0.001 or distance_correction < -0.001):
        speed.angular.z = 0.0
        speed.linear.x = distance_correction
        total_distance = total_distance + distance
      else:
        speed.angular.z = 0.0
        speed.linear.x = 0.0




        
      if(total_angle > 2 * math.pi):
        total_angle= total_angle%math.pi

      if(path != [] and speed.linear.x == 0.0 and speed.angular.z == 0.0):
        pointSelf = path[step]

      debug.x = last_error_angular
      debug.y = last_error_linear
      debug.z = np.rad2deg(total_angle)
      #Write your code here
      pub.publish(speed)
      pub2.publish(debug)
      start_time = rospy.get_time()
      check+=1

      rate.sleep()