#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from puzzlebot_operation.msg import pose

# Setup Variables, parameters and messages to be used (if required)


time = 0
pointSelf = [0,0]

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

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("path_generator")
    desired_time = rospy.get_param("/time")
    start_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    pub = rospy.Publisher("pose", pose, queue_size = 10)
    puntos = pose()
    input_path = rospy.get_param("/path")
    path = []

    check = 0

    print("Path Generator is Running")
    

    #Run the node
    while not rospy.is_shutdown():
      if(check == 0):
        for i in range(0,len(input_path)/2):
          path.append([input_path[i*2],input_path[i*2+1]])
        print(path)
      check +=1
      time = rospy.get_time() - start_time

      velocities = calculate_speeds(desired_time, path)
      if(velocities[0]>1.0 or velocities[1]>2.0):
        print("It is not possible to complete the path in the desired time frame due to dynamic limitations")
      else:
        puntos.pose = np.array(path).flatten().tolist()
        pub.publish(puntos)
      
      #Write your code here 
      

      rate.sleep()