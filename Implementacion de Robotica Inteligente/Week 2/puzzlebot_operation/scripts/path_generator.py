#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from puzzlebot_operation.msg import pose

# Setup Variables, parameters and messages to be used (if required)


time = 0


#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("path_generator")
    start_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    pub = rospy.Publisher("pose", pose, queue_size = 10)
    puntos = pose()
    path = rospy.get_param("/path")
    camino = []

    for i in range(0,len(path)):
      camino.append(path[i][0])
      camino.append(path[i][1])
    puntos = camino

    print("Path Generator is Running")

    #Run the node
    while not rospy.is_shutdown():

      time = rospy.get_time() - start_time
      
      #Write your code here 
      pub.publish(puntos)

      rate.sleep()