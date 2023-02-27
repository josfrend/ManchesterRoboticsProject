#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point

# Setup Variables, parameters and messages to be used (if required)

start_time = 0.0


#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("setPoint")
    start_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    pub = rospy.Publisher("set_point", set_point, queue_size = 10)
    mySetpoint = set_point()

    print("Set Point Signal Generator is Running")

    #Run the node
    while not rospy.is_shutdown():
      
      mySetpoint.time = rospy.get_time() - start_time
      mySetpoint.reference = 2 * np.sin(mySetpoint.time)
      #Write your code here 
      pub.publish(mySetpoint)

      rate.sleep()