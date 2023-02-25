#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point
from datetime import datetime

# Setup Variables, parameters and messages to be used (if required)

set_up_delay = 0
start_time = 0.0

now = datetime.now()

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Set_Point_Generator")
    start_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    pub = rospy.Publisher("set_point", set_point, queue_size = 10)
    mySetpoint = set_point()

    print("The Set Point Genertor is Running")

    #Run the node
    while not rospy.is_shutdown():
      
      mySetpoint.time = rospy.get_time() - start_time
      mySetpoint.reference = 3.0
      #Write your code here
      pub.publish(mySetpoint)

      rate.sleep()