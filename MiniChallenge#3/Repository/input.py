#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point
from std_msgs.msg import Int16

# Setup Variables, parameters and messages to be used (if required)

start_time = 0.0



#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("input")
    start_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)


    #Setup Publishers and subscribers here
    pub = rospy.Publisher("cmd_pwm", Int16, queue_size = 10)
    pub2 = rospy.Publisher("setpoint", Int16, queue_size = 10)
    mySetpoint = Int16()
    mySetpoint2 = Int16()
    mySetpoint2.data = 255

    print("Input Signal Generator is Running")

    #Run the node
    while not rospy.is_shutdown():
      typeSignal = rospy.get_param("/typeSignal")
      
      time = rospy.get_time() - start_time
      if(typeSignal == 1):
        mySetpoint.data = 250 * np.sin(time/5) + 250                 
      elif(typeSignal == 2):
        mySetpoint.data = 140 * np.sign(np.sin( np.pi * 1 * time / 5)) + 140
      else:                                                                                                   
        mySetpoint.data = 390
      #Write your code here 
      pub.publish(mySetpoint)
      pub2.publish(mySetpoint2)

      rate.sleep()
