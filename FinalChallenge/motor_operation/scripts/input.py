#!/usr/bin/env python
import rospy
import numpy as np
from motor_operation.msg import set_point
from std_msgs.msg import Float32

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
    pub = rospy.Publisher("set_point", set_point, queue_size = 10)
    mySetpoint = set_point()

    print("Input Signal Generator is Running")

    #Run the node
    while not rospy.is_shutdown():
      typeSignal = rospy.get_param("/typeSignal")
      stepSignal = rospy.get_param("/step")
      sinAmp = rospy.get_param("/sinAmp")
      sinPeriod = rospy.get_param("/sinPeriod")
      squarePeriod = rospy.get_param("/squarePeriod")
      squareAmp = rospy.get_param("/squareAmp")
      
      time = rospy.get_time() - start_time
      if(typeSignal == 1):
        mySetpoint.data = sinAmp * np.sin(time/sinPeriod)                 
      elif(typeSignal == 2):
        mySetpoint.data = squareAmp*np.sign(np.sin( np.pi * 1 * time / squarePeriod))
      else:                                                                                                   
        mySetpoint.data = stepSignal
      mySetpoint.time = rospy.get_time() - start_time
      #Write your code here 
      pub.publish(mySetpoint)

      rate.sleep()